//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! SITL (M14): the Rust flight code on the host, behind the same UDP sim-link the board serves (doc/specs/sim-link.md),
//! lockstep with the host's physics. tools/hitl_host.py `--sitl` flies it as it flies a board.
//!
//! The pieces are the board's own, from the same crates: the topology the board's bring-up builds from its config
//! (crates/controller `topology`, the `hitl` entry its only provider: every sensor it masks is simulated), a SENSORS
//! frame published into it as the `hitl` source (`HitlWriters`), and the board's control tick itself
//! (crates/controller `ControlCore`: the attitude filter, the stage machine and the flight law, in the board's order,
//! R3 of doc/specs/avionics-refactor.md), with the mission from the launch payload the host sends a board. The SITL is
//! the board's code path minus the platform, never a copy of it. The fins are the board's servos with no PWM behind
//! them (crates/drivers `Servo`, each from its config entry: its clamp and its step, `step_deg`), so a FINS answer
//! carries the angles they hold, as a board's does. What the host has none of is not here: the separation switch,
//! the checkpoint and the warm start, the GNSS drift calibration (no drift), the recorder (no rows).
//!
//! Lockstep: the clock is the frame's physics time (`sim_time_ms`, after a 10 s boot as the golden shims'), never the
//! wall's. Each frame first runs the control ticks due before it on what the earlier frames left, then is published,
//! then runs the tick due at its own time, then is answered: what a realtime board sees, every run the same.
//!
//!   sitl --config <board config JSON> [--mission <launch JSON>] [--port 1236] [--trace]

use std::net::UdpSocket;

use control::flight::FlightLaw;
use control::guidance::Landing;
use control::navigation::Position;
use controller::{ControlCore, FilterSetup, FlightSetup, HitlWriters, Hooks, SequencerSetup, Topology};
use drivers::sg90::{Servo, Settings};
use mission::Mission;
use recorder::Rows;
use sequencer::{FlightWord, Request, Stage};
use serde_json::Value;
use sim_link::{Fins, Sensors};

/// The clock at physics time zero, µs: a board's boot (the golden shims' 10 s), so no stamp starts at zero.
const BOOT_US: u64 = 10_000_000;

/// The sim-link's port when none is given (doc/specs/sim-link.md).
const SIM_PORT: u16 = 1236;

/// The largest datagram read.
const DATAGRAM_BYTES: usize = 256;

/// The FINS answer's fins, by the mixer's surface names (plan B's), in the frame's order.
const SURFACES: [&str; 3] = ["servo_eleron_left", "servo_eleron_right", "servo_yaw"];

/// The command line.
struct Arguments {
    /// The board config.
    config: String,
    /// The launch config (the payload the host sends a board's `update mission`).
    mission: Option<String>,
    /// The UDP port.
    port: u16,
    /// `--trace`: the filter's attitude every 100 ms on stderr (`ATT <ms> <heading_cd> <roll_cd> <pitch_cd>`) and the
    /// flight law's wind estimate every second (`WIND <ms> <east> <north>`, m/s the air moves toward), for the host to
    /// score against its truth (tools/hitl_host.py `ATTITUDE`).
    trace: bool,
}

/// The command line, or why it is wrong.
fn arguments() -> Result<Arguments, String> {
    let mut parsed = Arguments {
        config: String::new(),
        mission: None,
        port: SIM_PORT,
        trace: false,
    };
    let mut words = std::env::args().skip(1);
    while let Some(word) = words.next() {
        let mut value = || words.next().ok_or_else(|| format!("{word} takes a value"));
        match word.as_str() {
            "--config" => parsed.config = value()?,
            "--mission" => parsed.mission = Some(value()?),
            "--port" => parsed.port = value()?.parse().map_err(|_| "--port takes a number".to_string())?,
            "--trace" => parsed.trace = true,
            other => return Err(format!("unknown argument {other}")),
        }
    }
    if parsed.config.is_empty() {
        return Err(
            "usage: sitl --config <board config JSON> [--mission <launch JSON>] [--port 1236] [--trace]".to_string(),
        );
    }
    Ok(parsed)
}

/// The host's side of the control tick: the clock is the tick's own time (lockstep), the mission the launch
/// payload's, no drift, no recorder flag, and the console is stderr.
struct Host {
    /// The tick being run, µs.
    now_us: u64,
    /// The mission's landing zone.
    landing: Option<Landing>,
    /// The mission's set launch point.
    launch: Option<Position>,
    /// The fins by the law's surfaces (None for a surface no servo entry drives), as the board's fins thread holds
    /// them.
    servos: Vec<Option<Fin>>,
    /// `--trace`.
    trace: bool,
    /// When the last `WIND` line went out, ms.
    wind_ms: u64,
}

/// One fin: its servo and what the flight loop made it do.
struct Fin {
    /// The servo, its clamp and step from the fin's config entry.
    servo: Servo,
    /// The commands it wrote.
    writes: u32,
    /// The writes that changed its angle (a step of 0 rewrites a held one).
    moves: u32,
    /// The degrees those commands moved it.
    degrees: u32,
}

impl Hooks for Host {
    fn now_us(&mut self) -> u64 {
        self.now_us
    }

    fn mission(&mut self) -> (Option<Landing>, Option<Position>) {
        (self.landing, self.launch)
    }

    fn drift(&mut self) -> [f32; 2] {
        [0.0; 2]
    }

    fn flight_flag(&mut self, _raised: bool) {}

    fn moved(&mut self, stage: Stage) {
        eprintln!("sitl :: {} ms stage -> {}", self.now_us / 1000, stage.name());
    }

    fn veto_edge(&mut self) {}

    fn attitude(&mut self, row: [i32; 6], _seeded: bool, _blind: bool) {
        // the filter steps on the 20 ms grid: every fifth step, on the whole 100 ms
        if self.trace && (self.now_us / 1000).is_multiple_of(100) {
            eprintln!("ATT {} {} {} {}", self.now_us / 1000, row[0], row[1], row[2]);
        }
    }

    fn fins(&mut self, angles: &[i32]) {
        for (&angle, fin) in angles.iter().zip(&mut self.servos) {
            let Some(fin) = fin else {
                continue;
            };
            let held = fin.servo.angle();
            if let Some(command) = fin.servo.set_angle(f64::from(angle)) {
                fin.writes += 1;
                fin.moves += u32::from(command.angle != held);
                fin.degrees += command.angle.abs_diff(held);
            }
        }
    }

    fn vitals(&mut self, law: &FlightLaw) {
        let now_ms = self.now_us / 1000;
        if self.trace && now_ms >= self.wind_ms + 1000 {
            self.wind_ms = now_ms;
            let (east, north) = law.wind().components();
            eprintln!("WIND {now_ms} {east:.2} {north:.2}");
        }
    }
}

/// The control tick's lines (crates/controller logs as the board does) on stderr.
struct Stderr;

impl log::Log for Stderr {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        metadata.level() <= log::Level::Info
    }

    fn log(&self, record: &log::Record) {
        if self.enabled(record.metadata()) {
            eprintln!("{}", record.args());
        }
    }

    fn flush(&self) {}
}

/// The control tick of `document` (masked) on `topology`, a tick of `tick_us`: the board's duties, set up from the
/// config's `attitude`, `sequencer` and `flight` entries, no rows recorded.
fn control_core(document: &'static Value, topology: &'static Topology, tick_us: u64) -> Result<ControlCore, String> {
    let entries = board_config::directory(document);
    let entry = |implementation: &str| entries.iter().find(|entry| entry.implementation == implementation);
    let fields = |implementation: &str| {
        entry(implementation)
            .map(|entry| Value::Object(entry.fields.clone()))
            .ok_or_else(|| format!("no enabled '{implementation}' entry in the config"))
    };
    let sequencer = entry("sequencer").ok_or("no enabled 'sequencer' entry in the config")?;
    let settings = sequencer::Settings::from_config(&fields("sequencer")?).map_err(|error| error.to_string())?;
    let filter = match entry(ATTITUDE) {
        Some(attitude) => Some(FilterSetup {
            name: attitude.name,
            settings: control::attitude::Settings::from_json(&Value::Object(attitude.fields.clone())),
            rows: Rows::new(attitude.name, "row", None),
        }),
        None => {
            eprintln!("sitl :: no enabled attitude filter in the config: a dropped attitude leaves the law none");
            None
        }
    };
    Ok(ControlCore::new(
        topology,
        tick_us,
        BOOT_US,
        filter,
        Some(SequencerSetup {
            name: sequencer.name,
            settings,
            rows: Rows::new(sequencer.name, "record", None),
            vetoes: Rows::new(sequencer.name, "vetoes", None),
        }),
        Some(FlightSetup {
            entry: fields("flight")?,
            fins: document["fins"].clone(),
            rows: Rows::new("flight", "row", None),
            seed_airspeed: None,
        }),
    ))
}

/// The fins of `law`'s surfaces, each from its servo entry in `entries` (sg90 or mg90s, as the board's bring-up
/// reads it); None for a surface with none.
fn servos(entries: &[board_config::Entry<'_>], law: Option<&FlightLaw>) -> Vec<Option<Fin>> {
    let Some(law) = law else {
        return Vec::new();
    };
    let fin = |name: &str| {
        let entry = entries.iter().find(|entry| entry.name == name)?;
        let servo_type = match entry.implementation {
            "sg90" => drivers::sg90::SG90,
            "mg90s" => drivers::mg90s::MG90S,
            _ => return None,
        };
        let settings = Settings::from_config(&Value::Object(entry.fields.clone()), &servo_type);
        Some(Fin {
            servo: Servo::new(&servo_type, &settings),
            writes: 0,
            moves: 0,
            degrees: 0,
        })
    };
    law.mixer()
        .surfaces()
        .iter()
        .map(|surface| fin(&surface.name))
        .collect()
}

/// The FINS answer's fins: the angles the servos hold, whole degrees, by the frame's surfaces; None unless the law
/// drives all three.
fn fins(law: Option<&FlightLaw>, servos: &[Option<Fin>]) -> Option<[i16; 3]> {
    let surfaces = law?.mixer().surfaces();
    let mut fins = [0i16; 3];
    for (fin, name) in fins.iter_mut().zip(SURFACES) {
        let index = surfaces.iter().position(|surface| surface.name == name)?;
        *fin = servos.get(index)?.as_ref()?.servo.angle() as i16;
    }
    Some(fins)
}

/// The fins' tally at DONE, one line: each fin's writes, the moves among them and the degrees they moved it.
fn tally(law: Option<&FlightLaw>, servos: &[Option<Fin>]) -> String {
    let surfaces = law.map(|law| law.mixer().surfaces()).unwrap_or_default();
    let fins: Vec<String> = surfaces
        .iter()
        .zip(servos)
        .filter_map(|(surface, fin)| {
            let fin = fin.as_ref()?;
            Some(format!("{}={}/{}/{}", surface.name, fin.writes, fin.moves, fin.degrees))
        })
        .collect();
    format!("sitl :: servos (writes/moves/degrees) {}", fins.join(" "))
}

/// The attitude filter's implementation name in the config (its entry's `activity`).
const ATTITUDE: &str = "attitude";

fn main() -> Result<(), String> {
    let _ = log::set_logger(&Stderr).map(|()| log::set_max_level(log::LevelFilter::Info));
    let arguments = arguments()?;
    let text = std::fs::read_to_string(&arguments.config).map_err(|error| format!("{}: {error}", arguments.config))?;
    let mut document: Value = serde_json::from_str(&text).map_err(|error| format!("{}: {error}", arguments.config))?;
    let masked = board_config::mask_simulated(&mut document);
    eprintln!("sitl :: simulated, so masked: {}", masked.join(", "));
    let document: &'static Value = Box::leak(Box::new(document));
    // only the hitl entry and the attitude filter come up on the host: the hitl's quantities are every sensor the
    // config masked, the filter backs its attitude up as on a board
    let entries = board_config::directory(document);
    let outcomes = entries
        .iter()
        .map(|entry| {
            let outcome = if entry.implementation == "hitl" || entry.implementation == ATTITUDE {
                Ok(())
            } else {
                Err("not on the host".to_string())
            };
            (controller::candidate(document, entry), outcome)
        })
        .collect();
    let table = controller::Table::new(outcomes);
    let (topology, warnings) = controller::topology(&entries, &table);
    for line in warnings.iter().chain(&topology.describe()) {
        eprintln!("{line}");
    }
    let topology: &'static Topology = Box::leak(Box::new(topology));
    let source = entries
        .iter()
        .find(|entry| entry.implementation == "hitl")
        .map(|entry| entry.name)
        .ok_or("no enabled 'hitl' entry in the config")?;
    let mission_text = match &arguments.mission {
        Some(path) => Some(std::fs::read_to_string(path).map_err(|error| format!("{path}: {error}"))?),
        None => None,
    };
    let mission = Mission::load(mission_text.as_deref(), f32::INFINITY);
    let mut host = Host {
        now_us: BOOT_US,
        landing: mission
            .zone
            .map(|zone| Landing::new(zone.top_left, zone.bottom_right, mission.endgame_heading())),
        launch: mission.launch_point(None),
        servos: Vec::new(),
        trace: arguments.trace,
        wind_ms: 0,
    };
    let word = FlightWord::default();
    let mut writer = word.writer().ok_or("the flight word's writer is taken")?;
    // armed on the pad, as the host arms a board before the launch
    writer.apply(Request::Arm).map_err(|problem| problem.to_string())?;
    let mut writers = HitlWriters::new(topology, source);
    let tick_us = 1_000_000 / u64::from(board_config::schedule_hz(document).max(1));
    let mut core = control_core(document, topology, tick_us)?;
    host.servos = servos(&entries, core.law());
    let socket = UdpSocket::bind(("0.0.0.0", arguments.port)).map_err(|error| error.to_string())?;
    eprintln!("sitl :: sim-link on udp:{}, a tick every {tick_us} us", arguments.port);
    let mut last_seq: Option<u32> = None;
    let mut next_tick_us = BOOT_US;
    let mut tallied = false;
    let mut datagram = [0u8; DATAGRAM_BYTES];
    loop {
        let (length, sender) = socket.recv_from(&mut datagram).map_err(|error| error.to_string())?;
        let Ok(sensors) = Sensors::decode(&datagram[..length]) else {
            continue;
        };
        if last_seq.is_some_and(|last| !sim_link::newer(sensors.seq, last)) {
            continue;
        }
        last_seq = Some(sensors.seq);
        let now_us = BOOT_US + u64::from(sensors.sim_time_ms) * 1000;
        // the ticks before this frame on what the earlier frames left, then the frame, then the tick at its time
        while next_tick_us < now_us {
            tick(&mut core, &mut host, next_tick_us, tick_us, &mut writer);
            next_tick_us += tick_us;
        }
        writers.publish(&sensors, now_us);
        if next_tick_us == now_us {
            tick(&mut core, &mut host, next_tick_us, tick_us, &mut writer);
            next_tick_us += tick_us;
        }
        let state = writer.flight();
        let answer = Fins {
            seq: sensors.seq,
            uptime_ms: (now_us / 1000) as u32,
            stage: state.stage as u8,
            armed: state.armed,
            launched: state.launched,
            fins: fins(core.law(), &host.servos),
        };
        // logged before the answer: the host stops the binary once it reads DONE
        if state.stage == Stage::Done {
            eprintln!("sitl :: done at {} ms", now_us / 1000);
            if !tallied {
                eprintln!("{}", tally(core.law(), &host.servos));
                tallied = true;
            }
        }
        socket
            .send_to(&answer.encode(), sender)
            .map_err(|error| error.to_string())?;
    }
}

/// One control tick at `tick_at_us` of a `tick_us` grid, as the board's control thread runs it: DONE as it stood at
/// the wake, half a tick of slack.
fn tick(
    core: &mut ControlCore,
    host: &mut Host,
    tick_at_us: u64,
    tick_us: u64,
    writer: &mut sequencer::FlightWriter<'_>,
) {
    host.now_us = tick_at_us;
    let done = writer.flight().stage == Stage::Done;
    core.tick(tick_at_us, tick_us / 2, done, writer, host);
}
