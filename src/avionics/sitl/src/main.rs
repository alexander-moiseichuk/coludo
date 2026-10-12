//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! SITL (M14): the Rust flight code on the host, behind the same UDP sim-link the board serves (doc/specs/sim-link.md),
//! lockstep with the host's physics. tools/hitl_host.py `--sitl` flies it as it flies a board.
//!
//! The pieces are the board's own, from the same crates: the topology the board's bring-up builds from its config
//! (crates/controller `topology`, the `hitl` entry its only provider: every sensor it masks is simulated), a SENSORS
//! frame published into it as the `hitl` source (`HitlWriters`), the stage machine stepped on its per-source readings
//! (`StageSources`, `StageMachine`) and the flight law on the fused ones (`FlightQuantities`, `FlightLaw`), the mission
//! from the launch payload the host sends a board, and the attitude filter, stepped before the law on each tick as the
//! board's control thread steps it (in HITL the simulated attitude outranks it and the filter mirrors it, warm, until
//! that attitude drops out: then the filter carries the glide, as on a board -- without it here a dropped attitude
//! left the law with none, and the SITL flew 1.8 km off where a board would not). What the host has none of is not
//! here: the servos (a FINS answer carries the law's angles, as a board's centred fins at its start), the separation
//! switch, the checkpoint and the warm start, the GNSS drift calibration.
//!
//! Lockstep: the clock is the frame's physics time (`sim_time_ms`, after a 10 s boot as the golden shims'), never the
//! wall's. Each frame first runs the control ticks due before it on what the earlier frames left, then is published,
//! then runs the tick due at its own time, then is answered: what a realtime board sees, every run the same.
//!
//!   sitl --config <board config JSON> [--mission <launch JSON>] [--port 1236]

use std::net::UdpSocket;

use control::attitude::{Attitude, Pace, Quantities, assemble};
use control::flight::FlightLaw;
use control::guidance::Landing;
use controller::{FlightQuantities, HitlWriters, StageSources, Topology};
use mission::Mission;
use sequencer::{FlightWord, FlightWriter, Request, Stage, StageMachine};
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
}

/// The command line, or why it is wrong.
fn arguments() -> Result<Arguments, String> {
    let mut parsed = Arguments {
        config: String::new(),
        mission: None,
        port: SIM_PORT,
    };
    let mut words = std::env::args().skip(1);
    while let Some(word) = words.next() {
        let mut value = || words.next().ok_or_else(|| format!("{word} takes a value"));
        match word.as_str() {
            "--config" => parsed.config = value()?,
            "--mission" => parsed.mission = Some(value()?),
            "--port" => parsed.port = value()?.parse().map_err(|_| "--port takes a number".to_string())?,
            other => return Err(format!("unknown argument {other}")),
        }
    }
    if parsed.config.is_empty() {
        return Err("usage: sitl --config <board config JSON> [--mission <launch JSON>] [--port 1236]".to_string());
    }
    Ok(parsed)
}

/// The flight code of one board config.
struct Flight {
    /// The stage machine's step period, µs.
    stage_period_us: u64,
    /// When its next step is due, µs.
    stage_due_us: u64,
    /// The stage machine.
    machine: StageMachine,
    /// Its sources.
    sources: StageSources<'static>,
    /// The flight law.
    law: FlightLaw,
    /// What it reads.
    quantities: FlightQuantities<'static>,
    /// The mission's landing zone.
    landing: Option<Landing>,
    /// The mission's set launch point.
    launch: Option<control::navigation::Position>,
    /// Each FINS fin's index among the mixer's surfaces.
    surfaces: [Option<usize>; 3],
}

impl Flight {
    /// The flight code of `document` (masked) on `topology`, flying `mission`.
    fn new(document: &'static Value, topology: &'static Topology, mission: &Mission) -> Result<Flight, String> {
        let entries = board_config::directory(document);
        let entry = |implementation: &str| {
            entries
                .iter()
                .find(|entry| entry.implementation == implementation)
                .map(|entry| Value::Object(entry.fields.clone()))
                .ok_or_else(|| format!("no enabled '{implementation}' entry in the config"))
        };
        let settings = sequencer::Settings::from_config(&entry("sequencer")?).map_err(|error| error.to_string())?;
        let law = FlightLaw::new(&entry("flight")?, &document["fins"]);
        let surfaces = SURFACES.map(|name| law.mixer().surfaces().iter().position(|surface| surface.name == name));
        let landing = mission
            .zone
            .map(|zone| Landing::new(zone.top_left, zone.bottom_right, mission.endgame_heading()));
        Ok(Flight {
            stage_period_us: u64::from(settings.period_ms) * 1000,
            stage_due_us: 0,
            machine: StageMachine::new(settings),
            sources: StageSources::new(topology),
            law,
            quantities: FlightQuantities::new(topology),
            landing,
            launch: mission.launch_point(None),
            surfaces,
        })
    }

    /// One control tick at `now_us`: the stage machine when its step is due (the board's order: the machine, then the
    /// law), then the law.
    fn tick(&mut self, now_us: u64, writer: &mut FlightWriter<'_>) {
        if now_us >= self.stage_due_us {
            self.stage_due_us = (self.stage_due_us + self.stage_period_us).max(now_us + self.stage_period_us / 2);
            let flight = writer.flight();
            let step = self.machine.step(
                (now_us / 1000) as u32,
                flight.stage,
                self.sources.readings(now_us),
                flight.manual,
            );
            if let Some(transition) = step.advance {
                writer.advance(transition);
                eprintln!(
                    "sitl :: {} ms stage -> {} ({})",
                    now_us / 1000,
                    transition.to.name(),
                    transition.reason
                );
            }
        }
        let flight = writer.flight();
        let step = self
            .quantities
            .step(now_us, flight.stage, flight.armed, self.landing, self.launch, [0.0; 2]);
        self.law.step(&step);
    }

    /// The FINS answer's fins: the law's angles, whole degrees, by the frame's surfaces; None unless it drives all
    /// three.
    fn fins(&self) -> Option<[i16; 3]> {
        let angles = self.law.angles();
        let mut fins = [0i16; 3];
        for (fin, index) in fins.iter_mut().zip(self.surfaces) {
            *fin = *angles.get(index?)? as i16;
        }
        Some(fins)
    }
}

/// The attitude filter as the board's control thread steps it (firmware/src/control.rs `Filter`): paced on its
/// `period_ms`, reading the fused `rate`, `accel` and `mag`, its estimate pushed as its entry's `attitude` source.
struct Filter {
    /// The entry's name: the source of `attitude`, its log lines'.
    name: &'static str,
    /// The filter.
    attitude: Attitude,
    /// The quantities it reads and publishes into.
    quantities: Quantities<'static>,
    /// Which ticks step it.
    pace: Pace,
    /// The rank it publishes at (its `provides.attitude.priority`).
    rank: i32,
    /// The `attitude` writer.
    writer: Option<databoard::Writer<'static, [f32; 3]>>,
}

impl Filter {
    /// The config's attitude filter on a tick of `tick_us`; None (logged) without its entry or an `attitude` quantity.
    fn new(document: &'static Value, topology: &'static Topology, tick_us: u64) -> Option<Filter> {
        let entries = board_config::directory(document);
        let Some(entry) = entries.iter().find(|entry| entry.implementation == ATTITUDE) else {
            eprintln!("sitl :: no enabled attitude filter in the config: a dropped attitude leaves the law none");
            return None;
        };
        let attitude = topology.vector("attitude")?;
        let settings = control::attitude::Settings::from_json(&Value::Object(entry.fields.clone()));
        eprintln!(
            "{} :: every {} ms at rank {}, reading rate, accel and mag",
            entry.name, settings.period_ms, settings.rank
        );
        Some(Filter {
            name: entry.name,
            attitude: Attitude::new(settings, BOOT_US),
            quantities: Quantities {
                rate: topology.vector("rate"),
                acceleration: topology.vector("accel"),
                magnetic: topology.vector("mag"),
                attitude,
            },
            pace: Pace::new(settings.period_ms, tick_us),
            rank: settings.rank,
            writer: attitude.writer(entry.name),
        })
    }

    /// One control tick at `now_us`: when its pace is due, one step on the readings fresh now, its estimate pushed.
    fn tick(&mut self, now_us: u64) {
        if !self.pace.due(now_us) {
            return;
        }
        let readings = assemble(&self.quantities, self.rank, now_us);
        let step = self.attitude.tick(now_us, &readings);
        if let Some(line) = step.log {
            eprintln!("{} :: {line}", self.name);
        }
        if let (Some(orientation), Some(writer)) = (step.published, &mut self.writer) {
            writer.push(orientation.to_sample(), now_us);
        }
    }
}

/// The attitude filter's implementation name in the config (its entry's `activity`).
const ATTITUDE: &str = "attitude";

fn main() -> Result<(), String> {
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
    let mut flight = Flight::new(document, topology, &mission)?;
    let word = FlightWord::default();
    let mut writer = word.writer().ok_or("the flight word's writer is taken")?;
    // armed on the pad, as the host arms a board before the launch
    writer.apply(Request::Arm).map_err(|problem| problem.to_string())?;
    let mut writers = HitlWriters::new(topology, source);
    let tick_us = 1_000_000 / u64::from(board_config::schedule_hz(document).max(1));
    let mut filter = Filter::new(document, topology, tick_us);
    let socket = UdpSocket::bind(("0.0.0.0", arguments.port)).map_err(|error| error.to_string())?;
    eprintln!("sitl :: sim-link on udp:{}, a tick every {tick_us} us", arguments.port);
    let mut last_seq: Option<u32> = None;
    let mut next_tick_us = BOOT_US;
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
        // the filter before the stage machine and the law, as the board's control thread orders them
        while next_tick_us < now_us {
            if let Some(filter) = &mut filter {
                filter.tick(next_tick_us);
            }
            flight.tick(next_tick_us, &mut writer);
            next_tick_us += tick_us;
        }
        writers.publish(&sensors, now_us);
        if next_tick_us == now_us {
            if let Some(filter) = &mut filter {
                filter.tick(next_tick_us);
            }
            flight.tick(next_tick_us, &mut writer);
            next_tick_us += tick_us;
        }
        let state = writer.flight();
        let answer = Fins {
            seq: sensors.seq,
            uptime_ms: (now_us / 1000) as u32,
            stage: state.stage as u8,
            armed: state.armed,
            launched: state.launched,
            fins: flight.fins(),
        };
        socket
            .send_to(&answer.encode(), sender)
            .map_err(|error| error.to_string())?;
        if state.stage == Stage::Done {
            eprintln!("sitl :: done at {} ms", now_us / 1000);
        }
    }
}
