//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The HITL sim-link's board end (doc/specs/sim-link.md, M12), only with an enabled `hitl` entry: one thread (core 0,
//! priority 12, PSRAM stack) owns a UDP socket on the entry's `sim_port`. Each SENSORS frame the host sends that
//! decodes and is newer than the last one applied is published at once: every present quantity through the `hitl`
//! source's databoard writers (priority 0), stamped with the frame's arrival on this board's clock, `change` derived
//! from the altitude as a barometer derives it, and a row in each simulated sensor's own stream in its driver's shape,
//! so the session reads as a flight's. Each applied frame is answered with a FINS frame to its sender: the stage, the
//! arming, the launch, and no fins (M12 flies passive).
//!
//! A frame that is refused (crates/sim_link `Refusal`) or not newer is dropped and counted; `inspect hitl` shows the
//! counts. The real sensors the simulator replaces were masked at boot (board_config::mask_simulated), and the
//! entry refused to come up if one still runs (crate::devices).

use std::net::{SocketAddr, UdpSocket};
use std::sync::OnceLock;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::Duration;

use ::sim_link::{Fins, Refusal, Sensors};
use controller::HitlWriters;
use esp_idf_svc::hal::cpu::Core;
use recorder::{Cell, Rows};

use crate::devices::{HitlMember, Topology};
use crate::status::FLIGHT;
use firmware::threads::{self, Spec, Stack};
use firmware::time::uptime_us;

/// The thread's stack (PSRAM): the socket calls, one decode and the rows.
const STACK_SIZE: usize = 8 << 10;

/// The thread's priority on core 0 (avionics-port.md's thread table).
const PRIORITY: u8 = 12;

/// A receive buffer longer than any frame, so an oversized datagram reads as a wrong length, not a truncated frame.
const DATAGRAM_BYTES: usize = 256;

/// Knots per m/s, for the GNSS row's speed column (plan B hitl.py `_KNOTS`).
const KNOTS_PER_METRE_PER_SECOND: f32 = 1.943_84;

/// The barometer row's constant temperature and pressure: the simulator models neither (plan B hitl.py:315).
const SIMULATED_TEMPERATURE_C: f32 = 21.0;
/// See [`SIMULATED_TEMPERATURE_C`].
const SIMULATED_PRESSURE_PA: f32 = 100_000.0;

/// The pitot row's constant temperature, C x 100 (plan B hitl.py:288).
const SIMULATED_PITOT_TEMPERATURE: i32 = 2500;

/// The simulated sensors whose streams the frames are recorded into, by the config names their streams carry.
const IMU: &str = "imu_lsm6dso32";
/// See [`IMU`].
const BAROMETER: &str = "baro_icp10111";
/// See [`IMU`].
const GNSS: &str = "gnss";
/// See [`IMU`].
const LASER: &str = "laser_agl";
/// See [`IMU`].
const PITOT: &str = "airspeed_sdp810";
/// See [`IMU`].
const MAGNETOMETER: &str = "mag_bmm350";

/// Frames applied since boot.
static APPLIED: AtomicU32 = AtomicU32::new(0);
/// Frames refused, by reason: length, magic, version, kind, CRC, not finite.
static REFUSED: [AtomicU32; 6] = [const { AtomicU32::new(0) }; 6];
/// Frames that decoded but were not newer than the last applied (late, repeated).
static STALE: AtomicU32 = AtomicU32::new(0);
/// The last applied frame's seq and sim time (ms).
static LAST: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];
/// When the last frame was applied, uptime ms, wrapping (no 64-bit atomics on this core).
static LAST_MS: AtomicU32 = AtomicU32::new(0);
/// Whether the socket is bound.
static BOUND: AtomicBool = AtomicBool::new(false);
/// The entry's name and port, once the thread runs.
static RUNNING: OnceLock<(&'static str, u16)> = OnceLock::new();
/// The last frame's sender for inspect: its IPv4 address and its port (port 0: none yet, or not IPv4).
static SENDER: [AtomicU32; 2] = [const { AtomicU32::new(0) }; 2];

/// Whether the sim-link runs this boot: the link then joins again after the launch (crate::link), since the
/// simulated sensors ride the Wi-Fi.
pub fn running() -> bool {
    RUNNING.get().is_some()
}

/// Starts the sim-link thread for `member`, its quantities found in `topology`.
///
/// Errors: the thread would not spawn.
pub fn start(member: HitlMember, topology: &'static Topology) -> anyhow::Result<()> {
    // before the spawn: the link reads it from its first tick
    let _ = RUNNING.set((member.name, member.port));
    threads::spawn(
        Spec {
            name: c"sim_link",
            stack_size: STACK_SIZE,
            stack: Stack::Psram,
            priority: PRIORITY,
            core: Core::Core0,
        },
        move || run(member, topology),
    )?;
    Ok(())
}

/// The simulated sensors' streams, each recording when its entry has one registered, a refused row logged once a run
/// under the hitl entry's name (recorder::Rows).
struct Streams {
    /// imu_lsm6dso32.csv: accel g, rate centideg/s, irq_runs.
    imu: Rows<'static>,
    /// baro_icp10111.csv: altitude, temperature, pressure, elevation, change.
    barometer: Rows<'static>,
    /// gnss.csv: the fix.
    gnss: Rows<'static>,
    /// laser_agl.csv: agl, irq_runs.
    laser: Rows<'static>,
    /// airspeed_sdp810.csv: Pa x 100, cm/s, C x 100.
    pitot: Rows<'static>,
    /// mag_bmm350.csv: counts.
    magnetometer: Rows<'static>,
}

/// The thread: bind, then frames forever.
fn run(member: HitlMember, topology: &'static Topology) {
    let name = member.name;
    let source = name;
    // the hitl source's writers and the climb its altitude derives (crates/controller, the SITL's too)
    let mut writers = HitlWriters::new(topology, source);
    let stream = |entry: &str| Rows::new(name, "telemetry", crate::recorder::telemetry(entry));
    let mut streams = Streams {
        imu: stream(IMU),
        barometer: stream(BAROMETER),
        gnss: stream(GNSS),
        laser: stream(LASER),
        pitot: stream(PITOT),
        magnetometer: stream(MAGNETOMETER),
    };
    let socket = bind(name, member.port);
    BOUND.store(true, Ordering::Relaxed);
    log::info!("{name} :: sim-link on udp:{}", member.port);
    let mut last_seq: Option<u32> = None;
    let mut datagram = [0u8; DATAGRAM_BYTES];
    loop {
        let Ok((length, sender)) = socket.recv_from(&mut datagram) else {
            continue;
        };
        let now_us = uptime_us();
        let sensors = match Sensors::decode(&datagram[..length]) {
            Ok(sensors) => sensors,
            Err(refusal) => {
                REFUSED[refusal_index(refusal)].fetch_add(1, Ordering::Relaxed);
                continue;
            }
        };
        if last_seq.is_some_and(|last| !::sim_link::newer(sensors.seq, last)) {
            STALE.fetch_add(1, Ordering::Relaxed);
            continue;
        }
        last_seq = Some(sensors.seq);
        let change = writers.publish(&sensors, now_us);
        record(&sensors, change, now_us, &mut streams);
        APPLIED.fetch_add(1, Ordering::Relaxed);
        LAST[0].store(sensors.seq, Ordering::Relaxed);
        LAST[1].store(sensors.sim_time_ms, Ordering::Relaxed);
        LAST_MS.store((now_us / 1000) as u32, Ordering::Relaxed);
        if let SocketAddr::V4(sender) = sender {
            SENDER[0].store(u32::from(*sender.ip()), Ordering::Relaxed);
            SENDER[1].store(u32::from(sender.port()), Ordering::Relaxed);
        }
        let flight = FLIGHT.load();
        let answer = Fins {
            seq: sensors.seq,
            uptime_ms: (now_us / 1000) as u32,
            stage: flight.stage as u8,
            armed: flight.armed,
            launched: flight.launched,
            fins: fin_angles(),
        };
        let _ = socket.send_to(&answer.encode(), sender);
    }
}

/// The socket on `port`, once the TCP/IP stack is up (a socket before it asserts in lwIP), retried each second
/// while the bind fails (one line while it waits).
fn bind(name: &str, port: u16) -> UdpSocket {
    while !crate::link::stack_up() {
        std::thread::sleep(Duration::from_millis(100));
    }
    let mut said = false;
    loop {
        match UdpSocket::bind(("0.0.0.0", port)) {
            Ok(socket) => return socket,
            Err(error) => {
                if !said {
                    log::warn!("{name} :: udp:{port} not bound yet ({error}): retried every second");
                    said = true;
                }
                std::thread::sleep(Duration::from_secs(1));
            }
        }
    }
}

/// A row per simulated sensor the frame carries, in its driver's shape (plan B hitl.py:286-321).
fn record(sensors: &Sensors, change: Option<f32>, now_us: u64, streams: &mut Streams) {
    let Streams {
        imu,
        barometer,
        gnss,
        laser,
        pitot,
        magnetometer,
    } = streams;
    if let (Some([ax, ay, az]), Some(rate)) = (sensors.accel, sensors.rate) {
        let [gx, gy, gz] = rate.map(|degrees| (degrees * 100.0).round() as i32);
        let row = [
            Cell::F32(ax),
            Cell::F32(ay),
            Cell::F32(az),
            Cell::I32(gx),
            Cell::I32(gy),
            Cell::I32(gz),
            Cell::I32(1), // irq_runs: one sample handed over per publish (plan B's 1)
        ];
        imu.push(now_us, &row);
    }
    if let (Some(altitude), Some(elevation)) = (sensors.altitude, sensors.elevation) {
        let row = [
            Cell::F32(altitude),
            Cell::F32(SIMULATED_TEMPERATURE_C),
            Cell::F32(SIMULATED_PRESSURE_PA),
            Cell::F32(elevation),
            change.map_or(Cell::None, Cell::F32),
        ];
        barometer.push(now_us, &row);
    }
    if let (Some([latitude, longitude]), Some(speed), Some(course)) = (sensors.position, sensors.speed, sensors.course)
    {
        let row = [
            Cell::E7(latitude),
            Cell::E7(longitude),
            Cell::F32(speed * KNOTS_PER_METRE_PER_SECOND),
            Cell::F32(course),
        ];
        gnss.push(now_us, &row);
    }
    if let Some(agl) = sensors.agl {
        laser.push(now_us, &[Cell::F32(agl), Cell::I32(0)]);
    }
    if let (Some(pressure), Some(airspeed)) = (sensors.dynamic_pressure, sensors.airspeed) {
        let row = [
            Cell::I32((pressure * 100.0) as i32),
            Cell::I32((airspeed * 100.0) as i32),
            Cell::I32(SIMULATED_PITOT_TEMPERATURE),
        ];
        pitot.push(now_us, &row);
    }
    if let Some(mag) = sensors.mag {
        let row = mag.map(|counts| Cell::I32(counts.round() as i32));
        magnetometer.push(now_us, &row);
    }
}

/// The fins the board commands, as FINS carries them (left elevon, right elevon, yaw, plan B's surface names), None
/// unless all three are servos that came up: the host then flies them neutral.
fn fin_angles() -> Option<[i16; 3]> {
    // none until every fin was driven: a fin with no pulse yet is no angle (the host then flies them neutral)
    let angle = |name: &str| crate::fins::commanded(name).map(|angle| angle as i16);
    Some([
        angle("servo_eleron_left")?,
        angle("servo_eleron_right")?,
        angle("servo_yaw")?,
    ])
}

/// The [`REFUSED`] slot of `refusal`.
fn refusal_index(refusal: Refusal) -> usize {
    match refusal {
        Refusal::Length => 0,
        Refusal::Magic => 1,
        Refusal::Version => 2,
        Refusal::Kind => 3,
        Refusal::Crc => 4,
        Refusal::NotFinite => 5,
    }
}

/// What `inspect hitl` shows (the port's own; plan B's hitl inspected its body, now the host's).
pub struct Inspection {
    /// The UDP port.
    pub port: u16,
    /// Whether the socket is bound.
    pub bound: bool,
    /// Frames applied since boot.
    pub applied: u32,
    /// Frames refused, by reason.
    pub refused: [(&'static str, u32); 6],
    /// Frames not newer than the last applied.
    pub stale: u32,
    /// The last applied frame's seq and sim time (ms); None before the first.
    pub last: Option<(u32, u32)>,
    /// How long ago the last frame was applied, ms.
    pub age_ms: Option<u64>,
    /// The last frame's sender.
    pub sender: Option<String>,
}

/// The sim-link's state; None without a running `hitl` entry.
pub fn inspection() -> Option<Inspection> {
    let (_, port) = *RUNNING.get()?;
    let applied = APPLIED.load(Ordering::Relaxed);
    let names = ["length", "magic", "version", "kind", "crc", "not_finite"];
    let last_ms = LAST_MS.load(Ordering::Relaxed);
    Some(Inspection {
        port,
        bound: BOUND.load(Ordering::Relaxed),
        applied,
        refused: std::array::from_fn(|index| (names[index], REFUSED[index].load(Ordering::Relaxed))),
        stale: STALE.load(Ordering::Relaxed),
        last: (applied > 0).then(|| (LAST[0].load(Ordering::Relaxed), LAST[1].load(Ordering::Relaxed))),
        age_ms: (applied > 0).then(|| u64::from(((uptime_us() / 1000) as u32).wrapping_sub(last_ms))),
        sender: match SENDER[1].load(Ordering::Relaxed) {
            0 => None,
            port => Some(format!(
                "{}:{port}",
                std::net::Ipv4Addr::from(SENDER[0].load(Ordering::Relaxed))
            )),
        },
    })
}
