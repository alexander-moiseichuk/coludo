//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight loop on the control thread (M13; plan B tasks/flight.py, its law in crates/control flight): every
//! control tick the law reads the databoard's fresh quantities and the mission, and when it actuates, its whole-degree
//! fin angles go to the servos through their compare-and-set (crate::fins), so a held fin costs nothing. Every 100 ms
//! a flight.csv row samples what the loop believed and demanded (plan B's columns and units), and the vitals the
//! `health` panel shows are published through atomics.
//!
//! The mission is the executor's: it publishes the zone, the launch point and the endgame pattern after every line
//! ([`publish_mission`]) into a sequence-locked view the control thread reads without a lock.

use std::sync::OnceLock;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};

use control::flight::FlightLaw;
use control::guidance::{Heading, Landing};
use control::navigation::Position;
use controller::FlightQuantities;
use mission::Mission;
use recorder::{Cell, Rows};
use sequencer::{Flight as FlightWord, Stage};
use serde_json::Value;

use crate::devices::{FlightMember, Topology};
use crate::fins::Fins;

/// The flight.csv sample period, µs (plan B `telemetry_ms` 100: the control state SAMPLED, not traced).
const SAMPLE_US: u64 = 100_000;
/// At DONE, µs (M11b).
const DONE_SAMPLE_US: u64 = 1_000_000;

/// The mission view's sequence: odd while the executor writes it.
static MISSION_SEQUENCE: AtomicU32 = AtomicU32::new(0);
/// The zone's corners (1e-7 degree: top-left latitude, longitude, bottom-right latitude, longitude) and whether one
/// is set.
static MISSION_ZONE: [AtomicI32; 4] = [const { AtomicI32::new(0) }; 4];
/// Whether a zone is set.
static MISSION_ZONED: AtomicBool = AtomicBool::new(false);
/// The operator-set (or arm-pinned) launch point, 1e-7 degree, and whether one is set.
static MISSION_LAUNCH: [AtomicI32; 2] = [const { AtomicI32::new(0) }; 2];
/// Whether a launch point is set.
static MISSION_LAUNCHED: AtomicBool = AtomicBool::new(false);
/// The endgame pattern the mission decides for its zone, an index into [`Heading::ALL`].
static MISSION_HEADING: AtomicU32 = AtomicU32::new(0);

/// The flight loop's vitals for `health` (plan B flight.vitals), published every sample.
static VITALS: Vitals = Vitals::new();
/// Whether the flight loop runs, and its fins' names by the mixer's surfaces.
static RUNNING: OnceLock<Vec<&'static str>> = OnceLock::new();
/// The airspeed a warm start's crumb carried, f32 bits, for the law's seed; 0 when none.
static WARM_AIRSPEED: AtomicU32 = AtomicU32::new(0);

/// The vitals' atomics.
struct Vitals {
    /// The airspeed estimate, f32 bits.
    airspeed: AtomicU32,
    /// The fin cap, whole degrees.
    cap: AtomicI32,
    /// In a control stage.
    active: AtomicBool,
    /// The heading error, degrees.
    heading_error: AtomicI32,
    /// The wind speed (f32 bits) and its from-direction (whole degrees).
    wind: [AtomicU32; 2],
    /// The reach: known, reachable, margin m, distance m, headwind (f32 bits).
    reach: [AtomicI32; 4],
    /// Whether a reach is known.
    reach_known: AtomicBool,
    /// The endgame turn-radius floor, f32 bits; NaN when none.
    turn_radius: AtomicU32,
    /// The fin angles, by surface.
    angles: [AtomicI32; 4],
}

impl Vitals {
    /// All zero.
    const fn new() -> Vitals {
        Vitals {
            airspeed: AtomicU32::new(0),
            cap: AtomicI32::new(0),
            active: AtomicBool::new(false),
            heading_error: AtomicI32::new(0),
            wind: [const { AtomicU32::new(0) }; 2],
            reach: [const { AtomicI32::new(0) }; 4],
            reach_known: AtomicBool::new(false),
            turn_radius: AtomicU32::new(0),
            angles: [const { AtomicI32::new(0) }; 4],
        }
    }
}

/// Publishes `mission`'s zone, launch point and endgame pattern for the control thread (the executor, after every
/// line; main once before the threads start).
pub fn publish_mission(mission: &Mission) {
    MISSION_SEQUENCE.fetch_add(1, Ordering::AcqRel);
    match mission.zone {
        Some(zone) => {
            let corners = [
                zone.top_left.latitude_e7,
                zone.top_left.longitude_e7,
                zone.bottom_right.latitude_e7,
                zone.bottom_right.longitude_e7,
            ];
            for (published, value) in MISSION_ZONE.iter().zip(corners) {
                published.store(value, Ordering::Relaxed);
            }
            MISSION_ZONED.store(true, Ordering::Relaxed);
        }
        None => MISSION_ZONED.store(false, Ordering::Relaxed),
    }
    match mission.launch_point(None) {
        Some(point) => {
            MISSION_LAUNCH[0].store(point.latitude_e7, Ordering::Relaxed);
            MISSION_LAUNCH[1].store(point.longitude_e7, Ordering::Relaxed);
            MISSION_LAUNCHED.store(true, Ordering::Relaxed);
        }
        None => MISSION_LAUNCHED.store(false, Ordering::Relaxed),
    }
    let heading = mission.endgame_heading();
    let index = Heading::ALL.iter().position(|pattern| *pattern == heading).unwrap_or(0);
    MISSION_HEADING.store(index as u32, Ordering::Relaxed);
    MISSION_SEQUENCE.fetch_add(1, Ordering::AcqRel);
}

/// The mission as the control thread reads it: the landing zone and the set launch point, read again when the
/// executor wrote meanwhile.
fn mission_view() -> (Option<Landing>, Option<Position>) {
    loop {
        let before = MISSION_SEQUENCE.load(Ordering::Acquire);
        if before % 2 == 1 {
            continue;
        }
        let corner = |index: usize| MISSION_ZONE[index].load(Ordering::Relaxed);
        let landing = MISSION_ZONED.load(Ordering::Relaxed).then(|| {
            let heading = Heading::ALL[MISSION_HEADING.load(Ordering::Relaxed) as usize % Heading::ALL.len()];
            Landing::new(
                Position {
                    latitude_e7: corner(0),
                    longitude_e7: corner(1),
                },
                Position {
                    latitude_e7: corner(2),
                    longitude_e7: corner(3),
                },
                heading,
            )
        });
        let launch = MISSION_LAUNCHED.load(Ordering::Relaxed).then(|| Position {
            latitude_e7: MISSION_LAUNCH[0].load(Ordering::Relaxed),
            longitude_e7: MISSION_LAUNCH[1].load(Ordering::Relaxed),
        });
        if MISSION_SEQUENCE.load(Ordering::Acquire) == before {
            return (landing, launch);
        }
    }
}

/// The mission's launch point as the flight loop reads it (set by CC, or pinned at arm), (latitude, longitude) in
/// 1e-7 degrees: the beacons' map link while no fix came this boot (crate::beacon).
pub fn launch_e7() -> Option<(i32, i32)> {
    let (_, launch) = mission_view();
    launch.map(|point| (point.latitude_e7, point.longitude_e7))
}

/// Keeps a warm start's airspeed for the law's seed (crate::warm, before the control thread starts), logged here: a
/// float formatted on the control thread costs its stack.
pub fn keep_warm_airspeed(airspeed: f32) {
    WARM_AIRSPEED.store(airspeed.to_bits(), Ordering::Relaxed);
    log::info!("flight :: airspeed {airspeed} m/s kept as the law's seed (warm start)");
}

/// The flight loop, the control thread's.
pub struct FlightLoop {
    /// The law.
    law: FlightLaw,
    /// Each mixer surface's fin index, None for a surface with no servo.
    fins: Vec<Option<usize>>,
    /// What it reads.
    quantities: FlightQuantities<'static>,
    /// Its stream: a refused row logged once a run (recorder::Rows).
    rows: Rows<'static>,
    /// The next flight.csv sample, µs.
    next_sample_us: u64,
}

impl FlightLoop {
    /// The loop of `member` on `topology`, its surfaces bound to `fins` by name (a surface with no servo is logged:
    /// a third of the authority lost). Never inlined, nor [`FlightLoop::tick`]: their locals stay out of the control
    /// thread's loop frame.
    #[inline(never)]
    pub fn new(member: &FlightMember, topology: &'static Topology, fins: Option<&Fins>) -> FlightLoop {
        let mut law = FlightLaw::new(&member.entry, &member.fins);
        let seed = f32::from_bits(WARM_AIRSPEED.load(Ordering::Relaxed));
        if seed > 0.0 {
            law.seed_airspeed(seed);
        }
        let surfaces: Vec<&'static str> = law
            .mixer()
            .surfaces()
            .iter()
            .map(|surface| leak(&surface.name))
            .collect();
        let bound: Vec<Option<usize>> = surfaces
            .iter()
            .map(|surface| fins.and_then(|fins| fins.index(surface)))
            .collect();
        for (surface, index) in surfaces.iter().zip(&bound) {
            if index.is_none() {
                log::warn!(
                    "{} :: surface {surface} has NO driver -- reduced authority",
                    member.name
                );
            }
        }
        let _ = RUNNING.set(surfaces);
        FlightLoop {
            law,
            fins: bound,
            quantities: FlightQuantities::new(topology),
            rows: crate::recorder::rows(member.name),
            next_sample_us: 0,
        }
    }

    /// One control tick at `now_us` under `flight`: the law's step on the fresh quantities, the fins it actuated
    /// written, and every 100 ms the flight.csv row and the vitals.
    #[inline(never)]
    pub fn tick(&mut self, now_us: u64, flight: FlightWord, fins: Option<&mut Fins>) {
        let (landing, set_launch) = mission_view();
        let step = self.quantities.step(
            now_us,
            flight.stage,
            flight.armed,
            landing,
            set_launch,
            crate::gnss::drift(),
        );
        if self.law.step(&step)
            && let Some(fins) = fins
        {
            for (&angle, index) in self.law.angles().iter().zip(&self.fins) {
                if let Some(index) = *index {
                    fins.set_angle(index, f64::from(angle));
                }
            }
        }
        if now_us >= self.next_sample_us {
            // DONE (M11b): the recording a trickle, a row a second
            self.next_sample_us = now_us
                + if flight.stage == Stage::Done {
                    DONE_SAMPLE_US
                } else {
                    SAMPLE_US
                };
            self.sample(now_us, flight.stage);
        }
    }

    /// The flight.csv row (plan B's columns: centi-units where plan B wrote fixnums) and the vitals.
    fn sample(&mut self, now_us: u64, stage: Stage) {
        let law = &self.law;
        let guidance = law.guidance();
        let [roll, pitch, yaw] = law.commands();
        let wind = law.wind();
        let row = [
            Cell::I32(i32::from(stage as u8)),
            Cell::I32(i32::from(law.active())),
            Cell::I32((law.airspeed() * 100.0) as i32),
            Cell::I32(law.cap()),
            Cell::I32((guidance.roll_setpoint() * 100.0).round() as i32),
            Cell::I32((guidance.pitch_setpoint() * 100.0).round() as i32),
            Cell::I32(guidance.heading_error()),
            Cell::I32(roll),
            Cell::I32(pitch),
            Cell::I32(yaw),
            Cell::I32((wind.speed() * 100.0) as i32),
            Cell::I32(wind.direction() as i32),
            Cell::I32(i32::from(guidance.reckoning())),
        ];
        self.rows.push(now_us, &row);
        VITALS.airspeed.store(law.airspeed().to_bits(), Ordering::Relaxed);
        VITALS.cap.store(law.cap(), Ordering::Relaxed);
        VITALS.active.store(law.active(), Ordering::Relaxed);
        VITALS.heading_error.store(guidance.heading_error(), Ordering::Relaxed);
        VITALS.wind[0].store(wind.speed().to_bits(), Ordering::Relaxed);
        VITALS.wind[1].store(wind.direction().to_bits(), Ordering::Relaxed);
        match law.reach() {
            Some(reach) => {
                VITALS.reach[0].store(i32::from(reach.reachable), Ordering::Relaxed);
                VITALS.reach[1].store(reach.margin_m, Ordering::Relaxed);
                VITALS.reach[2].store(reach.distance_m, Ordering::Relaxed);
                VITALS.reach[3].store(reach.headwind_mps.to_bits() as i32, Ordering::Relaxed);
                VITALS.reach_known.store(true, Ordering::Relaxed);
            }
            None => VITALS.reach_known.store(false, Ordering::Relaxed),
        }
        VITALS.turn_radius.store(
            law.landing_turn_radius().unwrap_or(f32::NAN).to_bits(),
            Ordering::Relaxed,
        );
        for (published, &angle) in VITALS.angles.iter().zip(law.angles()) {
            published.store(angle, Ordering::Relaxed);
        }
    }
}

/// A surface name kept for the program (the mixer's names are config text, read once at boot).
fn leak(name: &str) -> &'static str {
    Box::leak(name.to_string().into_boxed_str())
}

/// The flight panel's vitals (plan B flight.vitals) as `health.flight` shows them; None when no flight loop runs.
/// `attitude` is the fused one, whole degrees.
pub fn vitals(attitude: Option<[i32; 3]>) -> Option<Value> {
    let surfaces = RUNNING.get()?;
    let load = |value: &AtomicU32| f32::from_bits(value.load(Ordering::Relaxed));
    let reach = VITALS.reach_known.load(Ordering::Relaxed).then(|| {
        serde_json::json!({
            "reachable": VITALS.reach[0].load(Ordering::Relaxed) != 0,
            "margin_m": VITALS.reach[1].load(Ordering::Relaxed),
            "distance_m": VITALS.reach[2].load(Ordering::Relaxed),
            "headwind_mps": (f32::from_bits(VITALS.reach[3].load(Ordering::Relaxed) as u32) * 10.0).round() / 10.0,
        })
    });
    let turn_radius = load(&VITALS.turn_radius);
    let fins: serde_json::Map<String, Value> = surfaces
        .iter()
        .zip(&VITALS.angles)
        .map(|(surface, angle)| (surface.to_string(), Value::from(angle.load(Ordering::Relaxed))))
        .collect();
    Some(serde_json::json!({
        "airspeed": (load(&VITALS.airspeed) * 10.0).round() / 10.0,
        "fin_cap": VITALS.cap.load(Ordering::Relaxed),
        "active": VITALS.active.load(Ordering::Relaxed),
        "reach": reach,
        "r_min_m": (!turn_radius.is_nan()).then(|| turn_radius.round() as i32),
        "attitude": attitude.map(|[heading, roll, pitch]| serde_json::json!({"heading": heading, "roll": roll, "pitch": pitch})),
        "fins": fins,
        "heading_error": VITALS.heading_error.load(Ordering::Relaxed),
        "wind": {"speed": (load(&VITALS.wind[0]) * 10.0).round() / 10.0, "from": load(&VITALS.wind[1]).round() as i32},
    }))
}

/// The airspeed estimate for the checkpoint's crumb, m/s; None while no flight loop runs.
pub fn airspeed() -> Option<f32> {
    RUNNING.get()?;
    Some(f32::from_bits(VITALS.airspeed.load(Ordering::Relaxed)))
}

/// The mixer surfaces with no servo (plan B `_readiness` `fins`), None while no flight loop runs.
pub fn unbound_surfaces() -> Option<Vec<&'static str>> {
    let surfaces = RUNNING.get()?;
    Some(
        surfaces
            .iter()
            .copied()
            .filter(|surface| crate::fins::inspection(surface).is_none())
            .collect(),
    )
}
