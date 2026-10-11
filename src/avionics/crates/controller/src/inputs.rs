//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight threads' inputs from the board's quantities ([`crate::Topology`]), one implementation for the firmware
//! and the SITL (M14): the flight law's step ([`FlightQuantities`]), the stage machine's per-source readings
//! ([`StageSources`]), and a simulated frame published as the `hitl` source ([`HitlWriters`]). A host that composed
//! its own would grade itself, not the board (the virtual_flight lesson: its private launch detector).

use control::change::Change;
use control::flight::Step;
use control::governor::Readings;
use control::guidance::{Fix, Inputs, Landing};
use control::navigation::Position;
use databoard::{Parameter, Sample, Writer};
use sequencer::{Binding, SourceReadings, Stage};
use sim_link::Sensors;

use crate::Topology;

/// The quantities the flight law reads.
pub struct FlightQuantities<'t> {
    /// [heading, roll, pitch], degrees.
    attitude: Option<&'t Parameter<[f32; 3]>>,
    /// [roll, pitch, yaw], deg/s.
    rate: Option<&'t Parameter<[f32; 3]>>,
    /// g.
    accel: Option<&'t Parameter<[f32; 3]>>,
    /// GNSS ground speed, m/s.
    speed: Option<&'t Parameter<f32>>,
    /// GNSS course, degrees.
    course: Option<&'t Parameter<f32>>,
    /// The pitot's airspeed, m/s.
    pitot: Option<&'t Parameter<f32>>,
    /// The fix.
    position: Option<&'t Parameter<[i32; 2]>>,
    /// The laser's height, m.
    agl: Option<&'t Parameter<f32>>,
    /// The baro height above the pad, m.
    elevation: Option<&'t Parameter<f32>>,
}

impl<'t> FlightQuantities<'t> {
    /// The law's quantities in `topology`.
    pub fn new(topology: &'t Topology) -> FlightQuantities<'t> {
        FlightQuantities {
            attitude: topology.vector("attitude"),
            rate: topology.vector("rate"),
            accel: topology.vector("accel"),
            speed: topology.scalar("speed"),
            course: topology.scalar("course"),
            pitot: topology.scalar("airspeed"),
            position: topology.position("position"),
            agl: topology.scalar("agl"),
            elevation: topology.scalar("elevation"),
        }
    }

    /// The law's step at `now_us` in `stage` (`armed`) from the fresh quantities: the mission's `landing` zone, the
    /// set launch point `set_launch` (else the fix), and the GNSS calibration's `drift`. The course's push time is the
    /// ground stamp: the same between samples, new with each.
    pub fn step(
        &self,
        now_us: u64,
        stage: Stage,
        armed: bool,
        landing: Option<Landing>,
        set_launch: Option<Position>,
        drift: [f32; 2],
    ) -> Step {
        let fix = self
            .position
            .and_then(|parameter| parameter.read(now_us))
            .map(|reading| Fix {
                position: Position {
                    latitude_e7: reading.value[0],
                    longitude_e7: reading.value[1],
                },
                age_ms: reading.age_us / 1000,
            });
        let course = self.course.and_then(|parameter| parameter.read(now_us));
        let speed = fresh(self.speed, now_us);
        Step {
            stage,
            armed,
            now_us,
            attitude: fresh(self.attitude, now_us),
            rate: fresh(self.rate, now_us),
            readings: Readings {
                acceleration: fresh(self.accel, now_us),
                gnss_speed: speed,
                pitot: fresh(self.pitot, now_us),
            },
            guidance: Inputs {
                landing,
                launch: set_launch.or(fix.map(|fix| fix.position)),
                position: fix,
                agl: fresh(self.agl, now_us),
                elevation: fresh(self.elevation, now_us),
                ..Inputs::default()
            },
            ground: course.zip(speed).map(|(course, speed)| [course.value, speed]),
            ground_stamp: course.map(|course| now_us.saturating_sub(u64::from(course.age_us))),
            drift,
        }
    }
}

/// `parameter`'s fused value at `now_us`, None when no source is fresh.
fn fresh<T: Sample>(parameter: Option<&Parameter<T>>, now_us: u64) -> Option<T> {
    parameter
        .and_then(|parameter| parameter.read(now_us))
        .map(|reading| reading.value)
}

/// The stage machine's sources: each class's bound to its slots (crates/sequencer [`Binding`]), and the quantities
/// they push into.
pub struct StageSources<'t> {
    /// The slots.
    binding: Binding,
    /// Every accelerometer's.
    accel: Option<&'t Parameter<[f32; 3]>>,
    /// Every barometer's and the GNSS's.
    elevation: Option<&'t Parameter<f32>>,
    /// The laser's.
    height_above_ground: Option<&'t Parameter<f32>>,
}

impl<'t> StageSources<'t> {
    /// The sources in `topology`, bound to their slots in rank order.
    pub fn new(topology: &'t Topology) -> StageSources<'t> {
        let accel = topology.vector("accel");
        let elevation = topology.scalar("elevation");
        let height_above_ground = topology.scalar("agl");
        let binding = Binding::new(
            accel.into_iter().flat_map(|parameter| parameter.sources()),
            elevation.into_iter().flat_map(|parameter| parameter.sources()),
            height_above_ground
                .into_iter()
                .flat_map(|parameter| parameter.sources()),
        );
        StageSources {
            binding,
            accel,
            elevation,
            height_above_ground,
        }
    }

    /// The slots.
    pub fn binding(&self) -> &Binding {
        &self.binding
    }

    /// Each bound source's own channel at `now_us`, fresh in its quantity's shared window (M9: the multi-source
    /// machine).
    pub fn readings(&self, now_us: u64) -> SourceReadings {
        self.binding.readings(
            |source| own(self.accel, source, now_us),
            |source| own(self.elevation, source, now_us),
            |source| own(self.height_above_ground, source, now_us),
        )
    }
}

/// `source`'s own channel of `parameter`, fresh in the parameter's shared window at `now_us`.
fn own<T: Sample>(parameter: Option<&Parameter<T>>, source: &str, now_us: u64) -> Option<T> {
    let parameter = parameter?;
    parameter.channel(source)?.recent(parameter.window_us(), now_us)
}

/// The `hitl` source's writers, one per quantity a SENSORS frame carries (doc/specs/sim-link.md), and the climb its
/// altitude derives (`change`, as every barometer's).
pub struct HitlWriters<'t> {
    /// g.
    accel: Option<Writer<'t, [f32; 3]>>,
    /// deg/s.
    rate: Option<Writer<'t, [f32; 3]>>,
    /// degrees.
    attitude: Option<Writer<'t, [f32; 3]>>,
    /// Calibrated counts.
    mag: Option<Writer<'t, [f32; 3]>>,
    /// m MSL.
    altitude: Option<Writer<'t, f32>>,
    /// m above the pad.
    elevation: Option<Writer<'t, f32>>,
    /// m/s, from the altitude.
    change: Option<Writer<'t, f32>>,
    /// m.
    agl: Option<Writer<'t, f32>>,
    /// m/s.
    airspeed: Option<Writer<'t, f32>>,
    /// Pa.
    dynamic_pressure: Option<Writer<'t, f32>>,
    /// 1e-7 degree.
    position: Option<Writer<'t, [i32; 2]>>,
    /// m/s over ground.
    speed: Option<Writer<'t, f32>>,
    /// degrees.
    course: Option<Writer<'t, f32>>,
    /// The climb rate from the altitude.
    climb: Change,
}

impl<'t> HitlWriters<'t> {
    /// `source`'s writers in `topology`.
    pub fn new(topology: &'t Topology, source: &str) -> HitlWriters<'t> {
        let scalar = |quantity: &str| topology.scalar(quantity).and_then(|parameter| parameter.writer(source));
        let vector = |quantity: &str| topology.vector(quantity).and_then(|parameter| parameter.writer(source));
        HitlWriters {
            accel: vector("accel"),
            rate: vector("rate"),
            attitude: vector("attitude"),
            mag: vector("mag"),
            altitude: scalar("altitude"),
            elevation: scalar("elevation"),
            change: scalar("change"),
            agl: scalar("agl"),
            airspeed: scalar("airspeed"),
            dynamic_pressure: scalar("dynamic_pressure"),
            position: topology
                .position("position")
                .and_then(|parameter| parameter.writer(source)),
            speed: scalar("speed"),
            course: scalar("course"),
            climb: Change::default(),
        }
    }

    /// Every quantity `sensors` carries pushed at `now_us`, and the climb its altitude (else its elevation) derives;
    /// that climb, None until its window fills.
    pub fn publish(&mut self, sensors: &Sensors, now_us: u64) -> Option<f32> {
        let change = sensors
            .altitude
            .or(sensors.elevation)
            .and_then(|altitude| self.climb.push(altitude, now_us));
        for (value, writer) in [
            (sensors.accel, &mut self.accel),
            (sensors.rate, &mut self.rate),
            (sensors.attitude, &mut self.attitude),
            (sensors.mag, &mut self.mag),
        ] {
            if let (Some(value), Some(writer)) = (value, writer) {
                writer.push(value, now_us);
            }
        }
        for (value, writer) in [
            (sensors.altitude, &mut self.altitude),
            (sensors.elevation, &mut self.elevation),
            (change, &mut self.change),
            (sensors.agl, &mut self.agl),
            (sensors.airspeed, &mut self.airspeed),
            (sensors.dynamic_pressure, &mut self.dynamic_pressure),
            (sensors.speed, &mut self.speed),
            (sensors.course, &mut self.course),
        ] {
            if let (Some(value), Some(writer)) = (value, writer) {
                writer.push(value, now_us);
            }
        }
        if let (Some(position), Some(writer)) = (sensors.position, &mut self.position) {
            writer.push(position, now_us);
        }
        change
    }
}
