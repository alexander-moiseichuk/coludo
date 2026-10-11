//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight law (src/glider tasks/flight.py): the control step that turns the sensed state into fin angles, one
//! call per control tick, composed of this crate's parts in plan B's order -- the governor (the airspeed estimate
//! and the dynamic-pressure fin cap, every tick), the stage gate, the attitude, the entry (holds captured, PIDs
//! reset), the guidance law, the three PIDs (the gyro rate as their D term), the mixer, and the wind estimator fed
//! once per new GNSS sample with the pad drift taken out.
//!
//! The caller reads the databoard and the mission (fresh values only: a stale quantity is None) and hands them in
//! as a [`Step`]; [`FlightLaw::step`] answers the fin angles to write, in the mixer's surface order. No clock, no
//! allocation per step: the firmware calls it from the control thread.
//!
//! Units are plan B's real-number law in f32 (decision D18): degrees, seconds, m/s. Where plan B floored a
//! centidegree command to whole degrees (`cmd // SCALE`), the port floors the degree command, so the fins see the
//! same whole-degree demand.

use serde_json::Value;

use crate::governor::{Governor, GovernorConfig, Readings};
use crate::guidance::{Guidance, GuidanceConfig, Inputs};
use crate::mixer::Mixer;
use crate::pid::{ANTI_WINDUP_SHIFT, Gains, Pid};
use crate::wind::WindEstimator;
use sequencer::Stage;

/// A gap past this many seconds since the last step (the first step, a stall) runs one nominal slice instead.
const LONGEST_SLICE_S: f32 = 0.5;

/// What the control step reads this tick.
#[derive(Clone, Copy, Debug)]
pub struct Step {
    /// The flight word's stage.
    pub stage: Stage,
    /// Armed: control acts only armed.
    pub armed: bool,
    /// The tick's time, µs.
    pub now_us: u64,
    /// The attitude [heading, roll, pitch] in degrees, None when no source is fresh.
    pub attitude: Option<[f32; 3]>,
    /// The gyro rates [roll, pitch, yaw] in deg/s, None when stale (the PIDs then differentiate the error).
    pub rate: Option<[f32; 3]>,
    /// The governor's readings: the acceleration, the GNSS ground speed, the pitot.
    pub readings: Readings,
    /// The guidance's inputs: the zone, the launch point, the fix, the laser and baro heights. Its airspeed and
    /// wind are this law's own and are filled in here.
    pub guidance: Inputs,
    /// The GNSS course (degrees) and ground speed (m/s) when both are fresh, None otherwise.
    pub ground: Option<[f32; 2]>,
    /// The course channel's stamp (its last sample's push time, or a count): the wind is fed once per new value,
    /// on the first control step after it arrived (plan B's `course.stamp()`). None before any sample.
    pub ground_stamp: Option<u64>,
    /// The pad drift [east, north] in m/s (gnss_calib), zero before it froze.
    pub drift: [f32; 2],
}

/// The flight law's state.
#[derive(Clone, Debug)]
pub struct FlightLaw {
    /// The fin mixer, whose limit the governor sets.
    mixer: Mixer,
    /// Roll, pitch and yaw.
    pids: [Pid; 3],
    /// The airspeed estimate and the fin cap.
    governor: Governor,
    /// The wind triangle.
    wind: WindEstimator,
    /// The per-stage setpoints and the heading error.
    guidance: Guidance,
    /// One nominal control slice, s (1 / `schedule_hz`, or `period_ms`).
    nominal_s: f32,
    /// The previous step's time, µs.
    last_us: Option<u64>,
    /// The last measured pitch, degrees: the governor's dive detector.
    pitch: f32,
    /// In a control stage, the PIDs engaged.
    active: bool,
    /// The control stage it is in.
    stage: Option<Stage>,
    /// The last per-axis demands, whole degrees: flight.csv's.
    commands: [i32; 3],
    /// The fin angles last written, in the mixer's surface order.
    angles: Vec<i32>,
    /// Control steps run.
    steps: u32,
    /// The ground speed under which a course is too noisy to feed the wind (`wind.min_speed`).
    wind_min_speed: f32,
    /// The course stamp the wind was last fed at.
    wind_stamp: Option<u64>,
    /// The last control step's guidance inputs, its airspeed and wind filled in: the reach estimate's.
    inputs: Inputs,
    /// The nominal L/D for the reach estimate (`glide_ratio`).
    glide_ratio: f32,
}

impl FlightLaw {
    /// The law of the flight entry `flight` (gains, schedule, guidance, wind) and the config's `fins` section
    /// (the mixer, the limit multiplier), as plan B's setup builds it.
    pub fn new(flight: &Value, fins: &Value) -> FlightLaw {
        let mut mixer = Mixer::new(&fins["mixer"]);
        let limit = mixer.limit() as f32;
        let number = |key: &str| flight[key].as_f64().map(|value| value as f32);
        let integral_limit = number("integral_limit").unwrap_or(limit);
        let pid = |axis: &str| {
            let gains = &flight["gains"][axis];
            let gain = |key: &str| gains[key].as_f64().unwrap_or(0.0) as f32;
            Pid::new(
                Gains {
                    proportional: gain("kp"),
                    integral: gain("ki"),
                    derivative: gain("kd"),
                },
                integral_limit,
                limit,
                ANTI_WINDUP_SHIFT,
            )
        };
        let schedule_hz = flight["schedule_hz"].as_f64().unwrap_or(100.0);
        let nominal_s = if schedule_hz > 0.0 {
            (1.0 / schedule_hz) as f32
        } else {
            number("period_ms").unwrap_or(20.0) / 1000.0
        };
        // the guidance's freshness gate: the GNSS channels' own windows (position 400 ms, speed 500 ms)
        let window_ms = 500;
        FlightLaw {
            pids: [pid("roll"), pid("pitch"), pid("yaw")],
            governor: Governor::new(
                GovernorConfig::new(flight),
                fins["limit_multiplier"].as_f64().unwrap_or(1.0) as f32,
            ),
            wind: WindEstimator::new(&flight["wind"]),
            guidance: Guidance::new(GuidanceConfig::from_json(flight, window_ms)),
            angles: mixer.mix(0.0, 0.0, 0.0).to_vec(),
            mixer,
            nominal_s,
            last_us: None,
            pitch: 0.0,
            active: false,
            stage: None,
            commands: [0; 3],
            steps: 0,
            wind_min_speed: flight["wind"]["min_speed"].as_f64().unwrap_or(3.0) as f32,
            wind_stamp: None,
            inputs: Inputs::default(),
            glide_ratio: number("glide_ratio").unwrap_or(5.5),
        }
    }

    /// One control step (plan B `_tick`). Returns whether it actuated: then [`FlightLaw::angles`] are the fin angles
    /// to write. Out of a control stage, disarmed, with no fresh attitude, or with the boost still on the rod, it
    /// actuates neutral; out of a control stage while nothing engaged it does not actuate at all, so an operator's
    /// fin command on the pad stands (plan B returns before `_actuate`).
    pub fn step(&mut self, input: &Step) -> bool {
        let elapsed = self.slice(input.now_us);
        self.governor.step(
            &mut self.mixer,
            elapsed,
            (input.stage as u8) < (Stage::Gliding as u8),
            self.pitch,
            &input.readings,
        );
        if self.guidance.setpoint(input.stage).is_none() || !input.armed {
            if self.active {
                self.neutral();
                self.active = false;
                self.stage = None;
                return true;
            }
            return false;
        }
        let Some([heading, roll, pitch]) = input.attitude else {
            self.neutral();
            return true;
        };
        self.pitch = pitch;
        if !self.active {
            self.active = true;
            self.guidance.enter(heading, roll, pitch);
            for pid in &mut self.pids {
                pid.reset();
            }
        }
        self.stage = Some(input.stage);
        let (wind_east, wind_north) = self.wind.components();
        self.inputs = Inputs {
            airspeed: self.governor.airspeed(),
            wind_east,
            wind_north,
            ..input.guidance
        };
        if !self.guidance.compute(input.stage, heading, input.now_us, &self.inputs) {
            self.neutral(); // the boost still on the rod: no q to bite
            return true;
        }
        self.run_pids(roll, pitch, elapsed, input.rate);
        self.steps += 1;
        if input.ground_stamp.is_some() && input.ground_stamp != self.wind_stamp {
            self.wind_stamp = input.ground_stamp;
            if let Some([course, speed]) = input.ground {
                self.feed_wind(course, speed, heading, input.drift);
            }
        }
        true
    }

    /// The fin angles last actuated, in the mixer's surface order (neutral before any).
    pub fn angles(&self) -> &[i32] {
        &self.angles
    }

    /// The zone's reach from the last control step's inputs (plan B flight.vitals `reach`), None without a zone.
    pub fn reach(&self) -> Option<crate::guidance::Reach> {
        self.guidance.reachability(self.glide_ratio, &self.inputs)
    }

    /// The endgame's turn-radius floor at the airspeed estimate, metres (plan B vitals `r_min_m`).
    pub fn landing_turn_radius(&self) -> Option<f32> {
        self.guidance.landing_turn_radius(self.governor.airspeed())
    }

    /// Seed the airspeed estimate (a warm start's crumb, plan B flight.seed_airspeed), its fin cap with it.
    pub fn seed_airspeed(&mut self, airspeed: f32) {
        self.governor.seed_airspeed(&mut self.mixer, airspeed);
    }

    /// The slice since the last step in seconds; the nominal one on the first step or after a gap past 0.5 s
    /// (plan B `_compute_dt`, whose integer milliseconds the PIDs read: the slice is truncated to whole ms).
    fn slice(&mut self, now_us: u64) -> f32 {
        let previous = self.last_us.replace(now_us);
        let elapsed_us = previous.map(|previous| now_us.saturating_sub(previous));
        match elapsed_us {
            Some(elapsed) if elapsed > 0 && (elapsed as f32) <= LONGEST_SLICE_S * 1e6 => {
                (elapsed / 1000) as f32 / 1000.0
            }
            _ => self.nominal_s,
        }
    }

    /// The three PIDs against the guidance's setpoints, their clamps on the governor's live cap, then the mixer
    /// (plan B `_run_pid`, `_actuate`).
    fn run_pids(&mut self, roll: f32, pitch: f32, elapsed: f32, rate: Option<[f32; 3]>) {
        let cap = self.mixer.limit();
        for pid in &mut self.pids {
            pid.set_limit(cap);
        }
        let [roll_rate, pitch_rate, yaw_rate] = rate.map_or([None; 3], |rate| rate.map(Some));
        let [roll_pid, pitch_pid, yaw_pid] = &mut self.pids;
        let roll_command = roll_pid.step(self.guidance.roll_setpoint() - roll, elapsed, roll_rate);
        let pitch_command = pitch_pid.step(self.guidance.pitch_setpoint() - pitch, elapsed, pitch_rate);
        let yaw_command = yaw_pid.step(self.guidance.heading_error() as f32, elapsed, yaw_rate);
        self.commands = [roll_command, pitch_command, yaw_command].map(|command| command.floor() as i32);
        let [roll, pitch, yaw] = self.commands.map(|command| command as f32);
        self.angles.copy_from_slice(self.mixer.mix(roll, pitch, yaw));
    }

    /// The fins at neutral: every surface at its base angle (plan B `_neutral`).
    fn neutral(&mut self) {
        self.angles.copy_from_slice(self.mixer.mix(0.0, 0.0, 0.0));
    }

    /// One wind observation from a new GNSS sample (plan B `_feed_wind`): the ground velocity less the pad drift,
    /// against the air velocity (the airspeed along `heading`). Nothing under the meaningful ground speed.
    fn feed_wind(&mut self, course: f32, speed: f32, heading: f32, drift: [f32; 2]) {
        if speed <= self.wind_min_speed {
            return;
        }
        let (mut course, mut speed) = (course, speed);
        if drift != [0.0; 2] {
            let (sin, cos) = course.to_radians().sin_cos();
            let east = speed * sin - drift[0];
            let north = speed * cos - drift[1];
            speed = (east * east + north * north).sqrt();
            course = east.atan2(north).to_degrees().rem_euclid(360.0);
        }
        self.wind.observe(course, speed, self.governor.airspeed(), heading);
    }

    /// The airspeed estimate, m/s.
    pub fn airspeed(&self) -> f32 {
        self.governor.airspeed()
    }

    /// The fin cap the governor set, whole degrees.
    pub fn cap(&self) -> i32 {
        self.mixer.limit()
    }

    /// In a control stage, the PIDs engaged.
    pub fn active(&self) -> bool {
        self.active
    }

    /// The last per-axis demands [roll, pitch, yaw], whole degrees.
    pub fn commands(&self) -> [i32; 3] {
        self.commands
    }

    /// The guidance law: its setpoints, heading error, reckoning and reach.
    pub fn guidance(&self) -> &Guidance {
        &self.guidance
    }

    /// The wind estimator.
    pub fn wind(&self) -> &WindEstimator {
        &self.wind
    }

    /// The mixer: its surfaces, in the order of the angles.
    pub fn mixer(&self) -> &Mixer {
        &self.mixer
    }

    /// The nominal L/D for the reach estimate.
    pub fn glide_ratio(&self) -> f32 {
        self.glide_ratio
    }

    /// Control steps run.
    pub fn steps(&self) -> u32 {
        self.steps
    }
}
