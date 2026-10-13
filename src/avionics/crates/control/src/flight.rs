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

/// BOOSTING's law (the flight entry's `boost_law`; doc/specs/boost-control.md).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoostLaw {
    /// Plan B's hold (`hold`, the default): the Euler roll and pitch captured at entry. Near a vertical nose the roll
    /// is noise and the pitch cannot tell which way the stack leans (doc/sims/TMS-7-boost_envelope: ~0.3 calibers).
    Hold,
    /// The lean of the body axis from the vertical (`lean`, BC3): the elevons steer the lean in their plane, the yaw
    /// fin the lean in its, and the roll loop only damps the roll rate.
    Lean,
}

impl BoostLaw {
    /// The law a `boost_law` value names: `lean`, else plan B's hold.
    fn of(value: &Value) -> BoostLaw {
        if value.as_str() == Some("lean") {
            BoostLaw::Lean
        } else {
            BoostLaw::Hold
        }
    }
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
    /// BOOSTING's law (`boost_law`).
    boost_law: BoostLaw,
    /// BOOSTING's own ceiling on the fin cap, whole degrees (`boost_limit_deg`; absent or not positive: none, plan
    /// B's governor cap alone). The operator's +/-10 of 2026-06-25: at the rod gate's 15 m/s the governor still
    /// allows 45 degrees, under thrust (doc/specs/boost-control.md BC4).
    boost_limit: Option<i32>,
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
            boost_law: BoostLaw::of(&flight["boost_law"]),
            boost_limit: number("boost_limit_deg")
                .filter(|limit| *limit >= 1.0)
                .map(|limit| limit as i32),
        }
    }

    /// One control step (plan B `_tick`). Returns whether it actuated: then [`FlightLaw::angles`] are the fin angles
    /// to write. Out of a control stage, disarmed, with no fresh attitude, or with the boost still on the rod, it
    /// actuates neutral; out of a control stage while nothing engaged it does not actuate at all, so an operator's
    /// fin command on the pad stands (plan B returns before `_actuate`).
    pub fn step(&mut self, input: &Step) -> bool {
        let elapsed = self.slice(input.now_us);
        let ceiling = if input.stage == Stage::Boosting {
            self.boost_limit
        } else {
            None
        };
        self.governor.set_ceiling(&mut self.mixer, ceiling);
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
        if input.stage == Stage::Boosting && self.boost_law == BoostLaw::Lean {
            self.run_lean(roll, pitch, elapsed, input.rate);
        } else {
            self.run_pids(roll, pitch, elapsed, input.rate);
        }
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
        self.actuate([roll_command, pitch_command, yaw_command]);
    }

    /// BOOSTING's lean law (doc/specs/boost-control.md BC3), the PIDs on the lean of the body axis from the
    /// vertical. In the marks frame (x the nose, y the left wing, z dorsal) the vertical is (sin pitch,
    /// sin roll cos pitch, cos roll cos pitch): continuous where the Euler roll and heading degenerate near a vertical
    /// nose, and naming which way the stack leans. The elevons (pitch) steer the lean toward the dorsal side, the yaw
    /// fin the lean toward the left wing (a positive yaw turns the nose right, away from it), each against its rate.
    /// The roll loop's error is 0: near vertical a roll angle is noise, so it only damps the roll rate (its D term).
    fn run_lean(&mut self, roll: f32, pitch: f32, elapsed: f32, rate: Option<[f32; 3]>) {
        let cap = self.mixer.limit();
        for pid in &mut self.pids {
            pid.set_limit(cap);
        }
        let (roll, pitch) = (roll.to_radians(), pitch.to_radians());
        let toward_left = (roll.sin() * pitch.cos()).clamp(-1.0, 1.0).asin().to_degrees();
        let toward_dorsal = (roll.cos() * pitch.cos()).clamp(-1.0, 1.0).asin().to_degrees();
        let [roll_rate, pitch_rate, yaw_rate] = rate.map_or([None; 3], |rate| rate.map(Some));
        let [roll_pid, pitch_pid, yaw_pid] = &mut self.pids;
        let roll_command = roll_pid.step(0.0, elapsed, roll_rate);
        let pitch_command = pitch_pid.step(toward_dorsal, elapsed, pitch_rate);
        let yaw_command = yaw_pid.step(-toward_left, elapsed, yaw_rate);
        self.actuate([roll_command, pitch_command, yaw_command]);
    }

    /// The per-axis demands floored to whole degrees (flight.csv's), then the mixer's fin angles (plan B `_actuate`).
    fn actuate(&mut self, demands: [f32; 3]) {
        self.commands = demands.map(|command| command.floor() as i32);
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

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    /// A law with unit gains (P 1, D 0.1, no I), BOOSTING and GLIDING control stages, the boost engaged at any speed.
    fn law(boost_law: &str) -> FlightLaw {
        let gains = json!({"kp": 1.0, "ki": 0.0, "kd": 0.1});
        let flight = json!({
            "gains": {"roll": gains, "pitch": gains, "yaw": gains},
            "schedule_hz": 100,
            "stages": {"boosting": {}, "gliding": {}},
            "boost_engage_speed": 0.0,
            "boost_law": boost_law,
        });
        FlightLaw::new(&flight, &json!({"mixer": {}}))
    }

    /// One boosting step at `attitude` [heading, roll, pitch] with the gyro at `rate`; the demands [roll, pitch, yaw].
    fn boost(law: &mut FlightLaw, now_us: u64, attitude: [f32; 3], rate: [f32; 3]) -> [i32; 3] {
        let step = Step {
            stage: Stage::Boosting,
            armed: true,
            now_us,
            attitude: Some(attitude),
            rate: Some(rate),
            readings: Readings::default(),
            guidance: Inputs::default(),
            ground: None,
            ground_stamp: None,
            drift: [0.0; 2],
        };
        assert!(law.step(&step));
        law.commands()
    }

    #[test]
    fn the_lean_law_steers_each_plane_by_its_own_fins() {
        // straight up: nothing to correct, whatever the roll angle reads near vertical
        let mut lean = law("lean");
        assert_eq!(boost(&mut lean, 10_000, [0.0, 137.0, 90.0], [0.0; 3]), [0, 0, 0]);
        // 10 deg off vertical toward the dorsal side's opposite (pitch 80, roll 0): the elevons raise the nose
        let mut lean = law("lean");
        let [roll, pitch, yaw] = boost(&mut lean, 10_000, [0.0, 0.0, 80.0], [0.0; 3]);
        assert!(pitch >= 9 && roll == 0 && yaw == 0, "{roll} {pitch} {yaw}");
        // the same lean with the stack rolled 90 deg: now it leans in the yaw fin's plane, and only the yaw fin acts
        // (the elevons within the floor's one degree: f32's cos 90 deg is -4e-8, and plan B floors the demand)
        let mut lean = law("lean");
        let [roll, pitch, yaw] = boost(&mut lean, 10_000, [0.0, 90.0, 80.0], [0.0; 3]);
        assert!(yaw <= -9 && pitch.abs() <= 1 && roll == 0, "{roll} {pitch} {yaw}");
    }

    #[test]
    fn the_lean_law_only_damps_the_roll() {
        let mut lean = law("lean");
        let [roll, ..] = boost(&mut lean, 10_000, [0.0, 0.0, 90.0], [-300.0, 0.0, 0.0]);
        assert!(roll > 0, "a negative spin is met by a positive roll demand: {roll}");
        let mut lean = law("lean");
        let [roll, ..] = boost(&mut lean, 10_000, [0.0, 45.0, 89.0], [0.0; 3]);
        assert_eq!(roll, 0, "a roll angle near vertical is not an error");
    }

    #[test]
    fn the_boost_ceiling_caps_the_fins_in_boosting_only() {
        // the lean law leaning 30 deg off vertical, with no readings: the governor's own cap is the unconfident
        // speed's (14 degrees), above a 10-degree boost ceiling
        let ceiled = |limit: Value| {
            let gains = json!({"kp": 1.0, "ki": 0.0, "kd": 0.1});
            let flight = json!({
                "gains": {"roll": gains, "pitch": gains, "yaw": gains},
                "schedule_hz": 100,
                "stages": {"boosting": {}, "gliding": {}},
                "boost_engage_speed": 0.0,
                "boost_law": "lean",
                "boost_limit_deg": limit,
            });
            FlightLaw::new(&flight, &json!({"mixer": {}}))
        };
        let glide = Step {
            stage: Stage::Gliding,
            armed: true,
            now_us: 20_000,
            attitude: Some([0.0, 0.0, -6.0]),
            rate: Some([0.0; 3]),
            readings: Readings::default(),
            guidance: Inputs::default(),
            ground: None,
            ground_stamp: None,
            drift: [0.0; 2],
        };
        let mut plain = ceiled(Value::Null);
        boost(&mut plain, 10_000, [0.0, 0.0, 60.0], [0.0; 3]);
        let own = plain.cap();
        assert!(own > 10, "the governor's own cap: {own}");
        let mut law = ceiled(json!(10));
        boost(&mut law, 10_000, [0.0, 0.0, 60.0], [0.0; 3]);
        assert_eq!(law.cap(), 10);
        assert!(
            law.angles().iter().all(|angle| (80..=100).contains(angle)),
            "{:?}",
            law.angles()
        );
        // the glide gets the governor's own cap back on its first step
        plain.step(&glide);
        law.step(&glide);
        assert_eq!(law.cap(), plain.cap());
        // NEGATIVE: no ceiling (absent, 0, a string) leaves the governor's cap, and the lean pushes past 10 degrees
        for limit in [Value::Null, json!(0), json!("10")] {
            let mut law = ceiled(limit.clone());
            boost(&mut law, 10_000, [0.0, 0.0, 60.0], [0.0; 3]);
            assert_eq!(law.cap(), own, "{limit}");
            assert!(
                law.angles().iter().any(|angle| (*angle - 90).abs() > 10),
                "{limit}: {:?}",
                law.angles()
            );
        }
    }

    #[test]
    fn plan_b_s_hold_stays_the_default() {
        // NEGATIVE: the hold, rolled 90 deg with the same lean, still pushes the elevons, which is what BC3 fixes
        for name in ["hold", "", "anything"] {
            let mut hold = law(name);
            boost(&mut hold, 10_000, [0.0, 90.0, 90.0], [0.0; 3]); // enters, capturing roll 90, pitch 90
            let [_, pitch, _] = boost(&mut hold, 20_000, [0.0, 90.0, 80.0], [0.0; 3]);
            assert!(pitch >= 9, "{name:?}: {pitch}");
        }
        // and the glide is the glide whatever the boost law
        let step = |law: &mut FlightLaw| {
            let step = Step {
                stage: Stage::Gliding,
                armed: true,
                now_us: 10_000,
                attitude: Some([0.0, 10.0, -6.0]),
                rate: Some([0.0; 3]),
                readings: Readings::default(),
                guidance: Inputs::default(),
                ground: None,
                ground_stamp: None,
                drift: [0.0; 2],
            };
            law.step(&step);
            law.commands()
        };
        assert_eq!(step(&mut law("lean")), step(&mut law("hold")));
    }
}
