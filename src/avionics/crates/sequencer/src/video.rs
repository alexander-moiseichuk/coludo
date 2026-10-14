//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The camera by stage: the rules of the firmware's `video` thread, the one owner of the camera's recorder
//! (doc/specs/avionics-port.md "Video by stage"). New in Rust: plan B records no video. The thread reads the flight
//! word and the fused accel every 250 ms and asks its [`Trigger`] what to do ([`Trigger::look`]), and it answers the
//! operator's `video start` and `video stop` by [`start_refused`] and [`stop_refused`]. Nothing here touches the
//! camera.
//!
//! - The rod trigger: the board armed in SETTING starts the flight recording ([`Owner::Flight`]) once the airframe
//!   stands nose up on the rod: the fused accel's +x (the body frame: +x the nose, +y left, +z up; specific force, g)
//!   at or above 0.87 g, within about 30 degrees of vertical, at 8 looks in a row, 2 s at the 250 ms look. It reads
//!   the accel, not the attitude filter, so it stands on its own. On a board with an accelerometer, a look with no
//!   fresh accel is a look off the rod (the count restarts): a stale channel never starts a recording on the bench. A
//!   board without one (no `accel` on its databoard) starts it at the episode's first look, as the arm alone did
//!   before the rod trigger, so it still records. The start replaces an operator's recording.
//! - An arm episode is a run of looks armed in SETTING: it begins at an arm there, or at a return to SETTING still
//!   armed (the way back from a false launch: `stage done`, the re-zero, `stage setting`), since DONE stopped the
//!   recording. Each episode counts its nose-up looks afresh and has one start: a start that fails, or a flight
//!   recording the operator's `video stop` ends, is not started again before the next episode.
//! - A disarm in SETTING stops the flight recording; an operator's recording runs on.
//! - DONE stops whatever records.
//! - Nothing starts outside SETTING, so the camera's register traffic on the shared I2C port 0 stays on the pad, and
//!   nothing stops in the air: the operator's `video stop` runs in SETTING and at DONE.
//!
//! Video is a utility (the operator: at least 20 delivered frames a second, and never at the flight's expense): a
//! camera that fails never blocks `arm`.

use core::fmt;

use crate::controller::{ChangeRefused, Flight, Stage};

/// The nose-up level, g: the fused accel's +x at or above it is the airframe standing within about 30 degrees of
/// vertical (cos 30 degrees = 0.866) on the pad's 1 g.
const NOSE_UP_G: f32 = 0.87;

/// How many looks in a row the nose must be up for the flight recording to start: 2 s at the video thread's 250 ms
/// look, long enough that a board handled on the bench does not start it.
const NOSE_UP_LOOKS: u8 = 8;

/// Who started the running recording.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Owner {
    /// The rod trigger: the board armed in SETTING, standing nose up on the rod, or without an accelerometer to tell.
    Flight,
    /// The operator: GCL `video start`.
    Operator,
}

/// What the video thread does after a look at the flight word ([`Trigger::look`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Duty {
    /// Start the flight recording, after stopping the operator's when one runs.
    StartFlight,
    /// Stop the flight recording: the board was disarmed on the pad.
    StopFlight,
    /// Stop whatever records: the flight is over.
    StopLanded,
}

/// Whether the board is armed on the pad: the state the flight recording starts in.
fn armed_in_setting(flight: Flight) -> bool {
    flight.armed && flight.stage == Stage::Setting
}

/// The rod trigger (see the module notes): whether the board has an accelerometer, and what the video thread keeps of
/// the arm episode between its looks. A disarm and an arm between two looks are one episode: the thread sees neither.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Trigger {
    /// Whether the board has an accelerometer (an `accel` quantity on its databoard): the nose decides the start.
    accelerometer: bool,
    /// The episode's looks in a row with the nose up.
    nose_up_looks: u8,
    /// Whether the episode has had its one start: made, failed, or found running already.
    started: bool,
}

impl Trigger {
    /// The trigger of a board with an `accelerometer` or without one, in no episode, as at every boot's first look.
    pub fn new(accelerometer: bool) -> Trigger {
        Trigger {
            accelerometer,
            nose_up_looks: 0,
            started: false,
        }
    }

    /// What the video thread does now, from the flight word, the fused accel and the recording it runs: one look of
    /// the arm episode, which a look not armed in SETTING ends.
    ///
    /// Args:
    /// * `now` - the flight now;
    /// * `accel` - the fused accel's specific force in g on the body axes (+x the nose, +y left, +z up), None when no
    ///   source is fresh (with an accelerometer: not nose up); not read without one;
    /// * `recording` - who started the running recording, None when nothing records.
    ///
    /// Returns: [`Duty::StartFlight`] armed in SETTING, once per episode, when the nose has been up (+x at or above
    /// 0.87 g) for 8 looks in a row, or at the episode's first look without an accelerometer, unless the flight
    /// recording runs already (it then counts as the episode's start); [`Duty::StopFlight`] in SETTING, disarmed,
    /// while the flight recording runs; [`Duty::StopLanded`] at DONE while anything records; None otherwise, every
    /// stage in the air included.
    pub fn look(&mut self, now: Flight, accel: Option<[f32; 3]>, recording: Option<Owner>) -> Option<Duty> {
        if !armed_in_setting(now) {
            *self = Trigger::new(self.accelerometer);
        }
        match now.stage {
            Stage::Done => recording.map(|_| Duty::StopLanded),
            Stage::Setting if now.armed => {
                if self.started || !self.on_the_rod(accel) {
                    return None;
                }
                self.started = true;
                (recording != Some(Owner::Flight)).then_some(Duty::StartFlight)
            }
            Stage::Setting => (recording == Some(Owner::Flight)).then_some(Duty::StopFlight),
            Stage::Boosting | Stage::Gliding | Stage::Landing => None,
        }
    }

    /// One armed look's count of the nose-up looks in a row: a look with the nose below [`NOSE_UP_G`], or with no
    /// fresh accel, restarts it.
    ///
    /// Returns: whether the airframe now stands on the rod: the nose up for [`NOSE_UP_LOOKS`] looks in a row, or true
    /// at once without an accelerometer, with nothing to tell.
    fn on_the_rod(&mut self, accel: Option<[f32; 3]>) -> bool {
        if !self.accelerometer {
            return true;
        }
        let nose_up = accel.is_some_and(|[nose_g, _, _]| nose_g >= NOSE_UP_G);
        // the start ends the count, so it never passes NOSE_UP_LOOKS
        self.nose_up_looks = if nose_up { self.nose_up_looks + 1 } else { 0 };
        self.nose_up_looks >= NOSE_UP_LOOKS
    }
}

/// Why the operator's `video start` or `video stop` does not run ([`start_refused`], [`stop_refused`]).
/// [`fmt::Display`] writes the text the operator is answered with, under [`Refusal::code`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// `video start` outside SETTING: it changes the board.
    Stage(ChangeRefused),
    /// `video stop` in the air: it would end the flight's video.
    Airborne(Stage),
    /// `video start` while a recording runs.
    AlreadyStarted,
    /// `video stop` while nothing records.
    NotStarted,
}

impl Refusal {
    /// The CC error code the refusal is answered with: `unsafe` for a stage that forbids the command, as the command
    /// gate answers; `video` for the recorder's state, as `video start` and `video stop` answer their failures.
    pub fn code(self) -> &'static str {
        match self {
            Refusal::Stage(_) | Refusal::Airborne(_) => "unsafe",
            Refusal::AlreadyStarted | Refusal::NotStarted => "video",
        }
    }
}

impl fmt::Display for Refusal {
    /// The refusal's text: a stage refusal names the stage; the recorder's state reads as the C recorder words it.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Refusal::Stage(refusal) => fmt::Display::fmt(refusal, formatter),
            Refusal::Airborne(stage) => write!(
                formatter,
                "refused -- stage is {} (video stops only in setting or done)",
                stage.name()
            ),
            Refusal::AlreadyStarted => formatter.write_str("already started"),
            Refusal::NotStarted => formatter.write_str("not started"),
        }
    }
}

/// Why the operator's `video start` must not run now, None when it may: only in SETTING (re-checked where the video
/// thread serves it, since the request may wait behind a start or a stop), and only while nothing records.
pub fn start_refused(flight: Flight, recording: Option<Owner>) -> Option<Refusal> {
    if let Some(refusal) = flight.stage.change_refused(false) {
        return Some(Refusal::Stage(refusal));
    }
    recording.map(|_| Refusal::AlreadyStarted)
}

/// Why the operator's `video stop` must not run now, None when it may: on the ground (SETTING, and DONE, where the
/// command gate refuses every other change), and only while a recording runs, whoever started it.
pub fn stop_refused(flight: Flight, recording: Option<Owner>) -> Option<Refusal> {
    if flight.stage.airborne() {
        return Some(Refusal::Airborne(flight.stage));
    }
    recording.is_none().then_some(Refusal::NotStarted)
}

#[cfg(test)]
mod tests {
    use super::{Duty, Owner, Refusal, Trigger, start_refused, stop_refused};
    use crate::controller::{Flight, Stage};

    /// What a recorder runs: nothing, the flight's, or the operator's.
    const RECORDINGS: [Option<Owner>; 3] = [None, Some(Owner::Flight), Some(Owner::Operator)];

    /// Standing on the rod: 1 g along +x, the nose.
    const NOSE_UP: Option<[f32; 3]> = Some([1.0, 0.0, 0.0]);

    /// Lying flat on the bench: 1 g along +z.
    const FLAT: Option<[f32; 3]> = Some([0.0, 0.0, 1.0]);

    /// Upside down on the rod: 1 g along -x.
    const NOSE_DOWN: Option<[f32; 3]> = Some([-1.0, 0.0, 0.0]);

    /// What a look reads of the accel: no fresh source, or each orientation.
    const ACCELS: [Option<[f32; 3]>; 4] = [None, NOSE_UP, FLAT, NOSE_DOWN];

    /// A flight at `stage`, armed or not, with no hold and no launch.
    fn at(stage: Stage, armed: bool) -> Flight {
        Flight {
            stage,
            armed,
            ..Flight::GROUND
        }
    }

    /// Every flight there is: 5 stages times the three flags.
    fn every_flight() -> Vec<Flight> {
        let mut flights = Vec::new();
        for stage in Stage::ALL {
            for bits in 0..8 {
                flights.push(Flight {
                    stage,
                    manual: bits & 1 != 0,
                    armed: bits & 2 != 0,
                    launched: bits & 4 != 0,
                });
            }
        }
        flights
    }

    /// `count` looks of `trigger` at one flight, accel and recording: the duties they ask for, in order.
    fn looks(
        trigger: &mut Trigger,
        count: usize,
        now: Flight,
        accel: Option<[f32; 3]>,
        recording: Option<Owner>,
    ) -> Vec<Option<Duty>> {
        (0..count).map(|_| trigger.look(now, accel, recording)).collect()
    }

    /// The duties of `count` looks that ask nothing.
    fn quiet(count: usize) -> Vec<Option<Duty>> {
        vec![None; count]
    }

    /// The duties of `count` looks that ask nothing, then one that starts the flight recording.
    fn quiet_then_start(count: usize) -> Vec<Option<Duty>> {
        let mut duties = quiet(count);
        duties.push(Some(Duty::StartFlight));
        duties
    }

    /// The trigger of a board with an accelerometer, in no episode.
    fn fitted() -> Trigger {
        Trigger::new(true)
    }

    /// The trigger of a board without an accelerometer, in no episode.
    fn unfitted() -> Trigger {
        Trigger::new(false)
    }

    /// The trigger at each point of an arm episode: with an accelerometer none, one nose-up look short of the start,
    /// and started; without one none and started.
    fn triggers() -> [Trigger; 5] {
        let armed = at(Stage::Setting, true);
        let mut short = fitted();
        assert_eq!(looks(&mut short, 7, armed, NOSE_UP, None), quiet(7));
        let mut started = short;
        assert_eq!(started.look(armed, NOSE_UP, None), Some(Duty::StartFlight));
        let mut unfitted_started = unfitted();
        assert_eq!(unfitted_started.look(armed, None, None), Some(Duty::StartFlight));
        [fitted(), short, started, unfitted(), unfitted_started]
    }

    #[test]
    fn nose_up_for_eight_looks_starts_the_flight_recording_once() {
        let armed = at(Stage::Setting, true);
        // the first look of a boot that is armed in SETTING already counts too
        let mut trigger = fitted();
        assert_eq!(looks(&mut trigger, 8, armed, NOSE_UP, None), quiet_then_start(7));
        assert_eq!(
            looks(&mut trigger, 40, armed, NOSE_UP, Some(Owner::Flight)),
            quiet(40),
            "it records"
        );
    }

    #[test]
    fn the_start_replaces_an_operator_recording() {
        let armed = at(Stage::Setting, true);
        let mut trigger = fitted();
        assert_eq!(
            looks(&mut trigger, 8, armed, NOSE_UP, Some(Owner::Operator)),
            quiet_then_start(7),
            "the operator's runs on while the nose-up looks count"
        );
        assert_eq!(
            unfitted().look(armed, None, Some(Owner::Operator)),
            Some(Duty::StartFlight),
            "no accelerometer"
        );
    }

    #[test]
    fn a_board_without_an_accelerometer_starts_at_the_first_armed_look() {
        let armed = at(Stage::Setting, true);
        let disarmed = at(Stage::Setting, false);
        // the arm alone starts it; the accel is not read
        for accel in ACCELS {
            assert_eq!(
                unfitted().look(armed, accel, None),
                Some(Duty::StartFlight),
                "{accel:?}"
            );
        }
        let mut trigger = unfitted();
        assert_eq!(looks(&mut trigger, 40, disarmed, None, None), quiet(40), "disarmed");
        assert_eq!(looks(&mut trigger, 1, armed, None, None), quiet_then_start(0));
        assert_eq!(
            looks(&mut trigger, 40, armed, None, None),
            quiet(40),
            "a failed start is not retried"
        );
        assert_eq!(trigger.look(disarmed, None, None), None);
        assert_eq!(
            looks(&mut trigger, 1, armed, None, None),
            quiet_then_start(0),
            "re-armed"
        );
    }

    #[test]
    fn a_flat_board_with_gaps_never_starts() {
        let armed = at(Stage::Setting, true);
        // a stale channel is no rod: flat with gaps, only gaps, and a gap at every 8th look of a nose up
        let mut trigger = fitted();
        for _ in 0..50 {
            assert_eq!(trigger.look(armed, FLAT, None), None);
            assert_eq!(trigger.look(armed, None, None), None);
        }
        assert_eq!(looks(&mut trigger, 100, armed, None, None), quiet(100));
        for _ in 0..10 {
            assert_eq!(looks(&mut trigger, 7, armed, NOSE_UP, None), quiet(7));
            assert_eq!(trigger.look(armed, None, None), None);
        }
    }

    #[test]
    fn seven_looks_then_flat_starts_nothing() {
        let armed = at(Stage::Setting, true);
        let mut trigger = fitted();
        assert_eq!(looks(&mut trigger, 7, armed, NOSE_UP, None), quiet(7));
        assert_eq!(looks(&mut trigger, 40, armed, FLAT, None), quiet(40));
    }

    #[test]
    fn a_look_off_the_rod_restarts_the_count() {
        let armed = at(Stage::Setting, true);
        // flat, upside down, and a gap in the accel
        for off in [FLAT, NOSE_DOWN, None] {
            let mut trigger = fitted();
            assert_eq!(looks(&mut trigger, 5, armed, NOSE_UP, None), quiet(5));
            assert_eq!(trigger.look(armed, off, None), None);
            assert_eq!(
                looks(&mut trigger, 8, armed, NOSE_UP, None),
                quiet_then_start(7),
                "{off:?}"
            );
        }
    }

    #[test]
    fn armed_off_the_rod_starts_nothing() {
        let armed = at(Stage::Setting, true);
        // flat on the bench, tilted 45 degrees, and upside down on the rod
        for off in [FLAT, Some([0.707, 0.0, 0.707]), NOSE_DOWN] {
            for recording in [None, Some(Owner::Operator)] {
                let mut trigger = fitted();
                assert_eq!(
                    looks(&mut trigger, 100, armed, off, recording),
                    quiet(100),
                    "{off:?} with {recording:?}"
                );
            }
        }
    }

    #[test]
    fn the_rod_begins_at_0_87_g() {
        let armed = at(Stage::Setting, true);
        let mut trigger = fitted();
        assert_eq!(
            looks(&mut trigger, 40, armed, Some([0.869, 0.0, 0.495]), None),
            quiet(40)
        );
        let mut trigger = fitted();
        assert_eq!(
            looks(&mut trigger, 8, armed, Some([0.87, 0.0, 0.49]), None),
            quiet_then_start(7)
        );
    }

    #[test]
    fn disarmed_and_nose_up_starts_nothing() {
        let disarmed = at(Stage::Setting, false);
        let armed = at(Stage::Setting, true);
        for recording in [None, Some(Owner::Operator)] {
            let mut trigger = fitted();
            assert_eq!(looks(&mut trigger, 100, disarmed, NOSE_UP, recording), quiet(100));
            // the count begins at the arm
            assert_eq!(
                looks(&mut trigger, 8, armed, NOSE_UP, recording),
                quiet_then_start(7),
                "{recording:?}"
            );
        }
    }

    #[test]
    fn a_rearm_after_a_failed_start_starts_again() {
        let armed = at(Stage::Setting, true);
        let disarmed = at(Stage::Setting, false);
        let mut trigger = fitted();
        assert_eq!(looks(&mut trigger, 8, armed, NOSE_UP, None), quiet_then_start(7));
        // the start failed: nothing records, and the episode does not try again
        assert_eq!(looks(&mut trigger, 40, armed, NOSE_UP, None), quiet(40));
        assert_eq!(trigger.look(disarmed, NOSE_UP, None), None);
        assert_eq!(looks(&mut trigger, 8, armed, NOSE_UP, None), quiet_then_start(7));
    }

    #[test]
    fn one_start_per_arm_episode() {
        let armed = at(Stage::Setting, true);
        // the operator's `video stop` ends the flight recording: not started again before the next episode
        let mut trigger = fitted();
        assert_eq!(looks(&mut trigger, 8, armed, NOSE_UP, None), quiet_then_start(7));
        assert_eq!(looks(&mut trigger, 4, armed, NOSE_UP, Some(Owner::Flight)), quiet(4));
        assert_eq!(looks(&mut trigger, 40, armed, NOSE_UP, None), quiet(40));
        // back to SETTING still armed with the flight recording running: it is the episode's start
        let mut trigger = fitted();
        assert_eq!(
            trigger.look(at(Stage::Gliding, true), NOSE_UP, Some(Owner::Flight)),
            None
        );
        assert_eq!(looks(&mut trigger, 8, armed, NOSE_UP, Some(Owner::Flight)), quiet(8));
        assert_eq!(looks(&mut trigger, 40, armed, NOSE_UP, None), quiet(40));
    }

    #[test]
    fn a_return_to_setting_still_armed_starts_it_again() {
        // the false launch's way back: DONE stopped the recording, `stage setting` keeps the arming
        let armed = at(Stage::Setting, true);
        let landed = Flight {
            manual: true,
            ..at(Stage::Done, true)
        };
        // with an accelerometer on the rod again, and without one
        for (mut trigger, accel, quiet_looks) in [(fitted(), NOSE_UP, 7), (unfitted(), None, 0)] {
            assert_eq!(
                looks(&mut trigger, quiet_looks + 1, armed, accel, None),
                quiet_then_start(quiet_looks)
            );
            assert_eq!(
                trigger.look(at(Stage::Boosting, true), accel, Some(Owner::Flight)),
                None
            );
            assert_eq!(trigger.look(landed, accel, Some(Owner::Flight)), Some(Duty::StopLanded));
            assert_eq!(
                looks(&mut trigger, quiet_looks + 1, armed, accel, None),
                quiet_then_start(quiet_looks),
                "{trigger:?}"
            );
        }
    }

    #[test]
    fn nothing_starts_outside_setting_armed() {
        for trigger in triggers() {
            for now in every_flight() {
                for accel in ACCELS {
                    for recording in RECORDINGS {
                        if trigger.clone().look(now, accel, recording) == Some(Duty::StartFlight) {
                            assert!(
                                now.stage == Stage::Setting && now.armed,
                                "{trigger:?} -> {now:?} with {accel:?} and {recording:?}"
                            );
                        }
                    }
                }
            }
        }
        // one look short of the start, then the launch inside one look: armed in the air, nothing starts, and the
        // count is gone
        let short = triggers()[1];
        for stage in [Stage::Boosting, Stage::Gliding, Stage::Landing, Stage::Done] {
            let mut trigger = short;
            assert_eq!(trigger.look(at(stage, true), NOSE_UP, None), None, "{stage:?}");
            assert_eq!(trigger, fitted(), "{stage:?}");
        }
    }

    #[test]
    fn a_disarm_in_setting_stops_only_the_flight_recording() {
        let disarmed = at(Stage::Setting, false);
        for trigger in triggers() {
            for accel in ACCELS {
                assert_eq!(
                    trigger.clone().look(disarmed, accel, Some(Owner::Flight)),
                    Some(Duty::StopFlight)
                );
                assert_eq!(trigger.clone().look(disarmed, accel, Some(Owner::Operator)), None);
                assert_eq!(trigger.clone().look(disarmed, accel, None), None);
            }
        }
        // a disarm in the air, then the return to SETTING: the flight recording ends there
        let mut trigger = fitted();
        assert_eq!(
            trigger.look(at(Stage::Gliding, false), NOSE_UP, Some(Owner::Flight)),
            None
        );
        assert_eq!(
            trigger.look(disarmed, NOSE_UP, Some(Owner::Flight)),
            Some(Duty::StopFlight)
        );
    }

    #[test]
    fn done_stops_whatever_records() {
        for trigger in triggers() {
            for armed in [false, true] {
                for accel in ACCELS {
                    let landed = at(Stage::Done, armed);
                    assert_eq!(
                        trigger.clone().look(landed, accel, Some(Owner::Flight)),
                        Some(Duty::StopLanded)
                    );
                    assert_eq!(
                        trigger.clone().look(landed, accel, Some(Owner::Operator)),
                        Some(Duty::StopLanded)
                    );
                    assert_eq!(trigger.clone().look(landed, accel, None), None, "nothing records");
                }
            }
        }
    }

    #[test]
    fn nothing_stops_in_the_air() {
        for trigger in triggers() {
            for now in every_flight().into_iter().filter(|flight| flight.stage.airborne()) {
                for accel in ACCELS {
                    for recording in RECORDINGS {
                        assert_eq!(
                            trigger.clone().look(now, accel, recording),
                            None,
                            "{trigger:?} -> {now:?} with {accel:?} and {recording:?}"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn the_operator_starts_in_setting_while_nothing_records() {
        assert_eq!(start_refused(at(Stage::Setting, false), None), None);
        assert_eq!(
            start_refused(at(Stage::Setting, true), None),
            None,
            "armed, after a failed flight start"
        );
    }

    #[test]
    fn the_operator_start_is_refused_off_the_pad_and_while_recording() {
        for stage in [Stage::Boosting, Stage::Gliding, Stage::Landing, Stage::Done] {
            for recording in RECORDINGS {
                let refusal = start_refused(at(stage, false), recording).unwrap();
                assert_eq!(refusal.code(), "unsafe");
                assert_eq!(
                    refusal.to_string(),
                    format!("refused -- stage is {} (changes only in setting)", stage.name())
                );
            }
        }
        for recording in [Some(Owner::Flight), Some(Owner::Operator)] {
            let refusal = start_refused(at(Stage::Setting, true), recording);
            assert_eq!(refusal, Some(Refusal::AlreadyStarted));
            assert_eq!(refusal.map(Refusal::code), Some("video"));
            assert_eq!(
                refusal.map(|refusal| refusal.to_string()).as_deref(),
                Some("already started")
            );
        }
    }

    #[test]
    fn the_operator_stops_any_recording_on_the_ground() {
        for stage in [Stage::Setting, Stage::Done] {
            for owner in [Owner::Flight, Owner::Operator] {
                assert_eq!(stop_refused(at(stage, true), Some(owner)), None, "{stage:?}, {owner:?}");
            }
        }
    }

    #[test]
    fn the_operator_stop_is_refused_in_the_air_and_with_nothing_recording() {
        for stage in [Stage::Boosting, Stage::Gliding, Stage::Landing] {
            for recording in RECORDINGS {
                let refusal = stop_refused(at(stage, true), recording);
                assert_eq!(refusal, Some(Refusal::Airborne(stage)));
                assert_eq!(refusal.map(Refusal::code), Some("unsafe"));
                assert_eq!(
                    refusal.map(|refusal| refusal.to_string()),
                    Some(format!(
                        "refused -- stage is {} (video stops only in setting or done)",
                        stage.name()
                    ))
                );
            }
        }
        for stage in [Stage::Setting, Stage::Done] {
            let refusal = stop_refused(at(stage, false), None);
            assert_eq!(refusal, Some(Refusal::NotStarted));
            assert_eq!(refusal.map(Refusal::code), Some("video"));
            assert_eq!(
                refusal.map(|refusal| refusal.to_string()).as_deref(),
                Some("not started")
            );
        }
    }
}
