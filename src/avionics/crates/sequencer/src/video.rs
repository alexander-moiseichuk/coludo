//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The camera by stage: the rules of the firmware's `video` thread, the one owner of the camera's recorder
//! (doc/specs/avionics-port.md "Video by stage"). New in Rust: plan B records no video. The thread reads the flight
//! word every 250 ms and asks [`look`] what to do, and it answers the operator's `video start` and `video stop` by
//! [`start_refused`] and [`stop_refused`]. Nothing here touches the camera.
//!
//! - The board armed in SETTING starts the flight recording ([`Owner::Flight`]) and replaces an operator's: an arm
//!   there, or a return to SETTING while still armed (the way back from a false launch: `stage done`, the re-zero,
//!   `stage setting`), since DONE stopped the recording.
//! - A disarm in SETTING stops the flight recording; an operator's recording runs on.
//! - DONE stops whatever records.
//! - Nothing starts outside SETTING, so the camera's register traffic on the shared I2C port 0 stays on the pad, and
//!   nothing stops in the air: the operator's `video stop` runs in SETTING and at DONE.
//!
//! Video is a utility (the operator: at least 20 delivered frames a second, and never at the flight's expense): a
//! camera that fails never blocks `arm`, and a start that fails at arm is not retried before the board is next armed
//! in SETTING.

use core::fmt;

use crate::controller::{ChangeRefused, Flight, Stage};

/// Who started the running recording.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Owner {
    /// The flight word: the board was armed in SETTING.
    Flight,
    /// The operator: GCL `video start`.
    Operator,
}

/// What the video thread does after a look at the flight word ([`look`]).
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

/// What the video thread does now, from the flight word at its last look and now, and the recording it runs.
///
/// Args:
/// * `seen` - the flight at the last look ([`Flight::GROUND`] before the first: a board armed in SETTING at the
///   first look starts its recording);
/// * `now` - the flight now;
/// * `recording` - who started the running recording, None when nothing records.
///
/// Returns: [`Duty::StartFlight`] when the board has become armed in SETTING since the last look and the flight
/// recording does not run; [`Duty::StopFlight`] in SETTING, disarmed, while the flight recording runs;
/// [`Duty::StopLanded`] at DONE while anything records; None otherwise, every stage in the air included.
pub fn look(seen: Flight, now: Flight, recording: Option<Owner>) -> Option<Duty> {
    match now.stage {
        Stage::Done => recording.map(|_| Duty::StopLanded),
        Stage::Setting if now.armed => {
            (!armed_in_setting(seen) && recording != Some(Owner::Flight)).then_some(Duty::StartFlight)
        }
        Stage::Setting => (recording == Some(Owner::Flight)).then_some(Duty::StopFlight),
        Stage::Boosting | Stage::Gliding | Stage::Landing => None,
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
    use super::{Duty, Owner, Refusal, look, start_refused, stop_refused};
    use crate::controller::{Flight, Stage};

    /// What a recorder runs: nothing, the flight's, or the operator's.
    const RECORDINGS: [Option<Owner>; 3] = [None, Some(Owner::Flight), Some(Owner::Operator)];

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

    #[test]
    fn an_arm_in_setting_starts_the_flight_recording() {
        let unarmed = at(Stage::Setting, false);
        let armed = at(Stage::Setting, true);
        assert_eq!(look(unarmed, armed, None), Some(Duty::StartFlight));
        assert_eq!(
            look(unarmed, armed, Some(Owner::Operator)),
            Some(Duty::StartFlight),
            "the operator's recording is replaced"
        );
        // the first look of a boot that is armed in SETTING already
        assert_eq!(look(Flight::GROUND, armed, None), Some(Duty::StartFlight));
    }

    #[test]
    fn a_return_to_setting_still_armed_starts_it_again() {
        // the false launch's way back: DONE stopped the recording, `stage setting` keeps the arming
        let landed = Flight {
            manual: true,
            ..at(Stage::Done, true)
        };
        assert_eq!(look(landed, at(Stage::Setting, true), None), Some(Duty::StartFlight));
        assert_eq!(
            look(
                at(Stage::Gliding, true),
                at(Stage::Setting, true),
                Some(Owner::Operator)
            ),
            Some(Duty::StartFlight)
        );
    }

    #[test]
    fn staying_armed_never_starts_again() {
        let armed = at(Stage::Setting, true);
        // a start that failed at arm left nothing recording: no retry before the next arm
        assert_eq!(look(armed, armed, None), None);
        assert_eq!(
            look(armed, armed, Some(Owner::Operator)),
            None,
            "the operator's, started after the arm"
        );
        // the flight recording runs already (a disarm and an arm between two looks)
        assert_eq!(look(at(Stage::Setting, false), armed, Some(Owner::Flight)), None);
        // back to SETTING with the flight recording still running: nothing to do
        assert_eq!(look(at(Stage::Gliding, true), armed, Some(Owner::Flight)), None);
    }

    #[test]
    fn nothing_starts_outside_setting() {
        for seen in every_flight() {
            for now in every_flight() {
                for recording in RECORDINGS {
                    if look(seen, now, recording) == Some(Duty::StartFlight) {
                        assert!(
                            now.stage == Stage::Setting && now.armed,
                            "{seen:?} -> {now:?} with {recording:?}"
                        );
                    }
                }
            }
        }
        // an arm and the launch inside one look: the board is armed in the air, and nothing starts
        for stage in [Stage::Boosting, Stage::Gliding, Stage::Landing, Stage::Done] {
            assert_eq!(
                look(at(Stage::Setting, false), at(stage, true), None),
                None,
                "{stage:?}"
            );
        }
    }

    #[test]
    fn a_disarm_in_setting_stops_only_the_flight_recording() {
        let armed = at(Stage::Setting, true);
        let disarmed = at(Stage::Setting, false);
        assert_eq!(look(armed, disarmed, Some(Owner::Flight)), Some(Duty::StopFlight));
        assert_eq!(look(armed, disarmed, Some(Owner::Operator)), None);
        assert_eq!(look(armed, disarmed, None), None);
        // a disarm in the air, then the return to SETTING: the flight recording ends there
        assert_eq!(
            look(at(Stage::Gliding, false), disarmed, Some(Owner::Flight)),
            Some(Duty::StopFlight)
        );
    }

    #[test]
    fn done_stops_whatever_records() {
        for armed in [false, true] {
            for seen in [at(Stage::Landing, armed), at(Stage::Done, armed)] {
                let landed = at(Stage::Done, armed);
                assert_eq!(look(seen, landed, Some(Owner::Flight)), Some(Duty::StopLanded));
                assert_eq!(look(seen, landed, Some(Owner::Operator)), Some(Duty::StopLanded));
                assert_eq!(look(seen, landed, None), None, "nothing records");
            }
        }
    }

    #[test]
    fn nothing_stops_in_the_air() {
        for seen in every_flight() {
            for now in every_flight().into_iter().filter(|flight| flight.stage.airborne()) {
                for recording in RECORDINGS {
                    assert_eq!(
                        look(seen, now, recording),
                        None,
                        "{seen:?} -> {now:?} with {recording:?}"
                    );
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
