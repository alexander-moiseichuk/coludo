//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The flight stages and the flight word, from src/glider controller.py: the stages (`Stage`: their ids, their
//! operator-facing names, which of them are on the ground and which airborne, and in which a command that changes the
//! board may run: plan B's cc_client `_in_flight`, narrowed), and the Controller's flight state
//! (`stage`, `manual`, `armed`) with the rules its `hold`, `resume`, `arm` and `disarm` change it by.
//!
//! **The flight word.** Plan B kept that state in three Controller attributes, set by five writers on one asyncio
//! loop. Rust packs it, with `launched`, into ONE `AtomicU32` ([`FlightWord`]): a reader's single load is a
//! consistent snapshot (a stage never pairs with another moment's hold or arming), and one writer ([`FlightWriter`],
//! not Clone, handed out once) commits every change: the stage machine's advances, the separation's, and the
//! operator's requests, which reach it through a [`crate::slot::RequestSlot`]. That writer lives on the control
//! thread, so a request is applied between two ticks and never races a detector.
//!
//! The rest of plan B's Controller (bring-up, the device directory) is the firmware's.

use core::fmt;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use crate::sequencer::Transition;

/// One flight stage. The ids are plan B's, so a warm-start crumb reads the same on both firmwares; 0 is the
/// NVS "no checkpoint" sentinel and never a live stage.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    /// On the pad: the sequencer is free to detect the launch.
    Setting = 1,
    /// Powered boost.
    Boosting = 2,
    /// Unpowered glide after separation.
    Gliding = 3,
    /// The endgame: the approach and the landing.
    Landing = 4,
    /// Landed; the flight is over.
    Done = 5,
}

impl Stage {
    /// Every stage, in id order.
    pub const ALL: [Stage; 5] = [
        Stage::Setting,
        Stage::Boosting,
        Stage::Gliding,
        Stage::Landing,
        Stage::Done,
    ];

    /// The operator-facing name (`stage`, `health`, `whoami`, and the stage column of sequencer.csv).
    pub fn name(self) -> &'static str {
        match self {
            Stage::Setting => "setting",
            Stage::Boosting => "boosting",
            Stage::Gliding => "gliding",
            Stage::Landing => "landing",
            Stage::Done => "done",
        }
    }

    /// The stage an operator names, None for a name that is not a stage.
    pub fn from_name(name: &str) -> Option<Stage> {
        Stage::ALL.into_iter().find(|stage| stage.name() == name)
    }

    /// The stage with this id, None for 0 (the sentinel) or an id past the last stage.
    pub fn from_id(id: u8) -> Option<Stage> {
        Stage::ALL.into_iter().find(|stage| *stage as u8 == id)
    }

    /// Nothing is flying: the stages active commands may run in (src/glider cc_client `_GROUND`).
    pub fn ground(self) -> bool {
        matches!(self, Stage::Setting | Stage::Done)
    }

    /// Off the ground: BOOSTING through LANDING (plan B `Stage.airborne`, the union of the powered boost and the
    /// passive glide). The stage machine's RSO backstop bounds exactly these: it once tested "not DONE" instead,
    /// which let a stale timer force DONE from SETTING on a board sitting on the pad.
    pub fn airborne(self) -> bool {
        !self.ground()
    }

    /// Why a command that changes the board must not run in this stage, None when it may.
    ///
    /// Changes run only in SETTING (the operator's rule; plan B's cc_client `_in_flight` allowed them at DONE too). A
    /// barometer's re-zero (`rezero`: its `calibrate`, its `update` of `ground` or `rezero`) also runs at DONE: it
    /// is the way back from a false launch without a power cycle (`stage done`, the re-zero, then `stage setting`).
    /// The executor checks it before a command runs, and the thread that serves the command checks it again, since a
    /// probe sweep takes seconds and a launch may be detected meanwhile. It allocates nothing: that thread may be the
    /// control thread or a bus owner off the pad, and the refusal's text is made where the operator is answered.
    ///
    /// Args: `rezero` the command re-zeroes a barometer.
    ///
    /// Returns: the refusal, naming this stage; None in SETTING, and at DONE for a re-zero.
    pub fn change_refused(self, rezero: bool) -> Option<ChangeRefused> {
        let allowed = self == Stage::Setting || (rezero && self == Stage::Done);
        (!allowed).then_some(ChangeRefused(self))
    }
}

/// A change refused in a stage ([`Stage::change_refused`]): the stage as a value, so the thread that refuses makes no
/// text. [`fmt::Display`] writes `refused -- stage is <stage> (changes only in setting)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChangeRefused(Stage);

impl fmt::Display for ChangeRefused {
    /// The refusal's text, naming the stage.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "refused -- stage is {} (changes only in setting)",
            self.0.name()
        )
    }
}

/// The flight word's stage field: bits 0-2 hold the [`Stage`] id.
const STAGE_BITS: u32 = 0b111;
/// The flight word's operator-hold bit.
const MANUAL_BIT: u32 = 1 << 3;
/// The flight word's arming bit.
const ARMED_BIT: u32 = 1 << 4;
/// The flight word's launch bit.
const LAUNCHED_BIT: u32 = 1 << 5;
/// Every bit a flight word uses: a word with any other bit set is not one. The request slot carries a flight in
/// the byte under its refused bit, so this must stay within 7 bits.
pub(crate) const FLIGHT_BITS: u32 = STAGE_BITS | MANUAL_BIT | ARMED_BIT | LAUNCHED_BIT;

/// `bit` when `set`, else 0: one flag of a packed word.
const fn bit_when(set: bool, bit: u32) -> u32 {
    if set { bit } else { 0 }
}

/// The board's flight state as one value: what the flight word packs.
///
/// Plan B's Controller `stage`, `manual` and `armed`, plus `launched`, which plan B did not need: Rust keys the
/// link's reconnects on it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Flight {
    /// The flight stage.
    pub stage: Stage,
    /// An operator hold (`stage <name>`): the stage machine leaves the stage alone (plan B `manual`).
    pub manual: bool,
    /// Actuation enabled: the arm gate passed (plan B `armed`). Disarmed fins hold neutral.
    pub armed: bool,
    /// The stage machine itself detected this flight's launch (its own SETTING -> BOOSTING advance), or a warm
    /// start restored a flight that was in the air ([`FlightWriter::restore`]), and no operator hold has come since.
    /// While it is set the link never rejoins, hunts or dials ([`Flight::reconnect_allowed`]).
    pub launched: bool,
}

impl Flight {
    /// A board on the pad: SETTING, no hold, disarmed, no launch. Every boot starts here; only a warm start's
    /// restore ([`FlightWriter::restore`]) moves the word on from it before the threads run.
    pub const GROUND: Flight = Flight {
        stage: Stage::Setting,
        manual: false,
        armed: false,
        launched: false,
    };

    /// The flight word: the stage id in bits 0-2, then `manual` (bit 3), `armed` (bit 4) and `launched` (bit 5).
    pub const fn pack(self) -> u32 {
        self.stage as u32
            | bit_when(self.manual, MANUAL_BIT)
            | bit_when(self.armed, ARMED_BIT)
            | bit_when(self.launched, LAUNCHED_BIT)
    }

    /// The flight a word packs, None when it is no flight word: a stage id that is no stage (0, the warm start's
    /// "no checkpoint" sentinel, or past DONE), or a bit set past `launched`.
    pub fn unpack(word: u32) -> Option<Flight> {
        if word & !FLIGHT_BITS != 0 {
            return None;
        }
        Some(Flight {
            stage: Stage::from_id((word & STAGE_BITS) as u8)?,
            manual: word & MANUAL_BIT != 0,
            armed: word & ARMED_BIT != 0,
            launched: word & LAUNCHED_BIT != 0,
        })
    }

    /// The flight after an operator request, by plan B's Controller rules (controller.py `hold`, `resume`, `arm`,
    /// `disarm`), or why an arm is refused.
    ///
    /// - Hold: the stage is forced, and held unless it is SETTING: `stage setting` is a return to the ground, not a
    ///   hold (a held SETTING suppressed every launch detector, and a board flew its whole flight recorded as
    ///   SETTING). Every hold clears `launched`: an operator move is never the machine's launch, so a board held
    ///   after landing (`stage gliding` from CC) gets its reconnects back.
    /// - Resume clears the hold and nothing else: it never makes the board launched again.
    /// - Arm enables actuation only on the ground under automatic sequencing ([`Flight::stage_problem`]), checked
    ///   here, when the control thread applies it. Plan B awaited its probe sweep and then armed whatever the stage
    ///   had become meanwhile, a launch detected during the sweep included.
    /// - Disarm always clears the arming: it is how the operator recovers.
    ///
    /// Args: `request` the operator's request.
    ///
    /// Returns: the new flight; for an arm off the ground or under a hold, the stage problem, and no change.
    pub fn apply(self, request: Request) -> Result<Flight, StageProblem> {
        match request {
            Request::Hold(stage) => Ok(Flight {
                stage,
                manual: stage != Stage::Setting,
                launched: false,
                ..self
            }),
            Request::Resume => Ok(Flight { manual: false, ..self }),
            Request::Arm => match self.stage_problem() {
                Some(problem) => Err(problem),
                None => Ok(Flight { armed: true, ..self }),
            },
            Request::Disarm => Ok(Flight { armed: false, ..self }),
        }
    }

    /// Why this board is not on the ground under automatic sequencing, None when it is (plan B cc_client
    /// `_stage_problem`): `arm` refuses it and `verify` reports it, both under the key `stage`.
    pub fn stage_problem(self) -> Option<StageProblem> {
        if self.manual {
            return Some(StageProblem::Held(self.stage));
        }
        (self.stage != Stage::Setting).then_some(StageProblem::NotSetting(self.stage))
    }

    /// Whether the link may rejoin, hunt or dial: not once the stage machine has detected the launch.
    ///
    /// In flight the glider leaves range and the link drops by itself; a session still up runs on until it does.
    /// Reconnecting would chase the pad network it left, and at DONE the recovery beacon's AP is the only network.
    /// An operator hold is no launch, so a bench `stage gliding` keeps the normal reconnects. The radio itself is
    /// never switched off: stopping and restarting Wi-Fi through the C6 coprocessor is an unproven path that can
    /// park the link until a reboot.
    pub fn reconnect_allowed(self) -> bool {
        !self.launched
    }
}

/// An operator request the control thread applies to the flight word ([`Flight::apply`]), posted through a
/// [`crate::slot::RequestSlot`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Request {
    /// `stage <name>`: force the stage and hold it (a ground test), except that SETTING is a return to the ground.
    Hold(Stage),
    /// `stage auto`: release the hold.
    Resume,
    /// `arm`: enable actuation, the stage re-checked when applied.
    Arm,
    /// `disarm`: actuation off, always.
    Disarm,
}

/// Why a board is not on the ground under automatic sequencing ([`Flight::stage_problem`]). [`fmt::Display`]
/// writes plan B's text exactly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StageProblem {
    /// An operator hold, at this stage.
    Held(Stage),
    /// This stage, not SETTING, with no hold.
    NotSetting(Stage),
}

impl fmt::Display for StageProblem {
    /// Plan B's `_stage_problem` text, with the way back.
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            StageProblem::Held(stage) => write!(
                formatter,
                "stage held at {} by an operator command -- `stage setting` to return to the ground",
                stage.name()
            ),
            StageProblem::NotSetting(stage) => write!(
                formatter,
                "stage is {}, not setting -- `stage setting`, then power-cycle before flight",
                stage.name()
            ),
        }
    }
}

/// The flight word: the one atomic every thread reads the [`Flight`] from, written only through its
/// [`FlightWriter`]. Release stores and Acquire loads: a reader that sees a commit sees what came before it.
#[derive(Debug)]
pub struct FlightWord {
    /// [`Flight::pack`] of the flight.
    word: AtomicU32,
    /// Whether [`FlightWord::writer`] has handed the writer out.
    writer_taken: AtomicBool,
}

impl FlightWord {
    /// A word at [`Flight::GROUND`], as every boot starts, its writer not yet handed out. `const`, so the firmware's
    /// word is a static. Any other flight is committed through the writer, a warm start's restore included.
    pub const fn new() -> FlightWord {
        FlightWord {
            word: AtomicU32::new(Flight::GROUND.pack()),
            writer_taken: AtomicBool::new(false),
        }
    }

    /// The word's address, for a C reader that loads it atomically (the video recorder's segment cuts, which stop once
    /// the board leaves SETTING): [`Flight::pack`]'s bits. Valid as long as the word, a static on the board.
    pub fn as_ptr(&self) -> *const u32 {
        self.word.as_ptr()
    }

    /// The flight now: one load, so its stage, hold, arming and launch are of one moment.
    pub fn load(&self) -> Flight {
        // only Flight::pack reaches the word (new, FlightWriter::commit), so it always unpacks
        Flight::unpack(self.word.load(Ordering::Acquire)).unwrap_or(Flight::GROUND)
    }

    /// The word's one writer: Some the first time, None ever after (even once that writer is dropped), so no
    /// second thread can ever commit.
    pub fn writer(&self) -> Option<FlightWriter<'_>> {
        if self.writer_taken.swap(true, Ordering::AcqRel) {
            return None;
        }
        Some(FlightWriter {
            word: self,
            flight: self.load(),
        })
    }
}

impl Default for FlightWord {
    /// [`FlightWord::new`]: at [`Flight::GROUND`] (a zero word would be no flight).
    fn default() -> FlightWord {
        FlightWord::new()
    }
}

/// The flight word's one writer: the control thread's, so the stage machine, the separation and the operator's
/// requests commit in one order, between two ticks. Not Clone, and [`FlightWord::writer`] hands out only one.
#[derive(Debug)]
pub struct FlightWriter<'word> {
    /// The word it writes.
    word: &'word FlightWord,
    /// The flight it last committed: the word's value, since nothing else writes it.
    flight: Flight,
}

impl FlightWriter<'_> {
    /// The flight as last committed: the word's value.
    pub fn flight(&self) -> Flight {
        self.flight
    }

    /// Commit the stage machine's own advance ([`crate::Step::advance`]): the stage moves, and the launch
    /// detector's SETTING -> BOOSTING, the only advance into BOOSTING, sets `launched`.
    pub fn advance(&mut self, transition: Transition) -> Commit {
        let launched = self.flight.launched || transition.to == Stage::Boosting;
        self.commit(Flight {
            stage: transition.to,
            launched,
            ..self.flight
        })
    }

    /// Commit a confirmed separation: BOOSTING moves to GLIDING, held or not. Plan B's driver advances a held
    /// BOOSTING too, and the hold stays (drivers/separation.py `_apply`).
    ///
    /// Returns: the commit; None, with nothing committed, in any other stage.
    pub fn separate(&mut self) -> Option<Commit> {
        (self.flight.stage == Stage::Boosting).then(|| {
            self.commit(Flight {
                stage: Stage::Gliding,
                ..self.flight
            })
        })
    }

    /// Apply an operator request by [`Flight::apply`]'s rules.
    ///
    /// Returns: the commit, or why an arm was refused (nothing committed then).
    pub fn apply(&mut self, request: Request) -> Result<Commit, StageProblem> {
        let after = self.flight.apply(request)?;
        Ok(self.commit(after))
    }

    /// Commit a warm start's restored flight, before any thread runs: the crumb's airborne stage and arming, no hold,
    /// and `launched`, since the board was in the air when it reset, so the link stays down (plan B's warmstart
    /// `set_stage`, then `arm()` when the crumb was armed). The stage machine then sees the stage as moved from
    /// outside, and re-bases its backstop on it.
    ///
    /// Args: `stage` the crumb's stage; `armed` the crumb's arming.
    ///
    /// Returns: the commit, for BOOSTING through LANDING; None, with nothing committed, for SETTING or DONE, which
    /// are never a warm start (a crumb reading DONE starts a new session in SETTING).
    pub fn restore(&mut self, stage: Stage, armed: bool) -> Option<Commit> {
        stage.airborne().then(|| {
            self.commit(Flight {
                stage,
                manual: false,
                armed,
                launched: true,
            })
        })
    }

    /// Store `after` in the word and report the change.
    fn commit(&mut self, after: Flight) -> Commit {
        let before = self.flight;
        self.word.word.store(after.pack(), Ordering::Release);
        self.flight = after;
        Commit { before, after }
    }
}

/// One committed change of the flight word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Commit {
    /// The flight before.
    pub before: Flight,
    /// The flight committed.
    pub after: Flight,
}

impl Commit {
    /// How this commit moves the recorder's flight flag, None when it stays: the flag follows `airborne()` of each
    /// committed stage, an operator hold's too (a bench hold makes flight blocks, which costs nothing).
    ///
    /// The flag marks the ring blocks newest wins gives up last, so it brackets the commit's sequencer.csv rows:
    /// [`FlightFlag::Raise`] goes before them, so the BOOSTING row lies in a flight block, and
    /// [`FlightFlag::Lower`] after them, so the DONE row does too. The recorder's writer applies a fall only once
    /// everything queued before it is programmed.
    pub fn flight_flag(&self) -> Option<FlightFlag> {
        match (self.before.stage.airborne(), self.after.stage.airborne()) {
            (false, true) => Some(FlightFlag::Raise),
            (true, false) => Some(FlightFlag::Lower),
            _ => None,
        }
    }
}

/// A change of the recorder's flight flag ([`Commit::flight_flag`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlightFlag {
    /// Raise it (`Recorder::set_flight(true)`) before queuing the commit's rows.
    Raise,
    /// Lower it (`Recorder::set_flight(false)`) after queuing the commit's rows.
    Lower,
}

#[cfg(test)]
mod tests {
    use super::{ChangeRefused, Commit, Flight, FlightFlag, FlightWord, Request, Stage, StageProblem};
    use crate::sequencer::{Reason, Transition};

    #[test]
    fn names_round_trip_and_ids_match_plan_b() {
        for (index, stage) in Stage::ALL.into_iter().enumerate() {
            assert_eq!(stage as u8, index as u8 + 1);
            assert_eq!(Stage::from_name(stage.name()), Some(stage));
            assert_eq!(Stage::from_id(stage as u8), Some(stage));
        }
    }

    #[test]
    fn rejects_the_sentinel_and_unknown_names() {
        assert_eq!(Stage::from_id(0), None);
        assert_eq!(Stage::from_id(6), None);
        assert_eq!(Stage::from_name("auto"), None);
        assert_eq!(Stage::from_name("Setting"), None);
    }

    #[test]
    fn only_setting_and_done_are_ground() {
        let ground: [bool; 5] = Stage::ALL.map(Stage::ground);
        assert_eq!(ground, [true, false, false, false, true]);
    }

    #[test]
    fn airborne_is_boosting_through_landing() {
        let airborne: [bool; 5] = Stage::ALL.map(Stage::airborne);
        assert_eq!(airborne, [false, true, true, true, false]);
    }

    #[test]
    fn changes_run_in_setting_and_a_rezero_at_done_too() {
        assert_eq!(Stage::Setting.change_refused(false), None);
        assert_eq!(Stage::Setting.change_refused(true), None);
        assert_eq!(Stage::Done.change_refused(true), None, "the false launch's recovery");
    }

    #[test]
    fn changes_are_refused_off_the_pad_with_the_stage_named() {
        for stage in [Stage::Boosting, Stage::Gliding, Stage::Landing] {
            for rezero in [false, true] {
                let refusal = stage.change_refused(rezero);
                assert_eq!(refusal, Some(ChangeRefused(stage)), "{stage:?}, rezero {rezero}");
                assert_eq!(
                    refusal.map(|refusal| refusal.to_string()),
                    Some(format!(
                        "refused -- stage is {} (changes only in setting)",
                        stage.name()
                    )),
                    "{stage:?}, rezero {rezero}"
                );
            }
        }
        assert_eq!(
            Stage::Done
                .change_refused(false)
                .map(|refusal| refusal.to_string())
                .as_deref(),
            Some("refused -- stage is done (changes only in setting)"),
            "at DONE only the re-zero runs"
        );
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

    /// The machine's own launch, as its step reports it.
    fn launch() -> Transition {
        Transition {
            to: Stage::Boosting,
            reason: Reason::LaunchAcceleration {
                magnitude_g: 8.0,
                dwell_ms: 100,
            },
        }
    }

    /// One of the machine's own advances past the launch.
    fn advance(to: Stage) -> Transition {
        Transition {
            to,
            reason: Reason::FlightTimeout,
        }
    }

    #[test]
    fn the_word_packs_every_flight_and_unpacks_it_back() {
        let flights = every_flight();
        let mut words: Vec<u32> = flights.iter().map(|flight| flight.pack()).collect();
        for (flight, word) in flights.iter().zip(&words) {
            assert_eq!(Flight::unpack(*word), Some(*flight));
            assert!(*word < 64, "a flight packs into bits 0-5: {word:#x}");
        }
        words.sort_unstable();
        words.dedup();
        assert_eq!(words.len(), 40, "every flight packs to its own word");
        assert_eq!(Flight::GROUND.pack(), Stage::Setting as u32);
        let launched_held = Flight {
            stage: Stage::Gliding,
            manual: true,
            armed: false,
            launched: true,
        };
        assert_eq!(launched_held.pack(), 3 | 1 << 3 | 1 << 5);
    }

    #[test]
    fn a_word_that_is_no_flight_is_refused() {
        // the warm start's sentinel, the ids past DONE, and a bit past `launched`
        for word in [0, 6, 7, 0b1000, 1 | 1 << 6, 1 | 1 << 31, u32::MAX] {
            assert_eq!(Flight::unpack(word), None, "{word:#x}");
        }
    }

    #[test]
    fn a_hold_forces_the_stage_and_holds_all_but_setting() {
        // from every flight to every stage, its own included: the arming stays, and the launch is always cleared
        for flight in every_flight() {
            for stage in Stage::ALL {
                let held = flight.apply(Request::Hold(stage)).unwrap();
                let expected = Flight {
                    stage,
                    manual: stage != Stage::Setting,
                    armed: flight.armed,
                    launched: false,
                };
                assert_eq!(held, expected, "{flight:?} held at {stage:?}");
            }
        }
        // `stage setting` from a held stage is a return to the ground, not a hold
        let held = Flight::GROUND.apply(Request::Hold(Stage::Gliding)).unwrap();
        assert_eq!(held.apply(Request::Hold(Stage::Setting)), Ok(Flight::GROUND));
    }

    #[test]
    fn resume_clears_only_the_hold() {
        for flight in every_flight() {
            let resumed = flight.apply(Request::Resume).unwrap();
            assert_eq!(
                resumed,
                Flight {
                    manual: false,
                    ..flight
                }
            );
        }
    }

    #[test]
    fn disarm_always_clears_the_arming() {
        for flight in every_flight() {
            assert_eq!(flight.apply(Request::Disarm), Ok(Flight { armed: false, ..flight }));
        }
    }

    #[test]
    fn arm_passes_only_in_setting_without_a_hold() {
        assert_eq!(
            Flight::GROUND.apply(Request::Arm),
            Ok(Flight {
                armed: true,
                ..Flight::GROUND
            })
        );
        for flight in every_flight() {
            let expected = if flight.manual {
                Err(StageProblem::Held(flight.stage))
            } else if flight.stage != Stage::Setting {
                Err(StageProblem::NotSetting(flight.stage))
            } else {
                Ok(Flight { armed: true, ..flight })
            };
            assert_eq!(flight.apply(Request::Arm), expected, "{flight:?}");
        }
    }

    #[test]
    fn the_stage_problems_read_as_plan_b_writes_them() {
        let held = Flight::GROUND.apply(Request::Hold(Stage::Gliding)).unwrap();
        assert_eq!(
            held.stage_problem().map(|problem| problem.to_string()).as_deref(),
            Some("stage held at gliding by an operator command -- `stage setting` to return to the ground")
        );
        let landed = Flight {
            stage: Stage::Done,
            ..Flight::GROUND
        };
        assert_eq!(
            landed.stage_problem().map(|problem| problem.to_string()).as_deref(),
            Some("stage is done, not setting -- `stage setting`, then power-cycle before flight")
        );
        assert_eq!(Flight::GROUND.stage_problem(), None);
    }

    #[test]
    fn the_word_has_one_writer() {
        let word = FlightWord::new();
        {
            let writer = word.writer();
            assert!(writer.is_some());
            assert!(word.writer().is_none(), "a second writer while the first lives");
        }
        assert!(word.writer().is_none(), "nor once it is gone");
    }

    #[test]
    fn every_reader_loads_what_the_writer_committed() {
        let word = FlightWord::new();
        assert_eq!(word.load(), Flight::GROUND);
        let mut writer = word.writer().unwrap();
        let commit = writer.apply(Request::Hold(Stage::Landing)).unwrap();
        assert_eq!(commit.before, Flight::GROUND);
        assert_eq!(word.load(), commit.after);
        assert_eq!(writer.flight(), commit.after);
        assert_eq!((commit.after.stage, commit.after.manual), (Stage::Landing, true));
        // a refused arm commits nothing
        assert_eq!(writer.apply(Request::Arm), Err(StageProblem::Held(Stage::Landing)));
        assert_eq!(word.load(), commit.after);
    }

    #[test]
    fn only_the_machines_own_launch_sets_launched() {
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        writer.apply(Request::Arm).unwrap();
        let commit = writer.advance(launch());
        assert!(commit.after.launched && commit.after.armed);
        // the rest of the flight keeps it: the separation, the landing, DONE
        assert!(writer.separate().unwrap().after.launched);
        assert!(writer.advance(advance(Stage::Landing)).after.launched);
        assert!(writer.advance(advance(Stage::Done)).after.launched);
        // a disarm does not end it
        assert!(writer.apply(Request::Disarm).unwrap().after.launched);
    }

    #[test]
    fn an_operator_boosting_is_never_a_launch() {
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        // `stage boosting`, `stage auto`: the machine then records the move as external, which commits nothing
        assert!(!writer.apply(Request::Hold(Stage::Boosting)).unwrap().after.launched);
        assert!(!writer.apply(Request::Resume).unwrap().after.launched);
        // the separation and the machine's own later advances do not make it one either
        assert!(!writer.separate().unwrap().after.launched);
        assert!(!writer.advance(advance(Stage::Landing)).after.launched);
        assert!(!word.load().launched);
    }

    #[test]
    fn the_link_reconnects_until_the_machines_launch_and_after_a_hold() {
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        assert!(writer.flight().reconnect_allowed(), "on the pad");
        writer.apply(Request::Arm).unwrap();
        assert!(writer.flight().reconnect_allowed(), "armed on the pad");
        for commit in [
            writer.advance(launch()),
            writer.separate().unwrap(),
            writer.advance(advance(Stage::Landing)),
            writer.advance(advance(Stage::Done)),
            writer.apply(Request::Disarm).unwrap(),
        ] {
            assert!(!commit.after.reconnect_allowed(), "{:?}", commit.after);
        }
        // landed, then `stage done` from the console, the false-launch recovery's first step: a hold at the stage
        // the board is already in is still no launch
        assert!(
            writer
                .apply(Request::Hold(Stage::Done))
                .unwrap()
                .after
                .reconnect_allowed()
        );
        assert!(writer.apply(Request::Resume).unwrap().after.reconnect_allowed());
        // an unarmed launch ends the reconnects too: they key on the detected launch alone
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        assert!(!writer.advance(launch()).after.reconnect_allowed());
        assert!(
            writer
                .apply(Request::Hold(Stage::Setting))
                .unwrap()
                .after
                .reconnect_allowed()
        );
    }

    #[test]
    fn a_separation_moves_only_boosting_and_keeps_the_hold() {
        for stage in [Stage::Setting, Stage::Gliding, Stage::Landing, Stage::Done] {
            // the operator's way there, unheld: `stage <name>`, then `stage auto`
            let word = FlightWord::new();
            let mut writer = word.writer().unwrap();
            writer.apply(Request::Hold(stage)).unwrap();
            writer.apply(Request::Resume).unwrap();
            assert_eq!(writer.separate(), None, "{stage:?}");
            assert_eq!(
                word.load(),
                Flight {
                    stage,
                    ..Flight::GROUND
                }
            );
        }
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        writer.apply(Request::Hold(Stage::Boosting)).unwrap();
        let commit = writer.separate().unwrap();
        assert_eq!((commit.after.stage, commit.after.manual), (Stage::Gliding, true));
    }

    #[test]
    fn a_restore_commits_the_crumbs_airborne_flight_as_launched() {
        for stage in [Stage::Boosting, Stage::Gliding, Stage::Landing] {
            for armed in [false, true] {
                let word = FlightWord::new();
                let mut writer = word.writer().unwrap();
                let commit = writer.restore(stage, armed).unwrap();
                let restored = Flight {
                    stage,
                    manual: false,
                    armed,
                    launched: true,
                };
                assert_eq!(
                    commit,
                    Commit {
                        before: Flight::GROUND,
                        after: restored
                    }
                );
                assert_eq!(Flight::unpack(restored.pack()), Some(restored));
                assert_eq!((word.load(), writer.flight()), (restored, restored));
                assert_eq!(commit.flight_flag(), Some(FlightFlag::Raise), "{stage:?}");
                assert!(
                    !word.load().reconnect_allowed(),
                    "the link stays down after a warm start"
                );
            }
        }
    }

    #[test]
    fn a_restored_flight_flies_on() {
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        writer.restore(Stage::Boosting, true).unwrap();
        let separated = writer.separate().unwrap().after;
        assert_eq!(
            separated,
            Flight {
                stage: Stage::Gliding,
                manual: false,
                armed: true,
                launched: true
            }
        );
        let landed = writer.advance(advance(Stage::Landing));
        assert_eq!(
            landed.after,
            Flight {
                stage: Stage::Landing,
                ..separated
            }
        );
        let done = writer.advance(advance(Stage::Done));
        assert_eq!(done.flight_flag(), Some(FlightFlag::Lower));
        assert!(done.after.launched);
        // an operator hold after the restore is no launch: it clears launched, the arming stays
        let held = writer.apply(Request::Hold(Stage::Gliding)).unwrap().after;
        assert_eq!(
            held,
            Flight {
                stage: Stage::Gliding,
                manual: true,
                armed: true,
                launched: false
            }
        );
        assert_eq!(word.load(), held);
    }

    #[test]
    fn a_restore_commits_the_crumbs_flight_whatever_the_word_held() {
        // armed, then held at GLIDING: a restore to a disarmed LANDING takes the crumb's arming and no hold
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        writer.apply(Request::Arm).unwrap();
        let held = writer.apply(Request::Hold(Stage::Gliding)).unwrap().after;
        let commit = writer.restore(Stage::Landing, false).unwrap();
        let restored = Flight {
            stage: Stage::Landing,
            manual: false,
            armed: false,
            launched: true,
        };
        assert_eq!(
            commit,
            Commit {
                before: held,
                after: restored
            }
        );
        assert_eq!(word.load(), restored);
    }

    #[test]
    fn a_restore_on_the_ground_commits_nothing() {
        for stage in [Stage::Setting, Stage::Done] {
            for armed in [false, true] {
                let word = FlightWord::new();
                let mut writer = word.writer().unwrap();
                assert_eq!(writer.restore(stage, armed), None, "{stage:?} armed {armed}");
                assert_eq!((word.load(), writer.flight()), (Flight::GROUND, Flight::GROUND));
            }
        }
        // the restore goes through the one writer: none is handed out beside it, before or after
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        assert!(word.writer().is_none());
        writer.restore(Stage::Gliding, false).unwrap();
        assert!(word.writer().is_none());
    }

    /// The flight flag's change for a commit from stage `from` to stage `to`.
    fn moving(from: Stage, to: Stage) -> Option<FlightFlag> {
        let before = Flight {
            stage: from,
            ..Flight::GROUND
        };
        Commit {
            before,
            after: Flight { stage: to, ..before },
        }
        .flight_flag()
    }

    #[test]
    fn the_flight_flag_rises_into_flight_and_falls_back_to_the_ground() {
        assert_eq!(moving(Stage::Setting, Stage::Boosting), Some(FlightFlag::Raise));
        assert_eq!(moving(Stage::Landing, Stage::Done), Some(FlightFlag::Lower));
        // a hold counts: the bench's `stage gliding` and back
        assert_eq!(moving(Stage::Setting, Stage::Gliding), Some(FlightFlag::Raise));
        assert_eq!(moving(Stage::Gliding, Stage::Setting), Some(FlightFlag::Lower));
        assert_eq!(moving(Stage::Gliding, Stage::Done), Some(FlightFlag::Lower));
        // inside the flight and on the ground, it stays
        assert_eq!(moving(Stage::Boosting, Stage::Gliding), None);
        assert_eq!(moving(Stage::Gliding, Stage::Landing), None);
        assert_eq!(moving(Stage::Done, Stage::Setting), None);
        assert_eq!(moving(Stage::Setting, Stage::Setting), None);
        // an arming changes no stage
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        assert_eq!(writer.apply(Request::Arm).unwrap().flight_flag(), None);
        assert_eq!(writer.advance(launch()).flight_flag(), Some(FlightFlag::Raise));
    }
}
