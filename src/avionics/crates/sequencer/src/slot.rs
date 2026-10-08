//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The request slot: how a thread asks the control thread to apply an operator [`Request`] (`stage`, `arm`,
//! `disarm`) to the flight word, with no lock, no allocation and no kernel call on the control thread's side.
//!
//! Plan B needed none: its one asyncio loop called the Controller directly. In Rust the flight word has one writer,
//! the control thread's ([`FlightWriter`]), so a requesting thread (the CC executor) posts its request into its own
//! slot and polls for the answer, and the control thread serves the slot once a tick ([`RequestSlot::serve`]).
//! Nothing notifies the control thread: its task notification counts the timer's ticks.
//!
//! One `AtomicU32` per requesting thread: `sequence` (bits 16-31) | `phase` (14-15) | `kind` (8-13) | `value`
//! (0-7). A request goes idle -> posted (the requester) -> taken (the control thread claims it) -> done (the answer
//! in `value`). A requester that gives up withdraws its request, posted -> idle, and one compare-and-swap decides
//! that race with the claim: a withdrawal that wins means the request was never applied; one that loses means it
//! was taken, and its answer follows within microseconds. The sequence tells a request's answer from an earlier
//! one's.
//!
//! Every operation is SeqCst: a request costs a handful of them, so the cost is nil, and no ordering has to be
//! argued.

use std::sync::atomic::AtomicU32;
use std::sync::atomic::Ordering::SeqCst;

use crate::controller::{Commit, FLIGHT_BITS, Flight, FlightWriter, Request, Stage, StageProblem};

/// What the control thread answers a request: the flight it committed, or why it refused an arm.
pub type Answer = Result<Flight, StageProblem>;

/// Where the sequence sits: bits 16-31.
const SEQUENCE_SHIFT: u32 = 16;
/// Where the phase sits: bits 14-15.
const PHASE_SHIFT: u32 = 14;
/// The phase's 2 bits, shifted down.
const PHASE_BITS: u32 = 0b11;
/// Where the kind sits: bits 8-13.
const KIND_SHIFT: u32 = 8;
/// The kind's 6 bits, shifted down.
const KIND_BITS: u32 = 0b11_1111;
/// The value: bits 0-7.
const VALUE_BITS: u32 = 0xFF;
/// In an answer's value: the request was refused, and the bits under it are the stage problem, as a flight.
const REFUSED: u32 = 1 << 7;

const _: () = assert!(FLIGHT_BITS < REFUSED, "an answer's flight fits under the refused bit");

/// The kind of a [`Request::Hold`]; its value is the stage id.
const HOLD: u32 = 1;
/// The kind of a [`Request::Resume`].
const RESUME: u32 = 2;
/// The kind of a [`Request::Arm`].
const ARM: u32 = 3;
/// The kind of a [`Request::Disarm`].
const DISARM: u32 = 4;

/// Where a request is, which says who acts on the slot next.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    /// Nothing waits: the requester may post.
    Idle = 0,
    /// A request waits for the control thread, and the requester may still withdraw it.
    Posted = 1,
    /// The control thread is applying it.
    Taken = 2,
    /// Applied, or refused: the answer waits for the requester, who may post again.
    Done = 3,
}

/// A slot word from its four fields.
fn slot_word(sequence: u16, phase: Phase, kind: u32, value: u32) -> u32 {
    (u32::from(sequence) << SEQUENCE_SHIFT) | ((phase as u32) << PHASE_SHIFT) | (kind << KIND_SHIFT) | value
}

/// A slot word's sequence.
fn sequence(word: u32) -> u16 {
    (word >> SEQUENCE_SHIFT) as u16
}

/// A slot word's phase.
fn phase(word: u32) -> Phase {
    match (word >> PHASE_SHIFT) & PHASE_BITS {
        0 => Phase::Idle,
        1 => Phase::Posted,
        2 => Phase::Taken,
        _ => Phase::Done,
    }
}

/// `word` moved to `phase`, its other fields kept.
fn with_phase(word: u32, phase: Phase) -> u32 {
    (word & !(PHASE_BITS << PHASE_SHIFT)) | ((phase as u32) << PHASE_SHIFT)
}

/// A request as the slot's kind and value.
fn encode_request(request: Request) -> (u32, u32) {
    match request {
        Request::Hold(stage) => (HOLD, stage as u32),
        Request::Resume => (RESUME, 0),
        Request::Arm => (ARM, 0),
        Request::Disarm => (DISARM, 0),
    }
}

/// The request a posted word carries, None for a kind or a stage id no request has (only [`encode_request`]
/// writes them, so never).
fn decode_request(word: u32) -> Option<Request> {
    let value = word & VALUE_BITS;
    match (word >> KIND_SHIFT) & KIND_BITS {
        HOLD => Stage::from_id(value as u8).map(Request::Hold),
        RESUME => Some(Request::Resume),
        ARM => Some(Request::Arm),
        DISARM => Some(Request::Disarm),
        _ => None,
    }
}

/// An answer as the slot's value: the committed flight, or [`REFUSED`] over the stage and hold refused.
fn encode_answer(answer: Answer) -> u32 {
    match answer {
        Ok(flight) => flight.pack(),
        Err(StageProblem::Held(stage)) => {
            REFUSED
                | Flight {
                    stage,
                    manual: true,
                    ..Flight::GROUND
                }
                .pack()
        }
        Err(StageProblem::NotSetting(stage)) => {
            REFUSED
                | Flight {
                    stage,
                    ..Flight::GROUND
                }
                .pack()
        }
    }
}

/// The answer a done word's value carries (only [`encode_answer`] writes it, so always Some).
fn decode_answer(value: u32) -> Option<Answer> {
    let flight = Flight::unpack(value & !REFUSED)?;
    if value & REFUSED == 0 {
        return Some(Ok(flight));
    }
    flight.stage_problem().map(Err)
}

/// A posted request's receipt: what its answer and its withdrawal are matched by.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ticket {
    /// The sequence the request was posted with.
    sequence: u16,
}

/// One requesting thread's slot (see the module notes): that thread posts, polls and withdraws; the control
/// thread serves. A second requesting thread gets a slot of its own.
#[derive(Debug, Default)]
pub struct RequestSlot {
    /// `sequence` | `phase` | `kind` | `value`.
    word: AtomicU32,
}

impl RequestSlot {
    /// An idle slot. `const`, so the firmware's slot is a static.
    pub const fn new() -> RequestSlot {
        RequestSlot {
            word: AtomicU32::new(0),
        }
    }

    /// Post `request` (the requesting thread). Its sequence is the last one's plus one, wrapping.
    ///
    /// Returns: its ticket; None, with nothing posted, while the previous request is still in the slot (posted,
    /// or taken and not answered yet): that one must be answered or withdrawn first.
    pub fn post(&self, request: Request) -> Option<Ticket> {
        let current = self.word.load(SeqCst);
        if matches!(phase(current), Phase::Posted | Phase::Taken) {
            return None;
        }
        let ticket = Ticket {
            sequence: sequence(current).wrapping_add(1),
        };
        let (kind, value) = encode_request(request);
        let posted = slot_word(ticket.sequence, Phase::Posted, kind, value);
        // the control thread leaves an idle or done slot alone, so only a second requester could fail this
        self.word.compare_exchange(current, posted, SeqCst, SeqCst).ok()?;
        Some(ticket)
    }

    /// The answer to `ticket`'s request (the requesting thread, polling).
    ///
    /// Returns: the answer once it is there, as often as asked; None while the request waits (posted or taken),
    /// and for a stale ticket (the slot holds a later request, or its own was withdrawn).
    pub fn answer(&self, ticket: Ticket) -> Option<Answer> {
        let current = self.word.load(SeqCst);
        if sequence(current) != ticket.sequence || phase(current) != Phase::Done {
            return None;
        }
        decode_answer(current & VALUE_BITS)
    }

    /// Withdraw `ticket`'s request before the control thread takes it (the requesting thread, at its deadline).
    ///
    /// Returns: true when it is withdrawn: never applied, and it never will be. False when the control thread took
    /// it first: it is applied (or refused), and its answer is there or follows within microseconds, so keep
    /// polling [`RequestSlot::answer`]. A stale ticket gets false too, and the later request is left alone.
    pub fn withdraw(&self, ticket: Ticket) -> bool {
        let current = self.word.load(SeqCst);
        if sequence(current) != ticket.sequence || phase(current) != Phase::Posted {
            return false;
        }
        let idle = slot_word(ticket.sequence, Phase::Idle, 0, 0);
        self.word.compare_exchange(current, idle, SeqCst, SeqCst).is_ok()
    }

    /// Serve the waiting request, if any (the control thread, once a tick): claim it, apply it through `writer`,
    /// and leave the answer in the slot.
    ///
    /// Returns: None when nothing waits, or the requester withdrew it first; else the writer's commit, or why it
    /// refused an arm.
    pub fn serve(&self, writer: &mut FlightWriter<'_>) -> Option<Result<Commit, StageProblem>> {
        let current = self.word.load(SeqCst);
        if phase(current) != Phase::Posted {
            return None;
        }
        let request = decode_request(current)?;
        let taken = with_phase(current, Phase::Taken);
        self.word.compare_exchange(current, taken, SeqCst, SeqCst).ok()?;
        let result = writer.apply(request);
        let answer = encode_answer(result.map(|commit| commit.after));
        // nobody else writes a taken slot: the requester neither withdraws nor posts over it
        self.word
            .store((with_phase(taken, Phase::Done) & !VALUE_BITS) | answer, SeqCst);
        Some(result)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::AtomicBool;
    use std::sync::atomic::Ordering::SeqCst;
    use std::time::{Duration, Instant};

    use super::{HOLD, Phase, RequestSlot, decode_answer, encode_answer, slot_word};
    use crate::controller::{Flight, FlightWord, Request, Stage, StageProblem};

    /// Every request there is.
    fn every_request() -> Vec<Request> {
        let mut requests: Vec<Request> = Stage::ALL.into_iter().map(Request::Hold).collect();
        requests.extend([Request::Resume, Request::Arm, Request::Disarm]);
        requests
    }

    #[test]
    fn every_request_and_answer_crosses_the_slot_unchanged() {
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        let slot = RequestSlot::new();
        for request in every_request() {
            let ticket = slot.post(request).unwrap();
            let expected = writer.flight().apply(request);
            let served = slot.serve(&mut writer).unwrap();
            assert_eq!(served.map(|commit| commit.after), expected, "{request:?}");
            assert_eq!(slot.answer(ticket), Some(expected), "{request:?}");
        }
        for stage in Stage::ALL {
            for problem in [StageProblem::Held(stage), StageProblem::NotSetting(stage)] {
                if problem == StageProblem::NotSetting(Stage::Setting) {
                    continue; // SETTING without a hold is no problem
                }
                assert_eq!(decode_answer(encode_answer(Err(problem))), Some(Err(problem)));
            }
        }
    }

    #[test]
    fn a_request_is_applied_once_and_answered_until_the_next() {
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        let slot = RequestSlot::new();
        assert!(slot.serve(&mut writer).is_none(), "an idle slot serves nothing");
        let ticket = slot.post(Request::Hold(Stage::Gliding)).unwrap();
        assert_eq!(slot.answer(ticket), None, "posted: no answer yet");
        let commit = slot.serve(&mut writer).unwrap().unwrap();
        assert_eq!(word.load(), commit.after);
        assert!(slot.serve(&mut writer).is_none(), "served once");
        for _ in 0..2 {
            assert_eq!(slot.answer(ticket), Some(Ok(commit.after)));
        }
        assert!(!slot.withdraw(ticket), "an answered request is not withdrawn");
    }

    #[test]
    fn an_arm_refused_at_apply_answers_the_stage_problem() {
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        let slot = RequestSlot::new();
        writer.apply(Request::Hold(Stage::Landing)).unwrap();
        let ticket = slot.post(Request::Arm).unwrap();
        assert_eq!(slot.serve(&mut writer), Some(Err(StageProblem::Held(Stage::Landing))));
        let answer = slot.answer(ticket).unwrap();
        assert_eq!(answer, Err(StageProblem::Held(Stage::Landing)));
        assert!(!word.load().armed);
        // released at DONE: refused all the same, for the other reason
        writer.apply(Request::Hold(Stage::Done)).unwrap();
        writer.apply(Request::Resume).unwrap();
        let ticket = slot.post(Request::Arm).unwrap();
        slot.serve(&mut writer);
        assert_eq!(
            slot.answer(ticket).unwrap().unwrap_err().to_string(),
            "stage is done, not setting -- `stage setting`, then power-cycle before flight"
        );
    }

    #[test]
    fn a_withdrawn_request_is_never_applied() {
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        let slot = RequestSlot::new();
        let ticket = slot.post(Request::Hold(Stage::Done)).unwrap();
        assert!(slot.withdraw(ticket));
        assert!(slot.serve(&mut writer).is_none());
        assert_eq!(slot.answer(ticket), None);
        assert_eq!(word.load(), Flight::GROUND);
        assert!(!slot.withdraw(ticket), "withdrawn once");
        // the slot is free again, and the next request goes through
        let next = slot.post(Request::Arm).unwrap();
        assert_ne!(next, ticket);
        assert!(slot.serve(&mut writer).unwrap().is_ok());
        assert!(slot.answer(next).unwrap().unwrap().armed);
    }

    #[test]
    fn a_taken_request_is_answered_not_withdrawn() {
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        let slot = RequestSlot::new();
        let ticket = slot.post(Request::Arm).unwrap();
        slot.serve(&mut writer).unwrap().unwrap();
        assert!(!slot.withdraw(ticket));
        assert!(slot.answer(ticket).unwrap().unwrap().armed);
    }

    #[test]
    fn a_stale_ticket_is_ignored() {
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        let slot = RequestSlot::new();
        let old = slot.post(Request::Hold(Stage::Gliding)).unwrap();
        slot.serve(&mut writer);
        assert!(slot.answer(old).is_some());
        let new = slot.post(Request::Resume).unwrap();
        assert_eq!(slot.answer(old), None, "the slot holds a later request");
        assert!(
            !slot.withdraw(old),
            "a stale withdrawal leaves the later request posted"
        );
        let commit = slot.serve(&mut writer).unwrap().unwrap();
        assert!(!commit.after.manual, "the later request applied");
        assert_eq!(slot.answer(new), Some(Ok(commit.after)));
        assert_eq!(slot.answer(old), None);
    }

    #[test]
    fn a_second_post_waits_for_the_first() {
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        let slot = RequestSlot::new();
        let first = slot.post(Request::Hold(Stage::Gliding)).unwrap();
        assert_eq!(slot.post(Request::Disarm), None, "posted");
        // the moment the control thread has taken it and not yet answered
        let posted = slot.word.load(SeqCst);
        slot.word.store(
            slot_word(first.sequence, Phase::Taken, HOLD, Stage::Gliding as u32),
            SeqCst,
        );
        assert_eq!(slot.post(Request::Disarm), None, "taken");
        slot.word.store(posted, SeqCst);
        slot.serve(&mut writer);
        assert!(slot.answer(first).is_some());
        assert!(slot.post(Request::Disarm).is_some(), "answered: the slot is free");
    }

    #[test]
    fn the_sequence_wraps() {
        let word = FlightWord::new();
        let mut writer = word.writer().unwrap();
        let slot = RequestSlot::new();
        slot.word.store(slot_word(u16::MAX, Phase::Done, 0, 0), SeqCst);
        let ticket = slot.post(Request::Arm).unwrap();
        assert_eq!(ticket.sequence, 0);
        slot.serve(&mut writer);
        assert!(slot.answer(ticket).unwrap().unwrap().armed);
    }

    /// The protocol's logic under a real race: a requester posts a hold, waits a varying moment and withdraws,
    /// while the control thread serves in a loop. Each request must be applied once (the withdrawal lost: its
    /// answer comes) or never (the withdrawal won), and the control thread's applications must be exactly the
    /// answered ones, in order. 10^5 rounds also wrap the 16-bit sequence.
    ///
    /// This covers the logic, not the memory orderings: the host is x86, whose TSO hides any reordering the
    /// weaker RISC-V model allows. That is why every slot operation is SeqCst rather than argued case by case.
    ///
    /// A lost answer fails the test instead of hanging it: the requester gives a taken request
    /// `ANSWER_DEADLINE`, and stops the control thread whatever it found.
    #[test]
    fn a_withdrawal_racing_the_claim_applies_each_request_once_or_never() {
        const ROUNDS: usize = 100_000;
        /// How long a taken request may stay unanswered before the test calls it lost.
        const ANSWER_DEADLINE: Duration = Duration::from_secs(10);
        let word = FlightWord::new();
        let slot = RequestSlot::new();
        let stop = AtomicBool::new(false);
        // the requester's side: the answered stages and the count withdrawn, or what went wrong
        let requester = || -> Result<(Vec<Stage>, usize), String> {
            let mut answered = Vec::new();
            let mut withdrawn = 0;
            let mut random: u32 = 0x9E37_79B9; // xorshift: a varying delay, the same on every run
            for round in 0..ROUNDS {
                let stage = Stage::ALL[round % Stage::ALL.len()];
                let ticket = slot
                    .post(Request::Hold(stage))
                    .ok_or_else(|| format!("round {round}: the slot is not free"))?;
                random ^= random << 13;
                random ^= random >> 17;
                random ^= random << 5;
                for _ in 0..random % 8 {
                    std::hint::spin_loop();
                }
                if round % 64 == 0 {
                    std::thread::yield_now(); // on a single core, lets the control thread claim some
                }
                if slot.withdraw(ticket) {
                    withdrawn += 1;
                    continue;
                }
                let deadline = Instant::now() + ANSWER_DEADLINE;
                let answer = loop {
                    if let Some(answer) = slot.answer(ticket) {
                        break answer;
                    }
                    if Instant::now() > deadline {
                        return Err(format!("round {round}: taken, and never answered"));
                    }
                    std::hint::spin_loop();
                };
                if answer.map(|flight| flight.stage) != Ok(stage) {
                    return Err(format!("round {round}: a hold at {stage:?} answered {answer:?}"));
                }
                answered.push(stage);
            }
            Ok((answered, withdrawn))
        };
        let (served, outcome) = std::thread::scope(|scope| {
            let control = scope.spawn(|| {
                let mut writer = word.writer().unwrap();
                let mut served = Vec::new();
                while !stop.load(SeqCst) {
                    if let Some(result) = slot.serve(&mut writer) {
                        served.push(result.map(|commit| commit.after.stage));
                    }
                    std::hint::spin_loop();
                }
                served
            });
            let outcome = requester();
            stop.store(true, SeqCst);
            (control.join().unwrap(), outcome)
        });
        let (answered, withdrawn) = outcome.unwrap();
        assert_eq!(answered.len() + withdrawn, ROUNDS);
        let answered_results: Vec<Result<Stage, StageProblem>> = answered.iter().copied().map(Ok).collect();
        assert_eq!(
            served, answered_results,
            "applied exactly the answered requests, once each"
        );
        assert!(
            withdrawn > 0 && !answered.is_empty(),
            "both sides of the race were taken: {withdrawn} withdrawn, {} answered",
            answered.len()
        );
    }
}
