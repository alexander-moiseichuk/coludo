//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! One boot of the board on the test's thread: the ring_writer, the ring_eraser and the ring_saver over one
//! in-memory flash and a memory [`Card`], stepped on a 10 ms tick of the clock the test moves, as their three core-0
//! threads take turns on the board: the writer every tick, the eraser every tick (logging each newest-wins loss into
//! recorder.log, as the firmware's eraser thread does), the saver every 100 ms (again at once while it commits full
//! batches). A [`Flight`] queues a flight's rows and log lines between the ticks: a pad, a boost of 5 g that
//! `recorder_flight` finds, a glide, a landing, and DONE.

use flash_ring::header::Session;
use flash_ring::{BlockTable, Eraser, Erasure, Loss, MemoryFlash, Position, Ring, Scan};
use recorder::{
    BATCH_BLOCKS, Cell, LineRing, Recorder, Registry, SaveError, Saved, Saver, StreamId, TICK_US, Telemetry, Writer,
};

use super::card::Card;
use super::{EPOCH, Parts, TestClock};

/// The saver steps every this many ticks: 100 ms.
pub const SAVER_EVERY: u64 = 10;

/// A batch the saver committed, and when.
#[derive(Clone, Copy, Debug)]
pub struct Commit {
    /// What it put on the card.
    pub saved: Saved,
    /// The uptime it was committed at.
    pub at_us: u64,
}

/// One boot's three ring tasks. See the module notes.
pub struct Board<'a> {
    /// The boot's recorder.
    pub recorder: &'static Recorder,
    /// Its clock, which every tick moves.
    pub clock: TestClock,
    /// The ring_writer.
    pub writer: Writer<'static, 'a, &'a MemoryFlash>,
    /// The ring_eraser.
    pub eraser: Eraser<'a, &'a MemoryFlash>,
    /// The ring_saver, while a card is mounted.
    pub saver: Option<Saver<'static, 'a, &'a MemoryFlash, Card>>,
    /// The flash, for a saver mounted later.
    flash: &'a MemoryFlash,
    /// Whether the eraser steps.
    pub erasing: bool,
    /// The uptime of the last tick.
    pub now_us: u64,
    /// The wall clock the writer's session.csv rows get, Unix seconds (before 2001: unset).
    pub epoch: i64,
    /// Every unsaved block newest wins erased, as the eraser reported it.
    pub losses: Vec<Loss>,
    /// For each loss, the writer's block when its line was queued: the line lies there or in the next block.
    pub loss_blocks: Vec<u32>,
    /// Every block erased as saved: its sequence, and the uptime of the erase.
    pub erased: Vec<(u32, u64)>,
    /// Every batch the saver committed.
    pub commits: Vec<Commit>,
    /// The card errors the saver reported.
    pub card_errors: u32,
    /// Ticks so far.
    ticks: u64,
}

impl<'a> Board<'a> {
    /// Boot `boot`'s board over `flash` after `scanned`, at `parts`'s clock: the writer on the pad, the eraser
    /// trusting `hint` as saved, and with `card` mounted a saver from `hint` (`BlockTable::saver_start`).
    pub fn new(
        flash: &'a MemoryFlash,
        scanned: &'a Scan,
        parts: &Parts,
        boot: u32,
        card: Option<Card>,
        hint: Option<Position>,
    ) -> Board<'a> {
        let label = Session::new(parts.recorder.inspect().session).unwrap();
        let ring = Ring::new(flash, &scanned.table, scanned.next_sequence, boot, label, false).unwrap();
        let writer = Writer::new(parts.recorder, ring).unwrap();
        let mut board = Board {
            recorder: parts.recorder,
            clock: parts.clock.clone(),
            writer,
            eraser: Eraser::new(flash, &scanned.table, hint).unwrap(),
            saver: None,
            flash,
            erasing: true,
            now_us: parts.clock.now(),
            epoch: EPOCH,
            losses: Vec::new(),
            loss_blocks: Vec::new(),
            erased: Vec::new(),
            commits: Vec::new(),
            card_errors: 0,
            ticks: 0,
        };
        if let Some(card) = card {
            board.mount(card, hint);
        }
        board
    }

    /// The block table the three tasks share.
    pub fn table(&self) -> &'a BlockTable {
        self.writer.ring().table()
    }

    /// Mounts `card`: a saver from `hint` (None: a new card, from the oldest block in flash).
    pub fn mount(&mut self, card: Card, hint: Option<Position>) {
        let start = self.table().saver_start(hint);
        self.saver = Some(Saver::new(self.recorder, self.flash, self.table(), card, start));
    }

    /// Mounts `card` as [`Board::mount`] does, the saver handing every line it stages to `queue` too (M19).
    #[allow(dead_code, reason = "the saver test's alone among the binaries sharing this module")]
    pub fn mount_with_uart(&mut self, card: Card, hint: Option<Position>, queue: &'static LineRing) {
        let start = self.table().saver_start(hint);
        self.saver = Some(Saver::new(self.recorder, self.flash, self.table(), card, start).with_uart(queue));
    }

    /// One 10 ms tick: the writer's step, the eraser's, and every [`SAVER_EVERY`] ticks the saver's.
    pub fn tick(&mut self) {
        self.now_us += TICK_US;
        self.ticks += 1;
        self.clock.set(self.now_us);
        self.writer.step(self.now_us, self.epoch);
        let now_ms = (self.now_us / 1000) as u32;
        if self.erasing {
            match self.eraser.erase_step(self.recorder.saver_position(), now_ms) {
                Ok(Erasure::Lost(loss)) => {
                    assert!(self.recorder.log("ring", &loss.to_string()), "a loss line is queued");
                    self.losses.push(loss);
                    self.loss_blocks.push(self.writer.ring().position().block_sequence);
                }
                Ok(Erasure::Saved { block_sequence, .. }) => self.erased.push((block_sequence, self.now_us)),
                Ok(Erasure::Foreign { .. } | Erasure::Idle) => {}
                Err(error) => panic!("an erase failed: {error:?}"),
            }
        }
        if self.ticks.is_multiple_of(SAVER_EVERY) {
            self.save(now_ms);
        }
    }

    /// The saver's steps at `now_ms`: again at once while it commits full batches.
    fn save(&mut self, now_ms: u32) {
        let Some(saver) = &mut self.saver else {
            return;
        };
        loop {
            match saver.step(now_ms) {
                Ok(Some(saved)) => {
                    self.commits.push(Commit {
                        saved,
                        at_us: self.now_us,
                    });
                    if saved.blocks < BATCH_BLOCKS {
                        return;
                    }
                }
                Ok(None) => return,
                Err(SaveError::Card(_)) => {
                    self.card_errors += 1;
                    return;
                }
                Err(SaveError::Flash(error)) => panic!("a flash read failed: {error:?}"),
            }
        }
    }

    /// Ticks `ticks` times with nothing queued: the merge horizon, the status lines and the saver's last batches.
    pub fn settle(&mut self, ticks: u64) {
        for _ in 0..ticks {
            self.tick();
        }
    }
}

/// The streams of a flight.
#[derive(Clone, Copy, Debug)]
pub struct Streams {
    /// `imu_lsm6dso32.csv`, plan B's fields: where recorder_flight finds the boost.
    pub imu: StreamId,
    /// `health.csv`, plan B's fields, once a second.
    pub health: StreamId,
    /// `sequencer.csv`: the stage changes, durable.
    pub sequencer: StreamId,
}

/// The registry of a flight's streams.
pub fn flight_registry() -> (Registry, Streams) {
    let mut registry = Registry::new();
    let imu = ["ax", "ay", "az", "gx", "gy", "gz", "irq_runs"];
    let health = [
        "temp",
        "mem_free",
        "load",
        "oom_s",
        "land_s",
        "leak_kbps",
        "rescues",
        "rescue_ms",
    ];
    let streams = Streams {
        imu: registry.add("imu_lsm6dso32.csv", &imu, -1, false).unwrap(),
        health: registry.add("health.csv", &health, -1, false).unwrap(),
        sequencer: registry.add("sequencer.csv", &["stage", "reason"], -1, true).unwrap(),
    };
    (registry, streams)
}

/// A flight's timeline, in ticks of the boot: the pad until `boost`, 0.6 s of 5 g, the glide, the landing at
/// `land`, DONE at `done`, then the pad again.
#[derive(Clone, Copy, Debug)]
pub struct Plan {
    /// BOOSTING.
    pub boost: u64,
    /// LANDING.
    pub land: u64,
    /// DONE.
    pub done: u64,
    /// On the pad (before the boost and after DONE) an IMU row every this many ticks; every tick in flight.
    pub pad_imu_every: u64,
}

/// The ticks of the boost: 0.6 s of 5 g, twice recorder_flight's 0.3 s hold.
const BOOST_TICKS: u64 = 60;

/// A flight's producers, all on the test's thread (the `misc` ring), queueing each tick's rows and log lines.
pub struct Flight {
    /// The boot's recorder.
    recorder: &'static Recorder,
    /// The IMU.
    imu: Telemetry<'static>,
    /// The supervisor's health rows.
    health: Telemetry<'static>,
    /// The sequencer's stage changes.
    sequencer: Telemetry<'static>,
    /// The timeline.
    plan: Plan,
    /// Rows queued per stream, IMU, health, sequencer.
    pub rows: [u32; 3],
    /// Log lines queued.
    pub logs: u32,
}

impl Flight {
    /// The producers of `streams` on `recorder`, flying `plan`.
    pub fn new(recorder: &'static Recorder, streams: Streams, plan: Plan) -> Flight {
        Flight {
            recorder,
            imu: Telemetry::new(recorder, streams.imu).unwrap(),
            health: Telemetry::new(recorder, streams.health).unwrap(),
            sequencer: Telemetry::new(recorder, streams.sequencer).unwrap(),
            plan,
            rows: [0; 3],
            logs: 0,
        }
    }

    /// Queues tick `tick`'s rows and log lines on the board's next tick, then ticks it.
    pub fn fly(&mut self, board: &mut Board<'_>, tick: u64) {
        self.queue(board.now_us + TICK_US, tick);
        board.tick();
    }

    /// Queues tick `tick`'s rows and log lines at `now_us`.
    pub fn queue(&mut self, now_us: u64, tick: u64) {
        let plan = self.plan;
        let flying = (plan.boost..plan.done).contains(&tick);
        let stage = match tick {
            0 => Some(("setting", "boot")),
            tick if tick == plan.boost => Some(("boosting", "launch")),
            tick if tick == plan.boost + BOOST_TICKS => Some(("gliding", "burnout")),
            tick if tick == plan.land => Some(("landing", "altitude")),
            tick if tick == plan.done => Some(("done", "landed")),
            _ => None,
        };
        if let Some((stage, reason)) = stage {
            self.recorder.set_flight(flying);
            self.sequencer
                .push(now_us, &[Cell::Text(stage), Cell::Text(reason)])
                .unwrap();
            self.rows[2] += 1;
            assert!(self.recorder.log("sequencer", &format!("stage -> {stage}")));
            self.logs += 1;
        }
        if flying || tick.is_multiple_of(plan.pad_imu_every) {
            let az = if (plan.boost..plan.boost + BOOST_TICKS).contains(&tick) {
                5.0
            } else if flying {
                0.4
            } else {
                1.0
            };
            let cells = [
                Cell::F32(0.01),
                Cell::F32(-0.02),
                Cell::F32(az),
                Cell::I32(12),
                Cell::I32(-7),
                Cell::I32(tick as i32 % 300),
                Cell::I32(tick as i32),
            ];
            self.imu.push(now_us, &cells).unwrap();
            self.rows[0] += 1;
        }
        if tick.is_multiple_of(100) {
            let cells = [
                Cell::F32(41.5),
                Cell::I32(1_900_000 - tick as i32),
                Cell::I32(23),
                Cell::None,
                Cell::None,
                Cell::I32(0),
                Cell::I32(0),
                Cell::I32(0),
            ];
            self.health.push(now_us, &cells).unwrap();
            self.rows[1] += 1;
        }
        if tick.is_multiple_of(25) {
            assert!(self.recorder.log("control", &format!("tick {tick}")));
            self.logs += 1;
        }
    }
}
