//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The uart_saver's walk (M19 stage 2, doc/specs/avionics-port.md M19): a [`Walk`] of the flash ring with a cursor of
//! its own, so the recorder's lines go out of the UART whether a card is mounted or not, and a reset costs the
//! receiver nothing. The firmware's uart_saver thread calls [`Relay::step`] again and again, writes each part's lines
//! to its UART, and once they have left it calls [`Relay::sent`]: the position published there is how far the
//! receiver has been sent, which the firmware keeps as the `uart` NVS hint and the next boot starts from. A part sent
//! again after a reset is the receiver's to drop (its files hold each line once: no line is newer than its file's
//! last).
//!
//! Stage 1 teed the ring_saver's lines into a queue instead. The saver tees a part when it stages it and the card has
//! it a commit later, so a part committed but still queued at a reset was never sent, and never sent again by the
//! next boot, which starts from the card's position (1.5 s of one session on the C6's card, 2026-10-11); and with no
//! card nothing was sent at all.
//!
//! **Each line names its session** ([`tee`]): a log line goes routed `<session>_recorder.log`, a session.csv line
//! `<session>_session.csv`, the body as the saver files it, its checks made under its own routing.
//!
//! **The eraser keeps what the relay has not sent:** [`Recorder::erase_horizon`] is the saver's position held back to
//! the relay's, so a saved block is erased only once the UART has sent it too.
//!
//! **Recency first:** a relay more than its `behind` blocks behind the writer jumps to the oldest block within it,
//! the blocks it passed counted ([`Relay::skipped`]): the last seconds before an impact matter most, and a UART slower
//! than the ring (a backlog after a long reset, a slower link) must not spend them on old ones.

use flash_ring::{BlockTable, Flash, Position};

use crate::recorder::Recorder;
use crate::saver::{Ahead, Walk, tee};

/// How far behind the writer the firmware's relay may fall before it jumps forward, in 4 KiB blocks: ~10 s of the
/// flight's ~26 KB/s of records.
pub const BEHIND_BLOCKS: u32 = 64;

/// The uart_saver's walk: see the module notes.
pub struct Relay<'r, 't, F: Flash> {
    /// This boot's recorder: the writer's position, and where the relay publishes its own.
    recorder: &'r Recorder,
    /// The cursor and the decoder.
    walk: Walk<'r, 't, F>,
    /// A line routed under its session, built for [`tee`].
    teed: Vec<u8>,
    /// How far behind the writer it may fall, in blocks.
    behind: u32,
    /// The blocks it jumped over to stay within `behind`.
    skipped: u32,
}

impl<'r, 't, F: Flash> Relay<'r, 't, F> {
    /// A relay from `start` (the `uart` hint through [`BlockTable::saver_start`], or this boot's first block),
    /// published as sent at once: nothing before it is the relay's to send.
    ///
    /// Args:
    /// * `recorder` - this boot's recorder.
    /// * `flash` - the relay's handle on the `log` partition.
    /// * `table` - the scan's block table, shared with the writer, the eraser and the saver.
    /// * `start` - where to begin.
    /// * `behind` - how far behind the writer it may fall, in blocks ([`BEHIND_BLOCKS`] on the board).
    pub fn new(
        recorder: &'r Recorder,
        flash: F,
        table: &'t BlockTable,
        start: Position,
        behind: u32,
    ) -> Relay<'r, 't, F> {
        recorder.relay_position.store(start);
        Relay {
            recorder,
            walk: Walk::new(recorder, flash, table, start),
            teed: Vec::new(),
            behind,
            skipped: 0,
        }
    }

    /// Reads and decodes the next part of the ring, handing each line, routed under its session, to `send`.
    ///
    /// Returns the cursor past the part, to pass to [`Relay::sent`] once its bytes have left; None when nothing is
    /// unread. Errors: a flash read failed; the cursor stays, and the next step reads the same part.
    pub fn step(&mut self, mut send: impl FnMut(&[u8])) -> Result<Option<Position>, F::Error> {
        if let Some(writer) = self.recorder.writer_position() {
            let floor = writer.block_sequence.saturating_sub(self.behind);
            let cursor = self.walk.cursor();
            if cursor.block_sequence < floor {
                self.skipped = self.skipped.saturating_add(floor - cursor.block_sequence);
                self.walk.jump(Position {
                    block_sequence: floor,
                    offset: 0,
                });
            }
        }
        match self.walk.peek()? {
            Ahead::CaughtUp => Ok(None),
            Ahead::Block(_) => {
                let teed = &mut self.teed;
                self.walk.decode(|line| send(tee(teed, &line)));
                Ok(Some(self.walk.cursor()))
            }
        }
    }

    /// Publishes `position` as sent: every line before it has left the UART. The eraser may now take the blocks
    /// before it (and before the saver's position) as done.
    pub fn sent(&self, position: Position) {
        self.recorder.relay_position.store(position);
    }

    /// The next byte to read.
    pub fn cursor(&self) -> Position {
        self.walk.cursor()
    }

    /// The blocks jumped over to stay within `behind` of the writer.
    pub fn skipped(&self) -> u32 {
        self.skipped
    }
}
