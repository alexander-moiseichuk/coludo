//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The panic log (doc/specs/avionics-port.md, "Panic log"): the `panic` partition keeps the board's last panics as
//! text records, round-robin over [`SLOT_BYTES`] slots, the way Linux's mtdoops keeps oopses in an MTD partition. A
//! panic is fatal or not:
//! * fatal: the board reset (a panic, a watchdog, a brownout). The text is what the board captured as it went down
//!   -- the panic handler's output and a Rust panic's message, which components/panic_capture keeps in RTC RAM across
//!   the reset -- and the next boot writes the record, with the OS up, never the panic handler;
//! * not fatal (an oops): the board noticed something broken and went on (a radio link it parked, a bus it had to
//!   reset). The firmware writes the record as it goes on.
//!
//! A slot holds a record when its first [`HEADER_BYTES`] start with a whole header line:
//!
//! ```text
//! COLUDO-PANIC 1 seq=12 fatal=1 cause=panic boot=351 session=000351 uptime_ms=1583111 firmware=avionics-0.1.0 config=3aa09342698a recorded=352 length=2817 intact=1
//! ```
//!
//! `seq` counts records over the partition's life, so the newest is the largest and the next record goes into the slot
//! after it, erasing the oldest (a u32: past 4 billion records the order would break, which a board never reaches). The text goes in first and the header last: a power cut leaves no header, so a torn
//! record is an empty slot, the next one written there. Anything else in a slot (an erased one, another firmware's
//! bytes) is no record either.
//!
//! Everything runs over flash_ring's [`Flash`] trait, so the host tests use its [`flash_ring::MemoryFlash`] and its
//! power cuts. `no_std` + `alloc`.

#![no_std]

extern crate alloc;

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec;
use alloc::vec::Vec;

use flash_ring::{BLOCK_BYTES, Flash};

/// One record's slot: two erase sectors.
pub const SLOT_BYTES: u32 = 2 * BLOCK_BYTES;

/// The room for the header line at the slot's start; the text follows it. The line's fixed part is under 170 bytes,
/// and each of its four text tags is held to [`TAG_BYTES`], so a header always fits.
pub const HEADER_BYTES: u32 = 512;

/// The longest text tag (cause, session, firmware, config) a header keeps; a longer one is cut.
pub const TAG_BYTES: usize = 64;

/// The most text a record keeps.
pub const TEXT_BYTES: u32 = SLOT_BYTES - HEADER_BYTES;

/// The header line's first word.
const MAGIC: &str = "COLUDO-PANIC";

/// The record layout's version, the header line's second word.
const VERSION: &str = "1";

/// What a tag holds for "not known".
const UNKNOWN: &str = "-";

/// What a record says about the panic it records: its header line's tags.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Tags {
    /// The record's number over the partition's life: the newest is the largest. [`PanicLog::append`] sets it.
    pub seq: u32,
    /// Whether the board reset (fatal), or went on (an oops).
    pub fatal: bool,
    /// What happened, one word: for a fatal panic the next boot's reset reason (panic, interrupt-watchdog,
    /// task-watchdog, watchdog, brownout, ...), for an oops what the firmware names it (c6-link-parked, ...).
    pub cause: String,
    /// The boot id it happened in (0 when not known).
    pub boot: u32,
    /// That boot's session label (empty when not known).
    pub session: String,
    /// Its uptime when it happened, in ms (0 when not known: a fatal reset with nothing captured).
    pub uptime_ms: u32,
    /// The firmware that ran (empty when not known).
    pub firmware: String,
    /// The board config's id it ran (empty when not known).
    pub config: String,
    /// The boot id that wrote the record: the next one for a fatal panic, the same one for an oops.
    pub recorded: u32,
    /// The text's length in bytes. [`PanicLog::append`] sets it.
    pub length: u32,
    /// Whether the text is whole: a fatal panic's capture checksum matched across the reset (an oops always is).
    pub intact: bool,
}

impl Tags {
    /// The header line, `\n` included. A tag's value keeps letters, digits and `._:+/-`; anything else becomes `_`,
    /// an empty value is written `-`, and a text tag is cut to [`TAG_BYTES`], so the line splits on spaces and stays
    /// within [`HEADER_BYTES`].
    pub fn header(&self) -> String {
        format!(
            "{MAGIC} {VERSION} seq={} fatal={} cause={} boot={} session={} uptime_ms={} firmware={} config={} \
             recorded={} length={} intact={}\n",
            self.seq,
            u8::from(self.fatal),
            token(&self.cause),
            self.boot,
            token(&self.session),
            self.uptime_ms,
            token(&self.firmware),
            token(&self.config),
            self.recorded,
            self.length,
            u8::from(self.intact),
        )
    }

    /// The tags of a slot's first bytes: None unless they start with a whole header line of this layout (ASCII,
    /// ending in `\n` within [`HEADER_BYTES`], with `seq` and `length`). A length past [`TEXT_BYTES`] is held to it.
    pub fn parse(bytes: &[u8]) -> Option<Tags> {
        let window = &bytes[..bytes.len().min(HEADER_BYTES as usize)];
        let end = window.iter().position(|&byte| byte == b'\n')?;
        let line = core::str::from_utf8(&window[..end]).ok()?;
        if !line.bytes().all(|byte| byte.is_ascii_graphic() || byte == b' ') {
            return None;
        }
        let mut words = line.split(' ');
        if words.next() != Some(MAGIC) || words.next() != Some(VERSION) {
            return None;
        }
        let mut tags = Tags::default();
        let (mut seq, mut length) = (None, None);
        for word in words {
            let (key, value) = word.split_once('=')?;
            let text = || {
                if value == UNKNOWN {
                    String::new()
                } else {
                    value.to_string()
                }
            };
            match key {
                "seq" => seq = Some(value.parse().ok()?),
                "fatal" => tags.fatal = value == "1",
                "cause" => tags.cause = text(),
                "boot" => tags.boot = value.parse().ok()?,
                "session" => tags.session = text(),
                "uptime_ms" => tags.uptime_ms = value.parse().ok()?,
                "firmware" => tags.firmware = text(),
                "config" => tags.config = text(),
                "recorded" => tags.recorded = value.parse().ok()?,
                "length" => length = Some(value.parse::<u32>().ok()?),
                "intact" => tags.intact = value == "1",
                _ => {} // a tag a later layout adds
            }
        }
        tags.seq = seq?;
        tags.length = length?.min(TEXT_BYTES);
        Some(tags)
    }
}

/// A tag's value as the header line writes it (see [`Tags::header`]).
fn token(value: &str) -> String {
    if value.is_empty() {
        return UNKNOWN.to_string();
    }
    value
        .chars()
        .take(TAG_BYTES)
        .map(|character| {
            if character.is_ascii_alphanumeric() || "._:+/-".contains(character) {
                character
            } else {
                '_'
            }
        })
        .collect()
}

/// Why a panic log call failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PanicLogError<E> {
    /// The flash failed.
    Flash(E),
    /// The partition holds no whole slot.
    TooSmall,
    /// The header line is longer than [`HEADER_BYTES`] (never, with the tags held to [`TAG_BYTES`]).
    HeaderTooLong,
}

/// The panic partition: its slots and the records they hold.
pub struct PanicLog<F: Flash> {
    /// The partition.
    flash: F,
    /// Each slot's record, by slot.
    slots: Vec<Option<Tags>>,
}

impl<F: Flash> PanicLog<F> {
    /// Reads every slot's header.
    ///
    /// Errors: a header read failed.
    pub fn open(flash: F) -> Result<Self, PanicLogError<F::Error>> {
        let count = flash.size() / SLOT_BYTES;
        let mut header = vec![0u8; HEADER_BYTES as usize];
        let mut slots = Vec::with_capacity(count as usize);
        for slot in 0..count {
            flash
                .read(slot * SLOT_BYTES, &mut header)
                .map_err(PanicLogError::Flash)?;
            slots.push(Tags::parse(&header));
        }
        Ok(Self { flash, slots })
    }

    /// How many records the partition holds at most.
    pub fn capacity(&self) -> u32 {
        self.slots.len() as u32
    }

    /// The records, oldest first, with their slots.
    pub fn records(&self) -> Vec<(u32, &Tags)> {
        let mut records: Vec<(u32, &Tags)> = self
            .slots
            .iter()
            .enumerate()
            .filter_map(|(slot, tags)| tags.as_ref().map(|tags| (slot as u32, tags)))
            .collect();
        records.sort_by_key(|(_, tags)| tags.seq);
        records
    }

    /// The newest record's tags, if any.
    pub fn newest(&self) -> Option<&Tags> {
        self.records().pop().map(|(_, tags)| tags)
    }

    /// Writes a record into the slot after the newest one (slot 0 on an empty log), erasing whatever it held: the
    /// text first (held to [`TEXT_BYTES`]), the header last. `tags.seq` and `tags.length` are set here.
    ///
    /// Returns the record's tags as written.
    ///
    /// Errors: no whole slot, or the flash failed; the slot then holds no record.
    pub fn append(&mut self, mut tags: Tags, text: &[u8]) -> Result<Tags, PanicLogError<F::Error>> {
        if self.slots.is_empty() {
            return Err(PanicLogError::TooSmall);
        }
        let (slot, seq) = match self.records().pop() {
            Some((slot, newest)) => ((slot + 1) % self.capacity(), newest.seq.wrapping_add(1)),
            None => (0, 1),
        };
        let text = &text[..text.len().min(TEXT_BYTES as usize)];
        tags.seq = seq;
        tags.length = text.len() as u32;
        let header = tags.header();
        if header.len() > HEADER_BYTES as usize {
            return Err(PanicLogError::HeaderTooLong);
        }
        let start = slot * SLOT_BYTES;
        self.slots[slot as usize] = None;
        for sector in 0..SLOT_BYTES / BLOCK_BYTES {
            self.flash
                .erase_sector(start + sector * BLOCK_BYTES)
                .map_err(PanicLogError::Flash)?;
        }
        if !text.is_empty() {
            self.flash
                .program(start + HEADER_BYTES, text)
                .map_err(PanicLogError::Flash)?;
        }
        self.flash
            .program(start, header.as_bytes())
            .map_err(PanicLogError::Flash)?;
        self.slots[slot as usize] = Some(tags.clone());
        Ok(tags)
    }

    /// The record with this `seq` as text: its header line, then its text. None when no slot holds it.
    ///
    /// Errors: the read failed.
    pub fn text(&self, seq: u32) -> Result<Option<Vec<u8>>, PanicLogError<F::Error>> {
        let Some((slot, tags)) = self.records().into_iter().find(|(_, tags)| tags.seq == seq) else {
            return Ok(None);
        };
        let mut text = tags.header().into_bytes();
        let start = text.len();
        text.resize(start + tags.length as usize, 0);
        self.flash
            .read(slot * SLOT_BYTES + HEADER_BYTES, &mut text[start..])
            .map_err(PanicLogError::Flash)?;
        Ok(Some(text))
    }
}

#[cfg(test)]
mod tests {
    use super::{HEADER_BYTES, PanicLog, PanicLogError, SLOT_BYTES, TAG_BYTES, TEXT_BYTES, Tags};
    use alloc::string::{String, ToString};
    use alloc::vec;
    use alloc::vec::Vec;
    use flash_ring::MemoryFlash;

    /// The tags of a fatal panic of boot `boot`.
    fn panic_of(boot: u32) -> Tags {
        Tags {
            fatal: true,
            cause: "panic".to_string(),
            boot,
            session: alloc::format!("{boot:06}"),
            uptime_ms: 1_583_111,
            firmware: "avionics-0.1.0".to_string(),
            config: "3aa09342698a".to_string(),
            recorded: boot + 1,
            intact: true,
            ..Tags::default()
        }
    }

    /// The (seq, boot) of every record, oldest first.
    fn listed(log: &PanicLog<MemoryFlash>) -> Vec<(u32, u32)> {
        log.records().iter().map(|(_, tags)| (tags.seq, tags.boot)).collect()
    }

    #[test]
    fn the_header_line_reads_back_and_values_stay_one_word() {
        let mut tags = panic_of(351);
        tags.seq = 12;
        tags.length = 2817;
        let line = tags.header();
        assert_eq!(
            line,
            "COLUDO-PANIC 1 seq=12 fatal=1 cause=panic boot=351 session=000351 uptime_ms=1583111 \
             firmware=avionics-0.1.0 config=3aa09342698a recorded=352 length=2817 intact=1\n"
        );
        assert!(line.len() <= HEADER_BYTES as usize);
        assert_eq!(Tags::parse(line.as_bytes()), Some(tags.clone()));
        // an oops: not fatal, recorded by its own boot
        let oops = Tags {
            fatal: false,
            cause: "c6-link-parked".to_string(),
            recorded: 351,
            ..tags.clone()
        };
        assert_eq!(Tags::parse(oops.header().as_bytes()), Some(oops));
        // a label with spaces or a newline cannot split the line; an empty value is written - and read back empty
        tags.session = "flight 3\nx".to_string();
        tags.config = String::new();
        let parsed = Tags::parse(tags.header().as_bytes()).expect("a header");
        assert_eq!(parsed.session, "flight_3_x");
        assert_eq!(parsed.config, "");
        // the longest header there can be fits: every number at its widest, every text tag past the cut
        let widest = Tags {
            seq: u32::MAX,
            fatal: true,
            cause: "c".repeat(300),
            boot: u32::MAX,
            session: "s".repeat(300),
            uptime_ms: u32::MAX,
            firmware: "f".repeat(300),
            config: "g".repeat(300),
            recorded: u32::MAX,
            length: TEXT_BYTES,
            intact: true,
        };
        let line = widest.header();
        assert!(line.len() <= HEADER_BYTES as usize, "{}", line.len());
        assert_eq!(
            Tags::parse(line.as_bytes()).expect("a header").session,
            "s".repeat(TAG_BYTES)
        );
    }

    #[test]
    fn anything_but_a_whole_header_is_no_record() {
        let line = {
            let mut tags = panic_of(7);
            tags.seq = 1;
            tags.header()
        };
        assert!(Tags::parse(line.as_bytes()).is_some());
        // erased, zeroed, torn before its newline, another layout, a missing seq, binary bytes
        assert_eq!(Tags::parse(&[0xFF; 256]), None);
        assert_eq!(Tags::parse(&[0; 256]), None);
        assert_eq!(Tags::parse(&line.as_bytes()[..line.len() - 1]), None);
        assert_eq!(Tags::parse(line.replacen(" 1 ", " 2 ", 1).as_bytes()), None);
        assert_eq!(Tags::parse(line.replacen("seq=1 ", "", 1).as_bytes()), None);
        assert_eq!(Tags::parse(b"COLUDO-PANIC 1 seq=1 length=3 \x01\n"), None);
        // a newline past the header's room is no header
        let mut long = vec![b'x'; HEADER_BYTES as usize + 8];
        long[HEADER_BYTES as usize + 4] = b'\n';
        assert_eq!(Tags::parse(&long), None);
        // a length past the room is held to it; an unknown tag is skipped
        let parsed = Tags::parse(b"COLUDO-PANIC 1 seq=3 length=99999 later=x\n").expect("a header");
        assert_eq!((parsed.seq, parsed.length), (3, TEXT_BYTES));
    }

    #[test]
    fn records_go_round_robin_and_the_oldest_is_overwritten() {
        let mut log = PanicLog::open(MemoryFlash::new(4 * SLOT_BYTES)).expect("an empty log");
        assert_eq!(log.capacity(), 4);
        assert!(log.records().is_empty());
        for boot in 1..=6 {
            let written = log.append(panic_of(boot), b"Guru Meditation Error").expect("appended");
            assert_eq!(written.seq, boot);
        }
        // six records into four slots: the two oldest are gone, the seq goes on
        assert_eq!(listed(&log), [(3, 3), (4, 4), (5, 5), (6, 6)]);
        assert_eq!(log.newest().map(|tags| tags.boot), Some(6));
        let slots: Vec<u32> = log.records().iter().map(|(slot, _)| *slot).collect();
        assert_eq!(slots, [2, 3, 0, 1]);
    }

    #[test]
    fn a_reopened_log_lists_the_same_records_and_goes_on() {
        let mut log = PanicLog::open(MemoryFlash::new(3 * SLOT_BYTES)).expect("an empty log");
        for boot in [40, 41, 42, 43] {
            log.append(panic_of(boot), b"text").expect("appended");
        }
        let mut again = PanicLog::open(MemoryFlash::from_image(log.flash.image())).expect("the same log");
        assert_eq!(listed(&again), [(2, 41), (3, 42), (4, 43)]);
        let next = again.append(panic_of(44), b"text").expect("appended");
        assert_eq!(next.seq, 5);
        assert_eq!(listed(&again), [(3, 42), (4, 43), (5, 44)]);
    }

    #[test]
    fn the_text_reads_back_behind_its_header_and_is_held_to_the_room() {
        let mut log = PanicLog::open(MemoryFlash::new(2 * SLOT_BYTES)).expect("an empty log");
        let text = b"Guru Meditation Error: Core  0 panic'ed (Load access fault).\nMEPC    : 0x4800abcd\n";
        let written = log.append(panic_of(9), text).expect("appended");
        assert_eq!(written.length, text.len() as u32);
        let read = log.text(written.seq).expect("read").expect("a record");
        assert_eq!(read, [written.header().as_bytes(), &text[..]].concat());
        assert_eq!(log.text(99).expect("read"), None);
        // a capture longer than the room keeps its first TEXT_BYTES
        let long = vec![b'z'; TEXT_BYTES as usize + 500];
        let written = log.append(panic_of(10), &long).expect("appended");
        assert_eq!(written.length, TEXT_BYTES);
        let read = log.text(written.seq).expect("read").expect("a record");
        assert_eq!(read.len(), written.header().len() + TEXT_BYTES as usize);
        // a record with no text (a watchdog reset with nothing captured) is a header alone
        let watchdog = Tags {
            cause: "watchdog".to_string(),
            ..panic_of(11)
        };
        let written = log.append(watchdog, b"").expect("appended");
        assert_eq!(written.length, 0);
        assert_eq!(
            log.text(written.seq).expect("read").expect("a record"),
            written.header().into_bytes()
        );
    }

    #[test]
    fn a_cut_while_writing_leaves_an_empty_slot_never_a_torn_record() {
        // every program and erase of the third append, cut at the byte counts that matter
        for operations in 0..4 {
            for torn in [0, 1, 40, usize::MAX] {
                let mut log = PanicLog::open(MemoryFlash::new(4 * SLOT_BYTES)).expect("an empty log");
                log.append(panic_of(1), b"first").expect("appended");
                log.append(panic_of(2), b"second").expect("appended");
                log.flash.cut_power(operations, torn);
                let finished = log.append(panic_of(3), b"third record text").is_ok();
                let again = PanicLog::open(MemoryFlash::from_image(log.flash.image())).expect("the same log");
                let records = listed(&again);
                // the third record is whole or absent; the earlier two survive (the slot written was free)
                assert!(
                    records == [(1, 1), (2, 2)] || records == [(1, 1), (2, 2), (3, 3)],
                    "{records:?}"
                );
                if records.len() == 3 {
                    let text = again.text(3).expect("read").expect("a record");
                    assert!(text.ends_with(b"third record text"));
                }
                // a call that returned Ok was written whole
                if finished {
                    assert_eq!(records.len(), 3);
                }
            }
        }
    }

    #[test]
    fn a_cut_overwriting_the_oldest_loses_only_that_one() {
        let mut log = PanicLog::open(MemoryFlash::new(2 * SLOT_BYTES)).expect("an empty log");
        log.append(panic_of(1), b"one").expect("appended");
        log.append(panic_of(2), b"two").expect("appended");
        // the third goes over the first: cut after the first erase
        log.flash.cut_power(1, 0);
        assert!(log.append(panic_of(3), b"three").is_err());
        let mut again = PanicLog::open(MemoryFlash::from_image(log.flash.image())).expect("the same log");
        assert_eq!(listed(&again), [(2, 2)]);
        // the next record takes the emptied slot
        assert_eq!(again.append(panic_of(4), b"four").expect("appended").seq, 3);
        assert_eq!(listed(&again), [(2, 2), (3, 4)]);
    }

    #[test]
    fn foreign_bytes_are_no_records_and_a_tiny_partition_refuses() {
        // a partition another firmware wrote: zeros in one slot, text in another
        let mut image = vec![0xFF; 3 * SLOT_BYTES as usize];
        image[..SLOT_BYTES as usize].fill(0);
        image[SLOT_BYTES as usize..SLOT_BYTES as usize + 11].copy_from_slice(b"hello\nworld");
        let mut log = PanicLog::open(MemoryFlash::from_image(image)).expect("a log");
        assert!(log.records().is_empty());
        assert_eq!(log.append(panic_of(5), b"x").expect("appended").seq, 1);
        assert_eq!(listed(&log), [(1, 5)]);
        // less than one slot: nothing to write into
        let mut tiny = PanicLog::open(MemoryFlash::new(SLOT_BYTES - 1)).expect("a log");
        assert_eq!(tiny.capacity(), 0);
        assert!(matches!(tiny.append(panic_of(6), b"x"), Err(PanicLogError::TooSmall)));
    }
}
