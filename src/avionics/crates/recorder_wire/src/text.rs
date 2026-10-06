//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The text of a recorder line before it is wrapped (src/glider recorder.py; doc/specs/avionics-port.md "Row,
//! log and index text"): a telemetry stream's header and rows, a log line, the session.csv index rows, the
//! session prefix and the Recorder's status message.
//!
//! Every function appends to the caller's buffer or returns the text, and none adds the line end: the wrapper
//! closes the line ([`crate::wrap`]).
//!
//! Deliberate differences from plan B, each one a text the host tools already read:
//!  * an F32 cell is Rust's `Display`: the shortest text that reads back as the same f32, never an exponent.
//!    Plan B printed `str(float)`, which on the board (MicroPython 1.29, single precision) is a round-trip text
//!    of up to 9 digits, not always the shortest (`0.982831936` where the shortest is `0.98283195`), with an
//!    exponent below 1e-4 (`1e-05`), `.0` on an integral value (`1.0` where this writes `1`), `nan` (`NaN`
//!    here) and `-0.0` (`-0` here). Compare floats numerically, never as text;
//!  * an E7 cell (a 1e-7 degree integer) and session.csv's `cc_lat`/`cc_lon` print 7 decimals from the integer
//!    (`25.5000000`), where plan B printed its float degrees (`25.5`);
//!  * a Text cell has `;` and control bytes made spaces, and a log line its control bytes. Plan B wrote them as
//!    they came: a `;` split the cell, a `\n` split the line (both halves then failed their checks), and a byte
//!    below 0x0A was dropped by recorderd, failing the line's CRC;
//!  * session.csv's `utc_offset` is emptied outside -720..=840 here as well, where plan B left that to
//!    mission.py, and the date comes from a Unix epoch (the Rust clock), not the 2000-based MicroPython RTC;
//!  * a `recorder.session` value that is not text names no session: plan B took `str(value)`, so `true` became
//!    the label `True`;
//!  * the status message lists `session`, `tlm`, `log` in that order, as plan B's dict does on CPython, then any
//!    members plan B did not have (the flash ring's); on the board MicroPython printed its own dict order (`tlm`,
//!    `log`, `session`). The readers search for `'session': '<s>'`, which both carry.

use alloc::string::String;
use alloc::vec::Vec;
use core::fmt::{self, Display, Write};

/// The shared session index's routing: bare, with no session prefix, so every boot appends to one file.
pub const SESSION_INDEX: &str = "session.csv";

/// The session index's CSV header (recorder.py `_SESSION_INDEX_HEADER`).
pub const SESSION_INDEX_HEADER: &str =
    "uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon";

/// A session label's and a session.csv text cell's longest, in bytes (recorder.py `_LABEL_MAX`, `_CELL_MAX`):
/// the label heads every telemetry routing, and board, firmware and source must fit one row.
const TEXT_LIMIT: usize = 32;

/// 2001-01-01T00:00:00Z in Unix seconds: a clock reading earlier was never set (recorder.py `_CLOCK_SET_S`,
/// there on the 2000 epoch), so it dates nothing.
const CLOCK_SET_EPOCH: i64 = 978_307_200;

/// The UTC offsets that exist, in minutes: UTC-12:00 to UTC+14:00. Outside them the cell stays empty.
const UTC_OFFSET_MINUTES: core::ops::RangeInclusive<i32> = -720..=840;

/// The legacy session prefix's random suffix runs over the six-digit numbers (recorder.py `randint(100000,
/// 999999)`): this many of them, from this first one.
const RANDOM_SUFFIXES: u32 = 900_000;

/// The first six-digit random suffix.
const RANDOM_SUFFIX_FIRST: u32 = 100_000;

/// 1e-7 degree units per degree, for an E7 cell's text.
const UNITS_PER_DEGREE: u32 = 10_000_000;

/// Seconds in a UTC day (no leap seconds, as Unix time counts).
const SECONDS_PER_DAY: i64 = 86_400;

/// A `core::fmt` sink appending to a byte buffer, so numbers format straight into a line.
struct Appender<'a>(&'a mut Vec<u8>);

impl Write for Appender<'_> {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        self.0.extend_from_slice(text.as_bytes());
        Ok(())
    }
}

/// Append `value`'s `Display` text. Appending to a Vec cannot fail, and no `Display` used here reports an
/// error of its own, so the result carries nothing.
fn append(text: &mut Vec<u8>, value: impl Display) {
    let _ = write!(Appender(text), "{value}");
}

/// Whether a byte is an ASCII control (below 0x20, or DEL): never part of a recorded text.
fn is_control(byte: u8) -> bool {
    byte < 0x20 || byte == 0x7F
}

/// Append `text` with every byte `replaced` says so made a space, keeping the length.
fn append_cleaned(line: &mut Vec<u8>, text: &str, replaced: impl Fn(u8) -> bool) {
    line.extend(text.bytes().map(|byte| if replaced(byte) { b' ' } else { byte }));
}

/// One value of a telemetry row (doc/specs/avionics-port.md "Producer side": a ring record's cell tags).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Cell<'a> {
    /// No value: prints `None`, as plan B's Python None did (health.csv's `oom_s`).
    None,
    /// A 32-bit integer, in decimal.
    I32(i32),
    /// A 64-bit integer, in decimal.
    I64(i64),
    /// A float, as Rust's `Display` writes it (see the module notes).
    F32(f32),
    /// A flag: `1` or `0`, as plan B pushed `1 if x else 0`.
    Bool(bool),
    /// An angle in 1e-7 degree units, printed as signed degrees with 7 decimals (`-80.3917950`).
    E7(i32),
    /// A word or a short text: `;` and control bytes are made spaces, so it stays one cell of one line.
    Text(&'a str),
}

impl Cell<'_> {
    /// Append this cell's text.
    fn append_to(&self, line: &mut Vec<u8>) {
        match *self {
            Cell::None => line.extend_from_slice(b"None"),
            Cell::I32(value) => append(line, value),
            Cell::I64(value) => append(line, value),
            Cell::F32(value) => append(line, value),
            Cell::Bool(value) => line.push(if value { b'1' } else { b'0' }),
            Cell::E7(units) => append_degrees(line, units),
            Cell::Text(text) => append_cleaned(line, text, |byte| byte == b';' || is_control(byte)),
        }
    }
}

/// Append 1e-7 degree `units` as degrees with exactly 7 decimals and a sign when negative: `-0.0000001`,
/// `214.7483647`, and i32::MIN too (`-214.7483648`).
fn append_degrees(line: &mut Vec<u8>, units: i32) {
    if units < 0 {
        line.push(b'-');
    }
    let magnitude = units.unsigned_abs();
    append(line, magnitude / UNITS_PER_DEGREE);
    let _ = write!(Appender(line), ".{:07}", magnitude % UNITS_PER_DEGREE);
}

/// Append a telemetry stream's CSV header, `uptime;<field>;...` (recorder.py `Telemetry`). As plan B, a stream
/// with no fields gets `uptime;`.
pub fn header(line: &mut Vec<u8>, fields: &[&str]) {
    line.extend_from_slice(b"uptime;");
    for (index, field) in fields.iter().enumerate() {
        if index > 0 {
            line.push(b';');
        }
        line.extend_from_slice(field.as_bytes());
    }
}

/// Append a telemetry row, `<uptime_us>;<cell>;...` (recorder.py `Telemetry.push`). As plan B, a row with no
/// cells is `<uptime_us>;`.
pub fn row(line: &mut Vec<u8>, uptime_us: u64, cells: &[Cell]) {
    append(line, uptime_us);
    line.push(b';');
    for (index, cell) in cells.iter().enumerate() {
        if index > 0 {
            line.push(b';');
        }
        cell.append_to(line);
    }
}

/// Append a recorder.log line, `<uptime_us> <descriptor> :: <message>` (recorder.py `Recorder.log`), with
/// control bytes made spaces.
///
/// `limit` is the longest line in bytes counting the `\n` the wrapper adds: a longer text is cut to
/// `limit - 1` bytes, wherever that falls (inside a UTF-8 character too), as plan B cut its encoded line to the
/// ring cell and put the `\n` back. A cut that dropped the line end would merge the line with the next one on
/// the wire, a telemetry row included.
pub fn log_line(line: &mut Vec<u8>, uptime_us: u64, descriptor: &str, message: &str, limit: usize) {
    let start = line.len();
    append(line, uptime_us);
    line.push(b' ');
    append_cleaned(line, descriptor, is_control);
    line.extend_from_slice(b" :: ");
    append_cleaned(line, message, is_control);
    line.truncate(start + (line.len() - start).min(limit.saturating_sub(1)));
}

/// A free-text session.csv cell (recorder.py `_cell`): `text` when it is printable ASCII (0x20-0x7E) without
/// `;` and at most 32 bytes, else empty. A `;` would split the cell, a control byte the line or its CRC, and a
/// long cell would crowd the row.
pub fn cell(text: &str) -> &str {
    let printable = text.bytes().all(|byte| (0x20..=0x7E).contains(&byte) && byte != b';');
    if printable && text.len() <= TEXT_LIMIT {
        text
    } else {
        ""
    }
}

/// Whether a `recorder.session` label is usable (recorder.py `_is_label`): ASCII letters, digits and `-`, at
/// least one letter, at most 32 bytes.
///
/// The letter keeps a label from reading as a boot id, the missing `_` keeps `<session>_<stream>.csv`
/// splittable at its first `_`, and the cap keeps the routing it heads short.
pub fn is_label(text: &str) -> bool {
    let allowed = text.bytes().all(|byte| byte.is_ascii_alphanumeric() || byte == b'-');
    allowed && text.len() <= TEXT_LIMIT && text.bytes().any(|byte| byte.is_ascii_alphabetic())
}

/// What a `recorder.session` config value names.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SessionLabel {
    /// Nothing (absent, empty or blank): the boot id names the files.
    Unset,
    /// A usable label: it names the files.
    Valid(String),
    /// A label that breaks the rule ([`is_label`]), as cleaned: ignored, so the boot id names the files, and
    /// worth a log line.
    Ignored(String),
}

/// Whether Python's `str.strip()` strips `character`: Rust's whitespace plus the separators U+001C to U+001F,
/// which Python counts and Rust does not.
fn is_python_whitespace(character: char) -> bool {
    character.is_whitespace() || ('\u{1c}'..='\u{1f}').contains(&character)
}

/// The label a `recorder.session` text names, as plan B reads it: stripped, its spaces made `-`
/// (`catapult run3` is `catapult-run3`), then kept only when [`is_label`] passes.
pub fn session_label(configured: &str) -> SessionLabel {
    let label = configured.trim_matches(is_python_whitespace).replace(' ', "-");
    if label.is_empty() {
        SessionLabel::Unset
    } else if is_label(&label) {
        SessionLabel::Valid(label)
    } else {
        SessionLabel::Ignored(label)
    }
}

/// This boot's file prefix (recorder.py `Recorder.session`; doc/specs/recorder-wire.md "Session prefix"): the
/// first of a valid label, the boot id as `%06u`, or the legacy `YYYYMMDD_HHMMSS_<6 digits>` when nothing
/// counted the boot.
///
/// Only the legacy form reads `epoch` (the clock in Unix seconds) and `random` (any random number, such as
/// esp_random(), mapped onto the six-digit suffixes): its random part keeps apart boots that start within the
/// same second of an unset clock.
pub fn session_prefix(label: &SessionLabel, boot_id: Option<u32>, epoch: i64, random: u32) -> String {
    if let SessionLabel::Valid(label) = label {
        return label.clone();
    }
    if let Some(boot_id) = boot_id {
        return alloc::format!("{boot_id:06}");
    }
    let (year, month, day, hour, minute, second) = civil(epoch);
    let suffix = RANDOM_SUFFIX_FIRST + random % RANDOM_SUFFIXES;
    alloc::format!("{year:04}{month:02}{day:02}_{hour:02}{minute:02}{second:02}_{suffix}")
}

/// One session.csv row (recorder.py `Recorder.index_session`): this boot tied to the wall clock, written at the
/// `boot`, at every successful time set and at the `anchor`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SessionRow<'a> {
    /// The uptime the row was written at.
    pub uptime_us: u64,
    /// The NVS boot id; None when nothing counted the boot (an empty cell).
    pub boot_id: Option<u32>,
    /// The session prefix ([`session_prefix`]).
    pub session: &'a str,
    /// The clock in Unix seconds: before 2001 it was never set, and utc and utc_offset stay empty.
    pub epoch: i64,
    /// The time setter's offset from UTC in minutes; None (and any value outside -720..=840) is an empty cell.
    pub utc_offset: Option<i32>,
    /// The board id from the config, a text cell ([`cell`]).
    pub board: &'a str,
    /// The firmware version, a text cell.
    pub firmware: &'a str,
    /// The config id, as the firmware announces it.
    pub config_id: &'a str,
    /// The row's kind: `boot`, `anchor`, or what set the time (`cc-auto`, `dashboard`); a text cell.
    pub source: &'a str,
    /// CC's own (latitude, longitude) in 1e-7 degree units; None for two empty cells.
    pub cc_position: Option<(i32, i32)>,
}

impl SessionRow<'_> {
    /// Append the row's text, the cells of [`SESSION_INDEX_HEADER`] in order.
    pub fn append_to(&self, line: &mut Vec<u8>) {
        append(line, self.uptime_us);
        line.push(b';');
        if let Some(boot_id) = self.boot_id {
            append(line, boot_id);
        }
        line.push(b';');
        line.extend_from_slice(self.session.as_bytes());
        line.push(b';');
        if self.epoch >= CLOCK_SET_EPOCH {
            let (year, month, day, hour, minute, second) = civil(self.epoch);
            let _ = write!(
                Appender(line),
                "{year:04}-{month:02}-{day:02}T{hour:02}:{minute:02}:{second:02}Z"
            );
        }
        line.push(b';');
        let offset = self.utc_offset.filter(|minutes| UTC_OFFSET_MINUTES.contains(minutes));
        if let (Some(minutes), true) = (offset, self.epoch >= CLOCK_SET_EPOCH) {
            append(line, minutes);
        }
        for text in [cell(self.board), cell(self.firmware), self.config_id, cell(self.source)] {
            line.push(b';');
            line.extend_from_slice(text.as_bytes());
        }
        line.push(b';');
        if let Some((latitude, _)) = self.cc_position {
            append_degrees(line, latitude);
        }
        line.push(b';');
        if let Some((_, longitude)) = self.cc_position {
            append_degrees(line, longitude);
        }
    }
}

/// A ring's figures in the Recorder's status message.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    /// Records queued now.
    pub count: u32,
    /// The most ever queued.
    pub max: u32,
    /// Records dropped.
    pub dropped: u32,
}

impl Counters {
    /// Append `{'count': n, 'max': n, 'dropped': n}`, a Python dict's repr.
    fn append_to(&self, text: &mut String) {
        let _ = write!(
            text,
            "{{'count': {}, 'max': {}, 'dropped': {}}}",
            self.count, self.max, self.dropped
        );
    }
}

/// The Recorder's status message, logged as `Recorder :: <message>` every `stats_ms` (recorder.py `report`):
/// `{'session': '<s>', 'tlm': {...}, 'log': {...}}` in Python dict repr, which recorder_flight searches for
/// `'session': '<s>'`.
///
/// `members` follow `log` in their order, each `(key, value)` written `, '<key>': <value>`, with `value` already
/// in Python repr: the flash writer's `'ring': {...}` (doc/specs/avionics-port.md, "The flash writer") and M4's
/// `'mover': {...}`, whose figures their owners format. Plan B's message has none, so `&[]` writes it exactly.
///
/// `session` and each key are written between single quotes as they are: a prefix is letters, digits, `-` and
/// `_`, and a key is an identifier, never a quote or backslash, which Python's repr would escape.
pub fn status_message(session: &str, telemetry: &Counters, log: &Counters, members: &[(&str, &str)]) -> String {
    let mut text = alloc::format!("{{'session': '{session}', 'tlm': ");
    telemetry.append_to(&mut text);
    text.push_str(", 'log': ");
    log.append_to(&mut text);
    for (key, value) in members {
        let _ = write!(text, ", '{key}': {value}");
    }
    text.push('}');
    text
}

/// The UTC (year, month, day, hour, minute, second) of a Unix epoch.
///
/// Howard Hinnant's civil_from_days, as crates/mission's clock text uses it (this crate depends on nothing, so
/// it keeps its own copy): shift the year to start in March so the leap day ends it, then split into 400-year
/// eras of 146097 days, which repeat exactly.
fn civil(epoch: i64) -> (i64, i64, i64, i64, i64, i64) {
    let days = epoch.div_euclid(SECONDS_PER_DAY);
    let seconds = epoch.rem_euclid(SECONDS_PER_DAY);
    let shifted = days + 719_468; // days since 0000-03-01
    let era = shifted.div_euclid(146_097);
    let day_of_era = shifted.rem_euclid(146_097); // 0..=146096
    let year_of_era = (day_of_era - day_of_era / 1460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100); // from 1 March
    let month_from_march = (5 * day_of_year + 2) / 153; // 0..=11
    let day = day_of_year - (153 * month_from_march + 2) / 5 + 1;
    let month = if month_from_march < 10 {
        month_from_march + 3
    } else {
        month_from_march - 9
    };
    let year = year_of_era + era * 400 + i64::from(month <= 2);
    (year, month, day, seconds / 3600, seconds % 3600 / 60, seconds % 60)
}

#[cfg(test)]
mod tests {
    use super::{Cell, Counters, SessionLabel, SessionRow, cell, civil, header, is_label, log_line, row};
    use super::{session_label, session_prefix, status_message};
    use alloc::string::{String, ToString};
    use alloc::vec::Vec;

    /// A row's text.
    fn row_text(uptime_us: u64, cells: &[Cell]) -> String {
        let mut line = Vec::new();
        row(&mut line, uptime_us, cells);
        String::from_utf8(line).unwrap()
    }

    #[test]
    fn a_header_lists_uptime_then_the_fields() {
        let mut line = Vec::new();
        header(&mut line, &["ax", "ay", "az"]);
        assert_eq!(line, b"uptime;ax;ay;az");
        line.clear();
        header(&mut line, &[]);
        assert_eq!(line, b"uptime;", "no fields: plan B's trailing ';'");
    }

    #[test]
    fn every_cell_kind_prints_as_specified() {
        let cells = [
            Cell::None,
            Cell::I32(-21),
            Cell::I64(1 << 40),
            Cell::F32(0.98),
            Cell::Bool(true),
            Cell::Bool(false),
            Cell::E7(255_143_790),
            Cell::Text("boosting"),
        ];
        assert_eq!(
            row_text(884_029, &cells),
            "884029;None;-21;1099511627776;0.98;1;0;25.5143790;boosting"
        );
        assert_eq!(
            row_text(u64::MAX, &[]),
            "18446744073709551615;",
            "no 2^30 wrap, and no cells"
        );
    }

    #[test]
    fn floats_are_shortest_and_never_exponents() {
        let text = |value: f32| row_text(0, &[Cell::F32(value)]);
        assert_eq!(text(1.764), "0;1.764");
        assert_eq!(text(1.0), "0;1", "no '.0', unlike plan B");
        assert_eq!(text(1e-5), "0;0.00001", "no exponent, unlike plan B");
        assert_eq!(text(1.5e10), "0;15000000000");
        assert_eq!(text(-0.0), "0;-0");
        assert_eq!(text(f32::NAN), "0;NaN");
        assert_eq!(text(f32::NEG_INFINITY), "0;-inf");
        // the f32 the board printed as 0.982831936 (2026-10-03, imu_lsm6dso32.csv): its shortest text reads back
        let board: f32 = "0.982831936".parse().unwrap();
        let text = text(board);
        assert_eq!(text, "0;0.98283195");
        assert_eq!(text[2..].parse::<f32>().unwrap(), board);
    }

    #[test]
    fn degrees_keep_seven_decimals_and_their_sign() {
        let text = |units: i32| row_text(0, &[Cell::E7(units)]);
        assert_eq!(text(-803_917_950), "0;-80.3917950");
        assert_eq!(text(-1), "0;-0.0000001", "a sign below one degree too");
        assert_eq!(text(0), "0;0.0000000");
        assert_eq!(text(i32::MAX), "0;214.7483647");
        assert_eq!(text(i32::MIN), "0;-214.7483648");
    }

    #[test]
    fn a_text_cell_stays_one_cell_of_one_line() {
        assert_eq!(
            row_text(5, &[Cell::Text("a;b\nc\td\x7f"), Cell::Text("é ~")]),
            "5;a b c d ;é ~"
        );
        assert_eq!(row_text(5, &[Cell::Text("")]), "5;");
    }

    /// A log line's text under `limit`.
    fn log_text(descriptor: &str, message: &str, limit: usize) -> Vec<u8> {
        let mut line = b"kept".to_vec();
        log_line(&mut line, 884_030, descriptor, message, limit);
        line.split_off(4)
    }

    #[test]
    fn a_log_line_is_cut_to_its_limit() {
        assert_eq!(
            log_text("sequencer", "stage -> boosting", 254),
            b"884030 sequencer :: stage -> boosting"
        );
        let line = log_text("X", &"y".repeat(300), 62);
        assert_eq!(line.len(), 61, "the '\\n' the wrapper adds makes 62");
        assert!(line.starts_with(b"884030 X :: yyy"));
        let exact = log_text("X", &"y".repeat(50), 62);
        assert_eq!(exact.len(), 61, "61 bytes and the line end fit 62 exactly: no cut");
        assert_eq!(log_text("X", &"y".repeat(51), 62).len(), 61, "one more is cut");
        // a cut may land inside a UTF-8 character, as plan B cut its encoded bytes
        let cut = log_text("X", &"é".repeat(40), 16);
        assert_eq!(cut, b"884030 X :: \xc3\xa9\xc3");
    }

    #[test]
    fn a_log_line_has_no_control_bytes() {
        assert_eq!(log_text("a\tb", "one\ntwo\r\x00;", 254), b"884030 a b :: one two  ;");
        // negative: a ';' and non-ASCII are fine in a log line
        assert_eq!(log_text("Mission", "a;b é", 254), "884030 Mission :: a;b é".as_bytes());
    }

    #[test]
    fn a_session_cell_is_printable_ascii_and_short() {
        for kept in [" cc-auto ~", "", "x", &"x".repeat(32), "2026.09.23.55c494a0c659"] {
            assert_eq!(cell(kept), kept);
        }
        for emptied in ["a;b", "a\tb", "a\nb", "café", "\x7f", &"x".repeat(33)] {
            assert_eq!(cell(emptied), "", "{emptied:?}");
        }
    }

    #[test]
    fn a_label_is_letters_digits_and_dashes_with_a_letter() {
        for label in ["A-1", "x", "catapult-run3", &("run-".to_string() + &"x".repeat(28))] {
            assert!(is_label(label), "{label}");
        }
        for label in [
            "",
            "-",
            "20261003",
            "123-456",
            "catapult_run3",
            "run/3",
            "réunion",
            "run;3",
            "a b",
        ] {
            assert!(!is_label(label), "{label}");
        }
        assert!(!is_label(&("run-".to_string() + &"x".repeat(29))), "33 bytes");
    }

    #[test]
    fn a_configured_session_is_cleaned_then_judged() {
        assert_eq!(
            session_label("catapult run3"),
            SessionLabel::Valid("catapult-run3".into())
        );
        assert_eq!(
            session_label("\u{1c} run\t"),
            SessionLabel::Valid("run".into()),
            "Python's strip set"
        );
        assert_eq!(session_label("  "), SessionLabel::Unset);
        assert_eq!(session_label(""), SessionLabel::Unset);
        assert_eq!(
            session_label("catapult_run3"),
            SessionLabel::Ignored("catapult_run3".into())
        );
        assert_eq!(
            session_label("a\u{a0}b"),
            SessionLabel::Ignored("a\u{a0}b".into()),
            "only spaces become '-'"
        );
    }

    #[test]
    fn the_session_prefix_takes_the_first_it_has() {
        let label = SessionLabel::Valid("hitl-f15".into());
        assert_eq!(session_prefix(&label, Some(123), 0, 0), "hitl-f15");
        assert_eq!(session_prefix(&SessionLabel::Unset, Some(123), 0, 0), "000123");
        assert_eq!(
            session_prefix(&SessionLabel::Unset, Some(1_234_567), 0, 0),
            "1234567",
            "never truncated"
        );
        let ignored = SessionLabel::Ignored("run_3".into());
        assert_eq!(
            session_prefix(&ignored, Some(7), 0, 0),
            "000007",
            "an ignored label names nothing"
        );
        // nothing counted the boot: the date and six random digits
        assert_eq!(
            session_prefix(&SessionLabel::Unset, None, 946_684_806, 798_573),
            "20000101_000006_898573"
        );
        assert_eq!(
            session_prefix(&SessionLabel::Unset, None, 1_791_037_267, 0),
            "20261003_142107_100000"
        );
        assert_eq!(
            session_prefix(&SessionLabel::Unset, None, 0, u32::MAX),
            "19700101_000000_".to_string() + &(100_000 + u32::MAX % 900_000).to_string()
        );
    }

    /// A dated time-set row on the board `taster`, to vary per test.
    fn time_set() -> SessionRow<'static> {
        SessionRow {
            uptime_us: 17_884_029,
            boot_id: Some(123),
            session: "000123",
            epoch: 1_791_037_267,
            utc_offset: Some(-240),
            board: "taster",
            firmware: "2026.09.23.55c494a0c659",
            config_id: "3f2a91",
            source: "cc-auto",
            cc_position: Some((255_144_000, -803_918_000)),
        }
    }

    /// A session row's text.
    fn session_text(session_row: SessionRow) -> String {
        let mut line = Vec::new();
        session_row.append_to(&mut line);
        String::from_utf8(line).unwrap()
    }

    #[test]
    fn a_session_row_carries_every_column() {
        assert_eq!(
            session_text(time_set()),
            "17884029;123;000123;2026-10-03T14:21:07Z;-240;taster;2026.09.23.55c494a0c659;3f2a91;cc-auto;\
             25.5144000;-80.3918000"
        );
        let anchor = SessionRow {
            uptime_us: 60_000_117,
            utc_offset: None,
            source: "anchor",
            cc_position: None,
            ..time_set()
        };
        assert_eq!(
            session_text(anchor),
            "60000117;123;000123;2026-10-03T14:21:07Z;;taster;2026.09.23.55c494a0c659;3f2a91;anchor;;"
        );
    }

    #[test]
    fn an_unset_clock_dates_nothing() {
        for epoch in [0, 946_684_800, 978_307_199] {
            let text = session_text(SessionRow { epoch, ..time_set() });
            assert!(text.starts_with("17884029;123;000123;;;taster;"), "{epoch}: {text}");
        }
        let first = session_text(SessionRow {
            epoch: 978_307_200,
            ..time_set()
        });
        assert!(
            first.starts_with("17884029;123;000123;2001-01-01T00:00:00Z;-240;"),
            "{first}"
        );
    }

    #[test]
    fn bad_cells_are_left_empty() {
        for utc_offset in [-721, 841, i32::MAX] {
            let text = session_text(SessionRow {
                utc_offset: Some(utc_offset),
                ..time_set()
            });
            assert!(text.contains(";2026-10-03T14:21:07Z;;taster;"), "{utc_offset}: {text}");
        }
        for utc_offset in [-720, 0, 840] {
            let text = session_text(SessionRow {
                utc_offset: Some(utc_offset),
                ..time_set()
            });
            assert!(text.contains(&alloc::format!("Z;{utc_offset};taster;")), "{text}");
        }
        let text = session_text(SessionRow {
            boot_id: None,
            board: "tms;7",
            firmware: "caf\u{e9}",
            source: "cc\tauto",
            ..time_set()
        });
        assert_eq!(
            text,
            "17884029;;000123;2026-10-03T14:21:07Z;-240;;;3f2a91;;25.5144000;-80.3918000"
        );
    }

    #[test]
    fn the_status_message_is_a_python_dict() {
        let telemetry = Counters {
            count: 1,
            max: 11,
            dropped: 0,
        };
        let text = status_message("000123", &telemetry, &Counters::default(), &[]);
        assert_eq!(
            text,
            "{'session': '000123', 'tlm': {'count': 1, 'max': 11, 'dropped': 0}, \
             'log': {'count': 0, 'max': 0, 'dropped': 0}}"
        );
    }

    #[test]
    fn members_follow_the_log_in_their_order() {
        let text = status_message(
            "000123",
            &Counters::default(),
            &Counters::default(),
            &[
                ("ring", "{'erased_ahead': 380, 'formatting': False}"),
                ("mover", "None"),
            ],
        );
        assert!(text.starts_with("{'session': '000123', 'tlm': {"), "{text}");
        assert!(
            text.ends_with(
                "'log': {'count': 0, 'max': 0, 'dropped': 0}, \
                 'ring': {'erased_ahead': 380, 'formatting': False}, 'mover': None}"
            ),
            "{text}"
        );
        // negative: no member adds nothing, not even a separator
        let plain = status_message("000123", &Counters::default(), &Counters::default(), &[]);
        assert!(plain.ends_with("'dropped': 0}}") && !plain.contains("ring"), "{plain}");
    }

    #[test]
    fn the_calendar_holds_at_its_edges() {
        assert_eq!(civil(0), (1970, 1, 1, 0, 0, 0));
        assert_eq!(civil(951_782_400), (2000, 2, 29, 0, 0, 0), "a leap day of a 400th year");
        assert_eq!(civil(4_107_542_400), (2100, 3, 1, 0, 0, 0), "2100 has no 29 February");
        assert_eq!(civil(-1), (1969, 12, 31, 23, 59, 59));
    }
}
