//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! The integrity wrapper every recorder line carries (doc/specs/recorder-wire.md, "Integrity wrapper"):
//!
//! ```text
//! [@<routing>@]{OPEN};<payload>;<CLOSE>\n
//! OPEN  = crc32 of the bytes '[<routing>@]<payload>'     8 lowercase hex digits
//! CLOSE = (!OPEN ^ U) mod 2^32                           U = the payload's leading decimal, else 0
//! ```
//!
//! [`wrap`] writes a line the way plan B's drain (`recorder._wrap`) and the host reference
//! (`tools/recorder_wire.py` `wrap`) both do; [`verify`], [`unwrap`], [`wrapped`] and [`classify`] read one
//! back as the host reference does (its `split` is [`classify`] here, since [`crate::split`] is recorderd's
//! demux). Everything works on bytes: the host reference decodes a line with `surrogateescape`, whose
//! round trip is exactly these bytes, and every index it compares at is an ASCII character's.
//!
//! The offline salvage of the host reference (`salvage`, `pieces`, `recover`, `fit`) is not ported: the board
//! never reads a wrapped line back. The ring_saver prints every line itself from a record of a chunk whose CRC
//! checked (doc/specs/avionics-port.md "The ring_saver"), and a cut tears a chunk whole, failing its CRC, not
//! the UART's lost bytes those rules repair.

use crate::crc::crc32;
use alloc::vec::Vec;

/// The bytes the wrapper adds around a payload, `{xxxxxxxx};` and `;<xxxxxxxx>`: an empty payload's whole body.
pub const WRAPPER_BYTES: usize = 22;

/// The wrapper's lowercase hex digits, indexed by nibble.
const HEX_DIGITS: &[u8; 16] = b"0123456789abcdef";

/// U: the payload's leading unsigned decimal (a row's uptime, a log line's microseconds), mod 2^32; 0 when
/// the payload does not start with a digit (a CSV header).
///
/// The digits wrap as they are read, so any length works, a u64 uptime and longer included: the same value
/// as the host reference's `int(digits) & 0xFFFFFFFF` and plan B's 32-bit viper register. A Rust uptime has
/// no 2^30 wrap of its own, so U is simply the uptime mod 2^32.
pub fn leading_number(payload: &[u8]) -> u32 {
    payload
        .iter()
        .take_while(|byte| byte.is_ascii_digit())
        .fold(0u32, |number, &digit| {
            number.wrapping_mul(10).wrapping_add(u32::from(digit - b'0'))
        })
}

/// (OPEN, CLOSE) for `payload` under `routing`: what a wrapped line must carry to verify.
///
/// `routing` is None for a log line, whose OPEN covers the payload alone. A line that had `@<routing>@` has a
/// routing even when it is empty: OPEN then covers `@` + payload.
pub fn checks(routing: Option<&[u8]>, payload: &[u8]) -> (u32, u32) {
    let open = match routing {
        Some(routing) => crc32(crc32(crc32(0, routing), b"@"), payload),
        None => crc32(0, payload),
    };
    (open, !open ^ leading_number(payload))
}

/// The wire line for `payload` under `routing` (None for a log line), with its `\n`.
///
/// `payload` is the record without a line end. It must not hold a `\n` (that splits the line) nor a byte below
/// 0x0A (recorderd and [`crate::split`] drop those, so the line fails its check): the row and log text of this crate
/// never does.
pub fn wrap(routing: Option<&str>, payload: &[u8]) -> Vec<u8> {
    let routing_bytes = routing.map_or(0, |routing| routing.len() + 2);
    let mut line = Vec::with_capacity(routing_bytes + payload.len() + WRAPPER_BYTES + 1);
    wrap_into(&mut line, routing, payload);
    line
}

/// [`wrap`], appended to `line`: the ring_saver's batch buffer for a file takes one line after another without a
/// copy.
pub fn wrap_into(line: &mut Vec<u8>, routing: Option<&str>, payload: &[u8]) {
    let (open, close) = checks(routing.map(str::as_bytes), payload);
    if let Some(routing) = routing {
        line.push(b'@');
        line.extend_from_slice(routing.as_bytes());
        line.push(b'@');
    }
    line.push(b'{');
    push_hex(line, open);
    line.extend_from_slice(b"};");
    line.extend_from_slice(payload);
    line.extend_from_slice(b";<");
    push_hex(line, close);
    line.extend_from_slice(b">\n");
}

/// Append `value` as 8 lowercase hex digits, most significant first.
fn push_hex(line: &mut Vec<u8>, value: u32) {
    for shift in (0..32).step_by(4).rev() {
        line.push(HEX_DIGITS[((value >> shift) & 0xF) as usize]);
    }
}

/// The value of a tag of exactly 8 ASCII hex digits, either case; None for anything else. [`unwrap`], and so
/// [`verify`], read tags this way.
///
/// Stricter than the host reference, which reads a tag with Python's `int(tag, 16)`: that also takes
/// surrounding whitespace, a sign, a `0x` prefix, underscores between digits and non-ASCII digits. Each is
/// damage to a tag the board wrote as 8 lowercase digits, so here it fails the line. The host then verifies a
/// few damaged lines this does not, such as a leading `0` flipped to a space (one bit): the fixture marks them.
/// Only that GOOD-to-BAD verdict differs: [`wrapped`] reads tags as leniently as the host, so a damaged line is
/// never taken for a legacy one here when the host calls it BAD.
fn hex_tag(tag: &[u8]) -> Option<u32> {
    if tag.len() != 8 {
        return None;
    }
    tag.iter().try_fold(0u32, |value, &digit| {
        Some((value << 4) | char::from(digit).to_digit(16)?)
    })
}

/// Whether the host reference's `_hex8` takes `tag`: 8 bytes that Python's `int(tag, 16)` reads as a number.
///
/// That is ASCII whitespace (space, `\t`, `\n`, `\v`, `\f`, `\r`) around one optional sign, an optional
/// `0x` or `0X` prefix with at most one `_` after it, then hex digits with single `_` between them.
/// [`wrapped`] reads a tag this way, so a line that keeps only a damaged tag is wrapper evidence here exactly
/// when it is to the host. Leniency only matters there, where it errs toward calling damage BAD.
///
/// Not mirrored: Python also takes non-ASCII decimal digits and spaces (`int` folds them to ASCII), and slices
/// by characters, not bytes. A tag holding one needs a whole multi-byte UTF-8 character in place of an ASCII
/// byte, which no flipped bit or cut produces; any byte from 0x80 up fails here.
fn host_reads_as_hex(tag: &[u8]) -> bool {
    let is_space = |byte: &u8| matches!(byte, b' ' | b'\t' | b'\n' | 0x0B | 0x0C | b'\r');
    if tag.len() != 8 {
        return false;
    }
    let start = tag.iter().position(|byte| !is_space(byte)).unwrap_or(tag.len());
    let end = tag
        .iter()
        .rposition(|byte| !is_space(byte))
        .map_or(start, |last| last + 1);
    let mut number = &tag[start..end];
    if let [b'+' | b'-', rest @ ..] = number {
        number = rest;
    }
    if let [b'0', b'x' | b'X', rest @ ..] = number {
        number = rest.strip_prefix(b"_").unwrap_or(rest);
    }
    // an empty group is a leading, trailing or doubled '_', or no digit at all: int() refuses each
    number
        .split(|&byte| byte == b'_')
        .all(|group| !group.is_empty() && group.iter().all(u8::is_ascii_hexdigit))
}

/// The three parts of a wrapped body `{OPEN};<payload>;<CLOSE>`, read but not verified.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Wrapper<'a> {
    /// The OPEN tag's value.
    pub open: u32,
    /// The record between the two tags.
    pub payload: &'a [u8],
    /// The CLOSE tag's value.
    pub close: u32,
}

/// The parts of a wrapped body `{xxxxxxxx};<payload>;<xxxxxxxx>`, or None when `body` does not have that shape
/// (the host reference's `unwrap`). Nothing is verified here.
pub fn unwrap(body: &[u8]) -> Option<Wrapper<'_>> {
    let length = body.len();
    if length < WRAPPER_BYTES
        || body[0] != b'{'
        || &body[9..11] != b"};"
        || &body[length - 11..length - 9] != b";<"
        || body[length - 1] != b'>'
    {
        return None;
    }
    Some(Wrapper {
        open: hex_tag(&body[1..9])?,
        payload: &body[11..length - 11],
        close: hex_tag(&body[length - 9..length - 1])?,
    })
}

/// The payload of a wrapped body when both checks pass under `routing` (None for a log line), else None.
pub fn verify<'a>(routing: Option<&[u8]>, body: &'a [u8]) -> Option<&'a [u8]> {
    let wrapper = unwrap(body)?;
    (checks(routing, wrapper.payload) == (wrapper.open, wrapper.close)).then_some(wrapper.payload)
}

/// Whether `body` carries a wrapper at EITHER end: the `{xxxxxxxx};` head at its first `{`, or the
/// `;<xxxxxxxx>` tail (the host reference's `wrapped`).
///
/// A line that lost its leading `@` keeps both ends, and a truncated one keeps the head: neither may pass as a
/// legacy line from before the wrapper. A tag counts when the host's `int(tag, 16)` reads it
/// (`host_reads_as_hex`), not only when [`unwrap`] does: a head damaged to `{cb34fe6 };` on a line that also
/// lost its tail's `>` is BAD on both sides, never LEGACY here.
pub fn wrapped(body: &[u8]) -> bool {
    let has_head = body.iter().position(|&byte| byte == b'{').is_some_and(|head| {
        body.get(head + 9..head + 11) == Some(b"};".as_slice())
            && body.get(head + 1..head + 9).is_some_and(host_reads_as_hex)
    });
    let length = body.len();
    let has_tail = length >= 11
        && &body[length - 11..length - 9] == b";<"
        && body[length - 1] == b'>'
        && host_reads_as_hex(&body[length - 9..length - 1]);
    has_head || has_tail
}

/// What a line read back proves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    /// Both checks pass.
    Good,
    /// It carries a wrapper (or part of one) that does not check out: damage.
    Bad,
    /// No wrapper at all: a capture from before the wrapper, or a host-built line.
    Legacy,
}

/// One line read back: its status, its routing and its payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Line<'a> {
    /// What the line proves.
    pub status: Status,
    /// The routing between the line's leading `@` and the next `@`; None for a log line.
    pub routing: Option<&'a [u8]>,
    /// The verified payload when Good; the body after the routing as it came when Bad or Legacy.
    pub payload: &'a [u8],
}

/// Read one line in any of the three shapes a capture holds: wrapped and good, wrapped and damaged, or legacy
/// (the host reference's `split`).
///
/// `line` may carry its line end: every trailing `\r` and `\n` is dropped first. A line starting with `@`
/// routes up to the next `@` wherever it is, an empty routing included (recorderd's own rule differs for
/// routings under 3 bytes, which the board never writes: [`crate::split`]).
pub fn classify(line: &[u8]) -> Line<'_> {
    let end = line
        .iter()
        .rposition(|&byte| byte != b'\r' && byte != b'\n')
        .map_or(0, |last| last + 1);
    let line = &line[..end];
    let (routing, body) = match line.split_first() {
        Some((b'@', rest)) => match rest.iter().position(|&byte| byte == b'@') {
            Some(close) => (Some(&rest[..close]), &rest[close + 1..]),
            None => (None, line),
        },
        _ => (None, line),
    };
    let (status, payload) = if !wrapped(body) {
        (Status::Legacy, body)
    } else {
        match verify(routing, body) {
            Some(payload) => (Status::Good, payload),
            None => (Status::Bad, body),
        }
    };
    Line {
        status,
        routing,
        payload,
    }
}

#[cfg(test)]
mod tests {
    use super::{Line, Status, WRAPPER_BYTES, Wrapper, checks, classify, leading_number, unwrap, verify, wrap};
    use super::{host_reads_as_hex, wrap_into, wrapped};
    use alloc::vec::Vec;

    /// test_recorder.py's first routed vector, as the board must write it.
    const ROW: &[u8] = b"@000123_imu_lsm6dso32.csv@{cb34fe60};884029;0.98;0.05;0.01;<34c67ca2>\n";

    #[test]
    fn a_row_wraps_as_plan_b_writes_it() {
        assert_eq!(wrap(Some("000123_imu_lsm6dso32.csv"), b"884029;0.98;0.05;0.01"), ROW);
        assert_eq!(
            wrap(None, b"123456789"),
            b"{cbf43926};123456789;<33500bcc>\n",
            "the CRC-32 check value"
        );
    }

    #[test]
    fn wrap_into_appends() {
        let mut frame = b"x".to_vec();
        wrap_into(&mut frame, None, b"boot :: no digits here");
        assert_eq!(frame, b"x{7afd6024};boot :: no digits here;<85029fdb>\n");
    }

    #[test]
    fn the_leading_number_wraps_mod_2_32() {
        assert_eq!(leading_number(b"884029;0.98"), 884_029);
        assert_eq!(leading_number(b"4294967295;0"), u32::MAX);
        assert_eq!(leading_number(b"4294967296;0"), 0);
        assert_eq!(
            leading_number(b"18446744073709551615;0"),
            u32::MAX,
            "u64::MAX is 2^32 - 1 mod 2^32"
        );
        assert_eq!(
            leading_number(b"99999999999 x :: y"),
            (99_999_999_999u64 % (1 << 32)) as u32
        );
        // negative: no leading digit is U = 0, so a header closes with plain !OPEN
        assert_eq!(leading_number(b"uptime;ax"), 0);
        assert_eq!(leading_number(b""), 0);
        assert_eq!(leading_number(b"-5;x"), 0);
        let (open, close) = checks(None, b"uptime;ax");
        assert_eq!(close, !open);
    }

    #[test]
    fn a_wrapped_line_verifies_under_its_own_routing_only() {
        let body = &ROW[26..ROW.len() - 1];
        assert_eq!(
            verify(Some(b"000123_imu_lsm6dso32.csv"), body),
            Some(b"884029;0.98;0.05;0.01".as_slice())
        );
        assert_eq!(verify(Some(b"000123_health.csv"), body), None, "another file's name");
        assert_eq!(verify(None, body), None, "as a log line");
        assert_eq!(verify(Some(b""), body), None, "an empty routing is a routing");
    }

    #[test]
    fn unwrap_reads_the_parts_and_refuses_other_shapes() {
        let body = b"{cbf43926};123456789;<33500bcc>";
        assert_eq!(
            unwrap(body),
            Some(Wrapper {
                open: 0xCBF4_3926,
                payload: b"123456789",
                close: 0x3350_0BCC
            })
        );
        assert_eq!(
            unwrap(b"{CBF43926};123456789;<33500BCC>").map(|parts| parts.open),
            Some(0xCBF4_3926)
        );
        let empty = b"{00000000};;<ffffffff>";
        assert_eq!(empty.len(), WRAPPER_BYTES);
        assert_eq!(unwrap(empty).map(|parts| parts.payload), Some(b"".as_slice()));
        // negative: one byte short, a missing brace, a non-hex digit, and the host's lenient forms
        assert_eq!(unwrap(&empty[1..]), None);
        assert_eq!(unwrap(b"cbf43926};123456789;<33500bcc>"), None);
        assert_eq!(unwrap(b"{cbf4392g};123456789;<33500bcc>"), None);
        assert_eq!(
            unwrap(b"{ bf43926};123456789;<33500bcc>"),
            None,
            "Python's int() strips the space"
        );
        assert_eq!(unwrap(b"{+bf43926};123456789;<33500bcc>"), None, "and takes a sign");
        assert_eq!(unwrap(b"{0x123456};123456789;<33500bcc>"), None, "and a 0x prefix");
        assert_eq!(unwrap(b"{cbf4_926};123456789;<33500bcc>"), None, "and underscores");
    }

    #[test]
    fn a_wrapper_at_either_end_counts() {
        assert!(wrapped(b"{cbf43926};123456789;<33500bcc>"));
        assert!(
            wrapped(b"000123_imu.csv@{cb34fe60};884029;0.98"),
            "a head after a lost '@'"
        );
        assert!(wrapped(b"884029;0.98;<34c67ca2>"), "a tail");
        // negative: a brace that is no head, a tail that is no tail, nothing at all
        assert!(!wrapped(b"x{not hex};<34c67cag>"));
        assert!(!wrapped(b"884029;0.98;0.05"));
        assert!(!wrapped(b""));
        assert!(!wrapped(b"{cbf43926}"), "a head needs the two bytes after its tag");
    }

    #[test]
    fn a_tag_counts_as_wrapper_evidence_when_python_reads_it() {
        for tag in [
            b"cb34fe60".as_slice(),
            b"CB34FE60",
            b"cb34fe6 ",
            b" b34fe60",
            b"\tb34fe6\r",
            b"\x0b\x0c34fe60",
            b"+b34fe60",
            b"-b34fe60",
            b"0x34fe60",
            b"-0X_4fe6",
            b"cb3_4fe6",
            b"c_b_3_4f",
            b"   0    ",
        ] {
            assert!(host_reads_as_hex(tag), "{}", tag.escape_ascii());
        }
        // negative: what int(tag, 16) refuses, and any length but 8
        for tag in [
            b"cb34fe6g".as_slice(),
            b"        ",
            b"+ b34fe6",
            b"+-b34fe6",
            b"++b34fe6",
            b"0x      ",
            b"0x_     ",
            b"0x__4fe6",
            b"_b34fe60",
            b"cb34fe6_",
            b"cb3__fe6",
            b"cb3 4fe6",
            b"\x1cb34fe6",
            b"cb34fe6\x00",
            b"cb34fe6\xff",
            b"x0b34fe6",
            b"0xx4fe60",
            b"cb34fe6",
            b"cb34fe600",
        ] {
            assert!(!host_reads_as_hex(tag), "{}", tag.escape_ascii());
        }
        // not mirrored: Python folds a non-ASCII digit (U+0660, two bytes) to '0'; here it is two foreign bytes
        assert!(!host_reads_as_hex("\u{660}b34fe6".as_bytes()));
    }

    #[test]
    fn a_damaged_tag_alone_is_damage_not_legacy() {
        // the head's last digit made a space and the tail's '>' lost: only the lenient head is left
        let line = b"@000123_imu_lsm6dso32.csv@{cb34fe6 };884029;0.98;0.05;0.01;<34c67ca2\n";
        assert_eq!(classify(line).status, Status::Bad);
        assert_eq!(unwrap(&line[26..line.len() - 1]), None, "unwrap stays strict");
        assert_eq!(classify(b"884029;0.98;<+4c67ca2>").status, Status::Bad, "a signed tail");
        // negative: a head and a tail int() refuses are no wrapper
        assert_eq!(classify(b"@a.csv@{cb3 4fe6};884029;0.98").status, Status::Legacy);
        assert_eq!(classify(b"884029;0.98;<34c6_ca_>").status, Status::Legacy);
    }

    #[test]
    fn classify_tells_good_bad_and_legacy() {
        assert_eq!(
            classify(ROW),
            Line {
                status: Status::Good,
                routing: Some(b"000123_imu_lsm6dso32.csv"),
                payload: b"884029;0.98;0.05;0.01"
            }
        );
        let mut damaged = ROW.to_vec();
        damaged[40] ^= 1;
        let line = classify(&damaged);
        assert_eq!(
            (line.status, line.routing),
            (Status::Bad, Some(b"000123_imu_lsm6dso32.csv".as_slice()))
        );
        assert_eq!(
            line.payload,
            &damaged[26..damaged.len() - 1],
            "a bad line keeps its raw body"
        );
        let legacy = classify(b"@a.csv@884029;0.98\r\n");
        assert_eq!(
            (legacy.status, legacy.payload),
            (Status::Legacy, b"884029;0.98".as_slice())
        );
        assert_eq!(classify(b"").status, Status::Legacy);
        let lost = classify(&ROW[1..]);
        assert_eq!(
            (lost.status, lost.routing),
            (Status::Bad, None),
            "a lost '@' is damage, not legacy"
        );
    }

    #[test]
    fn an_empty_routing_is_still_a_routing() {
        let line: Vec<u8> = wrap(Some(""), b"7;x");
        assert!(line.starts_with(b"@@{"));
        let read = classify(&line);
        assert_eq!(
            (read.status, read.routing, read.payload),
            (Status::Good, Some(b"".as_slice()), b"7;x".as_slice())
        );
    }
}
