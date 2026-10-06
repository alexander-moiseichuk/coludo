//! Coludo project, copyright under MIT license, Alexander Moiseichuk
//!
//! recorderd's demux (src/camera/recorded/recorderd.cpp `processing` and `processBuffer`): how a raw byte
//! stream becomes lines filed by routing. The Luckfox daemon is frozen, and the SD mover files a flash
//! frame's lines by the same rule (doc/specs/avionics-port.md "SD mover"), so a session pulled from either is
//! the same set of files.
//!
//! The rule, byte for byte:
//!  * every byte below 0x0A is dropped as it arrives. recorderd compares a `char`, which is unsigned on the
//!    Luckfox's ARM, so bytes from 0x80 up pass;
//!  * a `\n` ends the line, and the line keeps it;
//!  * a line starting with `@` is routed when another `@` follows at index 4 or later: the routing is the bytes
//!    between, so it is at least 3 bytes long and an `@` among its first three is kept in it. The payload is
//!    everything after that `@`, its `\n` included;
//!  * any other line goes whole to recorder.log, its leading `@` included.
//!
//! Not ported: recorderd's line buffer is 256 KiB, and a longer line wedges it for good (its loop stops
//! consuming bytes). A flash frame is at most 16 KiB, so no line comes near it.

use alloc::vec::Vec;

/// The lowest byte recorderd keeps: `\n`; everything below it is dropped.
const LINE_END: u8 = b'\n';

/// The index the routing's closing `@` must reach (recorderd's `pos > 3`): a routing of at least 3 bytes.
const ROUTING_CLOSE_MINIMUM: usize = 4;

/// Demux `stream` as recorderd does, calling `each(routing, payload)` for every complete line in order.
///
/// `routing` is None for a line recorderd writes to recorder.log, whose payload is then the whole line;
/// `payload` always ends with the line's `\n`, so it is exactly the bytes recorderd appends to the file. A line
/// that lost a byte below 0x0A is handed over without it (in a copy; any other line is borrowed from
/// `stream`).
///
/// Returns the bytes after the last `\n`, as they came: an unterminated line, which recorderd holds until the
/// next `\n` arrives, and which a whole flash frame never leaves (a torn frame does).
pub fn split(stream: &[u8], mut each: impl FnMut(Option<&[u8]>, &[u8])) -> &[u8] {
    let mut rest = stream;
    while let Some(end) = rest.iter().position(|&byte| byte == LINE_END) {
        let (line, after) = rest.split_at(end + 1);
        rest = after;
        if line.iter().any(|&byte| byte < LINE_END) {
            let kept: Vec<u8> = line.iter().copied().filter(|&byte| byte >= LINE_END).collect();
            route(&kept, &mut each);
        } else {
            route(line, &mut each);
        }
    }
    rest
}

/// File one complete line (`\n` included) by recorderd's `processBuffer` rule.
fn route(line: &[u8], each: &mut impl FnMut(Option<&[u8]>, &[u8])) {
    let close = match line.first() {
        Some(b'@') => line
            .iter()
            .skip(ROUTING_CLOSE_MINIMUM)
            .position(|&byte| byte == b'@')
            .map(|offset| offset + ROUTING_CLOSE_MINIMUM),
        _ => None,
    };
    match close {
        Some(close) => each(Some(&line[1..close]), &line[close + 1..]),
        None => each(None, line),
    }
}

#[cfg(test)]
mod tests {
    use super::split;
    use alloc::vec::Vec;

    /// One filed line: its routing (None for recorder.log) and its payload.
    type Filed = (Option<Vec<u8>>, Vec<u8>);

    /// (routing, payload) per line, and the unterminated tail.
    fn lines(stream: &[u8]) -> (Vec<Filed>, Vec<u8>) {
        let mut found = Vec::new();
        let tail = split(stream, |routing, payload| {
            found.push((routing.map(<[u8]>::to_vec), payload.to_vec()))
        });
        (found, tail.to_vec())
    }

    /// A routed (routing, payload) pair.
    fn routed(routing: &[u8], payload: &[u8]) -> Filed {
        (Some(routing.to_vec()), payload.to_vec())
    }

    /// A pair recorderd writes to recorder.log.
    fn unrouted(payload: &[u8]) -> Filed {
        (None, payload.to_vec())
    }

    #[test]
    fn routed_and_log_lines_are_filed() {
        let (found, tail) = lines(b"@000123_health.csv@{0};1;<2>\n{3};4 x :: y;<5>\n");
        assert_eq!(
            found,
            [
                routed(b"000123_health.csv", b"{0};1;<2>\n"),
                unrouted(b"{3};4 x :: y;<5>\n")
            ]
        );
        assert!(tail.is_empty());
    }

    #[test]
    fn a_routing_is_at_least_three_bytes() {
        let (found, _) = lines(b"@abc@x\n@ab@x\n@a@b@x\n@@@@@x\n@abcx\n@\n");
        assert_eq!(
            found,
            [
                routed(b"abc", b"x\n"),
                unrouted(b"@ab@x\n"),   // the '@' at index 3 is too early, and no later one follows
                routed(b"a@b", b"x\n"), // an '@' among the first three stays in the routing
                routed(b"@@@", b"x\n"), // ...even when they all are
                unrouted(b"@abcx\n"),   // never closed: the whole line, '@' included, to recorder.log
                unrouted(b"@\n"),
            ]
        );
    }

    #[test]
    fn bytes_below_the_line_end_are_dropped() {
        let (found, _) = lines(b"\x00@ab\tc@1;\x012\r\n\x09\n");
        assert_eq!(found, [routed(b"abc", b"1;2\r\n"), unrouted(b"\n")]);
        // negative: 0x0A and up are kept, the high bytes included (recorderd's char is unsigned on ARM)
        let (found, _) = lines(b"@abc@\x0b\x1f\x7f\x80\xff\n");
        assert_eq!(found, [routed(b"abc", b"\x0b\x1f\x7f\x80\xff\n")]);
    }

    #[test]
    fn an_empty_payload_is_still_written() {
        let (found, _) = lines(b"@abc@\n\n");
        assert_eq!(found, [routed(b"abc", b"\n"), unrouted(b"\n")]);
    }

    #[test]
    fn an_unterminated_tail_is_returned_untouched() {
        let (found, tail) = lines(b"@abc@1\n@abc@2;\x00\xff\xff");
        assert_eq!(found, [routed(b"abc", b"1\n")]);
        assert_eq!(tail, b"@abc@2;\x00\xff\xff", "as it came: nothing dropped from it");
        let (found, tail) = lines(b"");
        assert!(found.is_empty() && tail.is_empty());
        let (found, tail) = lines(b"no line end");
        assert!(found.is_empty());
        assert_eq!(tail, b"no line end");
    }
}
