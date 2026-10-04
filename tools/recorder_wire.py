"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The recorder line integrity wrapper -- the host reference (CPython, stdlib only). doc/specs/recorder-wire.md
defines it; this module is what every host tool verifies with, and what the board's viper implementation
is tested against.

    [@<routing>@]{OPEN};<payload>;<CLOSE>\\n
    OPEN  = crc32(the raw bytes of '[<routing>@]<payload>')   8 lowercase hex digits
    CLOSE = (~OPEN ^ U) & 0xFFFFFFFF                           U = the payload's leading decimal (uptime), else 0

Text here is a line's bytes decoded as UTF-8 with errors='surrogateescape', the codec every capture and
dump reader uses: encoded back the same way it is exactly the bytes that crossed the wire, whatever the
link did to them, so a CRC sees what the board sent.

`split()` reads one line in any of the three shapes a capture can hold -- wrapped and good, wrapped and
damaged, or a legacy line from before the wrapper. `recover()` gets back every record a damaged line still
proves: `salvage()` finds the routing a stranded record belongs to among the routings its boot is known to
use, and `pieces()` cuts a line holding several records apart. `fit()` places a recovered record's uptime
across the ticks_us wrap, and `session_files()` picks one session's files out of a dump listing.

    python3 tools/recorder_wire.py files <session> < listing   # the session's own names, one per line
"""

import binascii
import sys

GOOD: str = 'good'          # both checks pass
BAD: str = 'bad'            # carries a wrapper that does not check out (or a mangled one)
LEGACY: str = 'legacy'      # no wrapper at all: a capture from before the spec, or a host-built line
SESSION_INDEX: str = 'session.csv'  # the shared session index's routing: no session prefix, every boot
TICKS_PERIOD: int = 1 << 30  # MicroPython ticks_us wraps here (~1073.7 s ~ 17.9 min of board uptime)
_WRAPPER_BYTES: int = 22     # '{xxxxxxxx};' + ';<xxxxxxxx>': an empty payload's whole body
_MASK: int = 0xFFFFFFFF
_SESSION_STREAMS: int = 4    # another session repeats at least this many streams (session_files())


def leading_number(payload: str) -> int:
    """U: the payload's leading unsigned decimal (a row's uptime, a log line's ticks), else 0; mod 2**32."""
    digits = 0
    while digits < len(payload) and '0' <= payload[digits] <= '9':
        digits += 1
    return int(payload[:digits]) & _MASK if digits else 0


def checks(routing: str | None, payload: str) -> tuple:
    """
    (OPEN, CLOSE) as integers for `payload` under `routing`.

    Args:
        routing - the routing the line carries, or None for a log line. A line that had '@<routing>@'
            has a routing even when it is empty: '' covers '@' + payload.
        payload - the record between the two tags.

    Returns:
        (OPEN, CLOSE), each an unsigned 32-bit integer.
    """
    covered = payload if routing is None else routing + '@' + payload
    opening = binascii.crc32(covered.encode('utf-8', 'surrogateescape')) & _MASK
    return opening, (~opening ^ leading_number(payload)) & _MASK


def wrap(payload: str, routing: str | None = None) -> str:
    """
    The wire line for `payload`.

    Args:
        payload - the record to send.
        routing - the routing it goes to, or None (the default) for a log line.

    Returns:
        The line as the board sends it, with its '\\n'.
    """
    opening, closing = checks(routing, payload)
    body = '{%08x};%s;<%08x>' % (opening, payload, closing)
    return body + '\n' if routing is None else '@%s@%s\n' % (routing, body)


def unwrap(body: str) -> tuple | None:
    """
    (OPEN, payload, CLOSE) from a wrapped body `{xxxxxxxx};<payload>;<xxxxxxxx>`, or None when `body`
    does not have that shape. Both tags are read as hex; nothing is verified here.
    """
    if len(body) < _WRAPPER_BYTES or body[0] != '{' or body[9:11] != '};' or body[-11:-9] != ';<' or body[-1] != '>':
        return None
    try:
        return int(body[1:9], 16), body[11:-11], int(body[-9:-1], 16)
    except ValueError:
        return None


def verify(routing: str | None, body: str) -> str | None:
    """
    The payload of a wrapped body when it checks out.

    Args:
        routing - the routing to check it under, None for a log line.
        body - the wrapped body, `{OPEN};<payload>;<CLOSE>`.

    Returns:
        The payload when both checks pass under `routing`, else None.
    """
    parts = unwrap(body)
    if parts is None:
        return None
    opening, payload, closing = parts
    try:
        return payload if checks(routing, payload) == (opening, closing) else None
    except ValueError:  # a lone surrogate no byte decodes to (never from a surrogateescape read): damage
        return None


def split(line: str) -> tuple:
    """
    One capture line -> (status, routing, payload). `line` may carry its '\\n'. A routed line is
    `@<routing>@<body>`; anything else is a log body. LEGACY lines come back as they are, unverified; a
    BAD line keeps its routing (when it had one) and its raw body as the payload, for recover().
    """
    line = line.rstrip('\r\n')
    routing, body = None, line
    if line.startswith('@'):
        end = line.find('@', 1)
        if end > 0:
            routing, body = line[1:end], line[end + 1:]
    if not wrapped(body):
        return LEGACY, routing, body
    payload = verify(routing, body)
    return (GOOD, routing, payload) if payload is not None else (BAD, routing, body)


def _hex8(text: str) -> bool:
    """Exactly eight hex digits."""
    if len(text) != 8:
        return False
    try:
        int(text, 16)
        return True
    except ValueError:
        return False


def wrapped(body: str) -> bool:
    """
    True when `body` carries a wrapper at EITHER end -- the `{xxxxxxxx};` head or the `;<xxxxxxxx>` tail.
    A line that lost its leading '@' keeps both and must be salvaged, and a truncated one keeps the head:
    neither may pass as a legacy line. (A capture holding wrapped lines should also distrust any line
    with neither end: in a wrapped capture that is damage, not history.)
    """
    head = body.find('{')
    has_head = head >= 0 and body[head + 9:head + 11] == '};' and _hex8(body[head + 1:head + 9])
    has_tail = len(body) >= 11 and body[-11:-9] == ';<' and body[-1] == '>' and _hex8(body[-9:-1])
    return bool(has_head or has_tail)


def salvage(body: str, routings: list, prefixes: tuple) -> tuple:
    """
    The routing a damaged record proves, among those its boot is known to use.

    The record is read from its wrapper's '{' head, whatever precedes it: a lost second '@' leaves
    `@<name>{...`, a routing the Luckfox could not split, and a corrupted one leaves junk. A `<name>@`
    left in front (a lost leading '@') is tried first when the name carries one of `prefixes`: it proves
    a stream that no good row named. A name from another session is never tried, so its CRC, however
    sound, cannot file the row here. Then every known routing is tried, and the log (None) last.

    Args:
        body - the damaged line or piece, wrapper included.
        routings - every routing the boot is known to use.
        prefixes - session prefixes ('000123_') under which an unlisted `<name>@` may still prove a stream;
            () accepts known routings only. Never '', which every name carries.

    Returns:
        (routing, payload) for the routing that makes the record check out -- None for a log line -- or
        (None, None) when none does.
    """
    head = body.find('{')
    if head < 0:
        return None, None
    lead, record = body[:head], body[head:]
    candidates = list(routings) + [None]
    name = lead[:-1].lstrip('@')
    if lead.endswith('@') and name.startswith(prefixes):
        candidates.insert(0, name)
    for routing in candidates:
        payload = verify(routing, record)
        if payload is not None:
            return routing, payload
    return None, None


def pieces(body: str) -> list:
    """
    `body` cut where further records start inside it; [body] when none does.

    Two shapes put several records on one line, both from bytes the link lost. MERGED LINES lost only the
    newline between them, and are cut at each SEAM: a record's closing '>' followed by the next one's '{'
    (a log record) or '@' (a routed one). A SPLICE lost the first record's tail as well, the commonest
    damage on the 2026-10-03 dumps (~3 % of every stream's rows), so the cut is found from the next
    record's head instead: a routed head `@<routing>@{xxxxxxxx};` or a bare log head `{xxxxxxxx};`. When
    the lost run also took that record's opening '@' and part of its routing, what is left of the routing
    starts after the last ';' or '>' before it, and salvage() reads the remnant. At the very front, the
    same remnant is this record's own routing after a lost leading '@' -- not a cut. Each piece is then
    judged as a line of its own, so the whole records check out and only a truncated one is lost.
    """
    starts = set()
    seam = body.find('>')
    while 0 <= seam < len(body) - 1:
        if body[seam + 1] in ('{', '@'):
            starts.add(seam + 1)
        seam = body.find('>', seam + 1)
    head = body.find('{', 1)
    while head > 0:
        if body[head + 9:head + 11] == '};' and _hex8(body[head + 1:head + 9]):
            opening = body.rfind('@', 0, head - 1)        # the '@' that opens this record's routing
            remnant = max(body.rfind(';', 0, head), body.rfind('>', 0, head))
            if body[head - 1] != '@':
                starts.add(head)                          # a log record
            elif opening > 0 and ';' not in body[opening:head]:
                starts.add(opening)                       # a telemetry record
            elif remnant >= 0:
                starts.add(remnant + 1)                   # a routing remnant
        head = body.find('{', head + 1)
    cuts = sorted(starts)
    return [body[begin:end] for begin, end in zip([0] + cuts, cuts + [len(body)])]


def recover(routing: str | None, body: str, routings: list, prefixes: tuple) -> tuple:
    """
    Every record a damaged line still proves.

    The whole line is salvaged first: a corrupted or lost routing. Failing that, it is cut into pieces
    (merged lines, a splice), each checked under the routing it carries -- the first under the line's own
    -- and salvaged when that fails. A piece that proves nothing is lost, and so is one whose own head
    names another session.

    Args:
        routing - the routing the line arrived under (in a Luckfox dump, its file's name), None for a log line.
        body - the line's wrapped text after that routing.
        routings - every routing the boot is known to use.
        prefixes - as for salvage().

    Returns:
        ([(routing, payload, record)], lost): each proven record's routing (None for a log line), its
        payload, and the record from its '{' on, as its own file would hold it; and how many pieces were lost.
    """
    found, payload = salvage(body, routings, prefixes)
    if payload is not None:
        return [(found, payload, body[body.find('{'):])], 0
    parts = pieces(body)
    if len(parts) == 1:
        return [], 1
    records, lost = [], 0
    for number, part in enumerate(parts):
        own = routing if number == 0 else None
        if part.startswith('@') and part.find('@', 1) > 0:
            own, part = part[1:part.find('@', 1)], part[part.find('@', 1) + 1:]
        payload = verify(own, part)
        if payload is not None and own is not None and own not in routings and not own.startswith(prefixes):
            payload = None   # proven, but another session's record: not this capture's
        elif payload is None:
            own, payload = salvage(part, routings, prefixes)
        if payload is None:
            lost += 1
        else:
            records.append((own, payload, part[part.find('{'):]))
    return records, lost


def fit(recorded: int, low: int, high: int) -> int | None:
    """
    A recovered uptime unwrapped into the window its neighbours allow.

    Args:
        recorded - the uptime as recorded: raw ticks_us, below TICKS_PERIOD.
        low, high - the unwrapped bounds it must fall within, inclusive.

    Returns:
        The one recorded + k * TICKS_PERIOD (k >= 0) in [low, high]; None when there is none, or more
        than one (a window a wrap or longer cannot place a time in).
    """
    first = recorded + max(0, -((recorded - low) // TICKS_PERIOD)) * TICKS_PERIOD
    return first if first <= high < first + TICKS_PERIOD else None


def session_files(session: str, names: list) -> list:
    """
    The names in a dump listing that belong to `session`: `<session>_<stream>.csv`.

    A label may not hold '_' since doc/specs/recorder-wire.md, but an older one could, so another session's
    files can share this one's prefix: `hitl_f15_health.csv` beside `hitl_health.csv`. Under a label, the
    files under `<session>_<tag>_` are therefore left out as another session's when they look like one:
    at least _SESSION_STREAMS of them, every one repeating a stream of this session under the tag -- the
    same firmware writes the same streams. Two is not enough, and the bar is set on purpose. The Luckfox
    makes junk names by losing bytes, and a lost `imu_` turns `tms-7d_imu_bno055.csv` into
    `tms-7d_bno055.csv`; once the tails of both IMUs turn up as junk -- which a long lossy run does (a
    25-minute fuzz at 20 % damage made such tails every time) -- the real `imu_bno055` and `imu_lsm6dso32`
    look exactly like a session `imu` repeating two streams. No family of the board's streams shares a
    head word more than three times (`servo_`, `imu_`), so a tag repeating more is a session, and junk
    tails can never pass one family's real streams off as one. Dropping a real stream is silent, while
    another session's small run kept is a visibly foreign stream. A boot id and the legacy date prefix are
    digits, which no label is, so their files are all their own -- junk names included, which hold rows to
    salvage.

    Args:
        session - the session prefix: a boot id, a legacy date prefix or a label.
        names - the file names of the dump.

    Returns:
        This session's names, in listing order.
    """
    prefix = session + '_'
    rests = [name[len(prefix):] for name in names if name.startswith(prefix) and name.endswith('.csv')]
    if not any(character.isalpha() for character in session):
        return [prefix + rest for rest in rests]
    own = set(rests)
    foreign = []
    for tag in {rest[:cut] for rest in rests for cut in range(1, len(rest)) if rest[cut] == '_'}:
        under = [rest[len(tag) + 1:] for rest in rests if rest.startswith(tag + '_')]
        if len(under) >= _SESSION_STREAMS and all(stream in own for stream in under):
            foreign.append(tag + '_')
    return [prefix + rest for rest in rests if not rest.startswith(tuple(foreign))]


def main() -> None:
    """
    Command line: `files <session>` prints the session's own names among the names read on stdin, one per
    line -- the pull list of flight_pull.sh and hitl_collect.sh. The listing is read and written as bytes
    (UTF-8 with surrogateescape): the Luckfox names junk files with whatever bytes the link left, and one
    name that is not UTF-8 must not stop the pull.
    """
    if len(sys.argv) != 3 or sys.argv[1] != 'files':
        sys.exit('usage: recorder_wire.py files <session> < listing')
    listing = sys.stdin.buffer.read().decode('utf-8', 'surrogateescape').splitlines()
    for name in session_files(sys.argv[2], [line for line in listing if line]):  # a name kept whole, spaces too
        sys.stdout.buffer.write(name.encode('utf-8', 'surrogateescape') + b'\n')


if __name__ == '__main__':
    main()
