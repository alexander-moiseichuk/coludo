"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The single non-hot data path: telemetry + logs into PSRAM ring buffers, drained to the Luckfox
recorder over UART. See doc/specs/coludo.md ('Task Data-Flow', 'Logging', 'Telemetry', 'Storage Write
Constraints').

Recorder is a singleton: any module calls Recorder.log() / Recorder.tlm() globally. Producers enqueue
synchronously (struct.pack_into into a ring -- never slice-assignment, which is O(buffer length) on
this port); the async run() loop drains the rings to the UART via an asyncio.StreamWriter, telemetry
(first priority) before logs. Logs are best-effort (dropped when full); telemetry is important (raises
if a record will not fit).

That trade continues on the RECORDER side, deliberately -- logging is what is spent to make telemetry
trustworthy, so the two channels have different guarantees end to end:
  * TELEMETRY is committed PER LINE, so a row that reached the link is on disk. That is what makes a
    capture survive a crash mid-flight, and why anything that must be evidence belongs in tlm().
  * LOGS are buffered and flushed roughly every 1000 telemetry messages, so a SHORT session can end
    with log lines that were never written out. A log line is a convenience, never evidence -- do not
    reason about a flight from one, and do not put a value there that a capture needs.
  * SETUP-TIME messages reach NEITHER: drivers set up before the recorder task, so the log ring is
    empty and the line is discarded. Use print() there -- the only channel that early (measured: an
    sdp810 setup line never reached recorder.log, while print() shows on the console at boot).

ON THE WIRE every line carries an integrity wrapper, `[@<routing>@]{OPEN};<payload>;<CLOSE>\\n`
(doc/specs/recorder-wire.md; tools/recorder_wire.py is the host reference). It is added at DRAIN time
on the UART path only: the rings and the CC tee keep the raw line. Files are named `<session>_<stream>`
with the session from config, else this boot's NVS boot id.

EVERY BOOT IS LISTED in the shared `session.csv` (index_session()), whether or not anyone sets its clock:
a 'boot' row as run() starts, a row per successful time set (Mission, source 'cc-auto' / 'dashboard'),
and one 'anchor' row a minute in (_anchor). The header goes out again before the boot and anchor rows,
since the first may have reached nobody. While the RTC reads before 2001 the clock was never set, so
utc and utc_offset are empty on every kind of row and the boot id alone places the boot.
"""

import array
import asyncio
import random
import struct
import time

import config as config_module
import inspector

try:
    import micropython
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const, micropython  # the shim: @viper degrades to plain Python
    ptr8 = ptr32 = uint = int  # dummies so the shim'd @viper annotations and casts evaluate off-board

try:
    from machine import UART
except ImportError:  # host (CPython): board-only; the Luckfox UART is opened only on the board
    UART = None


_DEFAULT_CELL_SIZE = const(256)  # bytes per ring cell (record + 2-byte length header)
_DEFAULT_CAPACITY = const(1024)  # cells per ring
_DEFAULT_TXBUF = const(4096)     # UART TX ring; the 256-byte default silently truncates (see setup()).
                                 # 4096 rather than more because back-pressure is held at the RING
                                 # level below -- the buffer only has to absorb one high-water burst,
                                 # not a whole ring pass, and RAM is scarce on this board.
_DRAIN_HIGH_WATER = const(2048)  # half of _DEFAULT_TXBUF: flush when the pending bytes reach it
_LENGTH_BYTES = const(2)  # uint16 record-length header
_STATS_PERIOD_MS = const(1000)  # how often run() logs a buffer-stats line
_DEFAULT_TELEMETRY_MS = const(20)  # CODE fallback only (50 Hz). The SHIPPED config sets 0 = uncapped
                                   # (config_default recorder.telemetry_ms, and every launches/*/*/*.config),
                                   # so this applies only to a config that omits the recorder section.
# CONFIG knobs are milliseconds everywhere -- one unit in the file, converted to us at the boundary,
# so no reader has to remember which of two suffixes a given key used.
_WRAPPER_BYTES = const(22)  # '{xxxxxxxx};' + ';<xxxxxxxx>' the drain adds around each payload
_CRC32_POLYNOMIAL: int = 0xEDB88320  # reflected IEEE 802.3, the zlib / binascii one (not const: above 2**30)
_HEX_DIGITS: bytes = b'0123456789abcdef'  # the wrapper's lowercase hex, indexed by nibble
_SESSION_INDEX: str = 'session.csv'  # the shared index every boot appends to: routed bare, no session prefix
_SESSION_INDEX_HEADER: str = 'uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon'
_ANCHOR_AFTER_MS = const(60000)  # the 'anchor' row waits a minute: the Luckfox listens ~29 s after power-on
_CLOCK_SET_S = const(31622400)  # 2001-01-01 in the RTC's 2000 epoch (Unix 978307200): a clock this late was set
_LABEL_MAX = const(32)  # a `recorder.session` label's longest: it routes every line, so it must leave the cell room
_CELL_MAX = const(32)  # a session.csv text cell's longest: board, firmware and source together must fit one cell


class _RecorderError(ValueError):
    """Raised when an important (telemetry) record cannot be queued."""


def _cell(text: str) -> str:
    """
    A free-text session.csv cell: kept only when it is printable ASCII (0x20-0x7E) without ';', and at
    most _CELL_MAX characters.

    A ';' would split the cell, a '\\n' the line, and the Luckfox discards bytes below 0x0A, so a TAB
    would turn the row's CRC BAD. Non-ASCII has no business in a file the host splits by bytes. A long
    cell would push the whole row past the 254-byte ring cell, and the row would be lost as if the ring
    were full; the firmware stamp, the longest real value, is 23 characters.

    Args:
        text - the cell's value; it comes from JSON or a config, so a non-string (a number, None) is
            possible and is not text.

    Returns:
        `text` unchanged, or '' when it is not a string, is longer, or holds any other byte.
    """
    if not isinstance(text, str) or len(text) > _CELL_MAX:
        return ''
    for byte in text.encode():
        if byte < 0x20 or byte > 0x7E or byte == 0x3B:  # 0x3B is ';'
            return ''
    return text


def _is_label(text: str) -> bool:
    """
    Whether a `recorder.session` label is usable: ASCII letters, digits and '-', at least one letter,
    _LABEL_MAX characters at most.

    The letter keeps a label from looking like a boot id ('000123'); the missing '_' keeps
    `<session>_<stream>.csv` splittable at its first '_' (doc/specs/recorder-wire.md). The cap keeps the
    routing it heads on every telemetry line short: an unbounded label overflows the 254-byte cell, and
    then every tlm() raises.

    Args:
        text - the label, already stripped and with spaces made '-'.

    Returns:
        True when usable; False otherwise (an empty or over-long label included).
    """
    if len(text) > _LABEL_MAX:
        return False
    letter = False
    for byte in text.encode():
        if 65 <= byte <= 90 or 97 <= byte <= 122:  # 'A'..'Z', 'a'..'z'
            letter = True
        elif not (48 <= byte <= 57 or byte == 45):  # '0'..'9', '-'
            return False
    return letter


def _crc_table() -> array.array:
    """
    The CRC-32 lookup table: 256 uint32 entries for the reflected polynomial, as zlib builds it.

    Built ONCE, at setup with the GC on: most entries are above 2**30, i.e. boxed ints on this port, so
    the per-line work reads them through a viper pointer instead of doing this arithmetic per byte.

    Args:
        (none)

    Returns:
        array('I') of 256 entries.
    """
    table = array.array('I', range(256))
    for index in range(256):
        value = index
        for _bit in range(8):
            value = (value >> 1) ^ _CRC32_POLYNOMIAL if value & 1 else value >> 1
        table[index] = value
    return table


@micropython.viper
def _copy(source: ptr8, start: int, destination: ptr8, size: int) -> int:
    """
    Copy `size` bytes from `source[start:]` to the head of `destination`, allocating nothing.

    A slice would allocate its copy, and a memoryview over the source allocates the view -- per record,
    with the GC off. Viper does no bounds checks: the caller guarantees both ranges.

    Args:
        source - the buffer to copy from (ptr8).
        start - the first source byte.
        destination - the buffer to copy into (ptr8), written from index 0.
        size - how many bytes.

    Returns:
        size.
    """
    for index in range(size):
        destination[index] = source[start + index]
    return size


@micropython.viper
def _wrap(record: ptr8, size: int, line: ptr8, capacity: int, table: ptr32, digits: ptr8) -> int:
    """
    Wrap one raw record for the wire: `[@<routing>@]<payload>\\n` -> `[@<routing>@]{OPEN};<payload>;<CLOSE>\\n`.

    OPEN is the CRC-32 of `<routing>@<payload>` (the payload alone for a log line); CLOSE is ~OPEN ^ U,
    U the payload's leading decimal mod 2**32, 0 when it does not start with a digit (a CSV header).

    VIPER AND ALLOCATION-FREE, because the drain runs it per line with the GC off from BOOSTING to DONE:
    binascii.crc32 returns a boxed int above 2**30 and '%08x' allocates its string. Here the register, U
    and the hex digits stay machine words written straight into `line`. Two identities keep every value
    inside 32 bits with no wide literal (which viper would load as an object): OPEN is the inverted
    register, so its digits are the register's nibbles XOR 15; and CLOSE = ~OPEN ^ U is register ^ U.
    The same code is correct under the CPython shim, which is how the host checks it.

    Args:
        record - the raw record (ptr8); one trailing '\n' is its line end, and a missing one is added.
        size - the record's length in bytes.
        line - the output buffer (ptr8).
        capacity - the output buffer's length; nothing is written past it.
        table - the CRC-32 table (ptr32, _crc_table()).
        digits - the 16 lowercase hex digits (ptr8).

    Returns:
        The wrapped line's length, '\n' included; -1 when it would not fit `capacity` (nothing written).
    """
    end = size
    if end > 0 and record[end - 1] == 10:  # the record's own '\n': CLOSE goes before it
        end -= 1
    if end + _WRAPPER_BYTES + 1 > capacity:  # the wrapper + '\n'
        return -1
    payload = 0  # the first payload byte: past the routing's closing '@', or 0 for a log line
    if end > 0 and record[0] == 64:  # '@' opens a routing when a second '@' closes it
        index = 1
        while index < end and record[index] != 64:
            index += 1
        if index < end:
            payload = index + 1
    covered = 1 if payload else 0  # OPEN covers everything but a routing's leading '@'
    crc = uint(0xFFFF) << 16 | uint(0xFFFF)  # the register's all-ones start
    for index in range(covered, end):
        crc = uint(table[(crc ^ record[index]) & 0xFF]) ^ ((crc >> 8) & 0xFFFFFF)
    number = uint(0)  # U, wrapping mod 2**32 on this 32-bit machine word
    index = payload
    while index < end and record[index] >= 48 and record[index] <= 57:  # '0'..'9'
        number = number * 10 + uint(record[index] - 48)
        index += 1
    for index in range(payload):  # the routing, both '@' included, goes out as it came
        line[index] = record[index]
    out = payload
    line[out] = 123  # '{'
    out += 1
    shift = 28
    while shift >= 0:  # OPEN = ~register
        line[out] = digits[((crc >> shift) & 15) ^ 15]
        out += 1
        shift -= 4
    line[out] = 125  # '}'
    line[out + 1] = 59  # ';'
    out += 2
    for index in range(payload, end):
        line[out] = record[index]
        out += 1
    line[out] = 59  # ';'
    line[out + 1] = 60  # '<'
    out += 2
    close = crc ^ number  # CLOSE = ~OPEN ^ U
    shift = 28
    while shift >= 0:
        line[out] = digits[(close >> shift) & 15]
        out += 1
        shift -= 4
    line[out] = 62  # '>'
    line[out + 1] = 10  # '\n'
    return out + 2


class Ring:
    """
    Lock-free single-producer / single-consumer byte ring.

    The writer owns `head`, the reader owns `tail`; they never touch the same field, so it is safe
    between an ISR producer and a task consumer with no locks. Each cell holds <uint16 length>
    <payload>. write() uses pack_into (cost O(record)) and returns False if there is no room (the
    record is skipped, never overwriting unread data). read() returns a bytes copy (stable across an
    await). Holds `capacity - 1` records (one cell separates full from empty).
    """

    def __init__(self, capacity: int = _DEFAULT_CAPACITY, cell_size: int = _DEFAULT_CELL_SIZE):
        self.capacity: int = capacity or _DEFAULT_CAPACITY
        self.cell_size: int = cell_size or _DEFAULT_CELL_SIZE
        self.max_payload: int = self.cell_size - _LENGTH_BYTES
        self.storage: bytearray = bytearray(self.capacity * self.cell_size)
        self.head: int = 0  # writer-owned: next cell to write
        self.tail: int = 0  # reader-owned: next cell to read
        self.dropped: int = 0  # writer-owned: records skipped (too big, or full)

    def write(self, data: bytes) -> bool:
        size = len(data)
        if size > self.max_payload:
            self.dropped += 1
            return False
        head = self.head
        nxt = head + 1
        if nxt == self.capacity:
            nxt = 0
        if nxt == self.tail:  # full -> skip, do not overwrite unread data
            self.dropped += 1
            return False
        struct.pack_into('<H%ds' % size, self.storage, head * self.cell_size, size, data)
        self.head = nxt  # publish only after the record is written
        return True

    def read(self) -> bytes:
        """Return the oldest record as bytes (a copy) and advance, or None if empty."""
        tail = self.tail
        if self.head == tail:
            return None
        offset = tail * self.cell_size
        size = struct.unpack_from('<H', self.storage, offset)[0]
        record = bytes(self.storage[offset + _LENGTH_BYTES : offset + _LENGTH_BYTES + size])
        nxt = tail + 1
        self.tail = 0 if nxt == self.capacity else nxt
        return record

    def readinto(self, buffer: bytearray) -> int:
        """
        Copy the oldest record to the head of `buffer` and advance: read() without its allocations.

        read() builds a bytes copy, a slice and the tuple unpack_from returns -- per record, and the
        drain runs with the GC off from BOOSTING to DONE. Here the length is two byte reads
        (little-endian, as write() packs it) and the copy is a viper loop into the caller's buffer.

        Args:
            buffer - a bytearray at least as long as the record (max_payload covers every record).

        Returns:
            The record's length, or -1 when the ring is empty.

        Raises:
            ValueError - when the record is longer than `buffer`; it stays queued.
        """
        tail = self.tail
        if self.head == tail:
            return -1
        offset = tail * self.cell_size
        storage = self.storage
        size = storage[offset] | storage[offset + 1] << 8
        if size > len(buffer):
            raise ValueError('record of %d bytes > buffer of %d' % (size, len(buffer)))
        _copy(storage, offset + _LENGTH_BYTES, buffer, size)
        nxt = tail + 1
        self.tail = 0 if nxt == self.capacity else nxt
        return size

    def discard(self) -> None:
        """
        Drop every queued record without reading it -- O(1), zero allocation.

        For the case where the consumer has gone away and the buffered records are worthless (a lapsed
        CC tee window). Draining with read() would allocate a bytes copy per record; this just moves
        the reader forward. Safe from either side here because MicroPython's asyncio is cooperative
        and no await separates the two: nothing can interleave between reading `head` and storing it.
        """
        self.tail = self.head

    def count(self) -> int:
        """Records currently queued (a stats snapshot)."""
        delta = self.head - self.tail
        return delta if delta >= 0 else delta + self.capacity


class _TeeSink:
    """
    A poll-model CC mirror of a recorder stream (`log` and `tlm` both use one).

    While a deadline is armed, tee() copies each record into a bounded ring; drain(ms) returns the
    batch buffered since the last call and re-arms for `ms` more (<= 0 stops). The ring is lazily
    sized from the first window (~10 records/ms, capped) and reused. If no follow-up arrives before
    the window lapses, the next tee() discards the buffer and disables -- a lost link cannot grow
    memory. The UART sink is never touched; the tee is best-effort (a full ring drops, never raises).
    """

    def __init__(self):
        self._ring: Ring = None
        self._deadline: int = 0  # ticks_us window-end (0 = off)
        self._cell: int = _DEFAULT_CELL_SIZE

    def reset(self, cell_size: int) -> None:
        self._ring = None
        self._deadline = 0
        self._cell = cell_size

    def tee(self, data: bytes) -> None:
        if not self._deadline:
            return
        if time.ticks_diff(self._deadline, time.ticks_us()) > 0:
            self._ring.write(data)  # within the window (best-effort, bounded)
        else:
            """
            Window lapsed with no follow-up request -> stop and discard. DISCARD WITHOUT DECODING:
            this runs on the PRODUCER's path (tee() is reached from _enqueue on every log() and
            tlm_raw(), i.e. inside Telemetry.push at 100 Hz with GC off). _take() would build a list
            and decode every buffered record into a str only to drop it on the floor -- a burst of
            allocation, at the worst possible moment, for a result nobody reads.
            """
            self._deadline = 0
            if self._ring is not None:
                self._ring.discard()

    def _take(self) -> list:
        records = []
        if self._ring is not None:
            record = self._ring.read()
            while record is not None:
                records.append(record.decode().rstrip())
                record = self._ring.read()
        return records

    def drain(self, duration_ms: int) -> list:
        """
        Return the batch buffered since the last call and re-arm for `duration_ms` more.

        Freezes first (so no producer tees mid-drain -- the protocol is request->reply).

        Args:
            duration_ms - the next window in milliseconds (<= 0 stops after returning the batch).

        Returns:
            (records, dropped): the rows buffered since the last call (decoded, right-stripped), and
            how many the ring DISCARDED in that window. The count matters because the tee is
            best-effort by design -- a full ring drops rather than raising, so a live `tlm` session
            with a gap looks exactly like a quiet sensor. Reporting it turns an invisible loss into a
            number the operator can see.
        """
        self._deadline = 0
        if self._ring is None:
            if duration_ms <= 0:
                return [], 0
            self._ring = Ring(min(duration_ms * 10, 4 * _DEFAULT_CAPACITY), self._cell)  # first window sizes it
        dropped = self._ring.dropped
        self._ring.dropped = 0
        records = self._take()
        if duration_ms > 0:
            self._deadline = time.ticks_add(time.ticks_us(), duration_ms * 1000)
        return records, dropped


class Recorder:
    """The global telemetry + log singleton: enqueue synchronously, drain to the Luckfox UART async."""

    name = 'recorder'
    kind = 'recorder'
    _tlm: Ring = None
    _log: Ring = None
    _cc_log = _TeeSink()  # CC mirror of the log stream (the `log <ms>` command)
    _cc_tlm = _TeeSink()  # CC mirror of the telemetry stream (the `tlm <ms>` command)
    _uart = None  # asyncio.StreamWriter wrapping the recorder UART
    _flag = None  # ThreadSafeFlag set by producers, waited on by run()
    _prefix: str = ''  # whole session prefix from config (`recorder.session`, a valid label); '' -> the boot id
    _session: str = None  # the file prefix, settled on first use, fixed for the boot (see session())
    """
    This boot's id: the NVS counter main.py increments once per boot BEFORE the Recorder starts. setup()
    leaves it alone, because the tests call setup() many times per boot. None when nothing counted the
    boot (tests, HITL bring-ups): session() then falls back to the legacy date + random prefix.
    """
    boot_id: int = None
    _identity: str = ''  # session.csv's 'board;firmware;config_id' cells, built once by setup() (config is fixed)
    _index_header_sent: bool = False  # session.csv header queued; cleared again before the boot and anchor rows
    _setup_ms: int = 0  # ticks_ms when setup() ran: the anchor row's minute counts from here
    _anchored: bool = False  # the session.csv 'anchor' row went out (or was lost): once per boot
    _wrap_refused: bool = False  # a record _wrap() refused was logged: once per boot, the count is `dropped`
    _crc: array.array = None  # the CRC-32 table, built by the first setup()
    _raw: bytearray = None  # drain: the raw record being wrapped (one cell's payload)
    _line: bytearray = None  # drain: the wrapped line
    _lines: list = None  # drain: a memoryview of _line per length, made once so no write slices per line
    _tlm_max: int = 0  # high-water mark of queued telemetry records
    _log_max: int = 0  # high-water mark of queued log records
    _stats_ms: int = _STATS_PERIOD_MS
    _last_stats_ms: int = 0
    """
    global telemetry decimation (µs between emitted rows): every Telemetry stream whose own decimate_us is
    0 uses this, so `recorder.telemetry_ms` in the board config prorates ALL streams at once, while a stream
    that sets a non-zero telemetry_ms keeps its individual rate. The code fallback is 50 Hz; the
    SHIPPED config is 0 = uncapped, so in flight nothing decimates unless a profile asks for it.
    """
    telemetry_decimate_us: int = _DEFAULT_TELEMETRY_MS * 1000

    @classmethod
    def setup(cls, config: dict, uart=None) -> None:
        recorder = config.get('recorder', {})
        cell_size = recorder.get('cell_size', _DEFAULT_CELL_SIZE)
        cls._tlm = Ring(recorder.get('tlm_capacity', _DEFAULT_CAPACITY), cell_size)
        cls._log = Ring(recorder.get('log_capacity', _DEFAULT_CAPACITY), cell_size)
        cls._cc_log.reset(cell_size)  # off at boot: nothing mirrored to CC until it asks
        cls._cc_tlm.reset(cell_size)
        cls._session = None
        """
        The WHOLE session prefix can come from config (`recorder.session`), used verbatim -- a label the
        test system sets for one run. Absent the key, the prefix is this boot's NVS boot id (session()),
        which is unique per board without any clock. The id stays the SAFE default: a stale `session`
        left in a saved config would be reused by every boot and collide every time. A label that breaks
        the rule (_is_label) is ignored, not repaired, and logged once the log ring exists (below).
        """
        label = str(recorder.get('session', '') or '').strip().replace(' ', '-')
        cls._prefix = label if _is_label(label) else ''
        board = config.get('board', {})
        cls._identity = '%s;%s;%s' % (_cell(board.get('id', '')), _cell(board.get('firmware_version', 'dev')),
                                      config_module.config_id(config))  # once: it hashes the whole config
        cls._index_header_sent = False
        cls._anchored = False
        cls._wrap_refused = False
        """
        The drain's buffers, sized once: a cell's payload in, that plus the wrapper (and a '\n' for a
        record that came without one) out. One memoryview per output length is made HERE, because slicing
        a view per line allocates the new view -- with the GC off. ~16 B each, ~5 KB for 256-byte cells.
        """
        max_payload = cls._tlm.max_payload
        cls._raw = bytearray(max_payload)
        cls._line = bytearray(max_payload + _WRAPPER_BYTES + 1)
        view = memoryview(cls._line)
        cls._lines = [view[:length] for length in range(len(cls._line) + 1)]
        if cls._crc is None:
            cls._crc = _crc_table()
        cls._tlm_max = 0
        cls._log_max = 0
        cls._flag = asyncio.ThreadSafeFlag()
        if label and not cls._prefix:  # print() too: the Luckfox may not be listening this early
            message = "recorder.session %r ignored (letters, digits and '-' only, one letter at least, %d at most)" % (
                label, _LABEL_MAX)
            print('Recorder :: ' + message)
            cls.log('Recorder', message)
        cls._stats_ms = recorder.get('stats_ms', _STATS_PERIOD_MS)
        cls._setup_ms = cls._last_stats_ms = time.ticks_ms()
        cls.telemetry_decimate_us = recorder.get('telemetry_ms', _DEFAULT_TELEMETRY_MS) * 1000  # global rate knob
        if uart is None:
            entry = config_module.device(config, driver='recorder') or {'bus': 'uart', 'id': 1}
            kind, bus_id = entry.get('bus', 'uart'), entry.get('id', 1)
            spec = config_module.bus(config, kind, bus_id) or {'tx': 20, 'baud': 921600}
            """
            RX is passed ONLY when the config declares one. The recorder link is write-only -- the
            board streams telemetry to the Luckfox and never reads it back -- and on the v0.1 PCB
            GPIO21 has no copper at all (the netlist routes 20 of 22 GPIOs; this is one of the two
            absent). Naming it anyway claimed an unconnected pin as a UART input and left it floating,
            which is precisely the class of fault that made every servo appear to move on the bench.
            `rx=None` is not the escape hatch -- MicroPython rejects it with ValueError('invalid pin');
            the kwarg has to be omitted, which is what the dict-splat below does.
            """
            pins = {'tx': spec['tx']}
            if spec.get('rx') is not None:
                pins['rx'] = spec['rx']
            """
            txbuf, explicitly. The MicroPython default is 256 bytes, and drain() pushes the WHOLE ring
            in one pass -- 1024 cells of 256 bytes, so a burst can be orders of magnitude past that.
            An overrun does not raise; the UART silently drops the tail of whatever record it was
            mid-way through, which is exactly the corruption measured on real captures: a record cut
            mid-field with its newline gone and the next record running onto the same line, 20 times in
            1,004,804 rows across 48 flights (about one flight in three).

            Sizing rather than sleeping: at the measured 424 records/s, a "drain + sleep 1 ms per line"
            costs 0.4 s of sleep per second of flight at a true 1 ms, and 4.2 s/s at this board's
            MEASURED ~10 ms asyncio floor -- it would throttle telemetry roughly fourfold. A bigger
            buffer costs RAM once and nothing per record.
            """
            uart = UART(bus_id, baudrate=spec['baud'], txbuf=spec.get('txbuf', _DEFAULT_TXBUF), **pins)
        # accept a pre-wrapped async writer (tests) or wrap a raw UART for async drain
        cls._uart = uart if hasattr(uart, 'drain') else asyncio.StreamWriter(uart, {})
        inspector.Inspector.register(cls)

    @classmethod
    def timestamp(cls) -> int:
        """Monotonic-ish record timestamp. Currently raw microseconds; the unit may change."""
        return time.ticks_us()

    @classmethod
    def session(cls) -> str:
        """
        The per-boot file prefix, shared by every telemetry stream of this boot.

        The first of: a valid `recorder.session` label from config, verbatim; the boot id as '%06u';
        the legacy `YYYYMMDD_HHMMSS_<6-digit random>` when nothing counted the boot
        (doc/specs/recorder-wire.md).
        The boot id needs no clock, which is the point: the RTC reads 2000-01-01 until CC sets it, and
        the 2026-10-03 dumps named every session 2000-01-01.

        Args:
            (none)

        Returns:
            The session id string (cached after the first call).
        """
        if cls._session is None:
            if cls._prefix:  # the test system labelled this run -- trust it
                cls._session = cls._prefix
            elif cls.boot_id is not None:
                cls._session = '%06u' % cls.boot_id
            else:
                now = time.localtime()
                """
                A random suffix disambiguates boots that start before the RTC ticks (fast restarts share
                the same wall-clock second otherwise -> colliding session ids -> telemetry files clobbered
                / a header spliced mid-file on the Luckfox).

                SIX digits, not three. The original 3-digit suffix gave only 900 values, and the birthday
                bound makes that far weaker than it looks: N boots collide about N^2/2M times, so 150 boots
                over 900 values expects ~12 collisions -- and a Luckfox audit found exactly that. Boots
                that collide APPEND INTO EACH OTHER'S FILES, so two flights end up interleaved in one CSV
                with uptime restarting midway, which no amount of downstream parsing can separate. One
                session had 77 stream files instead of the usual 13 and a 542 MB accelerometer CSV.

                The RNG itself was fine (150 distinct suffixes observed, so it IS seeded per boot); the
                range was the defect. At 900000 values the same 150 boots expect 0.01 collisions, and even
                1000 boots expect 0.6.
                """
                cls._session = '%04d%02d%02d_%02d%02d%02d_%d' % (
                    now[0], now[1], now[2], now[3], now[4], now[5], random.randint(100000, 999999))
        return cls._session

    @classmethod
    def index_session(cls, seconds: int, utc_offset: int, source: str, cc_position: tuple) -> bool:
        """
        Append a row to the shared session index (`session.csv`): this boot, tied to the wall clock.

        A boot writes a 'boot' and an 'anchor' row (_index_row) and one per successful time set -- unless
        it dies inside its first minute or a row meets a full ring. Each dated row pairs an uptime with a
        UTC, so every uptime of the boot converts to wall-clock time.
        Routed bare ('@session.csv@', no session prefix), so all boots append to one file. It goes
        through the telemetry ring: wrapped on the wire, raising when it does not fit -- the callers log
        and lose such a row (the spec's rule for this file), because the boot or time set it records
        stands either way. The board name, firmware and config id are the ones setup() resolved.

        THE CLOCK IS READ HERE, NOT TRUSTED: a time set is accepted from 2000-01-01, so a setter's own
        bad clock can leave the RTC in 2000. Before 2001 the clock dates nothing, and both utc and
        utc_offset stay empty whatever the caller passed; the boot id still places the boot.

        Args:
            seconds - the RTC, time.time(): UTC seconds since 2000-01-01.
            utc_offset - the setter's offset from UTC in minutes, or None (unknown, or not a time set).
            source - the row's kind: 'boot', 'anchor', or what set the time ('cc-auto', 'dashboard');
                anything but a string of printable ASCII without ';' (_cell) becomes an empty cell.
            cc_position - CC's own (lat, lon), or None.

        Returns:
            True when queued; False when the Recorder is not set up (nothing to write to).

        Raises:
            _RecorderError - when the telemetry ring has no room; a header that did not fit goes out
                with the next row instead.
        """
        if cls._tlm is None:
            return False
        if not cls._index_header_sent:
            cls.tlm_raw(('@%s@%s\n' % (_SESSION_INDEX, _SESSION_INDEX_HEADER)).encode())
            cls._index_header_sent = True
        utc = ''
        if seconds >= _CLOCK_SET_S:
            field = time.gmtime(seconds)
            utc = '%04d-%02d-%02dT%02d:%02d:%02dZ' % (field[0], field[1], field[2], field[3], field[4], field[5])
        else:  # never set: an offset from an undated instant means nothing either
            utc_offset = None
        latitude, longitude = cc_position if cc_position is not None else ('', '')
        row = '%u;%s;%s;%s;%s;%s;%s;%s;%s' % (
            cls.timestamp(), '' if cls.boot_id is None else cls.boot_id, cls.session(), utc,
            '' if utc_offset is None else utc_offset, cls._identity, _cell(source), latitude, longitude)
        cls.tlm_raw(('@%s@%s\n' % (_SESSION_INDEX, row)).encode())
        return True

    @classmethod
    def _index_row(cls, source: str, seconds: int) -> None:
        """
        A session.csv row the Recorder writes on its own ('boot', 'anchor'): header first, never raising.

        The header goes out again right before it, since the boot's first header may have reached
        nobody: the Luckfox listens ~29 s after power-on. There is no time set behind such a row, so no
        utc_offset and no CC position. A full ring loses the row, logged: it is queued from run(), and
        raising there would end the drain loop and with it the boot's capture.

        Args:
            source - 'boot' or 'anchor'.
            seconds - the RTC, time.time(): UTC seconds since 2000-01-01.

        Returns:
            None; queues the header and the row, or logs that the row was lost.
        """
        cls._index_header_sent = False
        try:
            cls.index_session(seconds, None, source, None)
        except _RecorderError as error:  # logged and lost: there is no second try
            cls.log('Recorder', 'session.csv %s row lost: %s' % (source, error))

    @classmethod
    def _anchor(cls, now: int, seconds: int) -> bool:
        """
        The session.csv 'anchor' row: once per boot, a minute after setup(), whatever the clock reads.

        The 'boot' row and a cold boot's first sync go out before the Luckfox listens (~29 s after
        power-on), and a soft or watchdog reset or a warm start keeps the RTC, so CC never syncs it. One
        more row a minute in lists every boot the reader missed, dated when the clock is set and
        undated otherwise. Run off the stats tick in run(), so it is at most `stats_ms` late; `now` and
        `seconds` come in as arguments so a test need not wait a minute or touch the RTC.

        Args:
            now - time.ticks_ms() at the call.
            seconds - the RTC, time.time(): UTC seconds since 2000-01-01.

        Returns:
            True when this call wrote the row, or lost it to a full ring (logged): either way the boot's
            one anchor is spent. False when not due: already anchored, or under a minute in.
        """
        if cls._anchored or time.ticks_diff(now, cls._setup_ms) < _ANCHOR_AFTER_MS:
            return False
        cls._anchored = True
        cls._index_row('anchor', seconds)
        return True

    @classmethod
    def _enqueue(cls, ring, tee, data: bytes) -> bool:
        """
        Queue `data` to `ring` (the UART/Luckfox primary sink) and tee it to the CC mirror.

        Signals the drain loop on a successful store; the tee is the extra route and never gates the
        primary result.

        Args:
            ring - the primary destination ring.
            tee - the CC-mirror tee callable.
            data - the encoded record bytes.

        Returns:
            True when stored in the primary ring (so the caller picks best-effort drop for logs vs
            raise for telemetry), else False.
        """
        stored = ring.write(data)
        if stored:
            cls._flag.set()
        tee(data)
        return stored

    @classmethod
    def log(cls, descriptor: str, message: str) -> bool:
        """
        Best-effort log line "<ts> <descriptor> :: <message>" (-> recorder.log).

        Truncated to fit a cell; dropped (returns False) when the buffer is full or the Recorder is
        not set up.

        Args:
            descriptor - the source tag.
            message - the log text.

        Returns:
            True when queued, False when dropped (buffer full, or the Recorder is not set up).
        """
        if cls._log is None:
            return False  # not set up yet -> drop (logs are best-effort)
        data = ('%u %s :: %s\n' % (cls.timestamp(), descriptor, message)).encode()
        if len(data) > cls._log.max_payload:
            """
            KEEP THE NEWLINE. The wire protocol is line-framed, so a plain slice truncates the '\n'
            off the end and the over-long log line then MERGES with whatever record follows it on the
            UART. If that next record is a telemetry row, the row is swallowed into the log line and
            lost from its CSV -- silently, and on the channel the error policy promises is durable.
            Truncate the text instead and re-terminate.
            """
            data = data[: cls._log.max_payload - 1] + b'\n'
        return cls._enqueue(cls._log, cls._cc_log.tee, data)

    @classmethod
    def cc_logs(cls, duration_ms: int) -> dict:
        """
        Poll-model CC log streaming (the `log <ms>` command).

        Returns the lines buffered since the last call, re-arming the tee for `duration_ms` more
        (<= 0 stops).

        Args:
            duration_ms - the next window in milliseconds (<= 0 stops).

        Returns:
            {'lines': [...], 'dropped': n} -- the buffered log lines and how many the tee discarded.
        """
        lines, dropped = cls._cc_log.drain(duration_ms)
        return {'lines': lines, 'dropped': dropped}

    @classmethod
    def cc_telemetry(cls, duration_ms: int) -> dict:
        """
        Poll-model CC telemetry streaming (the `tlm <ms>` command).

        Returns the telemetry rows buffered since the last call, re-arming the tee for `duration_ms`
        more (<= 0 stops). An EXTRA route -- the UART/Luckfox telemetry is untouched.

        Args:
            duration_ms - the next window in milliseconds (<= 0 stops).

        Returns:
            {'samples': [...], 'dropped': n} -- the buffered rows and how many the tee discarded
            (non-zero means the operator is seeing a GAP, not a quiet sensor).
        """
        samples, dropped = cls._cc_tlm.drain(duration_ms)
        return {'samples': samples, 'dropped': dropped}

    @classmethod
    def tlm(cls, filename: str, content: str) -> None:
        """
        Queue an important telemetry line "@<session>_<filename>@<content>".

        Telemetry must not be lost silently.

        Args:
            filename - the destination stream file (without the session prefix).
            content - the CSV row (or header) text.

        Returns:
            None.

        Raises:
            _RecorderError - when the record will not fit or there is no room.
        """
        cls.tlm_raw(('@%s_%s@%s\n' % (cls.session(), filename, content)).encode())

    @classmethod
    def tlm_raw(cls, data: bytes) -> None:
        """
        Queue an ALREADY-ENCODED telemetry line (the hot path Telemetry.push uses).

        tlm() builds the wire line by formatting the session prefix around a row that the caller had
        already formatted -- copying the whole row a second time, per sample. A stream knows its own
        prefix, so it can format the complete line ONCE and hand the bytes straight here. Measured on
        the board: 240 -> 144 B per push for a 3-field row, and telemetry is the single biggest
        GC-off allocator on a busy capture.

        Args:
            data - the complete encoded record, newline included.

        Returns:
            None.

        Raises:
            _RecorderError - when the record will not fit or there is no room.
        """
        if not cls._enqueue(cls._tlm, cls._cc_tlm.tee, data):  # CC mirror + primary ring
            raise _RecorderError('telemetry dropped (%d bytes)' % len(data))

    @classmethod
    async def drain(cls) -> int:
        """
        Drain queued records to the UART, wrapped, telemetry first then logs.

        Args:
            (none)

        Returns:
            The records written; a record the wrapper refuses is counted in its ring's `dropped` instead.
        """
        queued = cls._tlm.count()
        if queued > cls._tlm_max:
            cls._tlm_max = queued
        queued = cls._log.count()
        if queued > cls._log_max:
            cls._log_max = queued
        drained = 0
        writer = cls._uart
        # Whether this writer reports its fill. The real StreamWriter does; the test stubs that stand in
        # for it need not, and production code must not require a stub to grow an attribute to stay
        # usable. Resolved ONCE -- `out_buf` is REBOUND on every write, so a captured reference goes
        # stale and only the name can be re-read.
        reports_fill = hasattr(writer, 'out_buf')
        """
        Flush on BUFFER FILL, not on a record count -- back-pressure only when there is pressure.

        The writer's pending bytes are directly observable (`out_buf`), so the flush happens when the
        buffer actually reaches its high water mark rather than every N records. That adapts to record
        size and to how fast the link is draining: a quiet stream never flushes early, a burst flushes
        as often as it needs to, and the UART is never handed more than it can hold.

        This is an await, not a sleep. A fixed `sleep_ms(1)` per line was considered and measured
        against: at the 424 records/s these flights produce it costs 0.4 s of sleep per second of
        flight if it truly slept 1 ms, and 4.2 s/s at this board's MEASURED ~10 ms asyncio floor --
        throttling telemetry roughly fourfold. drain() yields only as long as the UART actually needs.

        Honest note on effect: sizing txbuf and flushing early showed NO measurable reduction in wire
        corruption over a 24-flight run (40 events against a 41/42/49 baseline, inside its own spread).
        The sender was not the bottleneck. This is kept because it is strictly better-behaved than
        buffering a whole ring pass, not because it is demonstrated to fix anything.
        """
        """
        Each record is wrapped on the way out (_wrap), with NO allocation per line -- this loop runs
        with the GC off from BOOSTING to DONE. readinto() copies the record into _raw, _wrap writes the
        line into _line, and the write hands the StreamWriter a view made at setup. The writer copies
        whatever it cannot send at once (out_buf), so reusing _line for the next record is safe.
        """
        raw = cls._raw
        line = cls._line
        lines = cls._lines
        table = cls._crc
        capacity = len(line)
        for ring in (cls._tlm, cls._log):
            size = ring.readinto(raw)
            while size >= 0:
                length = _wrap(raw, size, line, capacity, table, _HEX_DIGITS)
                if length < 0:  # only for a line buffer below a cell + the wrapper, which setup() never sizes
                    cls._refused(ring)
                else:
                    writer.write(lines[length])
                    drained += 1
                    if reports_fill and len(writer.out_buf) >= _DRAIN_HIGH_WATER:
                        await writer.drain()
                size = ring.readinto(raw)
        if drained:
            await writer.drain()
        return drained

    @classmethod
    def _refused(cls, ring: Ring) -> None:
        """
        Account for a record _wrap() refused: counted in its ring's `dropped`, logged once per boot.

        Telemetry raises when it cannot be queued, but this record was queued and only failed on the way
        out, inside the drain -- raising there would stop the drain loop. So it is never silent instead:
        the count is in report() (the stats line, CC's `stats`) and the first refusal names the cause.

        Args:
            ring - the ring the record came from.

        Returns:
            None; bumps `ring.dropped`, and logs on the boot's first refusal.
        """
        ring.dropped += 1
        if not cls._wrap_refused:
            cls._wrap_refused = True
            cls.log('Recorder', 'wire wrapper refused a record (line buffer %d B < record + %d B); counted in '
                    'dropped' % (len(cls._line), _WRAPPER_BYTES + 1))

    @classmethod
    async def run(cls) -> None:
        """
        Event-driven drain loop: wait for a producer signal, then drain everything queued.

        Data is delivered as fast as possible with no fixed poll interval and zero idle CPU. Runs
        forever (a wedged board reboots via the watchdog); about every _stats_ms it logs a buffer-stats
        line and checks whether the boot's session.csv anchor row is due (_anchor).

        It opens with the boot's session.csv 'boot' row: here, not in setup(), so every bring-up that
        drains lists its boot -- main.py's and the HITL and test ones that skip it -- while the tests
        that call setup() and drain() directly see no extra record.

        Args:
            (none)

        Returns:
            None (runs forever).
        """
        cls._index_row('boot', time.time())
        while True:
            await cls._flag.wait()
            await cls.drain()
            now = time.ticks_ms()
            if time.ticks_diff(now, cls._last_stats_ms) >= cls._stats_ms:
                cls._last_stats_ms = now
                cls.log('Recorder', str(cls.report()))
                cls._anchor(now, time.time())

    @classmethod
    def inspect(cls) -> dict:
        return {
            'session': cls._session,
            'tlm_capacity': cls._tlm.capacity,
            'log_capacity': cls._log.capacity,
            'cell_size': cls._tlm.cell_size,
            'stats_ms': cls._stats_ms,
        }

    @classmethod
    def update(cls, props: dict) -> list:
        changed = []
        value = props.get('stats_ms')
        if isinstance(value, int) and value > 0 and value != cls._stats_ms:
            cls._stats_ms = value
            changed.append('stats_ms')
        return changed

    @classmethod
    def stats(cls) -> dict:
        return cls.report()

    @classmethod
    def report(cls) -> dict:
        return {
            'session': cls._session,
            'tlm': {'count': cls._tlm.count(), 'max': cls._tlm_max, 'dropped': cls._tlm.dropped},
            'log': {'count': cls._log.count(), 'max': cls._log_max, 'dropped': cls._log.dropped},
        }


class Telemetry:
    """
    A typed telemetry stream.

    Created with a destination file and its data field names; the first push emits the CSV header
    (uptime + fields), then each push emits a timestamped row. All streams in one boot share the
    Recorder session prefix, so file names are stable.

    `decimate_us` rate-limits the stream: push() emits only when at least `decimate_us` microseconds
    have passed since the last emitted row (a fast sensor can push every sample and have its telemetry
    decimated to a sane rate). `decimate_us=0` (the default) inherits the Recorder GLOBAL rate
    (`Recorder.telemetry_decimate_us`) -- so a stream opts into an individual rate by passing a
    non-zero value, else the board-wide `recorder.telemetry_ms` prorates it.

    THE GLOBAL IS RESOLVED AT USE, NOT AT CONSTRUCTION. It used to be folded into `self.decimate_us`
    in __init__, which quietly broke the inheritance it was documenting: drivers build their streams
    during their own setup(), and the controller runs every device's setup BEFORE the recorder task's,
    so a stream latched the CLASS DEFAULT and never saw the configured value. Measured on the board --
    with `recorder.telemetry_ms` 0 every stream still ran at 20000 us -- and it cost a config that
    claimed full-rate logging while capping the 100 Hz accelerometer at 50.
    """

    def __init__(self, filename: str, fields: tuple, decimate_us: int = 0):
        self.filename: str = filename
        self.fields: tuple = fields
        self.decimate_us = decimate_us  # 0 -> inherit the global, read through `window` at each use
        self._header: str = 'uptime;' + ';'.join(fields)  # constant CSV header, built once
        self._row_fmt: str = '%u;' + ';'.join('%s' for _ in fields)  # one reusable row-format string
        self._header_sent: bool = False
        self._line_fmt: str = None  # '@<session>_<file>@' + the row format, resolved on the first push
        # seed one FULL window back so the first push always emits. It must use THIS stream's own
        # window, not the global: seeding a 50 ms stream only 20 ms back decimates its very first row
        # away (caught by test_recorder). Resolving `or` here as well as in `window` is deliberate --
        # this one only has to make push #1 fire, while `window` must track a global set later.
        self._last_us: int = Recorder.timestamp() - (decimate_us or Recorder.telemetry_decimate_us)

    @property
    def window(self) -> int:
        """The decimation window in microseconds: this stream's own, else the Recorder global."""
        return self.decimate_us or Recorder.telemetry_decimate_us

    def due(self, now: int) -> bool:
        """
        Whether the decimation window has elapsed -- so a HOT-PATH producer can skip building its row.

        push() takes an already-built `values` tuple, so a 100 Hz caller that lets push() do the
        decimating still allocates that tuple 100x/s on a GC-off flight. Checking here first lets the
        caller return before building it, against the SAME clock push() uses -- one clock, so a jittery
        slice can never advance a private counter past a window push() then refuses (which would drop
        the sample silently).

        Args:
            now - the current Recorder.timestamp() / ticks_us.

        Returns:
            True when a push() now would emit rather than decimate. False when the Recorder is not
            running at all: with no ring to write to, push() would RAISE, and a hot-path producer
            calling it every tick would pay for a raised-and-caught exception per tick (measured in
            bench_flight, where the bench has no UART: it dominated the reported per-step cost). A
            stream that has nowhere to go is simply not due.
        """
        return Recorder._tlm is not None and time.ticks_diff(now, self._last_us) >= self.window

    def push(self, values) -> None:
        if not self._header_sent:
            Recorder.tlm(self.filename, self._header)
            self._header_sent = True
            # the session id is fixed for the boot, so the whole wire prefix folds into the row format
            # ONCE here (%s substitutes the row format literally) instead of wrapping every sample
            self._line_fmt = '@%s_%s@%s\n' % (Recorder.session(), self.filename, self._row_fmt)
        now = Recorder.timestamp()
        if time.ticks_diff(now, self._last_us) < self.window:
            return  # too soon since the last row -> decimate
        """
        ONE % pass over the precomputed full-line format, then one encode -- no per-field str()
        generator, no ';'.join list, and no second copy of the row to add the prefix (tlm() used to do
        that, which measured 240 B/push; this is 144 B). A tuple is passed through rather than rebuilt:
        every hot caller already holds one. ((now,) + values, not (now, *values): this compiler rejects
        display star-unpack.)
        """
        Recorder.tlm_raw((self._line_fmt % ((now,) + (values if type(values) is tuple else tuple(values))))
                         .encode())
        self._last_us = now
