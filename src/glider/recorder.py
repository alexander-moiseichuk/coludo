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
"""

import asyncio
import random
import struct
import time

import config as config_mod
import inspector

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const

try:
    from machine import UART
except ImportError:  # host (CPython): board-only; the Luckfox UART is opened only on the board
    UART = None


_DEFAULT_CELL_SIZE = const(256)  # bytes per ring cell (record + 2-byte length header)
_DEFAULT_CAPACITY = const(1024)  # cells per ring
_LENGTH_BYTES = const(2)  # uint16 record-length header
_STATS_PERIOD_MS = const(1000)  # how often run() logs a buffer-stats line
_DEFAULT_TELEMETRY_US = const(20000)  # global telemetry decimation default (50 Hz); a stream's 0 -> this


class _RecorderError(ValueError):
    """Raised when an important (telemetry) record cannot be queued."""


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
            self._deadline = 0  # window lapsed with no follow-up request -> stop and discard
            self._take()

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
    _session: str = None  # 'YYYYMMDD_HHMMSS', produced on first tlm(), fixed for the boot
    _tlm_max: int = 0  # high-water mark of queued telemetry records
    _log_max: int = 0  # high-water mark of queued log records
    _stats_ms: int = _STATS_PERIOD_MS
    _last_stats_ms: int = 0
    """
    global telemetry decimation (µs between emitted rows): every Telemetry stream whose own decimate_us is
    0 uses this, so `recorder.telemetry_us` in the board config prorates ALL streams at once, while a stream
    that sets a non-zero telemetry_us keeps its individual rate. Default 50 Hz.
    """
    telemetry_decimate_us: int = _DEFAULT_TELEMETRY_US

    @classmethod
    def setup(cls, config: dict, uart=None) -> None:
        recorder = config.get('recorder', {})
        cell_size = recorder.get('cell_size', _DEFAULT_CELL_SIZE)
        cls._tlm = Ring(recorder.get('tlm_capacity', _DEFAULT_CAPACITY), cell_size)
        cls._log = Ring(recorder.get('log_capacity', _DEFAULT_CAPACITY), cell_size)
        cls._cc_log.reset(cell_size)  # off at boot: nothing mirrored to CC until it asks
        cls._cc_tlm.reset(cell_size)
        cls._session = None
        cls._tlm_max = 0
        cls._log_max = 0
        cls._flag = asyncio.ThreadSafeFlag()
        cls._stats_ms = recorder.get('stats_ms', _STATS_PERIOD_MS)
        cls._last_stats_ms = time.ticks_ms()
        cls.telemetry_decimate_us = recorder.get('telemetry_us', _DEFAULT_TELEMETRY_US)  # global rate knob
        if uart is None:
            entry = config_mod.device(config, driver='recorder') or {'bus': 'uart', 'id': 1}
            kind, bus_id = entry.get('bus', 'uart'), entry.get('id', 1)
            spec = config_mod.bus(config, kind, bus_id) or {'tx': 20, 'baud': 921600}
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
            uart = UART(bus_id, baudrate=spec['baud'], **pins)
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
        The per-boot file prefix.

        Produced from the RTC the first time it is needed, then shared by every telemetry stream in
        this boot.

        Args:
            (none)

        Returns:
            The session id string 'YYYYMMDD_HHMMSS_rand' (cached after the first call).
        """
        if cls._session is None:
            now = time.localtime()
            """
            a random suffix disambiguates boots that start before the RTC ticks (fast restarts share the
            same wall-clock second otherwise -> colliding session ids -> telemetry files clobbered / a
            header spliced mid-file on the Luckfox).
            """
            cls._session = '%04d%02d%02d_%02d%02d%02d_%d' % (
                now[0], now[1], now[2], now[3], now[4], now[5], random.randint(100, 1000))
        return cls._session

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
            data = data[: cls._log.max_payload]
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
        """Drain queued records to the UART, telemetry first then logs. Returns records drained."""
        queued = cls._tlm.count()
        if queued > cls._tlm_max:
            cls._tlm_max = queued
        queued = cls._log.count()
        if queued > cls._log_max:
            cls._log_max = queued
        drained = 0
        writer = cls._uart
        for ring in (cls._tlm, cls._log):
            record = ring.read()
            while record is not None:
                writer.write(record)
                drained += 1
                record = ring.read()
        if drained:
            await writer.drain()
        return drained

    @classmethod
    async def run(cls) -> None:
        """
        Event-driven drain loop: wait for a producer signal, then drain everything queued.

        Data is delivered as fast as possible with no fixed poll interval and zero idle CPU. Runs
        forever (a wedged board reboots via the watchdog); logs a buffer-stats line about every
        _stats_ms.

        Args:
            (none)

        Returns:
            None (runs forever).
        """
        while True:
            await cls._flag.wait()
            await cls.drain()
            now = time.ticks_ms()
            if time.ticks_diff(now, cls._last_stats_ms) >= cls._stats_ms:
                cls._last_stats_ms = now
                cls.log('Recorder', str(cls.report()))

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
    (`Recorder.telemetry_decimate_us`, 50 Hz) -- so a stream opts into an individual rate by passing a
    non-zero value, else the board-wide `recorder.telemetry_us` prorates it.
    """

    def __init__(self, filename: str, fields: tuple, decimate_us: int = 0):
        self.filename: str = filename
        self.fields: tuple = fields
        self.decimate_us = decimate_us or Recorder.telemetry_decimate_us  # 0 -> the global default rate
        self._header: str = 'uptime;' + ';'.join(fields)  # constant CSV header, built once
        self._row_fmt: str = '%u;' + ';'.join('%s' for _ in fields)  # one reusable row-format string
        self._header_sent: bool = False
        self._line_fmt: str = None  # '@<session>_<file>@' + the row format, resolved on the first push
        self._last_us: int = Recorder.timestamp() - self.decimate_us  # one window back -> first push emits

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
        return Recorder._tlm is not None and time.ticks_diff(now, self._last_us) >= self.decimate_us

    def push(self, values) -> None:
        if not self._header_sent:
            Recorder.tlm(self.filename, self._header)
            self._header_sent = True
            # the session id is fixed for the boot, so the whole wire prefix folds into the row format
            # ONCE here (%s substitutes the row format literally) instead of wrapping every sample
            self._line_fmt = '@%s_%s@%s\n' % (Recorder.session(), self.filename, self._row_fmt)
        now = Recorder.timestamp()
        if time.ticks_diff(now, self._last_us) < self.decimate_us:
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
