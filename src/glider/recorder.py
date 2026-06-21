# Recorder — the single non-hot data path: telemetry + logs into PSRAM ring buffers, drained to
# the Luckfox recorder over UART. See specs/coludo.md ('Task Data-Flow', 'Logging', 'Telemetry',
# 'Storage Write Constraints').
#
# Recorder is a singleton: any module calls Recorder.log() / Recorder.tlm() globally. Producers
# enqueue synchronously (struct.pack_into into a ring -- never slice-assignment, which is
# O(buffer length) on this port); the async run() loop drains the rings to the UART via an
# asyncio.StreamWriter, telemetry (1st priority) before logs (2nd). Logs are best-effort (dropped
# when full); telemetry is important (raises if a record will not fit).

import asyncio
import struct
import time

import inspector

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)

    def const(value):
        return value


_DEFAULT_CELL_SIZE = const(256)  # bytes per ring cell (record + 2-byte length header)
_DEFAULT_CAPACITY = const(1024)  # cells per ring
_LENGTH_BYTES = const(2)  # uint16 record-length header
_STATS_PERIOD_MS = const(1000)  # how often run() logs a buffer-stats line


class _RecorderError(ValueError):
    """Raised when an important (telemetry) record cannot be queued."""


class Ring:
    """Lock-free single-producer / single-consumer byte ring. The writer owns `head`, the reader
    owns `tail`; they never touch the same field, so it is safe between an ISR producer and a task
    consumer with no locks. Each cell holds <uint16 length><payload>. write() uses pack_into
    (cost O(record)) and returns False if there is no room (the record is skipped, never
    overwriting unread data). read() returns a bytes copy (stable across an await). Holds
    `capacity - 1` records (one cell separates full from empty)."""

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


class Recorder:
    name = 'recorder'
    kind = 'recorder'
    _tlm: Ring = None
    _log: Ring = None
    _cc: Ring = None  # CC log ring, sized + allocated on the first `log <ms>` request, then reused
    _cc_deadline: int = 0  # ticks_us window-end while streaming to CC (0 = off); log() tees until it lapses
    _uart = None  # asyncio.StreamWriter wrapping the recorder UART
    _flag = None  # ThreadSafeFlag set by producers, waited on by run()
    _session: str = None  # 'YYYYMMDD_HHMMSS', produced on first tlm(), fixed for the boot
    _tlm_max: int = 0  # high-water mark of queued telemetry records
    _log_max: int = 0  # high-water mark of queued log records
    _stats_ms: int = _STATS_PERIOD_MS
    _last_stats_ms: int = 0

    @classmethod
    def setup(cls, config: dict, uart=None) -> None:
        recorder = config.get('recorder', {})
        cell_size = recorder.get('cell_size', _DEFAULT_CELL_SIZE)
        cls._tlm = Ring(recorder.get('tlm_capacity', _DEFAULT_CAPACITY), cell_size)
        cls._log = Ring(recorder.get('log_capacity', _DEFAULT_CAPACITY), cell_size)
        cls._cc = None  # lazily allocated + sized on the first `log <ms>` request, then reused
        cls._cc_deadline = 0  # off at boot: nothing is collected for CC until it asks
        cls._session = None
        cls._tlm_max = 0
        cls._log_max = 0
        cls._flag = asyncio.ThreadSafeFlag()
        cls._stats_ms = recorder.get('stats_ms', _STATS_PERIOD_MS)
        cls._last_stats_ms = time.ticks_ms()
        if uart is None:
            import config as config_mod
            from machine import UART

            entry = config_mod.device(config, driver='recorder') or {'bus': 'uart', 'id': 1}
            kind, bus_id = entry.get('bus', 'uart'), entry.get('id', 1)
            spec = config_mod.bus(config, kind, bus_id) or {'tx': 20, 'rx': 21, 'baud': 921600}
            uart = UART(bus_id, baudrate=spec['baud'], tx=spec['tx'], rx=spec['rx'])
        # accept a pre-wrapped async writer (tests) or wrap a raw UART for async drain
        cls._uart = uart if hasattr(uart, 'drain') else asyncio.StreamWriter(uart, {})
        inspector.Inspector.register(cls)

    @classmethod
    def timestamp(cls) -> int:
        """Monotonic-ish record timestamp. Currently raw microseconds; the unit may change."""
        return time.ticks_us()

    @classmethod
    def session(cls) -> str:
        """The per-boot file prefix, produced from the RTC the first time it is needed and then
        shared by every telemetry stream in this boot."""
        if cls._session is None:
            now = time.localtime()
            cls._session = '%04d%02d%02d_%02d%02d%02d' % (now[0], now[1], now[2], now[3], now[4], now[5])
        return cls._session

    @classmethod
    def log(cls, descriptor: str, message: str) -> bool:
        """Best-effort log line "<ts> <descriptor> :: <message>" (-> recorder.log). Truncated to
        fit a cell; dropped (returns False) when the buffer is full or the Recorder is not set up."""
        if cls._log is None:
            return False  # not set up yet -> drop (logs are best-effort)
        data = ('%u %s :: %s\n' % (cls.timestamp(), descriptor, message)).encode()
        if len(data) > cls._log.max_payload:
            data = data[: cls._log.max_payload]
        stored = cls._log.write(data)  # UART/Luckfox is the primary sink -> write it first
        if stored:
            cls._flag.set()
        if cls._cc_deadline:  # CC tee is the extra route -> never gates the primary return code
            if time.ticks_diff(cls._cc_deadline, cls.timestamp()) > 0:
                cls._cc.write(data)  # within the requested window (best-effort, bounded ring)
            else:
                cls._cc_deadline = 0  # window lapsed with no follow-up `log` -> stop and discard
                cls._cc_take()
        return stored

    @classmethod
    def _cc_take(cls) -> list:
        """Drain the CC ring -> its buffered lines, emptying it. Internal to cc_logs() and the
        window-lapse discard in log()."""
        lines = []
        if cls._cc is not None:
            record = cls._cc.read()
            while record is not None:
                lines.append(record.decode().rstrip())
                record = cls._cc.read()
        return lines

    @classmethod
    def cc_logs(cls, duration_ms: int) -> dict:
        """Poll-model CC log streaming (the `log <ms>` command): return {'lines': [...]} buffered since
        the last call and re-arm teeing for `duration_ms` more (<= 0 stops it). Freeze first (deadline
        0) so no producer tees mid-drain -- the protocol is request->reply, so no other `log` arrives
        until this batch reaches CC. The ring is sized from the first window (~10 records/ms) and
        reused, sharing the log cell size and capped at 4x the default ring. If no follow-up `log`
        arrives before the window lapses, the
        next log() discards the buffer and disables -- a lost link cannot grow memory. The UART path is
        never touched."""
        cls._cc_deadline = 0  # freeze teeing while we drain
        if cls._cc is None:
            if duration_ms <= 0:
                return {'lines': []}
            capacity = min(duration_ms * 10, 4 * _DEFAULT_CAPACITY)  # ~10 records/ms, capped
            cls._cc = Ring(capacity, cls._log.cell_size)  # first window sizes it; reused after
        lines = cls._cc_take()
        if duration_ms > 0:
            cls._cc_deadline = time.ticks_add(cls.timestamp(), duration_ms * 1000)  # re-arm
        return {'lines': lines}

    @classmethod
    def tlm(cls, filename: str, content: str) -> None:
        """Important telemetry line "@<session>_<filename>@<content>". Raises if the record will
        not fit or there is no room -- telemetry must not be lost silently."""
        data = ('@%s_%s@%s\n' % (cls.session(), filename, content)).encode()
        if not cls._tlm.write(data):
            raise _RecorderError('telemetry dropped (%d bytes)' % len(data))
        cls._flag.set()

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
        """Event-driven drain loop: wait for a producer signal, then drain everything queued, so
        data is delivered as fast as possible with no fixed poll interval and zero idle CPU. Runs
        forever (a wedged board reboots via the watchdog); logs a buffer-stats line about every
        _stats_ms."""
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
    """A typed telemetry stream. Created with a destination file and its data field names; the
    first push emits the CSV header (uptime + fields), then each push emits a timestamped row.
    All streams in one boot share the Recorder session prefix, so file names are stable.

    `decimate_us` rate-limits the stream: with it 0 every push() emits; with it set, push() emits
    only when at least `decimate_us` microseconds have passed since the last emitted row (a fast
    sensor can push every sample and have its telemetry decimated to a sane rate)."""

    def __init__(self, filename: str, fields: tuple, decimate_us: int = 0):
        self.filename: str = filename
        self.fields: tuple = fields
        self.decimate_us = decimate_us  # min gap between emitted rows (0 = emit every push)
        self._header_sent: bool = False
        self._last_us: int = Recorder.timestamp() - decimate_us  # one window back -> first push emits

    def push(self, values) -> None:
        if not self._header_sent:
            Recorder.tlm(self.filename, 'uptime;' + ';'.join(self.fields))
            self._header_sent = True
        now = Recorder.timestamp()
        if time.ticks_diff(now, self._last_us) < self.decimate_us:
            return  # too soon since the last row -> decimate
        Recorder.tlm(self.filename, '%u;%s' % (now, ';'.join(str(v) for v in values)))
        self._last_us = now
