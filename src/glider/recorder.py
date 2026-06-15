# Recorder — the single non-hot data path: telemetry + logs into PSRAM ring buffers, drained
# to the Luckfox recorder over UART. See specs/coludo.md ('Task Data-Flow', 'Logging',
# 'Telemetry', 'Storage Write Constraints').
#
# Recorder is a singleton: any module calls Recorder.log() / Recorder.tlm() globally. Producers
# enqueue synchronously (just struct.pack_into into a ring -- never slice-assignment, which is
# O(buffer length) on this port); the async run() loop drains the rings to UART, telemetry
# (1st priority) before logs (2nd), writing to UART before any subscriber.
#
# Records are complete UART-ready text lines.

import struct
import time
import asyncio


class Ring:
    '''Lock-free single-producer / single-consumer byte ring. The writer owns `head`, the
    reader owns `tail`; they never touch the same field, so it is safe between an ISR producer
    and a task consumer with no locks. Each slot holds <uint16 length><payload>. write() uses
    pack_into (cost O(record)) and returns False if there is no room (the record is skipped,
    never overwriting unread data). read() returns a bytes copy (stable across an await). Holds
    `slots - 1` records (one slot separates full from empty).'''

    def __init__(self, slots, slot_size):
        self.slots = slots
        self.slot = slot_size
        self.maxpay = slot_size - 2
        self.buf = bytearray(slots * slot_size)
        self.head = 0           # writer-owned: next write slot
        self.tail = 0           # reader-owned: next read slot
        self.dropped = 0        # writer-owned: records skipped (too big, or full)

    def write(self, data):
        n = len(data)
        if n > self.maxpay:
            self.dropped += 1
            return False
        head = self.head
        nxt = head + 1
        if nxt == self.slots:
            nxt = 0
        if nxt == self.tail:                # full -> skip, don't overwrite
            self.dropped += 1
            return False
        off = head * self.slot
        struct.pack_into('<H%ds' % n, self.buf, off, n, data)
        self.head = nxt                     # publish only after the record is written
        return True

    def read(self):
        '''Return the oldest record as bytes (a copy) and advance, or None if empty.'''
        tail = self.tail
        if self.head == tail:               # empty
            return None
        off = tail * self.slot
        n = struct.unpack_from('<H', self.buf, off)[0]
        out = bytes(self.buf[off + 2:off + 2 + n])
        nxt = tail + 1
        self.tail = 0 if nxt == self.slots else nxt
        return out

    def count(self):
        '''Records currently queued (a stats snapshot).'''
        d = self.head - self.tail
        return d if d >= 0 else d + self.slots


class Recorder:
    _tlm = None
    _log = None
    _uart = None
    _session = None             # 'YYYYMMDD_HHMMSS', produced lazily on first tlm(), fixed for the boot
    _drain_ms = 50
    _tlm_max = 0                # high-water mark of queued telemetry records
    _log_max = 0                # high-water mark of queued log records
    _stats_every = 20           # log a stats line every N drains (~1 s)

    @classmethod
    def setup(cls, config, uart=None):
        rc = config.get('recorder', {})
        slot = rc.get('slot_size', 192)
        cls._tlm = Ring(rc.get('tlm_slots', 512), slot)
        cls._log = Ring(rc.get('log_slots', 512), slot)
        cls._drain_ms = rc.get('drain_ms', 50)
        cls._session = None
        cls._tlm_max = 0
        cls._log_max = 0
        cls._stats_every = max(1, 1000 // cls._drain_ms)
        if uart is None:
            from machine import UART
            b = config['buses']['uart_recorder']
            uart = UART(1, baudrate=b['baud'], tx=b['tx'], rx=b['rx'])
        cls._uart = uart

    @classmethod
    def timestamp(cls):
        '''Monotonic-ish record timestamp. Currently raw microseconds; the unit may change.'''
        return time.ticks_us()

    @classmethod
    def session(cls):
        '''The per-boot file prefix, produced from the RTC the first time it is needed and then
        shared by every telemetry stream in this boot.'''
        if cls._session is None:
            t = time.localtime()
            cls._session = '%04d%02d%02d_%02d%02d%02d' % (t[0], t[1], t[2], t[3], t[4], t[5])
        return cls._session

    @classmethod
    def log(cls, descriptor, message):
        '''Append a log line "<ts> <descriptor> :: <message>" (-> recorder.log).'''
        line = '%u %s :: %s\n' % (cls.timestamp(), descriptor, message)
        return cls._log.write(line.encode())

    @classmethod
    def tlm(cls, filename, content):
        '''Append a telemetry line to a per-session file: "@<session>_<filename>@<content>".'''
        line = '@%s_%s@%s\n' % (cls.session(), filename, content)
        return cls._tlm.write(line.encode())

    @classmethod
    def drain(cls, sink, also=None, limit=None):
        '''Drain queued records to sink (telemetry first, then logs). sink.write(rec) happens
        before also(rec). Returns records drained (stops after `limit`).'''
        c = cls._tlm.count()                # capture pre-drain occupancy as the high-water mark
        if c > cls._tlm_max:
            cls._tlm_max = c
        c = cls._log.count()
        if c > cls._log_max:
            cls._log_max = c
        drained = 0
        for ring in (cls._tlm, cls._log):
            rec = ring.read()
            while rec is not None:
                sink.write(rec)
                if also is not None:
                    also(rec)
                drained += 1
                if limit is not None and drained >= limit:
                    return drained
                rec = ring.read()
        return drained

    @classmethod
    async def run(cls, sink=None, stop=None):
        '''Async drain loop. Drains to `sink` (or the configured UART) every drain_ms, and logs
        a buffer-stats line every _stats_every drains so usage is visible in recorder.log.'''
        s = sink if sink is not None else cls._uart
        n = 0
        while stop is None or not stop[0]:
            cls.drain(s)
            n += 1
            if n >= cls._stats_every:
                n = 0
                cls.log('Recorder', cls._stats())
            await asyncio.sleep_ms(cls._drain_ms)

    @classmethod
    def _stats(cls):
        return 'tlm max=%d drop=%d / log max=%d drop=%d' % (
            cls._tlm_max, cls._tlm.dropped, cls._log_max, cls._log.dropped)

    @classmethod
    def report(cls):
        return {'session': cls._session,
                'tlm': {'count': cls._tlm.count(), 'max': cls._tlm_max, 'dropped': cls._tlm.dropped},
                'log': {'count': cls._log.count(), 'max': cls._log_max, 'dropped': cls._log.dropped}}


class Telemetry:
    '''A typed telemetry stream. Created with a destination file and its data field names; the
    first push emits the CSV header (uptime + fields), then each push emits a timestamped row.
    All streams in one boot share the Recorder session prefix, so file names are stable.'''

    def __init__(self, filename, fields):
        self.filename = filename
        self.fields = fields
        self._header = False

    def push(self, values):
        if not self._header:
            Recorder.tlm(self.filename, 'uptime;' + ';'.join(self.fields))
            self._header = True
        row = '%u;%s' % (Recorder.timestamp(), ';'.join(str(v) for v in values))
        Recorder.tlm(self.filename, row)
