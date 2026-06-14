# Recorder — the single non-hot data path: telemetry + logs into PSRAM ring buffers, drained
# to the Luckfox recorder over UART. See specs/coludo.md ('Task Data-Flow', 'Logging',
# 'Telemetry', 'Storage Write Constraints').
#
# Recorder is a singleton: any module calls Recorder.log() / Recorder.tlm() globally. Producers
# enqueue synchronously (just struct.pack_into into a ring -- never slice-assignment, which is
# O(buffer length) on this port); the async run() loop drains the rings to UART, telemetry
# (1st priority) before logs (2nd), writing to UART before any subscriber.
#
# Records are complete UART-ready text lines. Timestamps use time.time_ns()//1000 (microseconds,
# monotonic, no wrap -- time.time_us() does not exist and ticks_us() wraps at ~1073 s).

import struct
import time
import asyncio


class Ring:
    '''Fixed-slot byte ring. Each slot holds <uint16 length><payload>. write() uses pack_into
    (cost O(record)); read() returns a bytes copy (stable across an await) and advances. On
    overflow the oldest record is dropped.'''

    def __init__(self, slots, slot_size):
        self.slots = slots
        self.slot = slot_size
        self.maxpay = slot_size - 2
        self.buf = bytearray(slots * slot_size)
        self.head = 0
        self.tail = 0
        self.count = 0
        self.dropped = 0

    def write(self, data):
        n = len(data)
        if n > self.maxpay:
            self.dropped += 1
            return False
        if self.count == self.slots:            # full -> drop oldest
            self.tail = (self.tail + 1) % self.slots
            self.count -= 1
            self.dropped += 1
        off = self.head * self.slot
        struct.pack_into('<H%ds' % n, self.buf, off, n, data)
        self.head = (self.head + 1) % self.slots
        self.count += 1
        return True

    def read(self):
        '''Return the oldest record as bytes (a copy) and advance, or None if empty.'''
        if self.count == 0:
            return None
        off = self.tail * self.slot
        n = struct.unpack_from('<H', self.buf, off)[0]
        out = bytes(self.buf[off + 2:off + 2 + n])
        self.tail = (self.tail + 1) % self.slots
        self.count -= 1
        return out


class Recorder:
    _tel = None
    _log = None
    _uart = None
    _bus = None
    _boot_us = 0          # time.time_ns()//1000 at setup (uptime reference)
    _boot_epoch = 0       # wall-clock seconds at boot (for the session prefix)
    _session = ''         # 'YYYYMMDD_HHMMSS', fixed for the whole boot
    _drain_ms = 50

    @classmethod
    def setup(cls, config, uart=None):
        rc = config.get('recorder', {})
        slot = rc.get('slot_size', 192)
        cls._tel = Ring(rc.get('tel_slots', 512), slot)
        cls._log = Ring(rc.get('log_slots', 512), slot)
        cls._drain_ms = rc.get('drain_ms', 50)
        cls._boot_us = time.time_ns() // 1000
        cls._boot_epoch = time.time() - time.ticks_ms() // 1000
        cls._bus = config.get('buses', {}).get('uart_recorder')
        cls._uart = uart
        cls._refresh_session()

    @classmethod
    def uptime_us(cls):
        '''Monotonic microseconds since boot (no wrap).'''
        return time.time_ns() // 1000 - cls._boot_us

    @classmethod
    def set_clock(cls, now_epoch):
        '''CC time-sync: back-date boot from uptime so the session prefix matches boot time.
        Does not touch the hardware RTC (keeps uptime monotonic).'''
        cls._boot_epoch = now_epoch - cls.uptime_us() // 1000000
        cls._refresh_session()

    @classmethod
    def _refresh_session(cls):
        t = time.localtime(cls._boot_epoch)
        cls._session = '%04d%02d%02d_%02d%02d%02d' % (t[0], t[1], t[2], t[3], t[4], t[5])

    @classmethod
    def session(cls):
        return cls._session

    @classmethod
    def log(cls, descriptor, message):
        '''Append a log line "<uptime_us> <descriptor> :: <message>" (-> recorder.log).'''
        line = '%d %s :: %s\n' % (cls.uptime_us(), descriptor, message)
        return cls._log.write(line.encode())

    @classmethod
    def tlm(cls, filename, content):
        '''Append a telemetry line to a per-session file: "@<session>_<filename>@<content>".'''
        line = '@%s_%s@%s\n' % (cls._session, filename, content)
        return cls._tel.write(line.encode())

    @classmethod
    def drain(cls, sink, also=None, limit=None):
        '''Drain queued records to sink (telemetry first, then logs). sink.write(rec) happens
        before also(rec). Returns records drained (stops after `limit`).'''
        drained = 0
        for ring in (cls._tel, cls._log):
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
        '''Async drain loop. Uses `sink` (or the configured UART). Stops when stop[0] is true.'''
        if sink is None:
            if cls._uart is None and cls._bus is not None:
                from machine import UART
                b = cls._bus
                cls._uart = UART(1, baudrate=b['baud'], tx=b['tx'], rx=b['rx'])
            sink = cls._uart
        while stop is None or not stop[0]:
            cls.drain(sink)
            await asyncio.sleep_ms(cls._drain_ms)

    @classmethod
    def report(cls):
        return {'session': cls._session,
                'tel': {'count': cls._tel.count, 'dropped': cls._tel.dropped},
                'log': {'count': cls._log.count, 'dropped': cls._log.dropped}}


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
        row = '%d;%s' % (Recorder.uptime_us(), ';'.join(str(v) for v in values))
        Recorder.tlm(self.filename, row)
