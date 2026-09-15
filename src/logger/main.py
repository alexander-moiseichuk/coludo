"""
TMS-7 nose logger: record the SEN0697 at 50 Hz from power-up to power-off, saving to flash as it goes.

Nothing is filtered: a flight can be a start and a drop inside one save window, so every sample is kept.
Records accumulate in RAM and are written as a new file every _SEGMENT records (~15 s). BOOT forces an
immediate save and blinks three times. When flash runs short the OLDEST logs are deleted -- only as many
as the new save needs.

Both sensors sample on their OWN clocks into their own FIFOs, and this loop drains them every 10 ms. That
is what makes a save lossless. Programming flash freezes the single core -- measured ~730 ms for a 72 KB
save, so ~180 ms for these 18 KB ones -- and through the freeze the BMI323 keeps queueing accel + gyro
(~3.4 s deep at 50 Hz) and the BMP581 keeps queueing pressure (32 frames, 0.64 s). The next drain collects
everything the stall would otherwise have cost. Temperature is read live: it moves too slowly to need a
FIFO, and putting it in the BMP581's halves that FIFO to 16 frames.

While NOTHING IS HAPPENING the rate drops to 1 Hz, and the first frame that moves restores 50 Hz -- so a
rocket lying hot in a closed zone for ten minutes costs ~14 KB instead of ~720 KB, and the pad wait costs
almost nothing, while the flight, the chute and every hand that picks it up stay at full rate. Stillness is
judged frame-to-frame, never against a stored resting pose: a pose would keep reading "moved" after the
airframe is set down in a new orientation, and the logger would never settle.

Timestamps come from the SENSOR's clock -- frame N is t0 + N * 20 ms, evenly spaced by construction. Each
file's header also carries the MCU clock at the save, so the host can check the two agree: a lost frame
shows as the sensor timeline falling a further 20 ms behind.

Files are bBBBBB_sSSSS.bin -- boot number (monotonic, kept in NVS) and segment -- and sort chronologically.
Header (20 bytes, <4sHHHHII): magic CLG3, record size, period ms, boot, segment, record count, MCU ms.
Record (24 bytes, <I6hii): ms, ax ay az gx gy gz, pressure, temperature -- all raw; decode.py scales them.

Nothing prints once sampling starts: with no USB host attached a console write can stall the loop.
"""

import gc
import os
import struct
import time

from machine import I2C, Pin
from micropython import const

try:
    from esp32 import NVS
except ImportError:
    NVS = None

_SDA = const(19)
_SCL = const(20)
_LED = const(15)              # plain status LED
_BOOT = const(9)              # BOOT: a strap only at reset, an ordinary input once running

_BMI323 = const(0x69)
_BMP581 = const(0x47)

_PERIOD_MS = const(20)        # 50 Hz, both sensors
_TICK_MS = const(10)          # drain cadence: twice per frame period
_FRAME = const(14)            # one BMI323 FIFO frame over I2C: 2 dummy bytes + accel x/y/z + gyro x/y/z
_FRAME_WORDS = const(6)
_PRESSURE_DEPTH = const(32)   # BMP581 FIFO depth, pressure-only frames
_RECORD = const(24)
_SEGMENT = const(750)         # records per file: ~15 s, 18 KB
_SLACK = const(256)           # headroom: a full BMI323 FIFO drains ~170 frames in one go
_HEADER = const(20)
_RESERVE = const(32768)       # flash kept free for filesystem metadata
_HEARTBEAT = const(50)        # LED toggle every 50 ticks (~0.5 s)
_BLINK_TICKS = const(15)      # 150 ms per blink phase, counted in ticks so sampling never pauses
_PRESS_TICKS = const(3)       # BOOT must read pressed on 3 consecutive ticks (~30 ms debounce)

# What counts as "something is happening". Raw LSB, so the test is integer comparisons in the hot loop.
_MOVE_LSB = const(100)        # 0.05 g of frame-to-frame accel change (2048 LSB/g); rest noise is ~5
_SPIN_LSB = const(80)         # ~5 dps on any axis (16.384 LSB/dps); rest noise measured ~1 dps
_LIFT_LSB = const(1536)       # ~24 Pa against the mark a second ago -- about 2 m, or 5 m/s under a chute
_QUIET_FRAMES = const(250)    # 5 s of stillness before decimating
_DECIMATE = const(50)         # while still, keep one frame a second

_MAGIC: bytes = b'CLG3'


def _write16(bus, register: int, value: int) -> None:
    """BMI323 registers are 16-bit, written LSB first."""
    bus.writeto_mem(_BMI323, register, bytes((value & 0xFF, value >> 8)))


def _setup(bus) -> int:
    """
    Configure both sensors into their FIFOs at 50 Hz; return a first pressure reading.

    Register values are from Bosch's own driver headers (bmi3_defs.h, bmp5_defs.h), not recalled.
    """
    bus.writeto_mem(_BMP581, 0x7E, b'\xb6')      # BMP581 soft reset: lands in standby, where it configures
    time.sleep_ms(5)
    bus.writeto_mem(_BMP581, 0x36, b'\x50')      # OSR_CONFIG: pressure on, 4x pressure / 1x temperature
    bus.writeto_mem(_BMP581, 0x18, b'\x02')      # FIFO_SEL: pressure-only frames (32 deep; 16 with temp)
    bus.writeto_mem(_BMP581, 0x16, b'\x00')      # FIFO_CONFIG: streaming
    bus.writeto_mem(_BMP581, 0x37, b'\xbd')      # ODR_CONFIG: deep standby off, 50 Hz, normal mode

    _write16(bus, 0x7E, 0xDEAF)                  # BMI323 soft reset
    time.sleep_ms(5)
    _write16(bus, 0x20, 0x7037)                  # ACC_CONF: high-performance, +/-16 g, 50 Hz
    _write16(bus, 0x21, 0x7047)                  # GYR_CONF: high-performance, +/-2000 dps, 50 Hz
    # The gyro reports 0x8000 (invalid) for its start-up time; wait it out before the FIFO starts, so the
    # first frame recorded is a real one. This also gives the BMP581 time for its first conversions.
    probe = bytearray(14)
    for _ in range(50):
        time.sleep_ms(10)
        bus.readfrom_mem_into(_BMI323, 0x03, probe)
        if probe[8:10] != b'\x00\x80' and probe[10:12] != b'\x00\x80' and probe[12:14] != b'\x00\x80':
            break
    _write16(bus, 0x36, 0x0600)                  # FIFO_CONF: accel + gyro frames, streaming
    _write16(bus, 0x37, 0x0001)                  # FIFO_CTRL: flush, so the first frame is fresh
    live = bytearray(3)
    bus.readfrom_mem_into(_BMP581, 0x20, live)   # a real pressure for the records before the first pairing
    return live[0] | (live[1] << 8) | (live[2] << 16)


def _moved(ax: int, ay: int, az: int, gx: int, gy: int, gz: int, pressure: int,
           previous_x: int, previous_y: int, previous_z: int, mark: int) -> bool:
    """
    Is anything happening? Frame-to-frame accel change, any rotation, or a climb against the mark.

    A pure function so it can be tested with synthetic values: proving it end to end on the bench would
    otherwise mean waiting out a segment at 1 Hz.
    """
    return (abs(ax - previous_x) > _MOVE_LSB or abs(ay - previous_y) > _MOVE_LSB
            or abs(az - previous_z) > _MOVE_LSB or abs(gx) > _SPIN_LSB or abs(gy) > _SPIN_LSB
            or abs(gz) > _SPIN_LSB or abs(pressure - mark) > _LIFT_LSB)


def _is_log(name: str) -> bool:
    """bBBBBB_sSSSS.bin -- nothing else is ever deleted."""
    return len(name) == 16 and name[0] == 'b' and name[6:8] == '_s' and name.endswith('.bin')


def _free() -> int:
    """Free filesystem bytes."""
    stat = os.statvfs('/')
    return stat[0] * stat[3]


def _make_room(need: int) -> None:
    """
    Delete the OLDEST logs, and only as many as it takes, until `need` bytes are free.

    Reclaimed space is counted from each deleted file's own size rather than re-read from statvfs, so the
    stop condition never depends on the filesystem's accounting catching up. File size under-credits (it
    ignores block rounding), so this can only err toward keeping data. Names sort chronologically:
    previous boots go before the current one.
    """
    free = _free()
    logs = sorted(name for name in os.listdir() if _is_log(name))
    while free < need and logs:
        name = logs.pop(0)
        free += os.stat(name)[6]
        os.remove(name)


def _next_boot() -> int:
    """Monotonic boot number: NVS if present, never below what the filesystem already holds."""
    boot = max([int(name[1:6]) for name in os.listdir() if _is_log(name)] or [0])
    store = None
    if NVS is not None:
        try:
            store = NVS('logger')
            boot = max(boot, store.get_i32('boot'))
        except OSError:
            pass
    boot += 1
    if store is not None:
        store.set_i32('boot', boot)
        store.commit()
    return boot


def _save(buffer, count: int, boot: int, segment: int) -> bool:
    """
    Write `count` records as one file, making room first. False if the write failed.

    A failed save drops that one segment and nothing else: the exception is caught here, because letting it
    escape would end the sampling loop and lose every segment still to come.
    """
    try:
        _make_room(_HEADER + count * _RECORD + _RESERVE)
        with open('b%05d_s%04d.bin' % (boot, segment), 'wb') as handle:
            handle.write(struct.pack('<4sHHHHII', _MAGIC, _RECORD, _PERIOD_MS, boot, segment, count,
                                     time.ticks_ms()))
            handle.write(memoryview(buffer)[:count * _RECORD])
        return True
    except OSError:
        return False


def run() -> None:
    """Drain both FIFOs every tick, save every _SEGMENT records and on BOOT."""
    led = Pin(_LED, Pin.OUT)
    boot_button = Pin(_BOOT, Pin.IN, Pin.PULL_UP)
    bus = I2C(0, scl=Pin(_SCL), sda=Pin(_SDA), freq=400000)
    pressure = _setup(bus)
    boot = _next_boot()
    gc.collect()                                 # allocate on a clean heap, then clean up after it
    buffer = bytearray((_SEGMENT + _SLACK) * _RECORD)
    level = bytearray(4)                         # 2 dummy bytes + FIFO_FILL_LEVEL (16-bit words)
    frame = bytearray(_FRAME)
    waiting_byte = bytearray(1)
    sample = bytearray(3)
    heat = bytearray(3)
    pressures = [0] * _PRESSURE_DEPTH
    gc.collect()
    print('logger :: boot %d, 50 Hz FIFO, 1 Hz when still, saving every %d records' % (boot, _SEGMENT))

    count, segment, beat, pressed, blink, frames_total = 0, 0, 0, 0, 0, 0
    origin = None
    active, quiet, mark = True, 0, pressure          # start at full rate: prove it is still before decimating
    previous_x, previous_y, previous_z = 0, 0, 0
    while True:
        started = time.ticks_ms()
        bus.readfrom_mem_into(_BMI323, 0x15, level)                  # FIFO_FILL_LEVEL
        frames = ((level[2] | (level[3] << 8)) & 0x07FF) // _FRAME_WORDS
        if frames:
            if origin is None:
                origin = time.ticks_add(started, -(frames - 1) * _PERIOD_MS)
            bus.readfrom_mem_into(_BMP581, 0x17, waiting_byte)       # FIFO_COUNT, frames
            waiting = min(waiting_byte[0] & 0x3F, _PRESSURE_DEPTH)
            for index in range(waiting):
                bus.readfrom_mem_into(_BMP581, 0x29, sample)         # FIFO_DATA: pressure XLSB, LSB, MSB
                pressures[index] = sample[0] | (sample[1] << 8) | (sample[2] << 16)
            bus.readfrom_mem_into(_BMP581, 0x1D, heat)               # live temperature
            temperature = heat[0] | (heat[1] << 8) | (heat[2] << 16)
            for index in range(frames):
                bus.readfrom_mem_into(_BMI323, 0x16, frame)          # FIFO_DATA: one frame
                aligned = index - (frames - waiting)                 # newest pressure pairs with newest IMU
                if aligned >= 0:
                    pressure = pressures[aligned]
                ax, ay, az, gx, gy, gz = struct.unpack_from('<6h', frame, 2)
                if _moved(ax, ay, az, gx, gy, gz, pressure, previous_x, previous_y, previous_z, mark):
                    active, quiet = True, 0
                else:
                    quiet += 1
                    if quiet >= _QUIET_FRAMES:
                        active = False
                previous_x, previous_y, previous_z = ax, ay, az
                frames_total += 1
                if frames_total % _DECIMATE == 0:
                    mark = pressure                                  # the climb test's one-second reference
                elif not active:
                    continue                                         # still: this frame is not worth a record
                offset = count * _RECORD
                struct.pack_into('<I', buffer, offset, time.ticks_add(origin, frames_total * _PERIOD_MS))
                buffer[offset + 4:offset + 16] = frame[2:_FRAME]
                struct.pack_into('<ii', buffer, offset + 16, pressure, temperature)
                count += 1

        forced = False
        if boot_button.value() == 0:
            pressed += 1
            forced = pressed == _PRESS_TICKS       # once per press, not once per tick held
        else:
            pressed = 0

        if count and (count >= _SEGMENT or forced):
            _save(buffer, count, boot, segment)
            segment += 1
            count = 0
            gc.collect()                           # the core is already stalled: collect in the same gap
            if forced:
                blink = 6 * _BLINK_TICKS           # three on/off pairs, counted down by the loop

        if blink:
            blink -= 1
            led.value(1 if (blink // _BLINK_TICKS) % 2 else 0)
        else:
            beat += 1
            if beat >= _HEARTBEAT:
                beat = 0
                led.value(1 - led.value())

        time.sleep_ms(max(0, _TICK_MS - time.ticks_diff(time.ticks_ms(), started)))


if __name__ == '__main__':            # at boot; `import main` from the REPL loads it without running
    run()
