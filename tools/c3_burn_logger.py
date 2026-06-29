# tools/c3_burn_logger.py -- standalone ESP32-C3 logger for the TMS-7 STATIC burn + separation ground
# test. Runs ON the C3 (MicroPython), NOT the P4 flight computer. It reads the ADXL375 (+/-200 g) and the
# LSM6DSO32 (+/-32 g accel + gyro) over a shared SPI bus and the separation switch on a GPIO, and writes a
# compact telemetry CSV to the C3's own flash (2 MB) -- a self-contained witness for the burn.
#
# Memory/flash thrift (the C3 has little RAM + 2 MB flash): every 10 samples (~100 ms) are reduced to
# min / avg / max. While IDLE (waiting for ignition, up to 1-2 min) it writes just ONE summary row per
# second. The moment a launch acceleration spike OR a separation is seen it latches FLIGHT and writes the
# full 100 ms summaries (10 rows/s) -- the interesting part is kept in detail -- then stops ~5 s after
# separation (the whole flight is < 5 s). So the long boring wait costs ~1 row/s and the flight is dense.
#
# ---- Wiring (ESP32-C3 supermini; left header 5,6,7,8,9,10,20,21 / right 4,3,2,1,0 + 5V,G,3V3) ----
#   SPI (shared):  SCK = GPIO4    MOSI = GPIO6    MISO = GPIO5
#   ADXL375    CS = GPIO7         LSM6DSO32 CS = GPIO1
#   data-ready INTs (OPTIONAL):  ADXL375 INT1 = GPIO10   LSM6DSO32 INT1 = GPIO0
#   (each sensor's CS+INT are grouped on one header -- ADXL on the left 7/10, LSM on the right 1/0)
#   separation switch = GPIO3  (pads route 3V3 -> GPIO3 when NESTED = HIGH; internal pull-down -> open = LOW)
#   sensors powered from 3V3 + G. Avoid GPIO2/8/9 (strapping) and GPIO20/21 (USB-serial REPL).
#
# The INT pins are OPTIONAL: wired, the loop reads exactly when a fresh conversion lands (~104 Hz, never
# misses a peak); unwired, it falls back to polling every ~15 ms. Wire them for a clean burn/landing capture.
#
# ---- Deploy + run ----  (the P4 firmware is untouched; this lives only on the C3)
#   mpremote connect /dev/ttyACM1 cp tools/c3_burn_logger.py :main.py     # auto-runs on every boot
#   mpremote connect /dev/ttyACM1 exec "import main; main.verify()"       # hardware check (after deploy,
#       3 s of live CSV to the REPL; no file). (mpremote `run` can't pass --verify -- it never sets argv.)
#   ... power the C3 from battery, ignite within ~2 min, let it fly, then USB in to pull ...
# Each boot writes the NEXT free file -- burn.00.csv, burn.01.csv, ... -- so an autonomous restart or a
# battery brownout NEVER overwrites a captured flight. Pull them over USB afterwards:
#   mpremote connect /dev/ttyACM1 fs ls                                   # list burn.NN.csv
#   mpremote connect /dev/ttyACM1 cp :burn.00.csv burn.00.csv            # pull each one
#
# ---- Output CSV + a VALIDATED example (manual shake + separate bench run) ----
# Columns: t_us (DEVICE UPTIME -- us since power-up), phase, sep, n, dt_us, then min/avg/max of
#   adxl_g, lsm_g, lsm_dps.
#   phase: start (boot state) | idle (1 row/s, decimated) | sep (a separation edge) | flight (full rate)
#   sep:   1 = nested (switch HIGH) / 0 = open    n,dt_us = samples in the row / their real span (-> ~Hz)
# A bench run (boot nested -> shake -> pull separation -> shake again) produced, e.g. (t_us = uptime):
#   318000,start,1,1,0,0.110,0.110,0.110,0.997,0.997,0.997,... <- start (~0.3 s after power-up): NESTED, 1 g
#   1006850,idle,1,159,986466,...                              <- idle: ~1 row/s, ~160 samples decimated
#   12476710,flight,1,10,52267,0.000,8.629,24.444,1.201,6.944,14.463,...   <- LAUNCH (shake1): 24 g / 14 g
#   14277405,sep,0,1,0,0.550,0.550,0.550,1.908,1.908,1.908,... <- SEPARATION edge (sep 1->0), timestamped
#   15741660,flight,0,10,55526,2.528,9.415,40.393,6.059,10.922,15.715,470.825,1346.848,2111.386
#                                                             <- shake2: ADXL 40 g, LSM 16 g, gyro 2111 dps
# Validated end-to-end: boot state + the separation edge are both logged, shakes captured at ~175 Hz
# (the data-ready INTs fire), auto-stopped 5 s after separation. LSM6DSO32 is the clean low-g channel
# (1.00 g at rest); the ADXL375 (49 mg/LSB, ~0.1-0.3 g offset at rest) is the high-g backstop -- it caught
# the sharp 40 g transient the LSM filtered to 16 g.

import asyncio
import math
import os
import struct
import sys
import time

from machine import SPI, Pin

# --- config (edit here), grouped per device ---

# shared SPI bus, mode 3 -- both parts ran clean at 10 MHz on the P4; 5 MHz is the safe ground choice
_SCK, _MOSI, _MISO, _SPI_HZ = 4, 6, 5, 5_000_000
_spi = SPI(1, baudrate=_SPI_HZ, polarity=1, phase=1, sck=Pin(_SCK), mosi=Pin(_MOSI), miso=Pin(_MISO))

# data-ready: either sensor's INT1 wakes the sample loop. An unwired INT pin sits LOW (pull-down) -> no
# edge -> the loop falls back to a _FALLBACK_MS poll. ThreadSafeFlag.set() is safe from the IRQ.
_ready = asyncio.ThreadSafeFlag()


def _on_ready(pin):
    _ready.set()


# ADXL375 -- +/-200 g high-g accel. CS + data-ready INT1 on the LEFT header.
_ADXL_CS, _ADXL_INT = 7, 10
_ADXL_ID, _ADXL_DEVID, _ADXL_DATAX0 = 0xE5, 0x00, 0x32   # id; X,Y,Z data = 6 bytes int16 LE
_ADXL_SCALE_G = 0.049                                    # ~49 mg/LSB
_cs_adxl = Pin(_ADXL_CS, Pin.OUT, value=1)
_int_adxl = Pin(_ADXL_INT, Pin.IN, Pin.PULL_DOWN)
_int_adxl.irq(_on_ready, Pin.IRQ_RISING)

# LSM6DSO32 -- +/-32 g accel + +/-2000 dps gyro. CS + data-ready INT1 on the RIGHT header.
_LSM_CS, _LSM_INT = 1, 0
_LSM_ID, _LSM_WHOAMI, _LSM_OUTX_L_G = 0x6C, 0x0F, 0x22   # id; gyro(6)+accel(6) = 12 bytes int16 LE
_LSM_SCALE_A, _LSM_SCALE_G = 0.000976, 0.070            # g/LSB @ +/-32 g ; dps/LSB @ +/-2000 dps
_cs_lsm = Pin(_LSM_CS, Pin.OUT, value=1)
_int_lsm = Pin(_LSM_INT, Pin.IN, Pin.PULL_DOWN)
_int_lsm.irq(_on_ready, Pin.IRQ_RISING)

# separation switch -- pads route 3V3 -> this pin when NESTED (HIGH); pull-down so open = LOW
_SEP = 3
_sep = Pin(_SEP, Pin.IN, Pin.PULL_DOWN)

# logging / behaviour
_FALLBACK_MS = 15          # loop wait when no data-ready INT fires (sensors ~104 Hz / ~9.6 ms)
_WINDOW = 10               # samples per min/avg/max summary (~100 ms)
_IDLE_WRITE_MS = 1000      # IDLE: one summary row per second
_LAUNCH_G = 3.0            # |a| over this (either accel) latches FLIGHT (E16 peak ~7.5 g)
_STOP_AFTER_SEP_MS = 5000  # stop this long after separation (the flight is < 5 s)
_STOP_AFTER_LAUNCH_MS = 30000  # stop 30 s after launch even if the switch never fires
_SEP_DEFER_US = 2000000    # ignore separation for first 2 s after boot (switch may be open pre-test)
_MAX_RUN_MS = 300000       # safety cap: never log past 5 min even if nothing fires
_PREFIX, _SUFFIX = 'burn.', '.csv'   # each boot writes the next free burn.NN.csv (NEVER overwrites)

_3H = '<3h'                          # precomputed format string for 3-channel struct.unpack


def _rd(cs, cmd, n):
    """Read n bytes starting at the command byte (0x80=read | 0x40=multi | reg)."""
    cs(0)
    _spi.write(bytes((cmd,)))
    data = _spi.read(n)
    cs(1)
    return data


def _wr(cs, reg, value):
    """Write one register (cmd byte = reg, bit7=0 write)."""
    cs(0)
    _spi.write(bytes((reg, value)))
    cs(1)


def _verify_id(cs, reg, expected, tries=5):
    """Read an id register, retrying -- the FIRST SPI read after bus bring-up can glitch (~10 % miss on
    this family, seen on the P4 too). Returns (last_value, ok)."""
    value = 0
    for _ in range(tries):
        value = _rd(cs, 0x80 | reg, 1)[0]
        if value == expected:
            return value, True
    return value, False


def setup_sensors():
    """Configure both parts; print + return (adxl_ok, lsm_ok) so the operator sees go/no-go before igniting."""
    adxl_id, adxl_ok = _verify_id(_cs_adxl, _ADXL_DEVID, _ADXL_ID)
    if adxl_ok:
        _wr(_cs_adxl, 0x31, 0x0B)   # DATA_FORMAT: full-res, 4-wire SPI
        _wr(_cs_adxl, 0x2C, 0x0A)   # BW_RATE: 100 Hz
        _wr(_cs_adxl, 0x2F, 0x00)   # INT_MAP: DATA_READY -> INT1
        _wr(_cs_adxl, 0x2E, 0x80)   # INT_ENABLE: DATA_READY
        _wr(_cs_adxl, 0x2D, 0x08)   # POWER_CTL: measure
    lsm_id, lsm_ok = _verify_id(_cs_lsm, _LSM_WHOAMI, _LSM_ID)
    if lsm_ok:
        _wr(_cs_lsm, 0x12, 0x44)    # CTRL3_C: BDU + IF_INC (auto-increment)
        _wr(_cs_lsm, 0x10, 0x44)    # CTRL1_XL: 104 Hz, +/-32 g
        _wr(_cs_lsm, 0x11, 0x4C)    # CTRL2_G: 104 Hz, +/-2000 dps
        _wr(_cs_lsm, 0x0D, 0x01)    # INT1_CTRL: accel data-ready -> INT1
    print('setup: ADXL375 id=0x%02X %s | LSM6DSO32 id=0x%02X %s | sep=%s' % (
        adxl_id, 'OK' if adxl_ok else 'FAIL', lsm_id, 'OK' if lsm_ok else 'FAIL',
        'NESTED' if _sep.value() else 'OPEN'))
    return adxl_ok, lsm_ok


def _mag(raw, scale):
    """|vector| of three int16 LE samples in `raw`, scaled."""
    buf = raw[:6]
    vals = struct.unpack(_3H, buf)
    return math.sqrt(sum((v * scale) ** 2 for v in vals))


def sample(adxl_ok, lsm_ok):
    """One reading: (adxl |a| g, lsm |a| g, lsm |gyro| dps, sep 0/1). Absent sensor -> 0.0."""
    adxl_g = _mag(_rd(_cs_adxl, 0xC0 | _ADXL_DATAX0, 6), _ADXL_SCALE_G) if adxl_ok else 0.0
    lsm_g = lsm_dps = 0.0
    if lsm_ok:
        raw = _rd(_cs_lsm, 0x80 | _LSM_OUTX_L_G, 12)   # gyro x,y,z then accel x,y,z
        lsm_dps = _mag(raw[:6], _LSM_SCALE_G)
        lsm_g = _mag(raw[6:12], _LSM_SCALE_A)
    return adxl_g, lsm_g, lsm_dps, _sep.value()


def _new():
    """Fresh accumulator: [min,max,sum] x3 metrics, then count, first-sample us, last-sample us -- the
    us pair lets each row carry the window's REAL span (n / dt_us = the true sample rate, with jitter)."""
    return [1e9, -1e9, 0.0, 1e9, -1e9, 0.0, 1e9, -1e9, 0.0, 0, 0, 0]


def _add(acc, adxl_g, lsm_g, lsm_dps, t_us):
    for i, v in enumerate((adxl_g, lsm_g, lsm_dps)):
        b = i * 3
        if v < acc[b]:
            acc[b] = v
        if v > acc[b + 1]:
            acc[b + 1] = v
        acc[b + 2] += v
    if acc[9] == 0:
        acc[10] = t_us       # first sample of the window
    acc[11] = t_us           # last sample of the window
    acc[9] += 1


def _row(handle, t_us, phase, sep, acc):
    """Write one CSV row: t_us, phase, sep, n, dt_us (window span) + min/avg/max of adxl_g, lsm_g, lsm_dps."""
    n = acc[9] or 1
    cells = [str(t_us), phase, str(sep), str(acc[9]), str(time.ticks_diff(acc[11], acc[10]))]
    for b in (0, 3, 6):
        cells += ['%.3f' % acc[b], '%.3f' % (acc[b + 2] / n), '%.3f' % acc[b + 1]]
    handle.write(','.join(cells) + '\n')
    try:
        handle.flush()        # the file handle flushes (durability vs brownout); sys.stdout (verify) has none
    except (AttributeError, OSError):
        pass


def _event(handle, t_us, phase, adxl_g, lsm_g, lsm_dps, sep):
    """Write a single-sample marker row -- the startup separation state ('start') or a separation edge
    ('sep') -- so the switch's history is explicit even between the decimated idle summaries."""
    acc = _new()
    _add(acc, adxl_g, lsm_g, lsm_dps, t_us)
    _row(handle, t_us, phase, sep, acc)


def _next_path():
    """The next free burn.NN.csv -- the device is autonomous on battery, so a restart/brownout must NOT
    overwrite a captured flight. Each boot picks the lowest unused index (00, 01, 02, ...)."""
    have = set(f for f in os.listdir() if f.startswith(_PREFIX) and f.endswith(_SUFFIX))
    n = 0
    while ('%s%02d%s' % (_PREFIX, n, _SUFFIX)) in have:
        n += 1
    return '%s%02d%s' % (_PREFIX, n, _SUFFIX)


async def main():
    adxl_ok, lsm_ok = setup_sensors()
    out = _next_path()
    handle = open(out, 'w')
    handle.write('t_us,phase,sep,n,dt_us,adxl_g_min,adxl_g_avg,adxl_g_max,'
                 'lsm_g_min,lsm_g_avg,lsm_g_max,lsm_dps_min,lsm_dps_avg,lsm_dps_max\n')
    handle.flush()
    print('logging to %s -- ignite within ~%ds' % (out, _MAX_RUN_MS // 1000))

    # t_us in every row is the DEVICE UPTIME (us since power-up): ticks_us() is esp_timer us-since-boot,
    # exact below its ~17.9 min wrap -- fine for a < 5 min run. Run-relative timing uses ticks_diff deltas.
    t0 = time.ticks_us()                   # uptime when logging began (~boot + main.py import)
    sec_acc, win_acc = _new(), _new()      # idle 1 s rows / flight 100 ms rows
    n = 0
    last_idle_us = t0
    phase = 'idle'
    launch_us = sep_us = None
    # record the separation state at startup -- if the switch is dead/miswired we see it now; and every
    # edge below. No 'sep' row after this means it never changed (the switch did not fire).
    adxl_g, lsm_g, lsm_dps, prev_sep = sample(adxl_ok, lsm_ok)
    _event(handle, t0, 'start', adxl_g, lsm_g, lsm_dps, prev_sep)
    rows = 1
    while True:
        adxl_g, lsm_g, lsm_dps, sep = sample(adxl_ok, lsm_ok)
        t_us = time.ticks_us()             # device uptime (us since power-up), per sample
        _add(sec_acc, adxl_g, lsm_g, lsm_dps, t_us)
        _add(win_acc, adxl_g, lsm_g, lsm_dps, t_us)
        n += 1

        if sep != prev_sep:                 # record EVERY separation edge with its (uptime) timestamp
            _event(handle, t_us, 'sep', adxl_g, lsm_g, lsm_dps, sep)
            prev_sep = sep
            rows += 1

        if phase == 'idle' and (adxl_g > _LAUNCH_G or lsm_g > _LAUNCH_G):
            phase = 'flight'
            launch_us = t_us
            win_acc = _new()                # drop the pre-launch idle samples from the first flight row
            print('LAUNCH at %dms uptime |a|=%.1fg' % (t_us // 1000, max(adxl_g, lsm_g)))
        if sep == 0 and sep_us is None and time.ticks_diff(t_us, t0) > _SEP_DEFER_US:
            sep_us = t_us
            if phase == 'idle':
                phase = 'flight'
                win_acc = _new()
            print('SEPARATION at %dms uptime' % (t_us // 1000))

        if phase == 'flight' and n % _WINDOW == 0:
            _row(handle, t_us, 'flight', sep, win_acc)
            win_acc = _new()
            rows += 1
        elif phase == 'idle' and time.ticks_diff(t_us, last_idle_us) >= _IDLE_WRITE_MS * 1000:
            _row(handle, t_us, 'idle', sep, sec_acc)
            sec_acc = _new()
            last_idle_us = t_us
            rows += 1

        if sep_us is not None and time.ticks_diff(t_us, sep_us) > _STOP_AFTER_SEP_MS * 1000:
            break
        if launch_us is not None and time.ticks_diff(t_us, launch_us) > _STOP_AFTER_LAUNCH_MS * 1000:
            print('LAUNCH timeout -- separation not seen, stopping')
            break
        if time.ticks_diff(t_us, t0) > _MAX_RUN_MS * 1000:
            print('timeout -- no launch/separation seen')
            break
        try:
            await asyncio.wait_for_ms(_ready.wait(), _FALLBACK_MS)  # wake on data-ready (~104 Hz) ...
        except asyncio.TimeoutError:
            pass  # ... else poll (INT unwired / silent)

    handle.close()
    print('done: %d rows -> %s (launch=%s sep=%s)' % (rows, out, launch_us is not None, sep_us is not None))


def verify():
    """Quick hardware verification: sample for ~3 s, print CSV to REPL. Returns True if all sensors OK."""
    adxl_ok, lsm_ok = setup_sensors()
    if not adxl_ok and not lsm_ok:
        print('VERIFY FAIL: no sensors responsive')
        return False
    print('VERIFY: sampling for ~3 s...')
    print('t_us,phase,sep,n,dt_us,adxl_g_min,adxl_g_avg,adxl_g_max,'
          'lsm_g_min,lsm_g_avg,lsm_g_max,lsm_dps_min,lsm_dps_avg,lsm_dps_max')
    start = time.ticks_us()
    win = _new()
    rows = 0
    while time.ticks_diff(time.ticks_us(), start) < 3_000_000:
        adxl_g, lsm_g, lsm_dps, sep = sample(adxl_ok, lsm_ok)
        t_us = time.ticks_diff(time.ticks_us(), start)
        _add(win, adxl_g, lsm_g, lsm_dps, t_us)
        if win[9] >= _WINDOW:
            _row(sys.stdout, t_us, 'verify', sep, win)
            win = _new()
            rows += 1
    print('VERIFY OK: %d rows (%d sensors)' % (rows, (1 if adxl_ok else 0) + (1 if lsm_ok else 0)))
    return True


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--verify':
        verify()
    else:
        asyncio.run(main())
