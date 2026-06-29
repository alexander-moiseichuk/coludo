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
#   ADXL375    CS = GPIO7         LSM6DSO32 CS = GPIO10
#   separation switch = GPIO3  (pads route 3V3 -> GPIO3 when NESTED = HIGH; internal pull-down -> open = LOW)
#   sensors powered from 3V3 + G. Avoid GPIO2/8/9 (strapping) and GPIO20/21 (USB-serial REPL).
#
# ---- Deploy + run ----  (the P4 firmware is untouched; this lives only on the C3)
#   mpremote connect /dev/ttyACM1 cp tools/c3_burn_logger.py :main.py     # auto-runs on every boot
#   ... power the C3, watch the REPL for 'setup', ignite within ~2 min, let it fly ...
#   mpremote connect /dev/ttyACM1 cp :burn.csv burn.csv                   # pull the log afterwards

import asyncio
import math
import struct
import time

from machine import SPI, Pin

# --- wiring + tuning (edit here) ---
_PIN_SCK, _PIN_MOSI, _PIN_MISO = 4, 6, 5
_PIN_CS_ADXL, _PIN_CS_LSM, _PIN_SEP = 7, 10, 3
_SPI_HZ = 5_000_000      # both parts ran clean at 10 MHz on the P4; 5 MHz is the safe ground-test choice
_SAMPLE_MS = 10          # ~100 Hz sample loop
_WINDOW = 10             # samples per min/avg/max summary (~100 ms)
_IDLE_WRITE_MS = 1000    # IDLE: one summary row per second
_LAUNCH_G = 3.0          # |a| over this (either accel) latches FLIGHT (E16 peak ~7.5 g)
_STOP_AFTER_SEP_MS = 5000  # stop this long after separation (the flight is < 5 s)
_MAX_RUN_MS = 300000     # safety cap: never log past 5 min even if nothing fires
_OUT = 'burn.csv'

# --- sensor registers (from the P4 drivers, simplified) ---
_ADXL_DEVID, _ADXL_ID = 0x00, 0xE5
_ADXL_DATAX0 = 0x32             # X,Y,Z = 6 bytes, int16 LE
_ADXL_SCALE_G = 0.049          # ADXL375 ~49 mg/LSB
_LSM_WHOAMI, _LSM_ID = 0x0F, 0x6C
_LSM_OUTX_L_G = 0x22           # gyro(6) + accel(6) = 12 bytes, int16 LE
_LSM_SCALE_A = 0.000976        # g/LSB at +/-32 g
_LSM_SCALE_G = 0.070           # dps/LSB at +/-2000 dps

_spi = SPI(1, baudrate=_SPI_HZ, polarity=1, phase=1,  # SPI mode 3 (ADXL/LSM)
           sck=Pin(_PIN_SCK), mosi=Pin(_PIN_MOSI), miso=Pin(_PIN_MISO))
_cs_adxl = Pin(_PIN_CS_ADXL, Pin.OUT, value=1)
_cs_lsm = Pin(_PIN_CS_LSM, Pin.OUT, value=1)
_sep = Pin(_PIN_SEP, Pin.IN, Pin.PULL_DOWN)


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


def setup_sensors():
    """Configure both parts; print + return (adxl_ok, lsm_ok) so the operator sees go/no-go before igniting."""
    adxl_id = _rd(_cs_adxl, 0x80 | _ADXL_DEVID, 1)[0]
    adxl_ok = adxl_id == _ADXL_ID
    if adxl_ok:
        _wr(_cs_adxl, 0x31, 0x0B)   # DATA_FORMAT: full-res, 4-wire SPI
        _wr(_cs_adxl, 0x2C, 0x0A)   # BW_RATE: 100 Hz
        _wr(_cs_adxl, 0x2D, 0x08)   # POWER_CTL: measure
    lsm_id = _rd(_cs_lsm, 0x80 | _LSM_WHOAMI, 1)[0]
    lsm_ok = lsm_id == _LSM_ID
    if lsm_ok:
        _wr(_cs_lsm, 0x12, 0x44)    # CTRL3_C: BDU + IF_INC (auto-increment)
        _wr(_cs_lsm, 0x10, 0x44)    # CTRL1_XL: 104 Hz, +/-32 g
        _wr(_cs_lsm, 0x11, 0x4C)    # CTRL2_G: 104 Hz, +/-2000 dps
    print('setup: ADXL375 id=0x%02X %s | LSM6DSO32 id=0x%02X %s | sep=%s' % (
        adxl_id, 'OK' if adxl_ok else 'FAIL', lsm_id, 'OK' if lsm_ok else 'FAIL',
        'NESTED' if _sep.value() else 'OPEN'))
    return adxl_ok, lsm_ok


def _mag(raw, scale, count):
    """|vector| of the first `count` int16 LE samples in `raw`, scaled."""
    vals = struct.unpack('<%dh' % count, raw[:count * 2])
    return math.sqrt(sum((v * scale) ** 2 for v in vals))


def sample(adxl_ok, lsm_ok):
    """One reading: (adxl |a| g, lsm |a| g, lsm |gyro| dps, sep 0/1). Absent sensor -> 0.0."""
    adxl_g = _mag(_rd(_cs_adxl, 0xC0 | _ADXL_DATAX0, 6), _ADXL_SCALE_G, 3) if adxl_ok else 0.0
    lsm_g = lsm_dps = 0.0
    if lsm_ok:
        raw = _rd(_cs_lsm, 0x80 | _LSM_OUTX_L_G, 12)   # gyro x,y,z then accel x,y,z
        lsm_dps = _mag(raw[0:6], _LSM_SCALE_G, 3)
        lsm_g = _mag(raw[6:12], _LSM_SCALE_A, 3)
    return adxl_g, lsm_g, lsm_dps, _sep.value()


def _new():
    """A fresh min/avg/max accumulator for the 3 metrics: [min,max,sum] x3 + count."""
    return [1e9, -1e9, 0.0, 1e9, -1e9, 0.0, 1e9, -1e9, 0.0, 0]


def _add(acc, adxl_g, lsm_g, lsm_dps):
    for i, v in enumerate((adxl_g, lsm_g, lsm_dps)):
        b = i * 3
        if v < acc[b]:
            acc[b] = v
        if v > acc[b + 1]:
            acc[b + 1] = v
        acc[b + 2] += v
    acc[9] += 1


def _row(handle, t_ms, phase, sep, acc):
    """Write one CSV row: t,phase,sep + min/avg/max of adxl_g, lsm_g, lsm_dps."""
    n = acc[9] or 1
    cells = [str(t_ms), phase, str(sep)]
    for b in (0, 3, 6):
        cells += ['%.3f' % acc[b], '%.3f' % (acc[b + 2] / n), '%.3f' % acc[b + 1]]
    handle.write(','.join(cells) + '\n')
    handle.flush()


async def main():
    adxl_ok, lsm_ok = setup_sensors()
    handle = open(_OUT, 'w')
    handle.write('t_ms,phase,sep,adxl_g_min,adxl_g_avg,adxl_g_max,'
                 'lsm_g_min,lsm_g_avg,lsm_g_max,lsm_dps_min,lsm_dps_avg,lsm_dps_max\n')
    handle.flush()
    print('logging to %s -- ignite within ~%ds' % (_OUT, _MAX_RUN_MS // 1000))

    start = time.ticks_ms()
    sec_acc, win_acc = _new(), _new()      # idle 1 s rows / flight 100 ms rows
    n = 0
    last_idle = start
    phase = 'idle'
    launch_ms = sep_ms = None
    rows = 0
    while True:
        adxl_g, lsm_g, lsm_dps, sep = sample(adxl_ok, lsm_ok)
        now = time.ticks_ms()
        _add(sec_acc, adxl_g, lsm_g, lsm_dps)
        _add(win_acc, adxl_g, lsm_g, lsm_dps)
        n += 1

        if phase == 'idle' and (adxl_g > _LAUNCH_G or lsm_g > _LAUNCH_G):
            phase = 'flight'
            launch_ms = now
            win_acc = _new()                # drop the pre-launch idle samples from the first flight row
            print('LAUNCH +%dms |a|=%.1fg' % (time.ticks_diff(now, start), max(adxl_g, lsm_g)))
        if sep == 0 and sep_ms is None:
            sep_ms = now
            if phase == 'idle':
                phase = 'flight'
                win_acc = _new()
            print('SEPARATION +%dms' % time.ticks_diff(now, start))

        if phase == 'flight' and n % _WINDOW == 0:
            _row(handle, time.ticks_diff(now, start), 'flight', sep, win_acc)
            win_acc = _new()
            rows += 1
        elif phase == 'idle' and time.ticks_diff(now, last_idle) >= _IDLE_WRITE_MS:
            _row(handle, time.ticks_diff(now, start), 'idle', sep, sec_acc)
            sec_acc = _new()
            last_idle = now
            rows += 1

        if sep_ms is not None and time.ticks_diff(now, sep_ms) > _STOP_AFTER_SEP_MS:
            break
        if time.ticks_diff(now, start) > _MAX_RUN_MS:
            print('timeout -- no launch/separation seen')
            break
        await asyncio.sleep_ms(_SAMPLE_MS)

    handle.close()
    print('done: %d rows -> %s (launch=%s sep=%s)' % (rows, _OUT, launch_ms is not None, sep_ms is not None))


asyncio.run(main())
