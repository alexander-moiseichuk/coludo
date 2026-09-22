"""
Decode TMS-7 nose-logger files to CSV -- one row per sample, every reading, nothing filtered.

Host-side (CPython). Reads every format the logger has written: `log.NN.bin` (CLG1, first bench build, no
temperature), `bBBBBB_sSSSS.bin` CLG2 (100 Hz polled) and CLG3 (50 Hz FIFO, MCU clock in the header).
Writes FILE.csv beside each input and prints a summary, then checks CONTINUITY across consecutive
segments -- each should start exactly one period after the last one ended, which is the proof that a
flash save cost no samples.

    python3 src/logger/decode.py launches/20261003/TMS-7/logger/*.bin
"""

import csv
import math
import struct
import sys

_G: float = 2048.0            # BMI323 LSB per g at +/-16 g
_DPS: float = 16.384          # BMI323 LSB per deg/s at +/-2000 dps
_R_AIR: float = 287.05        # J/(kg K), dry air
_GRAVITY: float = 9.80665     # m/s^2
_DECIMATE: int = 50           # mirrors main.py: one frame in 50 is kept while the airframe is still.
                              # Cannot be imported -- that is firmware for another chip -- so a change
                              # there must be mirrored here, and the continuity check below is what
                              # would notice: idle joins would start reading as lost samples.
_INVALID: int = -32768        # BMI323's "no sample yet" marker (0x8000) -- never a reading
_ACCEL_DUMMY: int = 0x7F01    # BMI323 FIFO: this frame carries no new accel sample (marker in the x word)
_GYRO_DUMMY: int = 0x7F02     # ... no new gyro sample
_TICKS: int = 1 << 30         # MicroPython ticks_ms wraps here

_COLUMNS: tuple = ('index', 't_s', 'dt_ms', 'ax_g', 'ay_g', 'az_g', 'a_g', 'gx_dps', 'gy_dps', 'gz_dps',
                   'pressure_pa', 'temp_c', 'alt_rel_m', 'tick_ms')


def _whole(data: bytes, offset: int, size: int, count: int) -> int:
    """
    How many of the header's `count` records are actually present, whole, in `data`.

    A segment cut short -- power lost mid-save, or a read-out tool's Ctrl-C landing during one -- still
    carries the header's full count. Unpacking past the end raised, and one such file took the whole
    batch down with it, so the flight's good segments could not be decoded either.
    """
    present = max(0, (len(data) - offset) // size) if size else 0
    if present < count:
        print('  truncated: %d of %d records present -- decoding the whole ones' % (present, count))
    return min(count, present)


def _records(data: bytes) -> tuple:
    """
    (header, rows). header: dict of boot, segment, period, mcu (None where the format lacks it).
    Each row is (ms, ax, ay, az, gx, gy, gz, pressure_raw, temperature_raw or None).
    """
    if data[:4] == b'CLG1':
        size, count, period = struct.unpack_from('<HIH', data, 4)
        count = _whole(data, 12, size, count)
        rows = [struct.unpack_from('<I6hi', data, 12 + i * size) + (None,) for i in range(count)]
        return {'boot': None, 'segment': None, 'period': period, 'mcu': None}, rows
    if data[:4] == b'CLG2':
        _, size, period, boot, segment, count = struct.unpack_from('<4sHHHHI', data, 0)
        count = _whole(data, 16, size, count)
        rows = [struct.unpack_from('<I6hii', data, 16 + i * size) for i in range(count)]
        return {'boot': boot, 'segment': segment, 'period': period, 'mcu': None}, rows
    if data[:4] == b'CLG3':
        _, size, period, boot, segment, count, mcu = struct.unpack_from('<4sHHHHII', data, 0)
        count = _whole(data, 20, size, count)
        rows = [struct.unpack_from('<I6hii', data, 20 + i * size) for i in range(count)]
        return {'boot': boot, 'segment': segment, 'period': period, 'mcu': mcu}, rows
    raise ValueError('not a logger file (magic %r)' % data[:4])


def _temperature(raw):
    """BMP581 temperature in degC: 24-bit two's complement, 1/65536 degC per LSB. None when not recorded."""
    if raw is None:
        return None
    return (raw - (1 << 24) if raw & 0x800000 else raw) / 65536.0


def _altitude(pascal: float, reference: float, celsius, reference_celsius) -> float:
    """
    Height above the file's first sample, in metres.

    Hypsometric with the MEASURED temperature when the file carries one. The standard-atmosphere formula
    assumes 15 C, and height per pascal scales with absolute temperature, so a 30 C pad reads ~5% low --
    about 15 m on a 300 m apogee. The layer temperature is the mean of its two ends. The BMP581 die sits
    beside the MCU and reads a few degrees warm; that is still far closer than 15 C on a Florida pad.
    """
    if celsius is None:
        return 44330.0 * (1.0 - (pascal / reference) ** (1 / 5.255))
    kelvin = (celsius + reference_celsius) / 2.0 + 273.15
    return _R_AIR * kelvin / _GRAVITY * math.log(reference / pascal)


def _cell(value, fmt: str) -> str:
    """A formatted reading, or an empty cell where the chip had none."""
    return '' if value is None else fmt % value


def reference_of(rows: list) -> tuple:
    """(pressure_pa, celsius) of the first sample with a real pressure, or (None, None)."""
    for row in rows:
        if row[7]:
            return row[7] / 64.0, _temperature(row[8])
    return None, None


def decode(path: str, header: dict, rows: list, reference: float, reference_celsius) -> dict:
    """
    Write PATH.csv, print a one-file summary; return the header plus first/last timestamps.

    The altitude `reference` is passed IN rather than taken from this file's first sample, because a
    flight spans many files -- a new one every ~15 s -- and re-zeroing each of them would put apogee at
    roughly zero in every segment. One reference per boot makes alt_rel_m comparable across the flight,
    which is the only way the apogee this payload exists to measure is visible at all.
    """
    label = 'bench' if header['boot'] is None else 'boot %d segment %d' % (header['boot'], header['segment'])
    start = rows[0][0]
    out = path.rsplit('.', 1)[0] + '.csv'
    peak_a = peak_w = 0.0
    gaps, heights = [], []
    with open(out, 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(_COLUMNS)
        previous = start
        for index, (ms, ax, ay, az, gx, gy, gz, pressure, temperature) in enumerate(rows):
            """
            The chip's "no sample" markers become empty cells and stay out of every peak -- but _INVALID
            applies to the GYRO ONLY. On a +/-16 g accel, -32768 is the negative saturation RAIL, a real
            reading of -16.0 g, and blanking it deletes exactly the sample a boost exists to capture: a
            mounting sign that puts thrust on the negative axis would report a railed boost as no boost
            at all. main.py only ever documents 0x8000 as a gyro start-up value, and the bench captures
            agree -- gx hit it twice per file (all three gyro axes at once, accel valid in the same
            record), ax never once.
            """
            dead_a, dead_w = ax == _ACCEL_DUMMY, gx in (_INVALID, _GYRO_DUMMY)
            a = [None if dead_a else v / _G for v in (ax, ay, az)]
            w = [None if dead_w else v / _DPS for v in (gx, gy, gz)]
            magnitude = None if None in a else (a[0] ** 2 + a[1] ** 2 + a[2] ** 2) ** 0.5
            pascal = pressure / 64.0 if pressure else None
            celsius = _temperature(temperature)
            height = None if pascal is None else _altitude(pascal, reference, celsius, reference_celsius)
            dt = (ms - previous) & 0xFFFFFFFF
            gaps.append(dt)
            if height is not None:
                heights.append(height)
            peak_a = max(peak_a, magnitude or 0.0)
            peak_w = max([peak_w] + [abs(v) for v in w if v is not None])
            writer.writerow([index, '%.3f' % (((ms - start) & 0xFFFFFFFF) / 1000.0), dt,
                             _cell(a[0], '%.4f'), _cell(a[1], '%.4f'), _cell(a[2], '%.4f'),
                             _cell(magnitude, '%.4f'), _cell(w[0], '%.2f'), _cell(w[1], '%.2f'),
                             _cell(w[2], '%.2f'), _cell(pascal, '%.2f'), _cell(celsius, '%.2f'), _cell(height, '%.2f'),
                             ms])
            previous = ms
    span = ((rows[-1][0] - start) & 0xFFFFFFFF) / 1000.0
    temperatures = [_temperature(r[8]) for r in rows if r[8] is not None]
    print('%s  [%s]  %d samples, %.1f s, %.1f Hz, max gap %d ms' % (
        out, label, len(rows), span, (len(rows) - 1) / span if span else 0, max(gaps[1:] or [0])))
    print('    peak |a| %.2f g   peak |gyro| %.0f dps   altitude %+.2f..%+.2f m   temperature %s' % (
        peak_a, peak_w, min(heights or [0]), max(heights or [0]),
        '%.2f..%.2f C' % (min(temperatures), max(temperatures)) if temperatures else 'not recorded'))
    if header['mcu'] is not None:
        # sensor clock vs MCU clock at the save: ~one period is normal; each LOST frame adds a period
        # signed: ticks wrap at 2^30, and the timeline can sit slightly AHEAD of the MCU clock because the
        # anchor is taken at a drain. What matters is that it does not GROW -- each lost frame adds a period.
        lag = ((header['mcu'] - rows[-1][0] + _TICKS // 2) % _TICKS) - _TICKS // 2
        print('    sensor timeline vs MCU clock at the save: %+d ms' % lag)
    return dict(header, first=rows[0][0], last=rows[-1][0])


def _continuity(summaries: list) -> None:
    """Consecutive segments of one boot must join with exactly one period between them."""
    joins = [(a, b) for a, b in zip(summaries, summaries[1:])
             if a['boot'] is not None and a['boot'] == b['boot'] and b['segment'] == a['segment'] + 1]
    if not joins:
        return
    """
    Two spacings are legal at a join, not one.

    A save that lands inside a quiet stretch joins one DECIMATION interval later, because the firmware is
    keeping one frame in _DECIMATE while the airframe is still. Treating that as loss would have this
    print 'samples lost' for the most common case of all -- a rocket sitting on the pad.
    """
    period = joins[0][0]['period']
    gaps = [(b['first'] - a['last']) % _TICKS for a, b in joins]
    legal = [period, period * _DECIMATE]
    lost = [gap for gap in gaps if gap not in legal]
    print('continuity over %d saves: %s' % (
        len(gaps),
        'LOSSLESS (joins at %s ms)' % sorted(set(gaps)) if not lost
        else 'SAMPLES LOST -- joins of %s ms, legal are %s' % (sorted(set(lost)), legal)))


if __name__ == '__main__':
    # One pass to read every file, so each boot's altitude reference comes from its LOWEST segment and
    # every later segment measures against the same ground.
    loaded = []
    for name in sorted(sys.argv[1:]):
        try:
            with open(name, 'rb') as handle:
                header, rows = _records(handle.read())
        except (ValueError, struct.error) as error:
            # one bad file (empty, foreign, a header cut short) is reported and SKIPPED -- it used to
            # abort the batch and cost every good segment of the flight along with it
            print('%s: SKIPPED -- %s' % (name, error))
            continue
        loaded.append((name, header, rows))
    references = {}
    for _name, header, rows in loaded:
        key = header['boot']
        if key not in references:
            references[key] = reference_of(rows)
    _continuity([decode(name, header, rows, *references[header['boot']]) for name, header, rows in loaded])
