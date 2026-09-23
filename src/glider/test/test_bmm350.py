"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the BMM350 driver (drivers/bmm350.py): registration, graceful setup when absent, the
24-bit axis decode, and the OPERATOR CALIBRATION that CC drives -- in particular that a partial turn is
REFUSED. Deterministic whether or not a BMM350 is wired (it is fitted only on v1.1 boards). Run by
`make test`.
"""

import asyncio
import math

import config_default
import task
from drivers import bmm350

"""
SUPPRESS NVS FOR THE WHOLE TEST. This runs on a FLIGHT board: without this, calibrate() below would
persist the fake circle's centre into the real 'bmm350_cal' key, and the next boot would restore it
and silently steer by it. A test constant has been latched into this board's live calibration once
before (the pitot tare) and it survived undetected for a day -- never again.
"""
bmm350._nvs = None


class _StubController:
    config = config_default.default()


class _FakeBus:
    """Feeds _read() a chosen (x, y, z) without touching hardware; z is fixed, only the circle matters."""

    def __init__(self):
        self.x: int = 0
        self.y: int = 0

    def _put(self, buf, offset, value):
        raw = value + (1 << 24) if value < 0 else value
        buf[offset], buf[offset + 1], buf[offset + 2] = raw & 0xFF, (raw >> 8) & 0xFF, (raw >> 16) & 0xFF

    async def read_into(self, _addr, _reg, buf):
        for index in range(len(buf)):
            buf[index] = 0
        self._put(buf, 2, self.x)
        self._put(buf, 5, self.y)
        self._put(buf, 8, 700)


def _probe():
    """A driver instance wired to a fake bus, with NVS suppressed so a test never writes the board's."""
    device = bmm350.Bmm350('mag', {}, _StubController())
    device._buf = bytearray(14)
    device._addr = 0x15
    device._bus = _FakeBus()
    device._span = None
    device._sectors = 0
    device._centre = None
    device._seen = 0
    device._calibration = None
    return device


async def _turn(device, degrees, centre_x=7500, centre_y=-4500, radius_x=4500, radius_y=4500, step=5):
    """Sweep the fake field through `degrees` of a circle offset by a hard iron, feeding every sample."""
    for angle in range(0, degrees, step):
        radians = math.radians(angle)
        device._bus.x = int(centre_x + radius_x * math.cos(radians))
        device._bus.y = int(centre_y + radius_y * math.sin(radians))
        await device._read()


async def amain():
    assert task.ACTIVITIES.get('bmm350') is bmm350.Bmm350  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = bmm350.Bmm350('mag', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    # a real bus but a bogus address (nothing acks) -> graceful False (Controller would skip it)
    absent = bmm350.Bmm350('mag', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, _StubController())
    assert await absent.setup() is False

    """
    Axis decode, both signs: 24-bit two's complement little-endian. A magnetometer reads NEGATIVE on
    half of every turn, so an unsigned decode still yields plausible-looking numbers -- and a heading
    that is wrong in exactly one half-plane.
    """
    device = _probe()
    device._bus.x, device._bus.y = -12345, 6789
    x, y, z = await device._read()
    assert (x, y, z) == (-12345, 6789, 700), (x, y, z)

    """
    NEGATIVE case 1: nothing seen yet, and a quarter turn. Both must REFUSE -- a quarter circle still
    yields a centre and two radii, confidently wrong, and a wrong hard iron biases heading by an amount
    that depends on where the nose points. Silent success here is worse than no calibration at all.
    """
    fresh = _probe()
    assert fresh.calibration(), 'a never-read magnetometer must ask for the turn'
    assert await fresh.calibrate() is not None, 'no samples must not calibrate'
    assert not fresh.calibrated()

    """
    The quarter turn is swept back and forth so it passes the sample-count floor: the gate under test is
    COVERAGE, and a refusal that only came from "too few samples" would leave it unexercised. This arc
    scores a perfect 1.00 span roundness (measured: x span == y span exactly) while putting the centre
    2250 counts out -- so the shape of the data cannot distinguish it, only the angles can.
    """
    quarter = _probe()
    for _ in range(3):
        await _turn(quarter, 90, step=1)
    assert quarter._seen >= 100, 'the sample floor must not be what refuses this: %d' % quarter._seen
    x_span, y_span = quarter._span[1] - quarter._span[0], quarter._span[3] - quarter._span[2]
    assert abs(x_span - y_span) < 150, 'the arc under test must look ROUND by span: %d vs %d' % (
        x_span, y_span)
    refusal = await quarter.calibrate()
    assert refusal is not None and 'sectors' in refusal, refusal
    assert quarter._covered() < 7, 'a quarter turn covers at most half the octants: %d' % quarter._covered()
    assert not quarter.calibrated(), 'a quarter turn must leave the axes raw'

    """
    NEGATIVE: a board SITTING STILL. Its field only moves by sensor noise (measured ~+/-100 counts on the
    v1.1 bench), and with a coverage test relative to the span alone that noise scattered into every
    octant: a still 7F reported "8 of 8 sectors covered" and calibrate() would have saved noise as the
    hard-iron calibration. It must cover nothing and be refused.
    """
    still = _probe()
    for index in range(400):
        still._bus.x = 13700 + (index * 37) % 201 - 100   # deterministic +/-100-count scatter
        still._bus.y = 16780 + (index * 53) % 201 - 100
        await still._read()
    assert still._covered() == 0, 'noise on a still board set %d sectors' % still._covered()
    refusal = await still.calibrate()
    assert refusal is not None and not still.calibrated(), refusal

    """
    NEGATIVE case 2: a full turn of a DEAD part (a constant field) has samples and perfect roundness of
    zero -- the span test is what catches it.
    """
    dead = _probe()
    for _ in range(200):
        await dead._read()
    refusal = await dead.calibrate()
    assert refusal is not None and 'alive' in refusal, refusal

    """
    NEGATIVE case 3, and the one span shape would get WRONG in the other direction: a genuinely
    ELLIPTICAL field (strong soft iron, here 2:1) is not a bad turn -- it is exactly what this
    calibration exists to correct, so a full turn of it must be ACCEPTED.
    """
    squashed = _probe()
    await _turn(squashed, 360, radius_x=6000, radius_y=3000, step=2)
    await _turn(squashed, 360, radius_x=6000, radius_y=3000, step=2)
    assert await squashed.calibrate() is None, 'an elliptical field is correctable, not a refusal'

    """
    POSITIVE: a full circle about a hard-iron centre. The captured centre must land on the real one, and
    afterwards the corrected axes must give the true heading back -- that is the whole point, so check
    the ANGLE, not the internals.
    """
    good = _probe()
    await _turn(good, 360, step=2)
    await _turn(good, 360, step=2)  # a second lap: the centre estimate settles on the first one
    assert good._covered() == 8, 'a full circle must reach every octant, got %d' % good._covered()
    assert await good.calibrate() is None, 'a full circle must calibrate'
    assert good.calibrated() and good.calibration() == '', 'calibrated -> nothing outstanding for CC'
    centre_x, centre_y, radius_x, radius_y = good._calibration
    assert abs(centre_x - 7500) <= 100 and abs(centre_y - -4500) <= 100, good._calibration
    assert abs(radius_x - 4500) <= 100 and abs(radius_y - 4500) <= 100, good._calibration

    worst = 0.0
    for degrees in (0, 37, 90, 154, 180, 271, 330):
        radians = math.radians(degrees)
        good._bus.x = int(7500 + 4500 * math.cos(radians))
        good._bus.y = int(-4500 + 4500 * math.sin(radians))
        corrected_x, corrected_y, _z = await good._read()
        recovered = math.degrees(math.atan2(corrected_y, corrected_x)) % 360
        error = abs((recovered - degrees + 180) % 360 - 180)
        assert error < 2.0, 'heading %d recovered as %.1f' % (degrees, recovered)
        worst = max(worst, error)

    """
    And the reason a hard iron cannot be left to attitude.py's learned track offset: UNCORRECTED, the
    same circle is read through an offset centre, so the error is not a constant bias -- it varies with
    heading and no single offset removes it.
    """
    raw = _probe()
    spread = []
    for degrees in (0, 90, 180, 270):
        radians = math.radians(degrees)
        raw._bus.x = int(7500 + 4500 * math.cos(radians))
        raw._bus.y = int(-4500 + 4500 * math.sin(radians))
        raw_x, raw_y, _z = await raw._read()
        spread.append((math.degrees(math.atan2(raw_y, raw_x)) - degrees + 180) % 360 - 180)
    assert max(spread) - min(spread) > 20.0, 'hard iron must show as a heading-DEPENDENT error, %r' % spread

    """
    A board with a RESTORED calibration that is re-calibrated on a partial turn must say REFUSED --
    calibration() returns '' once a calibration exists, and that '' used to be the answer, which reads
    as success while the old calibration silently stayed.
    """
    recal = bmm350.Bmm350('mag_recal_test', {}, _StubController())
    recal._calibration = (0, 0, 100, 100)  # restored from NVS
    recal._span = [-4000, 4000, -4000, 4000]    # a real field swing...
    recal._sectors = 0b00000111             # ...but only 3 of 8 sectors turned through
    recal._seen = 1000                      # plenty of samples: only the coverage is short
    answer = await recal.calibrate()
    assert answer and 'refused' in answer, answer
    assert recal._calibration == (0, 0, 100, 100), 'a refused re-calibration must keep the old one'

    """
    LIVE: where a BMM350 answers, it must MEASURE. The driver once never powered the OTP off, so every
    data register read 0x7F -- chip id and probe fine, and not one field sample ever produced. A still
    board's field only moves by noise, so the check is the part's own "no sample" pattern, plus the
    probe that now refuses it.
    """
    live = bmm350.Bmm350('mag_live', {'bus': 'i2c', 'id': 0, 'addr': 0x15,
                                      'provides': {'mag': {'priority': 9, 'timeout_ms': 500}}}, _StubController())
    if await live.setup():
        frames = []
        for _ in range(5):
            await asyncio.sleep_ms(20)
            await live._read()
            # offsets past the part's 2 dummy bytes -- literals, since an underscore const() is compiled away
            frames.append((live._axis(2), live._axis(5), live._axis(8)))
        assert all(frame[0] != 0x7F7F7F for frame in frames), 'the BMM350 is not measuring: %r' % frames
        assert await live.probe() is None
        print('   live BMM350 measuring: %r' % (frames[-1],))
    else:
        print('   no BMM350 answering -- live check skipped (fitted on v1.1 only)')

    print('ok: bmm350 registered; graceful-absent; partial turn refused; centre (%d, %d) r (%d, %d); '
          'heading worst %.2f deg, uncorrected spread %.0f deg' % (
              centre_x, centre_y, radius_x, radius_y, worst, max(spread) - min(spread)))


asyncio.run(amain())
