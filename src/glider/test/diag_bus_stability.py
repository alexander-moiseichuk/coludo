"""
Is every bus on this board carrying bits intact? Ten thousand reads per device, all buses.

The question a rewire actually raises is not "does each device answer" -- `diag_devices` covers that in
one read apiece -- but "does it answer CORRECTLY, every time, for as long as a flight". A marginal
joint, an under-driven pull-up or a harness run too fast for its length does not fail cleanly: it
corrupts one frame in hundreds, which a single read never sees and a flight turns into one wrong
altitude every half minute.

Two detectors, because the parts differ:

  * **CRC devices** (ICP-10111, SDP810) carry Sensirion checksums the drivers already validate, so a
    refused frame is corruption the part itself caught.
  * **everything else** is read at a register whose value is a CONSTANT -- a chip id. It cannot
    legitimately change, so any deviation is a bit that did not survive the wire. That makes a part
    with no checksum just as measurable as one with.

NAK and timeout are counted SEPARATELY from corruption. A device that stops answering is a connector
or a wedged part; a device that answers wrongly is signal integrity. Conflating them would let a loose
plug read as a bus that needs slowing down.

    mpremote connect $PORT run src/glider/test/diag_bus_stability.py
"""

import asyncio

import config
import i2cbus
import layout
import spibus
from drivers import sdp810 as sdp_driver  # its _frame_ok is the checker the flight path uses

_FRAMES: int = 10000  # per device; enough that a 0.05 % rate is ~5 events rather than 0 or 1

"""
(name, register, width, expected, addrsize) for the constant-value parts. addrsize None means SPI.
Every value here is the part's own identity register, taken from its driver -- a constant by
construction, which is what makes a mismatch unambiguous.
"""
_CONSTANT: tuple = (
    ('imu_bno055', 0x00, 1, 0xA0, 8),
    ('baro_bmp280', 0xD0, 1, 0x58, 8),
    ('power_ina226', 0xFF, 2, 0x2260, 8),
    ('laser_agl', 0x010F, 2, 0xEBAA, 16),
    ('imu_lsm6dso32', 0x0F, 1, 0x6C, None),
    ('accel_adxl375', 0x00, 1, 0xE5, None),
)

_CRC_DEVICES: tuple = ('baro_icp10111', 'airspeed_sdp810')


def _value(raw: bytes) -> int:
    """Big-endian integer from a 1- or 2-byte register read."""
    return raw[0] if len(raw) == 1 else (raw[0] << 8) | raw[1]


async def _constant(cfg: dict, name: str, reg: int, width: int, expected: int, addrsize) -> tuple:
    """
    Read one part's identity register `_FRAMES` times; return (good, corrupt, bus_error, where).

    Corrupt means the constant came back as something else -- the register cannot change, so the wire
    is the only explanation.
    """
    device = config.device(cfg, name=name)
    if device is None or not device.get('enabled', True):
        return None
    good = corrupt = bus_error = 0
    if addrsize is None:
        bus = spibus.bind(cfg, device)
        if bus is None:
            return None
        window = bus.device(cfg['pins'][device['cs_pin']],
                            mb_bit=None if 'lsm' in name else 6)
        where = 'spi:%s' % device.get('id', 1)
        for _ in range(_FRAMES):
            try:
                raw = await window.read(reg, width)
                good += 1 if _value(raw) == expected else 0
                corrupt += 0 if _value(raw) == expected else 1
            except Exception:
                bus_error += 1
    else:
        bus, addr = i2cbus.bind(cfg, device, device.get('addr', 0))
        if bus is None:
            return None
        where = 'i2c:%s' % device.get('id')
        for _ in range(_FRAMES):
            try:
                raw = await bus.read(addr, reg, width, addrsize=addrsize)
                good += 1 if _value(raw) == expected else 0
                corrupt += 0 if _value(raw) == expected else 1
            except Exception:
                bus_error += 1
    return good, corrupt, bus_error, where


async def _crc(flight, cfg: dict, name: str) -> tuple:
    """
    Exercise a CRC part's own read path `_FRAMES` times; return (good, corrupt, bus_error, where).

    Uses the driver rather than a raw register read, because the checksum IS the detector here and the
    driver is what validates it -- a refused frame lands as ValueError, a dead bus as OSError.
    """
    unit = flight.active(name)
    if unit is None:
        return None
    device = config.device(cfg, name=name)
    where = 'i2c:%s' % (device or {}).get('id')

    """
    The two CRC parts are read differently because they ARE different: the ICP-10111 validates inside
    its own `_measure()` and raises on a bad frame, while the SDP810 streams continuously and validates
    inline in its run loop, with no method to call. So the SDP810's 9-byte frame is pulled straight off
    the bus and handed to the driver's own `_frame_ok` -- the same checker the flight path uses, rather
    than a second implementation that could disagree with it.
    """
    good = corrupt = bus_error = 0
    if name == 'airspeed_sdp810':
        for _ in range(_FRAMES):
            try:
                frame = await unit._bus.readfrom(unit._addr, sdp_driver._FRAME)
                good += 1 if sdp_driver._frame_ok(frame) else 0
                corrupt += 0 if sdp_driver._frame_ok(frame) else 1
            except Exception:
                bus_error += 1
        return good, corrupt, bus_error, where

    for _ in range(_FRAMES):
        try:
            await unit._measure()
            good += 1
        except ValueError:
            corrupt += 1      # the part's own checksum refused the frame
        except Exception:
            bus_error += 1    # NAK / timeout: a different fault entirely
    return good, corrupt, bus_error, where


async def run():
    board, source, _errors = config.load()
    revision = layout.resolve(board, log=lambda line: None)
    import main
    flight = await main.bringup(board, log=lambda line: None)
    await asyncio.sleep_ms(1200)

    print('layout %s   %d frames per device' % (revision, _FRAMES))
    print('%-20s %-8s %8s %9s %9s  %s' % ('device', 'bus', 'good', 'corrupt', 'bus err', 'rate'))
    rows = []
    for name in _CRC_DEVICES:
        result = await _crc(flight, board, name)
        if result:
            rows.append((name,) + result)
    for name, reg, width, expected, addrsize in _CONSTANT:
        result = await _constant(board, name, reg, width, expected, addrsize)
        if result:
            rows.append((name,) + result)

    worst = 0.0
    for name, good, corrupt, bus_error, where in rows:
        total = good + corrupt + bus_error
        rate = 100.0 * corrupt / total if total else 0.0
        worst = max(worst, rate)
        flag = '' if not corrupt and not bus_error else '   <-- LOOK'
        print('%-20s %-8s %8d %9d %9d  %.3f %%%s' % (name, where, good, corrupt, bus_error, rate, flag))

    print('')
    print('worst corruption rate: %.3f %%   (v0.1 baseline on i2c:0 @ 400 kHz was 0.50 %%)' % worst)
    print('DONE')

asyncio.run(run())
