"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the NEO-6M GNSS driver (drivers/neo6mv2.py): @task.driver('neo6mv2') registration,
that it subclasses the shared gnss.Gnss base, the UBX binary framing/checksum, the command bytes in
send order -- the flight init, then the sky diagnostics -- for hz 5 / 1 / 10 and the edges (0, the
one-byte rate cap), graceful setup on an undefined bus, that the u-blox PUBX/UBX reconfiguration runs
without error on uart:2, and the sky switch's two frames (off in flight, on at the pad) for hz 10 / 5.
NMEA parsing/elevation, the sky row and when the switch fires are covered by test_gnss (the shared base).
Run by `make test`.
"""

import asyncio
import struct

import config_default
import gnss
import recorder
import task
from drivers import neo6mv2


class _FakeWriter:
    def write(self, data):
        pass

    async def drain(self):
        pass


class _Frames:
    """Stands in for the GNSS UART's writer: keeps every frame written, in order."""

    def __init__(self) -> None:
        self.frames: list = []

    def write(self, data: bytes) -> None:
        """Keep the frame."""
        self.frames.append(data)

    async def drain(self) -> None:
        """Nothing to flush."""
        pass


class _StubController:
    config = config_default.default()


def _spec(name):
    for sensor in config_default.default()['sensors']:
        if sensor['name'] == name:
            return sensor
    return {}


async def amain():
    assert task.ACTIVITIES.get('neo6mv2') is neo6mv2.Neo6mv2  # registered driver
    assert issubclass(neo6mv2.Neo6mv2, gnss.Gnss)  # shares the NMEA base

    # UBX framing + Fletcher checksum: a known-good UBX-CFG-RATE (measRate 1000 ms, navRate 1, GPS time)
    frame = neo6mv2._ubx(0x06, 0x08, struct.pack('<HHH', 1000, 1, 1))
    assert frame == b'\xb5\x62\x06\x08\x06\x00\xe8\x03\x01\x00\x01\x00\x01\x39', frame

    """
    The command bytes, in send order. hz 5 is EXACTLY what the 2026-10-04 bench sent and measured
    (src/gnss_bench): GSA and GSV every 50th fix, ~10 s, the RMC/GGA rates untouched. The diagnostics
    come every ~10 s at any rate -- 10 x hz fixes.
    """
    def _expected(hz, period_ms, diagnostics_every):
        return (
            gnss.nmea('PUBX,40,RMC,0,1,0,0,0,0'),
            gnss.nmea('PUBX,40,GGA,0,%d,0,0,0,0' % hz),
            gnss.nmea('PUBX,40,GLL,0,0,0,0,0,0'),
            gnss.nmea('PUBX,40,GSA,0,0,0,0,0,0'),
            gnss.nmea('PUBX,40,GSV,0,0,0,0,0,0'),
            gnss.nmea('PUBX,40,VTG,0,0,0,0,0,0'),
            neo6mv2._ubx(0x06, 0x08, struct.pack('<HHH', period_ms, 1, 1)),
            gnss.nmea('PUBX,40,GSA,0,%d,0,0,0,0' % diagnostics_every),
            gnss.nmea('PUBX,40,GSV,0,%d,0,0,0,0' % diagnostics_every),
        )

    assert neo6mv2._commands(5) == _expected(5, 200, 50), neo6mv2._commands(5)
    assert neo6mv2._commands(5)[7:] == (b'$PUBX,40,GSA,0,50,0,0,0,0*7B\r\n',  # the bench's bytes
                                        b'$PUBX,40,GSV,0,50,0,0,0,0*6C\r\n')
    assert neo6mv2._commands(1) == _expected(1, 1000, 10), neo6mv2._commands(1)
    assert neo6mv2._commands(10) == _expected(10, 100, 100), neo6mv2._commands(10)
    assert neo6mv2._commands(0) == neo6mv2._commands(1)  # no rate counts as 1 Hz, never a divide by zero
    # a port's rate is one byte: 25 Hz still fits (250), 30 Hz caps at 255 instead of an out-of-range 300
    assert neo6mv2._commands(25)[7:] == (gnss.nmea('PUBX,40,GSA,0,250,0,0,0,0'),
                                         gnss.nmea('PUBX,40,GSV,0,250,0,0,0,0'))
    assert neo6mv2._commands(30) == _expected(30, 33, 255), neo6mv2._commands(30)

    # an undefined bus -> graceful False, no UART touched
    no_bus = neo6mv2.Neo6mv2('gnss', {'bus': 'uart', 'id': 9}, _StubController())
    assert await no_bus.setup() is False

    # real uart:2: setup runs the u-blox PUBX/UBX reconfiguration without error and wires the channels
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    spec = dict(_spec('gnss'))
    spec['driver'] = 'neo6mv2'  # same component, swapped device
    unit = neo6mv2.Neo6mv2('gnss', spec, _StubController())
    assert await unit.setup() is True and unit.validate()

    """
    The sky switch, as literals: off is the flight init's own GSA and GSV at 0 -- the flight output exactly
    as before the diagnostics -- and on the diagnostics' GSA and GSV, each pair one write;
    gnss.Gnss._sky_window() writes one at each edge of the flight window. Setup kept the pair for the
    config's hz (10: the spec swaps only the driver) and left the diagnostics on. _configure() sends
    _commands() whole and in order and returns the pair -- to a recording writer here, so the module on
    uart:2 is not reconfigured again.
    """
    off = b'$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n$PUBX,40,GSV,0,0,0,0,0,0*59\r\n'
    at_10 = (off, b'$PUBX,40,GSA,0,100,0,0,0,0*4F\r\n$PUBX,40,GSV,0,100,0,0,0,0*58\r\n')
    assert (unit._sky_off, unit._sky_on) == at_10 and unit._diagnosing is True, (unit._sky_off, unit._sky_on)
    writer = _Frames()
    unit._writer = writer
    assert await unit._configure(10) == at_10
    assert writer.frames == list(neo6mv2._commands(10)), writer.frames
    writer.frames.clear()
    assert await unit._configure(5) == (off, b'$PUBX,40,GSA,0,50,0,0,0,0*7B\r\n$PUBX,40,GSV,0,50,0,0,0,0*6C\r\n')
    assert writer.frames == list(neo6mv2._commands(5)), writer.frames

    print('ok: neo6mv2 registered; subclasses gnss.Gnss; UBX framing; u-blox commands + sky '
          'diagnostics in order for hz 5/1/10/0/25/30; setup + graceful; the sky switch frames for hz 10/5')


asyncio.run(amain())
