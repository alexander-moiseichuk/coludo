"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the ATGM336H GNSS driver (drivers/atgm336h.py): @task.driver('atgm336h')
registration, that it subclasses the shared gnss.Gnss base, graceful setup on an undefined bus, the
CASIC command bytes in send order -- the flight init, then the sky diagnostics -- for hz 10 / 5 / 1 and
the edges, that the real reconfiguration runs without error on uart:2, and the sky switch's two frames
(off in flight, on at the pad) for hz 10 / 5. The NMEA parsing/elevation, the sky row and when the switch
fires are covered by test_gnss (the shared base). Run by `make test`.
"""

import asyncio

import config_default
import gnss
import recorder
import task
from drivers import atgm336h


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
    assert task.ACTIVITIES.get('atgm336h') is atgm336h.Atgm336h  # registered driver
    assert issubclass(atgm336h.Atgm336h, gnss.Gnss)  # shares the NMEA base

    """
    The command bytes, in send order. hz 10 is EXACTLY what the 2026-10-04 bench sent and measured
    (src/gnss_bench): GSA, GSV and the antenna text every 99th fix, ~9.9 s, the RMC/GGA rates untouched.
    The diagnostics come every ~10 s at any rate -- 10 x hz fixes, capped at the 2-digit 99.
    """
    assert atgm336h._commands(10) == (
        gnss.nmea('PCAS03,10,0,0,0,1,0,0,0,0,0,,,0,0'),
        gnss.nmea('PCAS02,100'),
        gnss.nmea('PMTK314,0,1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'),
        gnss.nmea('PMTK220,100'),
        gnss.nmea('PCAS03,10,0,99,99,1,0,0,99,0,0,,,0,0'),
    ), atgm336h._commands(10)
    assert atgm336h._commands(10)[4] == b'$PCAS03,10,0,99,99,1,0,0,99,0,0,,,0,0*02\r\n'  # the bench's bytes
    assert atgm336h._commands(5) == (
        gnss.nmea('PCAS03,5,0,0,0,1,0,0,0,0,0,,,0,0'),
        gnss.nmea('PCAS02,200'),
        gnss.nmea('PMTK314,0,1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'),
        gnss.nmea('PMTK220,200'),
        gnss.nmea('PCAS03,5,0,50,50,1,0,0,50,0,0,,,0,0'),
    ), atgm336h._commands(5)
    assert atgm336h._commands(1) == (
        gnss.nmea('PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0'),
        gnss.nmea('PCAS02,1000'),
        gnss.nmea('PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'),
        gnss.nmea('PMTK220,1000'),
        gnss.nmea('PCAS03,1,0,10,10,1,0,0,10,0,0,,,0,0'),
    ), atgm336h._commands(1)
    assert atgm336h._commands(0) == atgm336h._commands(1)  # no rate counts as 1 Hz, never a divide by zero
    assert atgm336h._commands(20)[4] == gnss.nmea('PCAS03,20,0,99,99,1,0,0,99,0,0,,,0,0')  # capped at 99

    # an undefined bus -> graceful False, no UART touched
    no_bus = atgm336h.Atgm336h('gnss', {'bus': 'uart', 'id': 9}, _StubController())
    assert await no_bus.setup() is False

    # real uart:2: setup runs the CASIC PCAS/PMTK reconfiguration without error and wires the channels
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    unit = atgm336h.Atgm336h('gnss', _spec('gnss'), _StubController())
    assert await unit.setup() is True and unit.validate()

    """
    The sky switch, as literals: off is the flight init's own PCAS03 -- RMC + GGA, the flight output exactly
    as before the diagnostics -- and on the diagnostics' PCAS03; gnss.Gnss._sky_window() writes one at each
    edge of the flight window. Setup kept the pair for the config's hz 10 and left the diagnostics on.
    _configure() sends _commands() whole and in order and returns the pair -- to a recording writer here,
    so the module on uart:2 is not reconfigured again.
    """
    at_10 = (b'$PCAS03,10,0,0,0,1,0,0,0,0,0,,,0,0*32\r\n', b'$PCAS03,10,0,99,99,1,0,0,99,0,0,,,0,0*02\r\n')
    assert (unit._sky_off, unit._sky_on) == at_10 and unit._diagnosing is True, (unit._sky_off, unit._sky_on)
    writer = _Frames()
    unit._writer = writer
    assert await unit._configure(10) == at_10
    assert writer.frames == list(atgm336h._commands(10)), writer.frames
    writer.frames.clear()
    assert await unit._configure(5) == (b'$PCAS03,5,0,0,0,1,0,0,0,0,0,,,0,0*06\r\n',
                                        b'$PCAS03,5,0,50,50,1,0,0,50,0,0,,,0,0*33\r\n')
    assert writer.frames == list(atgm336h._commands(5)), writer.frames

    print('ok: atgm336h registered; subclasses gnss.Gnss; CASIC commands + sky diagnostics in '
          'order for hz 10/5/1/0/20; setup + graceful; the sky switch frames for hz 10/5')


asyncio.run(amain())
