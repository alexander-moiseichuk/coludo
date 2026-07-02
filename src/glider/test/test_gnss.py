# On-board test for the shared GNSS base (gnss.py): the NMEA helpers and the Gnss base Task -- graceful
# setup on an undefined bus, and that a parsed RMC/GGA lands on the databoard (fed canned sentences, so
# it is deterministic without a satellite fix), including the GGA-derived elevation. A trivial
# _StubGnss subclass exercises the base independently of any module driver. Run by `make test`.

import asyncio

import config_default
import databoard
import gnss
import recorder


class _FakeWriter:
    def write(self, data):
        pass

    async def drain(self):
        pass


class _StubController:
    config = config_default.default()


class _StubGnss(gnss.Gnss):
    """A concrete Gnss with a no-op _configure -- exercises the base, no module-specific commands."""

    async def _configure(self, hz):
        pass


def _spec(name):
    for sensor in config_default.default()['sensors']:
        if sensor['name'] == name:
            return sensor
    return {}


def _line(body):  # a valid NMEA sentence (correct checksum) for feeding _parse
    return gnss.nmea(body).decode().strip()


async def amain():
    # NMEA helpers: checksum + ddmm.mmmm -> decimal degrees with hemisphere sign
    assert gnss.checksum_ok(_line('GPRMC,123519,A'))
    assert not gnss.checksum_ok('$GPRMC,123519,A*00')
    assert abs(gnss.degrees('4807.038', 'N') - 48.1173) < 1e-3
    assert gnss.degrees('01131.000', 'W') < 0 and gnss.degrees('', 'N') is None

    # an undefined bus -> graceful False, no UART touched
    no_bus = _StubGnss('gnss', {'bus': 'uart', 'id': 9}, _StubController())
    assert await no_bus.setup() is False

    # real uart:2: build the base (configures nothing), then feed canned NMEA
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    unit = _StubGnss('gnss', _spec('gnss'), _StubController())
    assert await unit.setup() is True

    # a valid RMC -> position on the databoard, fix True
    unit._parse(_line('GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W'))
    latitude, longitude = databoard.Databoard.value('position')
    assert abs(latitude - 48.1173) < 1e-3 and abs(longitude - 11.5167) < 1e-3 and unit._fix
    # RMC field-7 speed (knots) -> 'speed' channel in m/s for the airspeed governor (022.4 kn ~= 11.52 m/s)
    assert abs(databoard.Databoard.value('speed') - 22.4 * 0.514444) < 1e-2

    # GGA -> altitude + elevation: the first valid GGA fixes the ground (elevation 0), next is the delta
    unit._parse(_line('GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,'))
    assert abs(databoard.Databoard.value('altitude') - 545.4) < 1e-3
    assert abs(databoard.Databoard.value('elevation')) < 1e-3 and abs(unit._ground - 545.4) < 1e-3
    # GGA also carries the signal quality (for antenna/sky checks): fix quality 1, 8 satellites, HDOP 0.9
    assert unit._fix_quality == 1 and unit._satellites == 8 and abs(unit._hdop - 0.9) < 1e-3
    unit._parse(_line('GPGGA,123520,4807.038,N,01131.000,E,1,08,0.9,550.4,M,46.9,M,,'))
    assert abs(databoard.Databoard.value('elevation') - 5.0) < 1e-3  # 550.4 - 545.4 ground

    # a void fix (status V) clears `fix` and does NOT move position; a bad checksum is ignored
    previous = databoard.Databoard.value('position')
    unit._parse(_line('GPRMC,123520,V,,,,,,,230394,,'))
    assert unit._fix is False and databoard.Databoard.value('position') == previous
    unit._parse('$GPRMC,123521,A,1234.000,N,01234.000,E,0,0,230394,,*00')  # bad checksum
    assert databoard.Databoard.value('position') == previous

    print('ok: gnss base -- NMEA helpers; RMC->position+speed, GGA->altitude+elevation(ground); void/bad ignored')


asyncio.run(amain())
