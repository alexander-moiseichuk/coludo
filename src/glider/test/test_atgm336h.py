# On-board test for the ATGM336H GNSS driver (drivers/atgm336h.py): @task.driver('atgm336h')
# registration, the NMEA helpers, graceful setup on an undefined bus, and that a parsed RMC/GGA lands
# on the databoard (fed canned sentences, so it is deterministic without a satellite fix). The real
# UART is built on uart:2; run by `make test`.

import asyncio

import config_default
import databoard
import recorder
import task
from drivers import atgm336h


class _FakeWriter:
    def write(self, data):
        pass

    async def drain(self):
        pass


class _StubController:
    config = config_default.default()


def _spec(name):
    for sensor in config_default.default()['sensors']:
        if sensor['name'] == name:
            return sensor
    return {}


def _line(body):  # a valid NMEA sentence (correct checksum) for feeding _parse
    return atgm336h._nmea(body).decode().strip()


async def amain():
    assert task.ACTIVITIES.get('atgm336h') is atgm336h.Atgm336h  # registered driver

    # NMEA helpers: checksum + ddmm.mmmm -> decimal degrees with hemisphere sign
    assert atgm336h._checksum_ok(_line('GPRMC,123519,A'))
    assert not atgm336h._checksum_ok('$GPRMC,123519,A*00')
    assert abs(atgm336h._degrees('4807.038', 'N') - 48.1173) < 1e-3
    assert atgm336h._degrees('01131.000', 'W') < 0 and atgm336h._degrees('', 'N') is None

    # an undefined bus -> graceful False, no UART touched
    no_bus = atgm336h.Atgm336h('gnss', {'bus': 'uart', 'id': 9}, _StubController())
    assert await no_bus.setup() is False

    # real uart:2: build the driver (configures the module), then feed canned NMEA
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    gnss = atgm336h.Atgm336h('gnss', _spec('gnss'), _StubController())
    assert await gnss.setup() is True

    # a valid RMC -> position on the databoard, fix True
    gnss._parse(_line('GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W'))
    latitude, longitude = databoard.Databoard.value('position')
    assert abs(latitude - 48.1173) < 1e-3 and abs(longitude - 11.5167) < 1e-3 and gnss._fix

    # a GGA -> altitude on the databoard
    gnss._parse(_line('GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,'))
    assert abs(databoard.Databoard.value('altitude') - 545.4) < 1e-3

    # a void fix (status V) clears `fix` and does NOT move position; a bad checksum is ignored
    previous = databoard.Databoard.value('position')
    gnss._parse(_line('GPRMC,123520,V,,,,,,,230394,,'))
    assert gnss._fix is False and databoard.Databoard.value('position') == previous
    gnss._parse('$GPRMC,123521,A,1234.000,N,01234.000,E,0,0,230394,,*00')  # bad checksum
    assert databoard.Databoard.value('position') == previous

    print('ok: atgm336h registered; NMEA helpers; RMC->position, GGA->altitude; void/bad ignored')


asyncio.run(amain())
