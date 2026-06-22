# On-board test for the ATGM336H GNSS driver (drivers/atgm336h.py): @task.driver('atgm336h')
# registration, that it subclasses the shared gnss.Gnss base, graceful setup on an undefined bus, and
# that the real CASIC reconfiguration runs without error on uart:2. The NMEA parsing/elevation is
# covered by test_gnss (the shared base). Run by `make test`.

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

    # an undefined bus -> graceful False, no UART touched
    no_bus = atgm336h.Atgm336h('gnss', {'bus': 'uart', 'id': 9}, _StubController())
    assert await no_bus.setup() is False

    # real uart:2: setup runs the CASIC PCAS/PMTK reconfiguration without error and wires the channels
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    unit = atgm336h.Atgm336h('gnss', _spec('gnss'), _StubController())
    assert await unit.setup() is True and unit.validate()

    print('ok: atgm336h registered; subclasses gnss.Gnss; CASIC setup + graceful')


asyncio.run(amain())
