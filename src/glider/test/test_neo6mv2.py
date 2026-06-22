# On-board test for the NEO-6M GNSS driver (drivers/neo6mv2.py): @task.driver('neo6mv2') registration,
# that it subclasses the shared gnss.Gnss base, the UBX binary framing/checksum, graceful setup on an
# undefined bus, and that the u-blox PUBX/UBX reconfiguration runs without error on uart:2. NMEA
# parsing/elevation is covered by test_gnss (the shared base). Run by `make test`.

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

    # an undefined bus -> graceful False, no UART touched
    no_bus = neo6mv2.Neo6mv2('gnss', {'bus': 'uart', 'id': 9}, _StubController())
    assert await no_bus.setup() is False

    # real uart:2: setup runs the u-blox PUBX/UBX reconfiguration without error and wires the channels
    recorder.Recorder.setup(config_default.default(), uart=_FakeWriter())
    spec = dict(_spec('gnss'))
    spec['driver'] = 'neo6mv2'  # same component, swapped device
    unit = neo6mv2.Neo6mv2('gnss', spec, _StubController())
    assert await unit.setup() is True and unit.validate()

    print('ok: neo6mv2 registered; subclasses gnss.Gnss; UBX framing; u-blox setup + graceful')


asyncio.run(amain())
