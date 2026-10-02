"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the VL53L1X driver (drivers/vl53l1x.py): registration, graceful setup when absent,
and the property the whole two-laser arrangement rests on -- that a driver REFUSES the other family's
silicon at the shared 0x29 address. Deterministic whichever laser is fitted. Run by `make test`.
"""

import asyncio

import config_default
import layout
import task
from drivers import vl53l1x, vl53l4cx


class _StubController:
    config = config_default.default()


async def amain():
    assert task.ACTIVITIES.get('vl53l1x') is vl53l1x.Vl53l1x  # registered driver

    # an undefined bus -> graceful False, no hardware touched
    no_bus = vl53l1x.Vl53l1x('laser', {'bus': 'i2c', 'id': 9}, _StubController())
    assert await no_bus.setup() is False and not no_bus.validate()

    # a real bus but a bogus address (nothing acks) -> graceful False (Controller would skip it)
    absent = vl53l1x.Vl53l1x('laser', {'bus': 'i2c', 'id': 0, 'addr': 0x7F}, _StubController())
    assert await absent.setup() is False

    """
    THE CONTRACT: both lasers are declared in config and both sit at 0x29, so exactly one must come up.
    An I2C scan cannot tell them apart and `layout` does not try -- each driver checks its own model id
    (0xEACC vs 0xEBAA) and returns False for the other. If that check ever loosened, the wrong init
    block would be written to a real part: it ACKs, it configures, and it silently never ranges.

    Run against whatever is actually soldered, so this asserts on the PAIR rather than on one part:
    at most one may claim the socket, and on a board with a laser fitted exactly one must.
    """
    cfg = config_default.default()
    revision, _detail = layout.detect(cfg)
    layout.apply(cfg, revision)
    laser = [device for device in cfg['sensors'] if device['name'] == 'laser_agl'][0]
    # carry `provides` too: a driver that reaches provide() is a driver that PASSED its id check and
    # its init, so omitting it turns a successful bring-up into a KeyError and hides the real result
    spec = {key: laser[key] for key in ('bus', 'id', 'addr', 'provides')}

    l1x = vl53l1x.Vl53l1x('laser_l1x', dict(spec), _StubController())
    l4cx = vl53l4cx.Vl53l4cx('laser_l4cx', dict(spec), _StubController())
    up = []  # NOT a comprehension: MicroPython rejects `await` inside one ('await outside function')
    for name, device in (('vl53l1x', l1x), ('vl53l4cx', l4cx)):
        if await device.setup():
            up.append(name)
    assert len(up) <= 1, 'both drivers claimed 0x29 -- a model-id check is too loose: %s' % up
    # ...and when a laser DOES answer at 0x29, exactly one driver must take it: two refusals is a model-id
    # check too TIGHT, and a board with no working laser at all would otherwise pass here
    if revision is not None and '0x29' in _detail:
        assert len(up) == 1, 'a laser answers at 0x29 but neither driver claimed it: %s' % _detail

    """
    The part that claimed the socket must pass its OWN probe -- `verify` and `arm` refuse on a failed
    probe. The L1X probe once checked the L4CX's id byte (0xEB), so a healthy, ranging L1X failed it
    and every board carrying one refused to arm, while setup() above passed. And the other driver must
    FAIL its probe on this silicon, naming the wrong device rather than passing on a shared byte.
    """
    for name, device in (('vl53l1x', l1x), ('vl53l4cx', l4cx)):
        if not up:
            break
        verdict = await device.probe()
        if name in up:
            assert verdict is None, '%s claimed the socket but failed its own probe: %s' % (name, verdict)
        else:
            assert verdict and 'id' in verdict, '%s passed a probe on the other silicon: %r' % (name, verdict)
    if 'vl53l1x' in up:
        diagnosis = await l4cx.diagnose()
        assert 'wrong device' in diagnosis, 'L4CX diagnose on L1X silicon: %s' % diagnosis
    elif 'vl53l4cx' in up:
        diagnosis = await l1x.diagnose()
        assert 'wrong device' in diagnosis, 'L1X diagnose on L4CX silicon: %s' % diagnosis

    """
    And the layout side: the L1X entry must FOLLOW the L4CX's socket onto whatever bus the revision
    puts it on, or a v1.0 board would hunt it on the v0.1 bus. It must not vote separately, either --
    two candidate drivers for one socket are one piece of evidence, not two.
    """
    for candidate in ('laser_agl', 'laser_agl_l1x'):
        device = [d for d in cfg['sensors'] if d['name'] == candidate][0]
        assert (device['bus'], device['id']) == (laser['bus'], laser['id']), (
            '%s did not follow the socket: %s' % (candidate, device))
    before = layout.detect(cfg)[1]
    assert before.count('0x29') <= 1, 'the shared address is counted more than once: %s' % before

    print('ok: vl53l1x registered; graceful-absent; socket claimed by %s (own probe passes, the other '
          'fails); both follow %s i2c:%s' % (
        up[0] if up else 'neither (no laser wired)', revision, laser['id']))


asyncio.run(amain())
