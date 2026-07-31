"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the Task BASE (task.py): the @activity/@driver registry, the de-duplicated note()
health tracking, _pin_gpio resolution incl. the disabled-pin convention, find()/notify()/emit(), and the
inspect()/validate() surface.

Every driver and task inherits this, so a regression here surfaces as a confusing failure somewhere else
entirely -- and it had no direct test (findings §27.9). Also asserts the CONFIG CROSS-REFERENCES that
nothing checked (§27.10): every configured device names a driver that actually resolves in the registry,
every pin/bus it references exists, and every `provides` quantity has a consumer. Run by `make test`.
"""

import asyncio

import config
import config_default
import task


class _StubController:
    def __init__(self, cfg=None):
        self.config = cfg or config_default.default()
        self.tasks = {}


class _Probe(task.Task):
    """A minimal concrete Task -- the base is abstract (setup must be overridden)."""

    async def setup(self) -> bool:
        self._ok = True
        return True


def test_registry():
    """@activity/@driver register a class under a name; `driver` is the same decorator, aliased."""
    assert task.driver is task.activity  # drivers/ read as @task.driver, tasks/ as @task.activity

    @task.activity('unit_test_probe')
    class Registered(_Probe):
        pass

    assert task.ACTIVITIES['unit_test_probe'] is Registered
    assert Registered('x', {}, _StubController()) is not None  # the decorator returns the class unchanged
    del task.ACTIVITIES['unit_test_probe']  # keep the shared registry clean for the other tests


def test_note_dedupes_and_tracks_health():
    """
    note() prints ONCE per healthy->error transition and drives inspect()['healthy'].

    The de-duplication is not cosmetic: a persistently-failing 50 Hz read would otherwise format a string
    every tick (leaking in a GC-off flight) and flood USB-CDC until the REPL wedges.
    """
    unit = _Probe('probe', {}, _StubController())
    assert unit._healthy is True
    unit.note('probe :: %r', ValueError('first'))  # transition -> logs
    assert unit._healthy is False
    unit.note('probe :: %r', ValueError('second'))  # still failing -> silent, stays unhealthy
    assert unit._healthy is False
    unit.note(None)  # healthy pass -> re-arms
    assert unit._healthy is True
    unit.note('probe :: no-arg template')  # a template needing no argument must not raise
    assert unit._healthy is False
    unit.note(None)
    assert unit.inspect()['healthy'] is True  # surfaced to the operator


def test_pin_resolution():
    """
    _pin_gpio maps a component's pin NAME through the board pins map -- and honours 'wired off'.

    A `null` (or negative) entry means the optional feature is not wired on this board; it must resolve
    to None exactly like an absent pin, so every driver's `is None` guard skips the feature.
    """
    cfg = config_default.default()
    cfg['pins'] = {'servo_yaw': 26, 'off_by_null': None, 'off_by_negative': -1}
    controller = _StubController(cfg)
    assert _Probe('a', {'pin': 'servo_yaw'}, controller)._pin_gpio('pin') == 26
    assert _Probe('b', {}, controller)._pin_gpio('pin', 'servo_yaw') == 26  # default name when omitted
    assert _Probe('c', {'pin': 'off_by_null'}, controller)._pin_gpio('pin') is None
    assert _Probe('d', {'pin': 'off_by_negative'}, controller)._pin_gpio('pin') is None
    assert _Probe('e', {'pin': 'not_in_the_map'}, controller)._pin_gpio('pin') is None
    assert _Probe('f', {}, controller)._pin_gpio('pin') is None  # no field, no default


def test_find_and_events():
    """find() delegates to the Controller (None per missing name); notify/emit fan out to subscribers."""
    controller = _StubController()
    unit = _Probe('probe', {}, controller)
    controller.find = lambda names: [controller.tasks.get(n) for n in names]
    other = _Probe('other', {}, controller)
    controller.tasks['other'] = other
    assert unit.find(['other']) == [other]
    assert unit.find(['absent']) == [None]  # ALIGNED with names -- the contract warmstart relies on
    seen = []

    def subscriber(source, event):  # the contract is callback(task, event), not callback(event)
        seen.append((source.name, event))

    unit.notify(subscriber)
    unit.notify(subscriber)  # notify() DE-DUPLICATES, so a re-registered callback still fires once
    unit.emit('event')
    assert seen == [('probe', 'event')], seen
    unit.emit()  # event defaults to None
    assert seen[-1] == ('probe', None)


def test_validate_and_inspect():
    """validate() reports setup success; inspect() carries name/ok/healthy for the operator panel."""
    unit = _Probe('probe', {}, _StubController())
    assert unit.validate() is False  # not set up yet
    asyncio.run(unit.setup())
    assert unit.validate() is True
    status = unit.inspect()
    assert status['name'] == 'probe' and status['ok'] is True and status['healthy'] is True


def test_config_cross_references():
    """
    CONFIG CROSS-REFERENCES (findings §27.10): config_default is the single source of truth for the
    board, but nothing asserted that its parts agree with each other.

    Shape is checked by test_config and pins by test_pins; what was missing is whether every device
    actually RESOLVES -- a typo in a driver name, a pin that no map entry defines, or a bus id nothing
    declares, all validate fine and then fail at boot as a device that mysteriously never came up.
    """
    import drivers
    import tasks
    import warmstart  # noqa: F401
    drivers.load()
    tasks.load()
    """
    EXCEPTION worth knowing: `drivers.load()` + `tasks.load()` do NOT complete the registry. warmstart.py
    is a ROOT module that also carries an @task.activity ('checkpoint'), so the registry is only whole
    after main.py's imports -- which is why it is imported above, exactly as main.py does. The
    "adding a task is dropping a file in drivers/ or tasks/" story has this one exception; this test is
    the thing that would notice if a second one appeared.
    """
    cfg = config_default.default()
    pins = cfg.get('pins', {})
    buses = cfg.get('buses', {})
    devices = list(cfg.get('sensors', [])) + list(cfg.get('components', []))
    assert devices, 'the default config must declare devices'

    for device in devices:
        name = device.get('name')
        key = device.get('driver') or device.get('activity')
        assert key, 'device %r names neither a driver nor an activity' % name
        assert key in task.ACTIVITIES, 'device %r names %r, which no module registers' % (name, key)
        kind = device.get('bus')
        if kind is not None:  # a bus-attached device must reference a declared bus id
            assert config.bus(cfg, kind, device.get('id', 0)) is not None, \
                'device %r references undeclared bus %s:%s' % (name, kind, device.get('id'))
            assert kind in buses, 'device %r uses bus kind %r that buses does not declare' % (name, kind)
        for field, value in device.items():  # every *_pin reference must exist in the pins map
            if field.endswith('pin') and isinstance(value, str):
                assert value in pins, 'device %r references pin %r, absent from the pins map' % (name, value)

    """
    The provider/consumer closure check -- "every quantity a device PROVIDES is read by somebody" --
    lives in the HOST suite now (src/control/test/test_tools.py), not here.

    It used to compare against a hardcoded `consumed` set, so adding a provider failed this test until
    someone edited the set even when a real consumer existed. Deriving the set instead needs the
    Databoard.parameter() call sites, and the board runs .mpy -- there is no source here to scan. On
    the host the sources are present, so the set is derived and cannot drift. Deriving it also
    exposed what the hardcoded version hid: six quantities have no control consumer at all and exist
    for telemetry and the operator, which the old set silently asserted otherwise.
    """


test_registry()
test_note_dedupes_and_tracks_health()
test_pin_resolution()
test_find_and_events()
test_validate_and_inspect()
test_config_cross_references()
print('ok: task -- registry, note dedupe + health, pin resolution incl. wired-off, find/notify/emit, '
      'validate/inspect, config cross-references')
