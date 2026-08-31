"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

CROSS-REFERENCE test for the board config (findings §27.10): every device `config_default` declares
must actually be BUILDABLE. `config.validate()` checks the file's shape -- types, pin uniqueness, bus
refs -- and each driver's own test asserts that driver registers. Nothing checked the join between
them, so a typo'd `driver` name, a device pointing at a bus that is not defined, or a `pin` role with
no entry in `pins` all fail the same silent way: the Controller skips the device and the board flies
without a sensor nobody noticed was missing.

This is the one loop that closes: walk the REAL default config, and for every device assert its
implementation resolves in the registry, its bus exists, and its pin roles resolve. Run by `make test`.
"""

import config
import config_default
import drivers
import task
import tasks
import warmstart  # noqa: F401 -- see below: it registers @task.activity('checkpoint')


def main():
    """
    Populate task.ACTIVITIES exactly as main.bringup() does.

    drivers.load() / tasks.load() auto-discover their packages, but registration is NOT confined to
    them: `warmstart.py` is a ROOT module carrying @task.activity('checkpoint'), so it registers only
    because main.py imports it. Miss that import here and this test "discovers" a config error that does
    not exist -- which is exactly what happened when it was written. Mirror bring-up, do not approximate it.
    """
    drivers.load()
    tasks.load()
    cfg = config_default.default()
    assert not config.validate(cfg), config.validate(cfg)  # the default must always be self-valid

    devices = list(cfg.get('sensors') or []) + list(cfg.get('components') or [])
    assert devices, 'the default config declares no devices at all'
    enabled = [d for d in devices if d.get('enabled', True)]

    """
    IMPLEMENTATION resolves: a device names its code with `driver` (drivers/) or `activity` (tasks/).
    An unresolvable name is the silent-skip bug this test exists for -- the Controller cannot build it
    and the board boots one sensor short.
    """
    for device in devices:
        name = device.get('name')
        impl = device.get('driver') or device.get('activity')
        assert impl, 'device %r names neither a driver nor an activity' % name
        assert impl in task.ACTIVITIES, (
            'device %r wants %r, which is not registered (typo, or its module failed to import)'
            % (name, impl))

    """
    BUS resolves: a device addressing a bus must reference one the config defines, or config.bus()
    returns None at runtime and the driver fails setup for a reason that looks like bad hardware.
    """
    for device in devices:
        kind = device.get('bus')
        if kind is None:
            continue
        spec = config.bus(cfg, kind, device.get('id', 0))
        assert spec is not None, ('device %r addresses %s:%s, which the config does not define'
                                  % (device.get('name'), kind, device.get('id', 0)))

    """
    PIN roles resolve: every *_pin value must name an entry in `pins` (null / negative means the
    feature is deliberately unwired -- see the disabled-pin convention in board_layout.md).
    """
    pins = cfg.get('pins') or {}
    for device in devices:
        for key, role in device.items():
            if not key.endswith('pin') or role is None:
                continue
            assert role in pins, ('device %r wants pin role %r=%r, absent from `pins`'
                                  % (device.get('name'), key, role))

    # names are unique across sensors + components: the Controller keys its task map by name
    names = [d.get('name') for d in devices]
    assert len(names) == len(set(names)), 'duplicate device names: %r' % names

    # every declared `provides` quantity must name a real quantity string (a typo silently provides nothing)
    for device in devices:
        for quantity, spec in (device.get('provides') or {}).items():
            assert isinstance(quantity, str) and quantity, 'device %r provides a nameless quantity' % device
            assert 'priority' in spec, ('device %r provides %r with no priority (fusion cannot rank it)'
                                        % (device.get('name'), quantity))

    print('ok: config wiring -- %d devices (%d enabled), every impl/bus/pin/provides reference resolves'
          % (len(devices), len(enabled)))


main()
