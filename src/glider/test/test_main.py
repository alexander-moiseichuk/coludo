# On-board test for the board bring-up (main.py): the network-free bringup() wires the inspectable
# objects + Controller and starts the configured task loops, incl. the Recorder virtual driver,
# while skipping config sensors whose drivers are not implemented yet. Run by `make test`.

import asyncio

import config_default
import inspector
import main


async def amain():
    # a board with no Wi-Fi (e.g. FireBeetle 2): drop the connectivity components -- it must still
    # bring up everything else and run without CC.
    cfg = config_default.default()
    cfg['components'] = [c for c in cfg['components']
                         if (c.get('driver') or c.get('activity')) not in ('wifi', 'cc')]

    flight = await main.bringup(cfg, log=lambda message: None)

    # Mission (explicit) + the Controller and its enabled tasks all registered with the Inspector
    assert {'mission', 'controller', 'recorder', 'health', 'bluetooth'} <= set(inspector.Inspector.names())

    # every enabled component with a registered driver came up healthy (built purely from config)
    for name in ('recorder', 'health', 'bluetooth'):
        assert name in flight.tasks and flight.tasks[name].validate()

    # disabled-by-default + stripped components are absent; the board still runs standalone
    assert 'led' not in flight.tasks  # enabled: False by default
    assert 'wifi' not in flight.tasks and 'cc' not in flight.tasks

    # the gnss driver (atgm336h) is implemented and builds whenever enabled -- a UART has no
    # presence check, unlike the i2c/spi sensors which build only if their device answers on the bus
    assert 'gnss' in flight.tasks and flight.tasks['gnss'].validate()

    # the failures mechanism (hardware-independent invariants): a task that came up is never also
    # listed failed, and the always-set-up tasks (no bus presence check) never appear in failures.
    # The strict "every device connected" gate is the OPERATOR's check -- `verify` /
    # tools/board_check.py -- not the unit suite, which must run without every i2c/spi sensor wired.
    assert set(flight.failures) & set(flight.tasks) == set()  # up and failed are disjoint
    assert not (set(flight.failures) & {'gnss', 'recorder', 'health', 'bluetooth'})  # always set up

    await asyncio.sleep_ms(30)  # let the loops tick (the recorder drains)
    await flight.finish()
    assert flight.stage_name() == 'done' and flight.tasks == {}

    print('ok: main.bringup wires Mission/BoardHealth/Controller + Recorder driver, skips driverless sensors')


asyncio.run(amain())
