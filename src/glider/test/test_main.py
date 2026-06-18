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
    cfg['components'] = [c for c in cfg['components'] if c['driver'] not in ('wifi', 'cc')]

    flight = await main.bringup(cfg, log=lambda message: None)

    # Mission (explicit) + the Controller and its config tasks all registered with the Inspector
    assert {'mission', 'controller', 'recorder', 'led', 'health'} <= set(inspector.Inspector.names())

    # every enabled component with a registered driver came up healthy (built purely from config)
    for name in ('recorder', 'led', 'health'):
        assert name in flight.tasks and flight.tasks[name].validate()

    # no connectivity configured -> the board runs standalone, no Wi-Fi/CC tasks
    assert 'wifi' not in flight.tasks and 'cc' not in flight.tasks

    # sensors whose drivers are not implemented yet (adxl375, ...) are skipped, not fatal
    assert 'accel_adxl375' not in flight.tasks

    await asyncio.sleep_ms(30)  # let the loops tick (the recorder drains)
    await flight.finish()
    assert flight.state == 'done' and flight.tasks == {}

    print('ok: main.bringup wires Mission/BoardHealth/Controller + Recorder driver, skips driverless sensors')


asyncio.run(amain())
