# On-board test for the board bring-up (main.py): the network-free bringup() wires the inspectable
# objects + Controller and starts the configured task loops, incl. the Recorder virtual driver,
# while skipping config sensors whose drivers are not implemented yet. Run by `make test`.

import asyncio

import config_default
import inspector
import main


async def amain():
    flight = await main.bringup(config_default.default(), log=lambda message: None)

    # Mission (explicit) + the Controller and its config tasks all registered with the Inspector
    assert {'mission', 'controller', 'recorder', 'led', 'health'} <= set(inspector.Inspector.names())

    # every enabled component with a registered driver came up healthy (built purely from config)
    for name in ('recorder', 'led', 'health'):
        assert name in flight.tasks and flight.tasks[name].validate()

    # sensors whose drivers are not implemented yet (adxl375, ...) are skipped, not fatal
    assert 'accel_adxl375' not in flight.tasks

    await asyncio.sleep_ms(30)  # let the loops tick (the recorder drains)
    await flight.finish()
    assert flight.state == 'done' and flight.tasks == {}

    print('ok: main.bringup wires Mission/BoardHealth/Controller + Recorder driver, skips driverless sensors')


asyncio.run(amain())
