# On-board test for the board bring-up (main.py): the network-free bringup() wires the inspectable
# objects + Controller and starts the configured task loops, incl. the Recorder virtual driver,
# while skipping config sensors whose drivers are not implemented yet. Run by `make test`.

import asyncio

import config_default
import inspector
import main


async def amain():
    flight, health = await main.bringup(config_default.default(), log=lambda message: None)

    # the operator-facing objects self-registered with the Inspector
    assert {'mission', 'health', 'controller', 'recorder'} <= set(inspector.Inspector.names())

    # the Recorder virtual driver is the one component with a registered driver -> up and healthy
    assert 'recorder' in flight.tasks and flight.tasks['recorder'].validate()

    # sensors whose drivers are not implemented yet (adxl375, ...) are skipped, not fatal
    assert 'accel_adxl375' not in flight.tasks

    await asyncio.sleep_ms(30)  # let the loops tick (the recorder drains)
    await flight.finish()
    assert flight.state == 'done' and flight.tasks == {}

    print('ok: main.bringup wires Mission/BoardHealth/Controller + Recorder driver, skips driverless sensors')


asyncio.run(amain())
