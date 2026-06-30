# On-board (MicroPython) test for the shared servo slew gate (servo.py): the FIFO counting semaphore
# that bounds simultaneous fin slews, and the process-wide Gate.slew()/Gate.reset() shared instance.
# Run by `make test`.

import asyncio

import servo


async def amain():
    # N=1: the gate serialises -- two holders never overlap (order is one full pair then the other)
    gate = servo.Gate(1)
    order = []

    async def worker(tag):
        async with gate:
            order.append('in' + tag)
            await asyncio.sleep_ms(20)
            order.append('out' + tag)

    await asyncio.gather(worker('A'), worker('B'))
    assert order in (['inA', 'outA', 'inB', 'outB'], ['inB', 'outB', 'inA', 'outA']), order

    # N=2: two hold at once; a third blocks until a release hands it the permit (FIFO hand-off)
    gate2 = servo.Gate(2)
    await gate2.acquire()
    await gate2.acquire()
    held = []

    async def third():
        await gate2.acquire()
        held.append('got')

    pending = asyncio.create_task(third())
    await asyncio.sleep_ms(10)
    assert held == []  # both permits taken -> blocked
    gate2.release()
    await asyncio.sleep_ms(10)
    assert held == ['got']  # released -> handed the permit, no free-count change
    await pending

    # release with no waiters returns a permit to the pool
    gate3 = servo.Gate(1)
    await gate3.acquire()
    gate3.release()
    await gate3.acquire()  # the permit is back -> acquires without blocking
    gate3.release()

    # cancellation must NOT leak a permit (else the gate bleeds to 0 over a board's reconfigures).
    # (a) cancelled while still QUEUED -> just dequeues, the held permit count is unchanged
    gatec = servo.Gate(1)
    await gatec.acquire()                       # holder takes the only permit
    queued = asyncio.create_task(gatec.acquire())
    await asyncio.sleep_ms(5)                    # let it queue
    queued.cancel()
    try:
        await queued
    except asyncio.CancelledError:
        pass
    gatec.release()                             # holder releases -> permit back in the pool
    await asyncio.wait_for_ms(gatec.acquire(), 50)  # acquires at once -> no leak from the queued-cancel
    gatec.release()

    # (b) the grant/cancel race: release() hands a waiter the permit, but it is cancelled before it
    # resumes -> it must PASS THE PERMIT ON, not leak it
    gated = servo.Gate(1)
    await gated.acquire()                       # holder takes the only permit
    racer = asyncio.create_task(gated.acquire())
    await asyncio.sleep_ms(5)                    # let it queue
    gated.release()                             # hand the permit to the racer (event.set), then...
    racer.cancel()                              # ...cancel before it can resume -> must release the permit
    try:
        await racer
    except asyncio.CancelledError:
        pass
    got = []

    async def _take():
        await gated.acquire()
        got.append('ok')

    await asyncio.wait_for_ms(_take(), 50)       # would time out (FAIL) if the permit leaked
    assert got == ['ok'], 'permit leaked on the grant/cancel race'
    gated.release()

    # Gate.slew() is process-wide: created once (first permits win), Gate.reset() rebuilds it
    servo.Gate.reset()
    shared = servo.Gate.slew(3)
    assert servo.Gate.slew(1) is shared  # second call keeps the first gate (1 ignored)
    servo.Gate.reset()
    assert servo.Gate.slew(2) is not shared  # after reset a fresh gate is built

    print('ok: servo slew gate -- FIFO counting semaphore, N-limit serialisation, process-wide shared')


asyncio.run(amain())
