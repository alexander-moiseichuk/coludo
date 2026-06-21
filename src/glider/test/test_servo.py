# On-board (MicroPython) test for the shared servo slew gate (servo.py): the FIFO counting semaphore
# that bounds simultaneous fin slews, and the process-wide slew_gate()/reset_gate() factory. Run by
# `make test`.

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

    # slew_gate() is process-wide: created once (first permits win), reset_gate() rebuilds it
    servo.reset_gate()
    shared = servo.slew_gate(3)
    assert servo.slew_gate(1) is shared  # second call keeps the first gate (1 ignored)
    servo.reset_gate()
    assert servo.slew_gate(2) is not shared  # after reset a fresh gate is built

    print('ok: servo slew gate -- FIFO counting semaphore, N-limit serialisation, process-wide factory')


asyncio.run(amain())
