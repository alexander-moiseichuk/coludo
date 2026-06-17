# Minimal on-board sanity test for the runtime primitives the firmware relies on.
# Passes (exit 0) if everything works; any failed `assert` raises -> the runner reports FAIL.

import asyncio
import gc
import struct
import time


def main():
    # timing primitives
    t = time.ticks_us()
    assert time.ticks_diff(time.ticks_us(), t) >= 0, 'ticks_us not monotonic'

    # gc / heap
    gc.collect()
    assert gc.mem_free() > 100_000, 'unexpectedly low free heap'

    # struct.pack_into / unpack_from — the logger's in-place buffer primitive
    buf = bytearray(64)
    struct.pack_into('<I', buf, 0, 0x12345678)
    assert struct.unpack_from('<I', buf, 0)[0] == 0x12345678, 'pack/unpack mismatch'

    # asyncio runs and gathers tasks
    async def amain():
        hits = []

        async def worker():
            await asyncio.sleep_ms(1)
            hits.append(1)

        await asyncio.gather(worker(), worker())
        return len(hits)

    assert asyncio.run(amain()) == 2, 'asyncio gather did not run both tasks'

    print('ok: timing, gc, struct, asyncio')


main()
