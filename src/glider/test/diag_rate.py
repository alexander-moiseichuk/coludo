"""
Is the gyro 'rate' channel actually staying fresh, or does it drop out?

The device probe showed `accel` with a live source and `rate` with NO SOURCE in the same sweep, from
the SAME driver, pushed on adjacent lines. That cannot be a driver asymmetry -- it is a REDUNDANCY
asymmetry: `accel` has three providers and falls back to the ADXL375 when the LSM6DSO32 misses a
deadline, while `rate` has exactly one provider on the tightest timeout in the config (20 ms against a
10 ms period). So a brief starve is INVISIBLE on accel and total on rate.

That matters because 'rate' feeds the PID derivative term. A D term that silently goes absent is worse
than one that is absent by design: the loop keeps flying with P and I only, and nothing says so.

This samples both channels in a TIGHT loop and prints NOTHING until the end -- because printing over
USB CDC blocks for milliseconds and would itself starve a 10 ms sensor loop, manufacturing the very
dropout we are trying to measure. (The first version of the servo probe made exactly that mistake.)

    mpremote connect $PORT run src/glider/test/diag_rate.py
"""

import asyncio

import config
import databoard
import main

_SAMPLES: int = 400   # at 5 ms -> ~2 s of continuous observation
_INTERVAL_MS: int = 5


async def run():
    board, _source, _errors = config.load()
    await main.bringup(board, log=lambda line: None)
    await asyncio.sleep_ms(1500)   # let the IMU interrupt settle before judging it

    rate = databoard.Databoard.parameter('rate')
    accel = databoard.Databoard.parameter('accel')
    samples = []
    for _ in range(_SAMPLES):
        _rv, rate_source, rate_age = rate.read()
        _av, accel_source, accel_age = accel.read()
        samples.append((rate_source, rate_age, accel_source, accel_age))
        await asyncio.sleep_ms(_INTERVAL_MS)

    rate_live = sum(1 for row in samples if row[0] is not None)
    accel_live = sum(1 for row in samples if row[2] is not None)
    accel_sources = {}
    for row in samples:
        if row[2] is not None:
            accel_sources[row[2]] = accel_sources.get(row[2], 0) + 1
    rate_ages = [row[1] for row in samples if row[0] is not None]

    print('samples        : %d at %d ms' % (_SAMPLES, _INTERVAL_MS))
    print('rate  fresh    : %d/%d (%d%%)' % (rate_live, _SAMPLES, 100 * rate_live // _SAMPLES))
    print('accel fresh    : %d/%d (%d%%)' % (accel_live, _SAMPLES, 100 * accel_live // _SAMPLES))
    print('accel won by   : %s' % accel_sources)
    if rate_ages:
        print('rate age us    : min %d  max %d' % (min(rate_ages), max(rate_ages)))

    """
    The verdict. `rate` fresh on essentially every sample means the earlier NO SOURCE was an artefact of
    the probe's own printing, and the channel is healthy. Anything materially below accel's rate is a
    REAL dropout that the accel fallback has been hiding all along -- and it would take the D term with
    it, silently, in flight.
    """
    if rate_live < accel_live * 9 // 10:
        print('VERDICT: *** rate DROPS OUT (%d%% vs accel %d%%) -- the D term is intermittently absent'
              % (100 * rate_live // _SAMPLES, 100 * accel_live // _SAMPLES))
    elif rate_live > _SAMPLES * 9 // 10:
        print('VERDICT: rate is healthy -- the probe NO SOURCE was its own printing, not the sensor')
    else:
        print('VERDICT: both channels intermittent (%d%%) -- the sampler itself is being starved'
              % (100 * rate_live // _SAMPLES))
    print('DONE')

asyncio.run(run())
