# tools/board_check.py — "verify board setup and report problems". Runs ON the board: brings up every
# configured device from the board's own config and reports which are connected vs absent/miswired,
# then runs the probe self-tests. One command over serial, NO Wi-Fi/CC needed -- the network tasks set
# up locally and connect in the background, so this works right after install, before the AP /
# credentials are confirmed.
#
#   mpremote connect /dev/ttyACM0 run tools/board_check.py
#
# NOTE: probe runs ACTIVE self-tests -- it SWEEPS THE SERVOS. Bench use only, never with the airframe
# armed. PASS = no setup failures and every probe healthy; otherwise the problems are listed.

import asyncio

import config
import inspector
import main


async def check():
    cfg, source, errors = config.load()
    print('config: %s%s' % (source, (' ERRORS=%s' % errors) if errors else ''))
    flight = await main.bringup(cfg, log=lambda message: None)
    await asyncio.sleep_ms(400)  # let sensors take a first sample / channels populate

    configured = flight.directory()
    print('\n=== devices (%d configured) ===' % len(configured))
    for name in configured:
        if name in flight.tasks:
            print('  up    %s' % name)
        else:
            print('  DOWN  %s  <- %s' % (name, flight.failures.get(name, '?')))

    print('\n=== probe (active self-tests; sweeps servos) ===')
    probe_failures = {}
    for name in sorted(inspector.Inspector.names()):
        run = getattr(inspector.Inspector.get(name), 'probe', None)
        if run is None:
            continue
        result = await run()
        print('  %-20s %s' % (name, 'ok' if result is None else result))
        if result is not None:
            probe_failures[name] = result

    ok = not flight.failures and not probe_failures
    print('\n%s -- %d setup failure(s), %d probe failure(s)' % (
        'PASS' if ok else 'FAILURE', len(flight.failures), len(probe_failures)))
    if flight.failures:
        print('  not connected:', ', '.join('%s (%s)' % (n, r) for n, r in sorted(flight.failures.items())))
    if probe_failures:
        print('  probe failed: ', ', '.join('%s (%s)' % (n, r) for n, r in sorted(probe_failures.items())))
    await flight.finish()


asyncio.run(check())
