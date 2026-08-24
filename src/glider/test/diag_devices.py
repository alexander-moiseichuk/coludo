"""
Is every device on the assembled airframe alive? The post-assembly / post-rewire go-no-go.

Runs the REAL boot path (main.bringup) rather than a bus scan, so a device counts as up only if its
own driver could set it up -- a part that acks on I2C but fails its chip-id check is not "alive". Then
the on-demand probe() self-tests, then the fused channels, so the three questions an operator actually
has are answered in order: did it come up, does it work, is data flowing.

Reports the FAILURE REASON per device (controller.failures), because "down" alone does not distinguish
an unplugged part from a miswired one. Where several devices share a module (BNO055 + BMP280 are one
sen0253 board), read them together: both down is one connector, not two faults.

USE THE REAL BOOT PATH. A script that builds a Controller without drivers.load() finds an EMPTY
registry and reports every device down, including software-only tasks like wifi -- a false total
failure that looks alarming and means nothing.

WARNING: probe() is the ACTIVE self-test -- the servo probes SWEEP THE FINS. Keep clear.

    mpremote connect $PORT run src/glider/test/diag_devices.py
"""

import asyncio

import config
import databoard
import inspector
import main

_SETTLE_MS: int = 2000  # let the periodic tasks publish before the channels are read


def _report_setup(board: dict, flight) -> None:
    """
    Print which configured devices came up, and why each one did not.

    Args:
        board - the loaded board config (its `sensors` + `components` are the expected roster).
        flight - the Controller returned by bringup, carrying `failures`.

    Returns:
        None.
    """
    print()
    print('=== SETUP ===')
    up = 0
    for device in board.get('sensors', []) + board.get('components', []):
        name = device['name']
        if not device.get('enabled', True):
            print('  %-20s -- disabled in config' % name)
        elif flight.active(name) is not None:
            print('  %-20s UP' % name)
            up += 1
        else:
            print('  %-20s *** DOWN: %s ***' % (name, flight.failures.get(name, 'unknown')))
    print('  -> %d up, %d down' % (up, len(flight.failures)))


async def _report_probe() -> None:
    """
    Print each inspectable's probe() verdict (the fins sweep here).

    Returns:
        None.
    """
    print()
    print('=== PROBE (active self-tests -- FINS MOVE) ===')
    for name, result in sorted((await inspector.Inspector.probe_all()).items()):
        print('  %-20s %s' % (name, 'pass' if result is None else '*** FAIL: %s ***' % result))


def _report_channels() -> None:
    """
    Print every fused quantity with the source that won it, or NO SOURCE when nothing is fresh.

    read() rather than value(): value() extrapolates a stale channel unbounded, so a dead sensor still
    reads plausible. A quantity whose providers are all down shows NO SOURCE here, which is the point.

    Returns:
        None.
    """
    print()
    print('=== CHANNELS (fused values flowing) ===')
    for quantity in sorted(databoard.Databoard.inspect()):
        value, source, _age = databoard.Databoard.read(quantity)
        print('  %-18s %-16s %s' % (quantity, source or '(NO SOURCE)',
                                    'NO DATA' if value is None else str(value)[:46]))


async def main_diag() -> None:
    """Bring the board up exactly as boot does, then report setup / probe / channels."""
    board, source, errors = config.load()
    print('config   : %s  (version %s)' % (source, board.get('version')))
    if errors:
        print('config errors: %s' % errors)
    print('board    : %s' % board.get('board', {}).get('id'))

    flight = await main.bringup(board, log=lambda line: None)  # quiet -- the report below IS the output
    _report_setup(board, flight)
    await _report_probe()
    await asyncio.sleep_ms(_SETTLE_MS)
    _report_channels()
    print('DONE')


asyncio.run(main_diag())
