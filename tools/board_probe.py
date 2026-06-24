# board_probe.py — run ON a board to sketch an initial board.config: identity (name, unique_id),
# and which UART/I2C/SPI ids + GPIOs are constructable. A discovery aid, not a pass/fail test.
#
#   mpremote connect /dev/ttyACM0 run tools/board_probe.py
#
# HAZARD: some ids/pins HARD-CRASH this port (e.g. I2C(2), and flash/PSRAM GPIOs) and reset the
# board -- a try/except cannot catch that. So every result is printed immediately (streamed), the
# crashy I2C probe runs last, and the GPIO sweep is opt-in. If the board resets mid-run, the
# streamed output up to that point is still valid; rerun skipping the id/pin that reset it.
#
# Captured results, one round per board: doc/board-probe.md.

import os

import machine
import ubinascii


def identity():
    print('# identity')
    print('machine  : %s' % os.uname().machine)
    print('release  : %s' % os.uname().release)
    print('unique_id: %s' % ubinascii.hexlify(machine.unique_id()).decode())
    print('freq_hz  : %d' % machine.freq())
    try:
        print('Pin.board: %s' % sorted(a for a in dir(machine.Pin.board) if not a.startswith('_')))
    except Exception as error:
        print('Pin.board: (%r)' % error)


def buses():
    # UART/SPI first, then I2C (its higher ids are the known crashers) so we keep the most data.
    print('\n# bus defaults (id 0..5; a crash here means that id does not exist safely)')
    for name in ('UART', 'SPI', 'I2C'):
        cls = getattr(machine, name)
        for i in range(6):
            try:
                print('%s(%d): %s' % (name, i, cls(i)))
            except Exception as error:
                print('%s(%d): -- %r' % (name, i, error))


def gpios(low=0, high=55):
    # Riskiest sweep: reconfiguring a flash/PSRAM pin as input can reset the board.
    print('\n# constructable GPIOs %d..%d as input' % (low, high - 1))
    usable = []
    for n in range(low, high):
        try:
            machine.Pin(n, machine.Pin.IN)
            usable.append(n)
        except Exception:
            pass
    print('constructable: %s' % usable)


identity()
buses()
# gpios()  # opt-in: uncomment only if you accept that flash-pin construction may reset the board
