"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Recover a board held in RESET by the serial bridge's control lines -- the wedge a USB power-cycle
cannot fix.

The P4's REPL arrives through a CH34x bridge (1a86:55d3) whose DTR/RTS drive EN/IO0. A crashed or
killed `mpremote` can leave those lines asserted, which holds the MCU in reset. The signature is
distinctive and misleading:

    * the port still ENUMERATES (the bridge is a separate chip, powered independently of the MCU);
    * `mpremote` says "could not enter raw repl";
    * the serial line is COMPLETELY SILENT -- not even boot output.

That last point is how to tell it apart from a busy REPL: a board running main.py still prints log
lines; a held-reset board prints nothing at all. And because the lines are driven by the HOST's serial
state, cycling the board's USB power changes nothing -- which is what makes this one so easy to
misdiagnose as dead hardware.

The cure is to open the port with DTR/RTS DE-asserted before open(), then strobe EN.

  python3 tools/board_unwedge.py                 # default /dev/ttyACM0
  python3 tools/board_unwedge.py --port /dev/ttyACM1 --listen 12
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit('board_unwedge needs pyserial:  pip install pyserial')


def unwedge(port: str, listen: float) -> int:
    """
    De-assert the control lines, strobe EN, and report whatever the board says.

    Args:
        port - the serial device the bridge presents.
        listen - seconds to listen for boot output after the reset.

    Returns:
        0 when the board talks back, 1 when it stays silent.
    """
    try:
        # de-assert BEFORE open(): opening with them asserted re-applies the very reset we are clearing
        handle = serial.Serial()
        handle.port = port
        handle.baudrate = 115200
        handle.timeout = 1
        handle.dtr = False
        handle.rts = False
        handle.open()
    except Exception as error:
        print('cannot open %s: %r' % (port, error), file=sys.stderr)
        return 1
    print('%s open with DTR/RTS de-asserted -- strobing EN' % port)
    handle.rts = True    # EN low: assert reset
    time.sleep(0.15)
    handle.rts = False   # release -> the MCU runs
    handle.dtr = False   # IO0 high -> normal boot, not the bootloader
    handle.reset_input_buffer()

    deadline = time.time() + listen
    buffered = b''
    while time.time() < deadline:
        buffered += handle.read(4096)
    handle.close()
    if not buffered:
        print('STILL SILENT after %.0f s.' % listen)
        print('  Not this wedge, then. Next: check the board has power, try the other /dev/ttyACM*,')
        print('  or reflash -- a silent board with a live bridge can also mean corrupted firmware.')
        return 1
    text = buffered.decode('utf-8', 'replace')
    print('--- %d bytes ---' % len(buffered))
    print(text[-1200:])
    print('--- board is talking again; mpremote should work now ---')
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description='Clear a DTR/RTS-held board reset (see module docstring).')
    parser.add_argument('--port', default='/dev/ttyACM0', help='serial port (default /dev/ttyACM0)')
    parser.add_argument('--listen', type=float, default=10.0, help='seconds to listen after reset')
    args = parser.parse_args()
    return unwedge(args.port, args.listen)


if __name__ == '__main__':
    sys.exit(main())
