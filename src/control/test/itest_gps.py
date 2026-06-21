# Integration test: read a real serial GPS on the Control host and report when it reaches a usable
# launch fix (3D + 4+ satellites). Hardware-gated — needs a GPS plugged in (default /dev/ttyUSB0);
# the `itest_` prefix keeps it out of the default `make test` (host-only) run.
#
#   python3 src/control/test/itest_gps.py            (GPS_DEVICE / GPS_BAUD env override the port)
#
# A cold receiver takes tens of seconds to a few minutes to fix; this watches for up to TIMEOUT_S and
# prints the fix as it improves, exiting 0 on the first usable fix and 1 if it never gets one.

import asyncio
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))  # src/control
import gps  # noqa: E402

DEVICE = os.environ.get('GPS_DEVICE', '/dev/ttyUSB0')
BAUD = int(os.environ.get('GPS_BAUD', '9600'))
TIMEOUT_S = float(os.environ.get('GPS_TIMEOUT', '180'))


async def main() -> int:
    unit = gps.Gps()
    try:
        reader = await gps.open_serial(DEVICE, BAUD)
    except OSError as error:
        print('itest_gps: cannot open %s (%s) — plug a GPS in or set GPS_DEVICE' % (DEVICE, error))
        return 1
    print('itest_gps: reading %s @ %d for up to %ds — waiting for a 3D fix with %d+ satellites' % (
        DEVICE, BAUD, TIMEOUT_S, gps.IDEAL_SATELLITES))

    last = None
    loop = asyncio.get_event_loop()
    deadline = loop.time() + TIMEOUT_S
    while loop.time() < deadline:
        try:
            raw = await asyncio.wait_for(reader.readline(), timeout=deadline - loop.time())
        except asyncio.TimeoutError:
            break
        if not raw or not unit.feed(raw.decode('ascii', 'ignore')):
            continue
        snapshot = (unit.fix.fix_3d, unit.fix.satellites)
        if snapshot != last:  # print only when the fix state changes
            last = snapshot
            print('  fix_3d=%s sats=%d pos=%s,%s' % (
                unit.fix.fix_3d, unit.fix.satellites, unit.fix.latitude, unit.fix.longitude))
        if unit.fix.usable:
            print('ok: usable launch fix — %s' % unit.status())
            return 0
    print('itest_gps: no usable fix within %ds (last %s)' % (TIMEOUT_S, unit.status()))
    return 1


sys.exit(asyncio.run(main()))
