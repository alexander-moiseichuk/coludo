"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for gps.py: NMEA parsing (GGA position/sats/altitude, GSA 2D/3D), fix-quality
logic (usable = 3D + 4 satellites + a position), checksum + malformed-line robustness, and the
launch-position dict handed to `assist`. Pure parsing -- no serial hardware. Run by `make test`.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import gps  # noqa: E402  (subject under test)


def nmea(body: str) -> str:
    """Wrap an NMEA body in `$...*hh` with a correct XOR checksum."""
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return '$%s*%02X' % (body, checksum)


def main():
    # a 3D GSA + a GGA with 8 satellites -> a usable fix with position and altitude
    unit = gps.Gps(log=lambda message: None)
    assert unit.feed(nmea('GPGSA,A,3,01,02,03,04,05,,,,,,,,2.5,1.3,2.1'))
    assert unit.feed(nmea('GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,'))
    status = unit.status()
    assert status['fix_3d'] and status['satellites'] == 8 and status['usable'], status
    assert abs(status['latitude'] - 48.1173) < 1e-4, status
    assert abs(status['longitude'] - 11.51667) < 1e-4, status
    assert abs(status['altitude'] - 545.4) < 1e-3, status
    assert unit.lines == 2
    # position() is the mission dict `assist` would push
    assert unit.position() == {'latitude': status['latitude'], 'longitude': status['longitude'],
                               'altitude': status['altitude']}

    # southern / western hemispheres flip the sign
    south = gps.Gps(log=lambda message: None)
    south.feed(nmea('GNGSA,A,3,01,02,03,04,,,,,,,,,2.0,1.0,1.5'))  # GN talker id also accepted
    south.feed(nmea('GNGGA,000000,3349.000,S,15112.000,W,1,06,1.0,30.0,M,0,M,,'))
    assert south.fix.latitude < 0 and south.fix.longitude < 0
    assert south.fix.usable  # 3D + 6 sats

    # a 2D fix is never usable, even with plenty of satellites
    two_d = gps.Gps(log=lambda message: None)
    two_d.feed(nmea('GPGSA,A,2,01,02,03,04,05,06,07,08,,,,,3.0,2.0,2.0'))
    two_d.feed(nmea('GPGGA,123519,4807.038,N,01131.000,E,1,09,0.9,545.4,M,46.9,M,,'))
    assert two_d.fix.satellites == 9 and not two_d.fix.fix_3d
    assert not two_d.fix.usable and two_d.position() is None

    # a 3D fix with too few satellites is not the ideal launch condition
    sparse = gps.Gps(log=lambda message: None)
    sparse.feed(nmea('GPGSA,A,3,01,02,03,,,,,,,,,,4.0,3.0,3.0'))
    sparse.feed(nmea('GPGGA,123519,4807.038,N,01131.000,E,1,03,2.0,545.4,M,46.9,M,,'))
    assert sparse.fix.fix_3d and sparse.fix.satellites == 3
    assert not sparse.fix.usable and sparse.position() is None

    # robustness: a bad checksum, a non-NMEA line, and a malformed field are all rejected, and none
    # of them corrupt the running fix
    robust = gps.Gps(log=lambda message: None)
    robust.feed(nmea('GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,'))
    good_sats = robust.fix.satellites
    assert robust.feed('$GPGGA,bad,checksum*00') is False  # wrong checksum
    assert robust.feed('not an nmea sentence') is False
    assert robust.feed(nmea('GPGGA,123519,xx,N,01131.000,E,1,zz,0.9,545.4,M,46.9,M,,')) is False  # bad fields
    assert robust.feed(nmea('GPVTG,054.7,T,034.4,M,005.5,N,010.2,K')) is False  # unhandled sentence
    assert robust.fix.satellites == good_sats and robust.lines == 1  # only the first line counted

    # altitude omitted from the position dict when the GGA carries none
    no_alt = gps.Gps(log=lambda message: None)
    no_alt.feed(nmea('GPGSA,A,3,01,02,03,04,,,,,,,,,2.0,1.0,1.5'))
    no_alt.feed(nmea('GPGGA,123519,4807.038,N,01131.000,E,1,07,0.9,,M,,M,,'))
    assert no_alt.fix.usable and no_alt.position() == {'latitude': no_alt.fix.latitude,
                                                       'longitude': no_alt.fix.longitude}

    # a STALE fix is not usable: `sync GNSS` would persist it as the launch point. Fresh first...
    stale = gps.Gps(log=lambda message: None)
    stale.feed(nmea('GPGSA,A,3,01,02,03,04,,,,,,,,,2.0,1.0,1.5'))
    stale.feed(nmea('GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,'))
    assert stale.fix.usable and stale.status()['age'] < 1.0, stale.status()
    # ...then the receiver goes silent past STALE_S
    stale.fix.seen -= gps.STALE_S + 1.0
    assert not stale.fix.usable and stale.position() is None, stale.status()
    assert stale.status()['age'] > gps.STALE_S and not stale.status()['usable'], stale.status()
    # a GSA alone never makes a fix fresh: only a GGA carries the position it vouches for
    assert gps.Gps(log=lambda message: None).status()['age'] is None

    # serve() on a missing device REPORTS it and returns -- it must NOT raise (it is gathered with
    # hub.run(), so an unhandled open failure would take the whole hub down).
    import asyncio
    reported = []
    asyncio.run(gps.Gps(log=reported.append).serve('/dev/coludo-no-such-gps', 9600))
    assert any('unavailable' in message for message in reported), reported

    # a receiver LOST after it opened drops its fix at once and is reopened, not abandoned
    opened = []

    async def fake_open(device, baud):
        reader = asyncio.StreamReader()
        if not opened:  # the first stream carries a usable fix and then ends (the unplug)
            for sentence in ('GPGSA,A,3,01,02,03,04,,,,,,,,,2.0,1.0,1.5',
                             'GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,'):
                reader.feed_data((nmea(sentence) + '\r\n').encode())
            reader.feed_eof()
        opened.append(device)
        return reader  # the reopened stream never ends

    async def lose_and_reopen(unit):
        task = asyncio.create_task(unit.serve('/dev/coludo-fake-gps', 9600))
        while len(opened) < 2:
            await asyncio.sleep(0.005)
        task.cancel()

    real_open, real_reopen = gps.open_serial, gps._REOPEN_S
    gps.open_serial, gps._REOPEN_S = fake_open, 0.01
    try:
        lost_log = []
        lost = gps.Gps(log=lost_log.append)
        asyncio.run(asyncio.wait_for(lose_and_reopen(lost), 5.0))
    finally:
        gps.open_serial, gps._REOPEN_S = real_open, real_reopen
    assert lost.lines == 2, lost.lines  # the fix WAS there before the loss
    assert not lost.fix.usable and not lost.fix.has_position and lost.position() is None, lost.status()
    assert any('lost' in message for message in lost_log), lost_log
    assert any('back on' in message for message in lost_log), lost_log

    print('ok: gps NMEA parse (GGA/GSA), usable=3D+4sat+fresh, hemisphere signs, checksum/malformed '
          'robustness, a lost receiver drops its fix and is reopened')


main()
