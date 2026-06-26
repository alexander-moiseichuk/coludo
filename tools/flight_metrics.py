# tools/flight_metrics.py -- miss / in-zone / max-from-pad / duration for each capture in a directory,
# computed from the GNSS track vs the HPRC pad + landing zone (sim_model.HPRC). Miss = touchdown distance
# to the zone centre; duration = GNSS-track span (proxy for flight time from launch).
# Usage: flight_metrics.py <dir-of-<scenario>.txt>

import math
import os
import sys

_M_PER_DEG = 111320.0
_PAD = (25.514379, -80.391795)
_TL, _BR = (25.514944, -80.392972), (25.514583, -80.391111)
_CENTER = ((_TL[0] + _BR[0]) / 2.0, (_TL[1] + _BR[1]) / 2.0)
_COSLAT = math.cos(math.radians(_PAD[0]))
_ORDER = ('noise05', 'noise10', 'noise25', 'noise50', 'noise100',
          'wind00', 'wind03', 'wind06', 'wind09', 'wind12', 'corner_spike', 'corner_stress')


def _meters(a: tuple, b: tuple) -> float:
    return math.hypot((a[0] - b[0]) * _M_PER_DEG, (a[1] - b[1]) * _M_PER_DEG * _COSLAT)


def _track(path: str) -> list:
    gnss = []
    for line in open(path):
        if 'gnss.csv@' in line:
            row = line.strip().split('gnss.csv@', 1)[1]
            if not row.startswith('uptime'):
                fields = row.split(';')
                gnss.append((int(fields[0]), float(fields[1]), float(fields[2])))
    return gnss


def metrics(path: str):
    """(miss_m, in_zone, max_from_pad_m, duration_s) for a capture, or None when it has no GNSS."""
    gnss = _track(path)
    if not gnss:
        return None
    touchdown = (gnss[-1][1], gnss[-1][2])
    in_zone = (_BR[0] <= touchdown[0] <= _TL[0]) and (_TL[1] <= touchdown[1] <= _BR[1])
    return (_meters(touchdown, _CENTER), in_zone,
            max(_meters((p[1], p[2]), _PAD) for p in gnss), (gnss[-1][0] - gnss[0][0]) / 1e6)


if __name__ == '__main__':
    directory = sys.argv[1]
    print('%-14s %6s %6s %8s %6s' % ('scenario', 'miss', 'zone', 'maxpad', 'dur'))
    for scenario in _ORDER:
        capture = os.path.join(directory, scenario + '.txt')
        if not os.path.exists(capture):
            continue
        result = metrics(capture)
        if result:
            print('%-14s %6.0f %6s %8.0f %6.1f'
                  % (scenario, result[0], 'yes' if result[1] else 'no', result[2], result[3]))
