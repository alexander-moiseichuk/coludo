#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

An airframe study over the SITL (M14): the Rust flight code flown lockstep against sim_model, every combination of
airframe x glider mass x scenario x seeded run, scored by the physics (tools/hitl_host.py's LANDED line: the miss
from the zone's centre, inside the zone, the time from ignition to DONE).

  tools/sitl_matrix.py --config <board config JSON> -o <outdir> [--airframes tms7 tms8] [--masses 235 285]
                       [--runs 5] [--motor F15] [--boost-cdas 0.0015 0.0020 0.0030]

Writes <outdir>/runs.csv (';', one row per flight) and <outdir>/summary.csv, and prints the summary: per airframe and
mass, the median miss, the flights in the zone, the median time aloft, the flights that never reached DONE. The
scenarios are the pre-flight report's ten (doc/sims/TMS-7-preflight: tools/hitl_matrix_host.sh's twelve without
noise50 and noise100, unusable there); run r seeds the noise and the spikes with r, so a run repeats exactly.
"""
import argparse
import os
import re
import statistics
import subprocess
import sys

_ROOT: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
_SITL: str = os.path.join(_ROOT, 'src', 'avionics', 'target', 'release', 'sitl')
# name, noise, wind m/s, wind from deg, spike: hitl_matrix_host.sh's table without noise50/noise100
SCENARIOS: tuple = (
    ('noise05', 0.05, 0.0, 210.0, False), ('noise10', 0.10, 0.0, 210.0, False), ('noise25', 0.25, 0.0, 210.0, False),
    ('wind00', 0.10, 0.0, 210.0, False), ('wind03', 0.10, 3.0, 210.0, False), ('wind06', 0.10, 6.0, 210.0, False),
    ('wind09', 0.10, 9.0, 210.0, False), ('wind12', 0.10, 12.0, 210.0, False),
    ('corner_spike', 0.10, 0.0, 210.0, True), ('corner_stress', 0.50, 12.0, 210.0, True),
)
_LANDED: re.Pattern = re.compile(r'^LANDED miss=([\d.]+) inside=(True|False) aloft=([\d.]+) apogee=(\d+)$',
                                 re.MULTILINE)
_COLUMNS: tuple = ('airframe', 'mass_g', 'boost_cda', 'scenario', 'run', 'outcome', 'miss_m', 'inside', 'aloft_s',
                   'apogee_m')


def fly(config: str, airframe: str, mass: int, scenario: tuple, run: int, motor: str, boost_cda=None) -> dict:
    """
    One SITL flight; its row (`_COLUMNS`). A flight with no LANDED line reads outcome `none`, its figures -1.
    `boost_cda` overrides the airframe's (None: its own, `-` in the row).
    """
    name, noise, wind, direction, spike = scenario
    command = [sys.executable, os.path.join(_ROOT, 'tools', 'hitl_host.py'), '--sitl', _SITL, '--config', config,
               '--airframe', airframe, '--motor', motor, '--glider-g', str(mass), '--noise', str(noise),
               '--wind', str(wind), '--wind-dir', str(direction), '--seed', str(run)] + (['--spike'] if spike else []) \
        + (['--boost-cda', str(boost_cda)] if boost_cda is not None else [])
    out = subprocess.run(command, capture_output=True, text=True, timeout=600).stdout
    outcome = 'done' if '\nDONE' in '\n' + out else ('timeout' if 'TIMEOUT' in out else 'none')
    landed = _LANDED.search(out)
    return {'airframe': airframe, 'mass_g': mass, 'boost_cda': '-' if boost_cda is None else boost_cda,
            'scenario': name, 'run': run, 'outcome': outcome,
            'miss_m': float(landed.group(1)) if landed else -1.0,
            'inside': landed is not None and landed.group(2) == 'True',
            'aloft_s': float(landed.group(3)) if landed else -1.0,
            'apogee_m': int(landed.group(4)) if landed else -1}


def summary(rows: list) -> list:
    """
    Per airframe, mass and boost CdA: flights, median miss, in-zone count, median time aloft, median apogee, flights
    short of DONE.
    """
    combos = sorted({(row['airframe'], row['mass_g'], str(row['boost_cda'])) for row in rows})
    lines = []
    for airframe, mass, cda in combos:
        mine = [row for row in rows if (row['airframe'], row['mass_g'], str(row['boost_cda'])) == (airframe, mass, cda)]
        landed = [row for row in mine if row['miss_m'] >= 0]
        lines.append({
            'airframe': airframe, 'mass_g': mass, 'boost_cda': cda, 'flights': len(mine),
            'median_miss_m': round(statistics.median(row['miss_m'] for row in landed), 1) if landed else -1.0,
            'in_zone': sum(1 for row in mine if row['inside']),
            'median_aloft_s': round(statistics.median(row['aloft_s'] for row in landed), 1) if landed else -1.0,
            'median_apogee_m': statistics.median(row['apogee_m'] for row in landed) if landed else -1,
            'not_done': sum(1 for row in mine if row['outcome'] != 'done'),
        })
    return lines


def _write(path: str, rows: list, columns: tuple) -> None:
    with open(path, 'w') as stream:
        stream.write(';'.join(columns) + '\n')
        for row in rows:
            stream.write(';'.join(str(row[column]) for column in columns) + '\n')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('--config', required=True, help="a board's saved config JSON (the HITL copy is made of it)")
    parser.add_argument('-o', '--out', required=True, help='the output directory')
    parser.add_argument('--airframes', nargs='+', default=['tms7', 'tms8'])
    parser.add_argument('--masses', nargs='+', type=int, default=[235, 285])
    parser.add_argument('--runs', type=int, default=5)
    parser.add_argument('--motor', default='F15')
    parser.add_argument('--boost-cdas', nargs='+', type=float, help="sweep the boost CdA (m^2); default the airframe's")
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rows = []
    for airframe in args.airframes:
        for mass in args.masses:
            for cda in args.boost_cdas or [None]:
                for run in range(1, args.runs + 1):
                    for scenario in SCENARIOS:
                        rows.append(fly(args.config, airframe, mass, scenario, run, args.motor, cda))
                print('flown %s %d g cda %s: %d flights' % (airframe, mass, cda or 'own', args.runs * len(SCENARIOS)),
                      flush=True)
    _write(os.path.join(args.out, 'runs.csv'), rows, _COLUMNS)
    lines = summary(rows)
    _write(os.path.join(args.out, 'summary.csv'), lines, tuple(lines[0]))
    for line in lines:
        print(';'.join('%s=%s' % item for item in line.items()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
