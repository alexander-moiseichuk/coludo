#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The card-layout A/B under a HITL flight (doc/specs/avionics-port.md "SD and flash file layout"): one row per run from
its tools/hitl_host.py output and its recorder.log, and a table per layout -- the control tick's lateness, the video's
missed frames and card figures, the ring_saver's backlog over the flight.

  tools/partition_ab.py <run dir>... [-o <outdir>]   # each dir holds host.log and recorder.log

Writes runs.csv (';', one row per run) and prints the table by layout. A run dir comes from
tools/hitl_partition_ab.sh: hitl_host.py --video --clean, then the session's recorder.log.

The backlog is how far the saver trails the writer in the flash ring, from the recorder's status line once a second
(`'writer': (block, offset), 'saver': (block, offset)`), over the flight: from the stage's `boosting` line to its `done`
line. A block is 4 KiB.
"""
import argparse
import os
import re
import sys

_BLOCK: int = 4096
_OVER_BYTES: int = 64 << 10
_POSITIONS: re.Pattern = re.compile(r"'writer': \((\d+), (\d+)\), 'saver': \((\d+), (\d+)\)")
_LINE: re.Pattern = re.compile(r'^\{[0-9a-f]{8}\};(\d+) (.*);<[0-9a-f]{8}>$')
_COLUMNS: tuple = ('board', 'layout', 'session', 'outcome', 'late_max_us', 'overruns', 'work_max_us', 'captured',
                   'missed', 'missed_pct', 'dropped', 'write_max_ms', 'sd_busy_pct', 'encode_max_ms', 'unsaved_s',
                   'backlog_mean_kb', 'backlog_peak_kb', 'backlog_over_64kb_s')


def named(line: str) -> dict:
    """A reply line's `key=value` params (`CONTROL ok control schedule_hz=100 ...`); the rest left out."""
    return dict(token.split('=', 1) for token in line.split() if '=' in token)


def host_figures(text: str) -> dict:
    """
    One hitl_host.py output's figures: the session, the outcome, and its CONTROL / VIDEO / RING / DF replies' params.

    Raises:
        ValueError - no SESSION line (not a hitl_host run).
    """
    figures = {'outcome': 'none'}
    for line in text.splitlines():
        label, _, rest = line.partition(' ')
        if label == 'SESSION':
            figures['session'] = rest.split()[0]
        elif label in ('DONE', 'TIMEOUT'):
            figures['outcome'] = label.lower()
        elif label in ('CONTROL', 'VIDEO', 'RING', 'DF') and rest.startswith('ok '):
            figures[label.lower()] = named(rest)
    if 'session' not in figures:
        raise ValueError('no SESSION line')
    return figures


def backlog(text: str) -> list:
    """
    The saver's backlog over the flight, in bytes, one value a status line: from the `boosting` line to the `done` one
    of recorder.log.
    """
    values, flying = [], False
    for raw in text.splitlines():
        match = _LINE.match(raw)
        if not match:
            continue
        message = match.group(2)
        if 'stage -> boosting' in message:
            flying = True
        elif 'stage -> done' in message:
            break
        positions = _POSITIONS.search(message) if flying else None
        if positions:
            writer_block, writer_offset, saver_block, saver_offset = (int(value) for value in positions.groups())
            values.append(max(0, (writer_block - saver_block) * _BLOCK + writer_offset - saver_offset))
    return values


def row(board: str, host: str, recorder: str) -> dict:
    """One run's row (the `_COLUMNS`): its host.log and recorder.log text."""
    figures = host_figures(host)
    control, video = figures.get('control', {}), figures.get('video', {})
    ring, df = figures.get('ring', {}), figures.get('df', {})
    captured, missed = int(video.get('captured', 0)), int(video.get('missed', 0))
    values = backlog(recorder)
    return {
        'board': board, 'layout': df.get('layout', '?'), 'session': figures['session'], 'outcome': figures['outcome'],
        'late_max_us': int(control.get('late_max_us', -1)), 'overruns': int(control.get('overruns', -1)),
        'work_max_us': int(control.get('work_max_us', -1)), 'captured': captured, 'missed': missed,
        'missed_pct': round(100.0 * missed / captured, 2) if captured else -1.0,
        'dropped': int(video.get('dropped', -1)),
        'write_max_ms': round(int(video.get('write_us_max', -1000)) / 1000.0, 1),
        'sd_busy_pct': float(video.get('sd_busy_pct', -1)),
        'encode_max_ms': round(int(video.get('encode_us_max', -1000)) / 1000.0, 1),
        'unsaved_s': float(ring.get('unsaved_s', -1)),
        'backlog_mean_kb': round(sum(values) / len(values) / 1024.0, 1) if values else -1.0,
        'backlog_peak_kb': round(max(values) / 1024.0, 1) if values else -1.0,
        'backlog_over_64kb_s': sum(1 for value in values if value > _OVER_BYTES),
    }


def table(rows: list) -> str:
    """
    The markdown table by layout: each figure's mean over the runs, and its worst run in brackets (every figure here
    is worse when larger).
    """
    layouts = sorted({entry['layout'] for entry in rows})
    figures = _COLUMNS[4:]
    lines = ['| | ' + ' | '.join('%s (%d runs)' % (layout, sum(1 for entry in rows if entry['layout'] == layout))
                                 for layout in layouts) + ' |',
             '|---|' + '---|' * len(layouts)]
    for figure in figures:
        cells = []
        for layout in layouts:
            values = [entry[figure] for entry in rows if entry['layout'] == layout]
            cells.append('%.4g [%.4g]' % (sum(values) / len(values), max(values)))
        lines.append('| %s | %s |' % (figure, ' | '.join(cells)))
    return '\n'.join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('runs', nargs='+', help='run dirs: <board>_<n>/ with host.log and recorder.log')
    parser.add_argument('-o', '--out', default='.', help='where runs.csv goes (default: .)')
    args = parser.parse_args()
    rows = []
    for run in args.runs:
        with open(os.path.join(run, 'host.log')) as host, open(os.path.join(run, 'recorder.log'),
                                                               errors='replace') as recorder:
            board = os.path.basename(os.path.normpath(run)).rsplit('_', 1)[0]
            rows.append(row(board, host.read(), recorder.read()))
    with open(os.path.join(args.out, 'runs.csv'), 'w') as stream:
        stream.write(';'.join(_COLUMNS) + '\n')
        for entry in rows:
            stream.write(';'.join(str(entry[column]) for column in _COLUMNS) + '\n')
    print(table(rows))
    return 0


if __name__ == '__main__':
    sys.exit(main())
