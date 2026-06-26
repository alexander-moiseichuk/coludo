# tools/assemble_capture.py -- reassemble the per-stream CSVs the Luckfox writes for one recorder session
# (<session>_<file>.csv, pulled with `adb pull`) into the interleaved recorder wire-format capture that
# flight_telemetry / flight_report / flight_svg read. Stage transitions are synthesized from sequencer.csv
# as `... controller :: stage -> X` log lines so the reports mark them.
# Usage: assemble_capture.py <session> <indir> <out.txt>

import glob
import os
import sys


def assemble(session: str, indir: str, out: str) -> int:
    """Merge <indir>/<session>_*.csv into the capture `out`; return the row count."""
    lines = []
    for path in sorted(glob.glob(os.path.join(indir, session + '_*.csv'))):
        name = os.path.basename(path)[len(session) + 1:]  # strip the '<session>_' prefix
        for row in open(path):
            row = row.rstrip('\r\n')
            if row:
                lines.append('@%s_%s@%s' % (session, name, row))
        if name == 'sequencer.csv':  # synthesize stage-event log lines for the report markers
            for row in open(path):
                fields = row.strip().split(';')
                if len(fields) >= 2 and fields[0].isdigit():
                    lines.append('%s controller :: stage -> %s' % (fields[0], fields[1]))
    with open(out, 'w') as handle:
        handle.write('\n'.join(lines) + '\n')
    return len(lines)


if __name__ == '__main__':
    session_id, in_dir, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
    print('assembled %s (%d rows)' % (out_path, assemble(session_id, in_dir, out_path)))
