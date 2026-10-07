"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The host tools' verdict on SD cards the ring_saver wrote in a Rust test (src/avionics/crates/recorder/tests/
saver_tools.rs; doc/specs/avionics-port.md, M4's proof): for each session directory, what recorder_flight's
wrapped reader, assemble_capture and flight_telemetry make of it, one JSON object a line, so that one Python
process judges every card of a sweep.

    python3 sd_tools.py <tools dir> <session> <directory> [<directory> ...]

Per directory (a pulled session directory: <session>_<stream>.csv, recorder.log, session.csv):
  * recorder_flight, the reader `recorder_flight.py <directory> --session <session>` runs: each stream's
    [good, salvaged in, rejected, duplicates dropped] (its board.log and the lines that prove no stream under their
    own keys), the lines elsewhere that prove nothing, board.log's lines, and the sources of the session.csv rows
    that date this session's boot;
  * assemble_capture, then flight_telemetry strictly over its capture: the line verdicts, the duplicates dropped
    (by assemble_capture, and then by the reader), the files flagged as two boots spliced, the rows per stream and
    the log lines.
Stdlib only, with the tools' own modules.
"""

import contextlib
import io
import json
import os
import re
import sys
import tempfile


def _verdict(session: str, directory: str) -> dict:
    """
    What the tools make of one session directory.

    Args:
        session - the session prefix, e.g. '000123'.
        directory - the session directory.

    Returns:
        The JSON-able verdict the module docstring lists.
    """
    import assemble_capture
    import flight_telemetry
    import recorder_flight

    _streams, tally, clock, elsewhere, board = recorder_flight._wrapped_streams(directory, session)
    rows = recorder_flight._index_rows(clock)
    dated = [row['source'] for row in rows if row['session'] == session and recorder_flight._utc(row)]
    with tempfile.TemporaryDirectory() as scratch:
        capture = os.path.join(scratch, 'capture.txt')
        printed = io.StringIO()
        with contextlib.redirect_stdout(printed):
            assemble_capture.assemble(session, directory, capture)
        streams, logs = flight_telemetry.load(capture)
    assembled = re.search(r'duplicates\s+(\d+) rows dropped', printed.getvalue())
    return {
        'directory': directory,
        'tally': tally,
        'elsewhere': elsewhere,
        'board': len(board),
        'dated': dated,
        'assembled_duplicates': int(assembled.group(1)) if assembled else 0,
        'counts': flight_telemetry.line_counts(),
        'duplicates': flight_telemetry.duplicates(),
        'spliced': flight_telemetry.spliced(streams),
        'rows': {name: len(stream.rows) for name, stream in streams.items()},
        'logs': len(logs),
    }


def main() -> None:
    """Command line: see the module docstring."""
    if len(sys.argv) < 4:
        sys.exit('usage: sd_tools.py <tools dir> <session> <directory> [<directory> ...]')
    sys.path.insert(0, sys.argv[1])
    for directory in sys.argv[3:]:
        print(json.dumps(_verdict(sys.argv[2], directory)))


if __name__ == '__main__':
    main()
