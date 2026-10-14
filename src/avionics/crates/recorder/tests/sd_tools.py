"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The host tools' verdict on SD cards the ring_saver wrote in a Rust test (src/avionics/crates/recorder/tests/
saver_tools.rs; doc/specs/avionics-port.md, M4's proof): for each card, what recorder_flight's wrapped reader,
assemble_capture and flight_telemetry make of one session on it, one JSON object a line, so that one Python process
judges every card of a sweep.

    python3 sd_tools.py <tools dir> <session> <card> [<card> ...]

A card is a directory laid out as the board's /sdcard (doc/specs/avionics-port.md "SD and flash file layout",
crates/recorder layout.rs): the session's files in <card>/<session>/recordings/ (<session>_<stream>.csv,
recorder.log, session.csv), which is what the tools read, as tools/board_pull.sh hands them <outdir>/recordings/; and
the global index <card>/sessions.csv, every boot's session.csv rows. Per card:
  * recorder_flight, the reader `recorder_flight.py <recordings> --session <session>` runs: each stream's
    [good, salvaged in, rejected, duplicates dropped] (its board.log and the lines that prove no stream under their
    own keys), the lines elsewhere that prove nothing, board.log's lines, the sources of the session's own
    session.csv rows that name it (each once), and those of them that date this session's boot;
  * assemble_capture, then flight_telemetry strictly over its capture: the line verdicts, the duplicates dropped
    (by assemble_capture, and then by the reader), the files flagged as two boots spliced, the rows per stream and
    the log lines;
  * the global index: the sources of its rows that name this session (each once, however many duplicates a cut
    left), every line of it verified under the routing `session.csv`, and the lines of it that did not verify.
Stdlib only, with the tools' own modules.
"""

import contextlib
import io
import json
import os
import re
import sys
import tempfile

RECORDINGS: str = 'recordings'  # a session's recordings directory: <card>/<session>/recordings (layout RECORDINGS)
GLOBAL_INDEX: str = 'sessions.csv'  # every boot's session.csv rows, at the card's root (layout GLOBAL_INDEX)
_SESSION_CELL: int = 2  # a session.csv row's `session` cell: uptime, boot, session, ...
_SOURCE_CELL: int = 8  # its `source` cell (boot, cc-auto, dashboard, anchor)


def _indexed(session: str, card: str) -> tuple:
    """
    The global index's rows that name `session`.

    Args:
        session - the session prefix.
        card - the card's root.

    Returns:
        (the sorted distinct sources of those rows, the lines of the index that did not verify).
    """
    import recorder_wire

    sources, damaged = set(), 0
    path = os.path.join(card, GLOBAL_INDEX)
    with open(path, encoding='utf-8', errors='surrogateescape', newline='\n') as handle:
        for line in handle.read().splitlines():
            payload = recorder_wire.verify(recorder_wire.SESSION_INDEX, line)
            if payload is None:
                damaged += 1
                continue
            cells = payload.split(';')
            if cells[0].isdigit() and len(cells) > _SOURCE_CELL and cells[_SESSION_CELL] == session:
                sources.add(cells[_SOURCE_CELL])
    return sorted(sources), damaged


def _verdict(session: str, card: str) -> dict:
    """
    What the tools make of one session on one card.

    Args:
        session - the session prefix, e.g. '000123'.
        card - the card's root.

    Returns:
        The JSON-able verdict the module docstring lists.
    """
    import assemble_capture
    import flight_telemetry
    import recorder_flight

    directory = os.path.join(card, session, RECORDINGS)
    _streams, tally, clock, elsewhere, board = recorder_flight._wrapped_streams(directory, session)
    rows = recorder_flight._index_rows(clock)
    dated = [row['source'] for row in rows if row['session'] == session and recorder_flight._utc(row)]
    sources = sorted({row['source'] for row in rows if row['session'] == session})
    with tempfile.TemporaryDirectory() as scratch:
        capture = os.path.join(scratch, 'capture.txt')
        printed = io.StringIO()
        with contextlib.redirect_stdout(printed):
            assemble_capture.assemble(session, directory, capture)
        streams, logs = flight_telemetry.load(capture)
    assembled = re.search(r'duplicates\s+(\d+) rows dropped', printed.getvalue())
    indexed, index_damaged = _indexed(session, card)
    return {
        'directory': directory,
        'tally': tally,
        'elsewhere': elsewhere,
        'board': len(board),
        'dated': dated,
        'sources': sources,
        'assembled_duplicates': int(assembled.group(1)) if assembled else 0,
        'counts': flight_telemetry.line_counts(),
        'duplicates': flight_telemetry.duplicates(),
        'spliced': flight_telemetry.spliced(streams),
        'rows': {name: len(stream.rows) for name, stream in streams.items()},
        'logs': len(logs),
        'indexed': indexed,
        'index_damaged': index_damaged,
    }


def main() -> None:
    """Command line: see the module docstring."""
    if len(sys.argv) < 4:
        sys.exit('usage: sd_tools.py <tools dir> <session> <card> [<card> ...]')
    sys.path.insert(0, sys.argv[1])
    for card in sys.argv[3:]:
        print(json.dumps(_verdict(sys.argv[2], card)))


if __name__ == '__main__':
    main()
