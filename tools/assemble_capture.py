"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Reassemble the per-stream CSVs the Luckfox writes for one recorder session (<session>_<file>.csv,
pulled with `adb pull`) into the interleaved recorder wire-format capture that flight_telemetry /
flight_report / flight_svg read. Stage transitions are synthesized from sequencer.csv as
`... controller :: stage -> X` log lines so the reports mark them.

The session is a boot id (`000123`) on current firmware, a `recorder.session` label, or the older
`YYYYMMDD_HHMMSS_<tag>`; recorder_wire.session_files() decides which files are its own. Rows go out as the
Luckfox holds them, byte for byte (read and written as UTF-8 with errors='surrogateescape'): a wrapped row
(doc/specs/recorder-wire.md) becomes its original wire line again, since its file's name is its routing,
so flight_telemetry checks and salvages it. The lines this tool adds itself -- the stage marks and an
optional note -- are wrapped too when the capture is, or the strict reader of a wrapped capture would
reject them as damage.

Usage: assemble_capture.py <session> <indir> <out.txt> [note]
  note: one host log line appended at the end, e.g. hitl_collect's '0 capture :: build ...'
"""

import glob
import os
import sys

import recorder_wire


def _host_line(text: str, wrapped: bool) -> str:
    """
    A log line this tool adds to the capture.

    Args:
        text - the log line.
        wrapped - whether the capture carries the wrapper.

    Returns:
        The line wrapped like the board's own when the capture is wrapped, else as it is.
    """
    return recorder_wire.wrap(text).rstrip('\n') if wrapped else text


def assemble(session: str, indir: str, out: str, note: str | None = None) -> int:
    """
    Merge <indir>/<session>_*.csv into the capture `out`.

    Args:
        session - the recorder session id (the '<session>_' filename prefix).
        indir - directory holding the pulled per-stream CSVs.
        out - path to write the interleaved capture to.
        note - a host log line to append, or None.

    Returns:
        The number of lines written to `out`.
    """
    files = []
    names = sorted(os.path.basename(path) for path in glob.glob(os.path.join(indir, session + '_*.csv')))
    for name in recorder_wire.session_files(session, names):
        with open(os.path.join(indir, name), 'rb') as handle:  # a damaged byte fails its row's check, not the tool
            rows = [raw.decode('utf-8', 'surrogateescape').rstrip('\r\n') for raw in handle]
        files.append((name, [row for row in rows if row]))
    # one row that checks out under its file's name makes the capture wrapped; a lookalike does not
    wrapped = any(recorder_wire.verify(routing, row) is not None for routing, rows in files for row in rows)
    lines = []
    inventory = {}
    for routing, rows in files:
        name = routing[len(session) + 1:]  # strip the '<session>_' prefix
        lines.extend('@%s@%s' % (routing, row) for row in rows)
        inventory[name] = len(rows)
        if name == 'sequencer.csv':  # synthesize stage-event log lines for the report markers
            for row in rows:
                payload = recorder_wire.verify(routing, row)
                if wrapped and payload is None:
                    continue  # a damaged row must not become a trusted stage mark
                fields = (row if payload is None else payload).strip().split(';')
                if len(fields) >= 2 and fields[0].isdigit():
                    lines.append(_host_line('%s controller :: stage -> %s' % (fields[0], fields[1]), wrapped))
    if note:
        lines.append(_host_line(note, wrapped))
    with open(out, 'w', encoding='utf-8', errors='surrogateescape') as handle:
        handle.write('\n'.join(lines) + '\n')
    """
    REPORT WHAT WENT IN. A capture missing streams assembles into a file that looks exactly like a
    whole flight -- there is no marker of absence -- and every downstream tool then renders a partial
    flight as a complete one. That class has bitten twice (§27.1 hardcoded stream list, §27.8 a
    5-stream fixture standing in for an 8-stream flight). The stream set is config-dependent (per-device
    streams take their name from the device), so this cannot assert a fixed list; naming what IS here,
    with row counts, is what lets a human or a caller notice what is NOT.
    """
    for name in sorted(inventory):  # a junk name's bytes that are not UTF-8 print escaped, never raise
        printable = name.encode('utf-8', 'surrogateescape').decode('utf-8', 'backslashreplace')
        print('  %-28s %6d rows' % (printable, inventory[name]))
    print('  %-28s %6d streams' % ('TOTAL', len(inventory)))
    return len(lines)


if __name__ == '__main__':
    session_id, in_dir, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
    note_line = sys.argv[4] if len(sys.argv) > 4 else None
    print('assembled %s (%d rows)' % (out_path, assemble(session_id, in_dir, out_path, note_line)))
