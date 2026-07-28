"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Pre-run gates shared by the default scripts and by `make check`.

Nothing in this repo is gated by CI (deliberately -- local gates only), so the gating has to live where
the work actually starts: a simulation run checks its INSTALL and its DATA CONSISTENCY before burning
minutes producing a capture nobody can read. The same checks back `make check` at the repo root.

Two kinds of gate:
  * install  -- the interpreter can import what this run needs (the glider tree, optional plotly, the
                mpy-cross the board gate uses). A missing dep should fail in the first second, not after
                a 20-run sweep.
  * data     -- the config validates, and the TELEMETRY SCHEMA agrees between the sim and the board. The
                second one is the check that would have caught findings §27.1 (the sim recorded a fused
                fins.csv while a board records per-servo streams, so every fin-aware tool silently found
                nothing on a real capture).

  python3 tools/preflight.py            # run every gate, report, exit non-zero on failure
  python3 tools/preflight.py --quiet    # only report failures (what the scripts use)
"""

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_GLIDER = os.path.join(_HERE, '..', 'src', 'glider')


def _ok(label: str, detail: str = '') -> tuple:
    return (True, label, detail)


def _bad(label: str, detail: str) -> tuple:
    return (False, label, detail)


def check_glider_importable() -> tuple:
    """The shared glider modules import on the host (config/sim/control math are host-portable by design)."""
    if _GLIDER not in sys.path:
        sys.path.insert(0, _GLIDER)
    try:
        import config_default  # noqa: F401
        import sim_model  # noqa: F401
    except Exception as error:
        return _bad('glider tree importable', '%r -- is src/glider present?' % error)
    return _ok('glider tree importable', _GLIDER)


def check_config() -> tuple:
    """config_default validates against its own schema -- a broken default breaks every board and sim."""
    if _GLIDER not in sys.path:
        sys.path.insert(0, _GLIDER)
    try:
        import config
        import config_default
        errs = config.validate(config_default.default())
    except Exception as error:
        return _bad('config_default validates', '%r' % error)
    if errs:
        return _bad('config_default validates', '; '.join(errs))
    return _ok('config_default validates', 'schema %s' % config_default.CONFIG_VERSION)


def check_telemetry_schema() -> tuple:
    """
    The SIM and the BOARD must record the same stream shapes.

    This is the gate for findings §27.1: the sim used to record one fused `fins.csv` while a real board
    records one stream per servo, so every fin-aware tool came back empty on a board capture and nobody
    noticed until the shapes were compared. Here the sim's own capture header is parsed and the streams a
    renderer depends on must resolve -- via the same role-based lookup the tools use, so a rename that
    breaks the tools breaks this check first.
    """
    if _HERE not in sys.path:
        sys.path.insert(0, _HERE)
    try:
        import flight_telemetry
        import virtual_flight
    except Exception as error:
        return _bad('telemetry schema', 'cannot import the sim/parser: %r' % error)
    try:
        rows = virtual_flight._Capture()
        rows.header()
        streams, _logs = flight_telemetry.parse('\n'.join(rows._lines))
    except Exception as error:
        return _bad('telemetry schema', 'sim header did not parse: %r' % error)
    required = {
        'attitude': ('roll', 'pitch'),
        'position': ('lat', 'lon'),
        'fins': ('eleron_left', 'eleron_right', 'yaw'),
        'control state': ('fin_cap',),
        'airspeed': ('dynamic_pressure',),
    }
    missing = [role for role, fields in required.items()
               if flight_telemetry.find_stream(streams, *fields) is None]
    if missing:
        return _bad('telemetry schema', 'sim capture has no stream for: %s' % ', '.join(missing))
    return _ok('telemetry schema', '%d streams, all renderer roles resolve' % len(streams))


def check_plotly() -> tuple:
    """plotly backs the HTML reports -- optional, so this is reported but never fatal."""
    try:
        import plotly  # noqa: F401
    except ImportError:
        return _ok('plotly (optional)', 'ABSENT -- flight_report/HTML will not render')
    return _ok('plotly (optional)', 'present')


def check_mpy_cross() -> tuple:
    """The board syntax gate deploy.sh uses; optional on a host that only runs sims."""
    for name in os.listdir(_HERE):
        if name.startswith('mpy-cross') and os.access(os.path.join(_HERE, name), os.X_OK):
            return _ok('mpy-cross (optional)', name)
    return _ok('mpy-cross (optional)', 'ABSENT -- board compile gate unavailable')


_GATES = (check_glider_importable, check_config, check_telemetry_schema, check_plotly, check_mpy_cross)


def run(quiet: bool = False) -> int:
    """
    Run every gate; return the number that FAILED.

    Args:
        quiet - report only failures (what a script calls before doing real work).

    Returns:
        The failure count (0 = good to go).
    """
    failures = 0
    for gate in _GATES:
        passed, label, detail = gate()
        if not passed:
            failures += 1
            print('preflight FAIL : %s -- %s' % (label, detail), file=sys.stderr)
        elif not quiet:
            print('preflight ok   : %-26s %s' % (label, detail))
    return failures


def gate(context: str = 'run') -> None:
    """
    Run the gates and ABORT the caller when any fails -- the entry point for a default script.

    Args:
        context - what is about to happen, for the message ('simulation', 'report', ...).

    Returns:
        None; exits(2) when a gate fails.
    """
    if run(quiet=True):
        sys.exit('preflight: refusing to start the %s (fix the failures above, '
                 'or run `python3 tools/preflight.py` for the full report)' % context)


def main() -> int:
    parser = argparse.ArgumentParser(description='Coludo pre-run gates (install + data consistency).')
    parser.add_argument('--quiet', action='store_true', help='report only failures')
    args = parser.parse_args()
    failures = run(quiet=args.quiet)
    if failures:
        print('preflight: %d gate(s) FAILED' % failures, file=sys.stderr)
        return 1
    if not args.quiet:
        print('preflight: all gates passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
