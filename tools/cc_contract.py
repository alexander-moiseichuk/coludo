#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Check a board's CC answers against the protocol contract (src/control/contract, doc/specs/cc-protocol.md),
tier by tier, and print what the board can do: per tier PASS, NOT YET (a documented answer of a board that
lacks the feature -- `err unsupported`, `err badcmd` for push-*) or FAIL with the reason.

  tools/cc_contract.py fake                              # plan B's own dispatcher on CPython: the reference
  tools/cc_contract.py taster --tier core --tier stream  # a board through the running hub
  tools/cc_contract.py taster --serial /dev/ttyACM0      # a Rust board's console, no hub
  tools/cc_contract.py taster --diff fake                # and how its answers differ from plan B's
and the M1 proof on the taster, where every case M1 owes must pass (an M1 board drives no fin, so
--allow-active moves nothing there; on plan B it sweeps every fin):
  tools/cc_contract.py taster --milestone M1 --beats 30 --allow-active --allow-launch-write --allow-stage --allow-reboot

BOARD is `fake` or a board id: through the hub (POST /api/cmd, --hub) unless --serial names the console.
By default the run only reads, plus `disarm` and `probe mission` (a data check that moves nothing, inside a
log window). Everything that changes a board needs its flag, and nothing else is ever sent:
  --allow-active        arm, probe, verify, calibrate <device>, detect, bustune -- the probes sweep every fin
                        over its full travel
  --allow-stage         hold the stage at gliding (only a board seen disarmed: it then holds its fins
                        neutral), check the in-flight refusal, then return to setting
  --allow-launch-write  the launch-config round trip: the launch id is written and put back. NEVER on a plan
                        B board whose launch.config carries `sites`: plan B's save drops them and stores any
                        unsaved `update mission` field, and its get-config launch shows neither
  --allow-config-write  re-save the saved board config (a board that had none gets reset-config after)
  --allow-reboot        reboot the board and prove it came back from a new boot
The runner refuses a board that is airborne or armed, and with any flag one whose stage or arm state it
cannot read. A run that stops on an error or Ctrl-C undoes what it wrote (disarm, stage setting, the launch id,
reset-config) before it exits.

--milestone M<n> claims the board has reached that port-plan milestone (doc/specs/avionics-port.md): a not-yet
answer to a case owed by then is a violation. The hub tier (through the hub only) watches the board's dashboard
row for --beats heartbeats.

Exit status: 0 no contract violation (not-yet answers included), 1 a violation, 2 the run could not start or
did not finish (the board unreachable, refused, or the run aborted).
"""

import argparse
import json
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'control'))
from contract import cases, normalize, transports  # noqa: E402 -- the contract package lives in src/control

_VERDICTS: tuple = (cases.FAIL, cases.NOT_YET, cases.PASS, cases.SKIP)  # a tier's verdict: its worst outcome
_FAKE: str = 'fake'  # the BOARD (and --diff) name of plan B on CPython
_MILESTONE: re.Pattern = re.compile(r'[Mm](\d+)')  # a port-plan milestone as the plan names it: M1, M13


def _milestone(text: str) -> int:
    """
    The --milestone value: `M1` -> 1.

    Args:
        text - the option's text.

    Returns:
        The milestone number.

    Raises:
        argparse.ArgumentTypeError - the text is not M<number>.
    """
    match = _MILESTONE.fullmatch(text)
    if match is None:
        raise argparse.ArgumentTypeError('a milestone is M<number> (M1, M13), not %r' % text)
    return int(match.group(1))


def _transport(board: str, serial_port: str | None, hub_url: str) -> transports.Transport:
    """
    The transport BOARD and the options name.

    Args:
        board - `fake` or a board id.
        serial_port - the console port, or None for the hub.
        hub_url - the hub's web bridge.

    Returns:
        The open transport.

    Raises:
        transports.Unavailable - the hub, the board or the port cannot be reached.
    """
    if board == _FAKE:
        return transports.Fake()
    if serial_port is not None:
        return transports.Serial(serial_port)
    return transports.Hub(board, hub_url)


def _run(transport: transports.Transport, tiers: tuple, allowed: tuple, board_id: str | None,
         milestone: int | None, beats: int) -> list:
    """Run the cases (cases.run) and close the transport, whatever happens."""
    try:
        return cases.run(transport, tiers, allowed, board_id, milestone, beats)
    finally:
        transport.close()


def summary(results: list) -> dict:
    """
    Each tier's verdict and counts.

    Args:
        results - the cases.Result list.

    Returns:
        {tier: {verdict, pass, not yet, FAIL, skip}} in run order; the verdict is the worst outcome present
        (FAIL, then not yet, then pass), skip only when nothing ran.
    """
    tiers = {}
    for result in results:
        counts = tiers.setdefault(result.tier, {outcome: 0 for outcome in _VERDICTS})
        counts[result.outcome] += 1
    for counts in tiers.values():
        counts['verdict'] = next(outcome for outcome in _VERDICTS if counts[outcome] or outcome == cases.SKIP)
    return tiers


def violations(results: list) -> int:
    """The number of contract violations: FAIL results."""
    return sum(1 for result in results if result.outcome == cases.FAIL)


def _print_report(name: str, tiers: tuple, allowed: tuple, milestone: int | None, results: list) -> None:
    """The capability report: a line per tier, then a line per case that did not pass."""
    print('contract: %s -- tiers %s; allowed: %s; milestone: %s'
          % (name, ', '.join(tiers), ', '.join(allowed) if allowed else 'none',
             'none (not-yet answers are reported)' if milestone is None else 'M%d' % milestone))
    for tier, counts in summary(results).items():
        detail = ', '.join('%d %s' % (counts[outcome], outcome) for outcome in _VERDICTS if counts[outcome])
        print('  %-11s %-8s %s' % (tier, counts['verdict'].upper(), detail))
        for result in results:
            if result.tier == tier and result.outcome != cases.PASS:
                print('    %-8s %s: %s' % (result.outcome, result.name, result.reason))
    print('violations: %d' % violations(results))


def _print_diff(found: list, known: list) -> None:
    """The --diff section: the known differences, then each case whose masked answers differ, with its lines."""
    print('known differences (by design; not counted):')
    for line in known or ['none']:
        print('  %s' % line)
    print('diff against fake (masked: %s; known: %s; %s compared by shape; free text masked):'
          % (', '.join(sorted(normalize.VOLATILE)), ', '.join(sorted(normalize.KNOWN)),
             ', '.join(sorted(normalize.CONTENT))))
    for label, lines in found:
        print('  %s' % label)
        for line in lines:
            print('    %s' % line)
    print('%d case(s) differ' % len(found))


def _json_report(name: str, tiers: tuple, allowed: tuple, milestone: int | None, results: list,
                 found: list | None, known: list | None) -> dict:
    """The --json report: the same content as the printed one, machine-readable."""
    report = {
        'transport': name,
        'requested_tiers': list(tiers),
        'allowed': list(allowed),
        'milestone': milestone,
        'tiers': summary(results),
        'violations': violations(results),
        'cases': [{'tier': result.tier, 'name': result.name, 'request': result.request, 'outcome': result.outcome,
                   'reason': result.reason, 'replies': list(result.replies), 'note': result.note}
                  for result in results],
    }
    if found is not None:
        report['diff'] = [{'case': label, 'differences': lines} for label, lines in found]
        report['known'] = known
    return report


def main(arguments: list | None = None) -> int:
    """
    Run the contract against BOARD and report.

    Args:
        arguments - the command line (sys.argv[1:] when None).

    Returns:
        The exit status: 0 no violation, 1 a violation, 2 the run could not start or did not finish.
    """
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('board', metavar='BOARD', help='`fake` (plan B on CPython) or a board id')
    parser.add_argument('--tier', action='append', choices=cases.TIERS,
                        help='a tier to run (repeatable; default every tier)')
    parser.add_argument('--diff', choices=(_FAKE,), help='also run the cases on plan B and show what differs')
    parser.add_argument('--serial', metavar='PORT', help="the board's console port instead of the hub")
    parser.add_argument('--hub', default=transports.HUB_URL, help='the hub web bridge (default %(default)s)')
    for flag in cases.ALLOW_FLAGS:
        parser.add_argument('--allow-' + flag.replace('_', '-'), dest='allow_' + flag, action='store_true')
    parser.add_argument('--milestone', type=_milestone, metavar='M<n>',
                        help='the port-plan milestone the board claims: a not-yet answer it owes by then FAILs')
    parser.add_argument('--beats', type=int, default=cases.HUB_BEATS, metavar='N',
                        help='heartbeats the hub tier watches (default %(default)s; the M1 proof asks for 30)')
    parser.add_argument('--json', action='store_true', help='print a machine-readable report instead')
    options = parser.parse_args(arguments)
    if options.beats < 1:
        parser.error('--beats: the hub tier reads at least one heartbeat')

    tiers = tuple(tier for tier in cases.TIERS if tier in (options.tier or cases.TIERS))
    allowed = tuple(flag for flag in cases.ALLOW_FLAGS if getattr(options, 'allow_' + flag))
    board_id = None if options.board == _FAKE else options.board
    try:
        transport = _transport(options.board, options.serial, options.hub)
        name = transport.name
        results = _run(transport, tiers, allowed, board_id, options.milestone, options.beats)
        found = known = None
        if options.diff:
            reference = _run(transports.Fake(), tiers, allowed, None, options.milestone, options.beats)
            found = normalize.diff(results, reference)
            known = normalize.known(results, reference)
    except (transports.Unavailable, cases.Refused, cases.Aborted) as error:
        print('cc_contract: %s' % error, file=sys.stderr)
        return 2
    if options.json:
        print(json.dumps(_json_report(name, tiers, allowed, options.milestone, results, found, known), indent=1))
    else:
        _print_report(name, tiers, allowed, options.milestone, results)
        if found is not None:
            _print_diff(found, known)
    return 1 if violations(results) else 0


if __name__ == '__main__':
    raise SystemExit(main())
