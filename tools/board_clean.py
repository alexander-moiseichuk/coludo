#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Clear a Rust board's card before a test run, over the hub (the SD component's `ls` and `rm`, doc/specs/cc-protocol.md):
every session's directory but the running boot's, at the card's root and under `videos/`, so each run of an A/B
starts from an empty card (the operator, 2026-10-11). What it removes is gone for good: pull first what is to be kept.

  tools/board_clean.py <board>    # the board's id on the hub; the board must be in SETTING

Kept: the running session (the board refuses it anyway), `videos/` itself (on a two-partition card the videos
partition's mount point), the global index `sessions.csv` and the entry file `entry.gcl`. tools/hitl_host.py --clean runs the same after its
reboot, so the run's own session is the one kept.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'control'))
from contract import transports  # noqa: E402

_VIDEOS: str = 'videos'
_KEPT: tuple = ('sessions.csv', 'entry.gcl')  # the global index, the entry file (M21)


def doomed(entries: list, session: str, under: str = '') -> list:
    """
    The paths `clean` removes from one listing: every entry but the running session, `videos/` and the global index.

    Args:
        entries - the `ls` reply's entries: [name, bytes] per file, [name/, None] per directory.
        session - the running session.
        under - the listed directory, '' for the card's root.

    Returns:
        The paths from the card's root.
    """
    paths = []
    for name, _size in entries:
        bare = name.rstrip('/')
        if bare == session or (not under and (bare.lower() == _VIDEOS or bare in _KEPT)):
            continue
        paths.append(under + '/' + bare if under else bare)
    return paths


def clean(board: object, session: str) -> list:
    """
    Remove every session but `session` from the board's card.

    Args:
        board - a hitl_host `_Board` (`ask` and `raw`) on the board.
        session - the running session, kept.

    Returns:
        The paths removed.

    Raises:
        SystemExit - a listing or a removal the board refused.
    """
    removed = []
    for under in ('', _VIDEOS):
        status, listing = board.ask('ls path=/%s' % under)
        if status != 'ok':
            if under:  # a card with no videos yet has no videos/
                continue
            raise SystemExit('ls refused: %s' % listing)
        for path in doomed(listing['entries'], session, under):
            reply = board.raw('rm confirm path=%s' % path)  # the word first: the hub builds named params last
            if not reply.startswith('ok'):
                raise SystemExit('rm %s: %s' % (path, reply))
            removed.append(path)
    return removed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('board', help="the board's id on the hub")
    args = parser.parse_args()
    import hitl_host  # here: it pulls in the sim, which only the CLI needs
    board = hitl_host._Board(transports.Hub(args.board))
    session = board.ask('whoami')[1].get('session')
    if not session:
        raise SystemExit('whoami named no session')
    removed = clean(board, session)
    print('removed %d: %s' % (len(removed), ' '.join(removed)))
    print('df: %s' % board.raw('df'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
