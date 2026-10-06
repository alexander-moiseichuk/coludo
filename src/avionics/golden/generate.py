"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The golden-vector generator: runs plan B (src/glider, frozen at tag micropython_plan_B) on CPython through
shims.py and writes what it answers as the fixtures the Rust port (src/avionics) is tested against.

Every fixture has one builder, golden/build_<fixture>.py, exposing build() -> dict: the whole fixture
document, whose `meta` carries only `source` (the plan B files it reads). This driver adds the provenance --
tag, commit, glider_tree (git rev-parse HEAD:src/glider), python (major.minor), generator -- and writes
golden/<fixture>.json as json.dumps(indent=1, sort_keys=True). Builders are separate files, so a fixture is
added without touching this one.

The provenance has to be true, so the driver refuses to run (exit 2) unless:
  * the interpreter is CPython 3.12, the oracle's version: another minor may answer differently (repr,
    json, int parsing), and a patch level must not make every fixture stale, so only major.minor is kept;
  * src/glider has no uncommitted change: the builders read the working tree, and glider_tree and commit
    name HEAD;
  * src/glider outside test/ is what the tag micropython_plan_B froze, so `tag` names the plan B the
    fixture really came from. A plan B fix moves the tag (or _TAG) on purpose; until then nothing is
    generated from a plan B the tag does not describe.

Each builder runs in a FRESH interpreter. Plan B keeps registries in class state (the Inspector, the
Databoard, the Recorder, layout.RESOLVED), so a builder run after another would otherwise answer differently
-- `objects` would list whatever the earlier one registered -- and a fixture would depend on which others
were generated with it. PYTHONHASHSEED is pinned too, so no set iteration order can leak into a fixture.

`--check` regenerates in memory and diffs against the files on disk, ignoring meta.commit (it moves with
every commit; glider_tree moves only when src/glider does). A fixture whose only difference is meta.commit is
also left untouched when writing, so regenerating after an unrelated commit changes no file.

Run from anywhere:
    python3 src/avionics/golden/generate.py               # write every fixture
    python3 src/avionics/golden/generate.py cc_dispatch   # only the named ones
    python3 src/avionics/golden/generate.py --check       # exit 1 when any fixture on disk is stale
"""

import argparse
import contextlib
import difflib
import glob
import importlib
import json
import os
import subprocess
import sys

import shims

_HERE: str = os.path.dirname(os.path.abspath(__file__))
_TAG: str = 'micropython_plan_B_2'  # the git tag plan B is frozen at: the original plus the bus re-arm fix (76c138c)
_PYTHON: str = '3.12'  # the oracle's CPython major.minor (meta.python)
_GLIDER: str = ':(top)src/glider'  # plan B, as a git pathspec from the repository root
_GLIDER_TESTS: str = ':(top,exclude)src/glider/test'  # its tests and benches, which the tag does not pin
_BUILDER_PREFIX: str = 'build_'
_DIFF_LINES: int = 40  # a stale fixture's diff is cut here: enough to see what moved


def _git(*arguments: str) -> str:
    """
    One git query about this checkout.

    Args:
        arguments - the git arguments (run from this directory, so any checkout location works).

    Returns:
        git's output, stripped.

    Raises:
        subprocess.CalledProcessError - git failed (not a checkout, no HEAD).
    """
    return subprocess.run(['git', '-C', _HERE, *arguments], capture_output=True, text=True, check=True).stdout.strip()


def _fixtures() -> list:
    """
    Every fixture that has a builder.

    Args:
        (none)

    Returns:
        The fixture names (build_<name>.py), sorted.
    """
    pattern = os.path.join(_HERE, _BUILDER_PREFIX + '*.py')
    return sorted(os.path.basename(path)[len(_BUILDER_PREFIX) : -len('.py')] for path in glob.glob(pattern))


def _render(document: dict) -> str:
    """The fixture file text for a document: one key per line, sorted, so a diff shows what moved."""
    return json.dumps(document, indent=1, sort_keys=True) + '\n'


def _build(fixture: str) -> None:
    """
    Child mode: build one fixture with plan B and print its file text.

    Plan B's own prints (boot and layout lines) go to stderr, so stdout carries the fixture alone.

    Args:
        fixture - the fixture name.

    Returns:
        None; writes the rendered fixture to stdout.

    Raises:
        SystemExit - the builder's meta holds anything but `source`: the provenance is this driver's.
    """
    shims.install()
    with contextlib.redirect_stdout(sys.stderr):
        document = importlib.import_module(_BUILDER_PREFIX + fixture).build()
    meta = document.get('meta')
    if not isinstance(meta, dict) or set(meta) != {'source'}:
        raise SystemExit('%s%s.py: meta must hold exactly `source`, the generator writes the rest'
                         % (_BUILDER_PREFIX, fixture))
    meta.update(tag=_TAG, commit=_git('rev-parse', 'HEAD'), glider_tree=_git('rev-parse', 'HEAD:src/glider'),
                python=_PYTHON, generator=fixture)
    sys.stdout.write(_render(document))


def _generate(fixture: str) -> str:
    """
    Build one fixture in a fresh interpreter.

    Args:
        fixture - the fixture name.

    Returns:
        The fixture's file text.

    Raises:
        SystemExit - the builder failed or printed nothing; its stderr is in the message.
    """
    environment = dict(os.environ, PYTHONHASHSEED='0')
    result = subprocess.run([sys.executable, os.path.abspath(__file__), '--build', fixture],
                            capture_output=True, text=True, env=environment)
    if result.returncode != 0 or not result.stdout:
        raise SystemExit('%s: the builder failed (exit %d)\n%s' % (fixture, result.returncode, result.stderr))
    return result.stdout


def _stale(path: str, text: str) -> list:
    """
    How the fixture file at `path` differs from freshly generated `text`, meta.commit aside.

    Args:
        path - the fixture file on disk.
        text - the regenerated fixture text.

    Returns:
        The unified-diff lines (on disk -> regenerated), or a one-line reason when the file is missing or no
        fixture; [] when they match.
    """
    try:
        with open(path) as handle:
            disk = handle.read()
    except OSError:
        return ['%s: missing' % path]
    try:
        commit = json.loads(disk)['meta']['commit']
    except (ValueError, KeyError, TypeError):
        return ['%s: not a fixture (no meta.commit)' % path]
    fresh = json.loads(text)
    fresh['meta']['commit'] = commit
    return list(difflib.unified_diff(disk.splitlines(), _render(fresh).splitlines(), path + ' (on disk)',
                                     path + ' (regenerated)', lineterm='', n=1))


def _refusal() -> str:
    """
    Why the fixtures cannot be generated or checked here, or '' when they can (see the module docstring).

    Args:
        (none)

    Returns:
        A one-line reason: the wrong interpreter, uncommitted plan B changes, an unknown tag, or plan B
        moved past the tag (the files named); '' when the provenance would be true.
    """
    running = '%d.%d' % sys.version_info[:2]
    if running != _PYTHON:
        return 'CPython %s is the oracle, this is %s' % (_PYTHON, running)
    if _git('status', '--porcelain', '--', _GLIDER):
        return 'src/glider has uncommitted changes: the fixtures would claim a tree nobody can check out'
    try:
        _git('rev-parse', '--verify', '--quiet', _TAG + '^{commit}')
    except subprocess.CalledProcessError:
        return 'no tag %s in this checkout (fetch the tags)' % _TAG
    moved = _git('diff', '--name-only', _TAG, 'HEAD', '--', _GLIDER, _GLIDER_TESTS)
    if moved:
        return 'src/glider is not plan B as %s froze it (%s): move the tag on purpose first' % (
            _TAG, ', '.join(moved.split()))
    return ''


def main() -> int:
    """
    Write or check the fixtures named on the command line (all of them when none are).

    Args:
        (none -- reads sys.argv)

    Returns:
        The exit code: 0 done or in sync; 1 a fixture is stale (--check); 2 an unknown fixture name, no
        builders at all, or a provenance _refusal().
    """
    parser = argparse.ArgumentParser(description='Generate the golden fixtures from plan B (src/glider).')
    parser.add_argument('fixtures', nargs='*', help='fixture names (default: every build_<fixture>.py)')
    parser.add_argument('--check', action='store_true', help='diff against the files on disk; exit 1 if stale')
    parser.add_argument('--build', help=argparse.SUPPRESS)  # child mode, one fixture: see _generate()
    arguments = parser.parse_args()
    if arguments.build:
        _build(arguments.build)
        return 0
    refusal = _refusal()
    if refusal:
        print('error: %s' % refusal, file=sys.stderr)
        return 2
    known = _fixtures()
    unknown = [name for name in arguments.fixtures if name not in known]
    if unknown or not known:
        print('error: %s -- builders here: %s' % ('no fixture ' + ', '.join(unknown) if unknown else 'no builders',
                                                  ', '.join(known) or 'none'), file=sys.stderr)
        return 2
    selected = arguments.fixtures or known
    stale = 0
    for fixture in selected:
        path = os.path.join(_HERE, fixture + '.json')
        text = _generate(fixture)
        difference = _stale(path, text)
        if arguments.check:
            if difference:
                stale += 1
                print('STALE %s' % fixture)
                print('\n'.join(difference[:_DIFF_LINES]))
            else:
                print('ok    %s' % fixture)
        elif difference:
            with open(path, 'w') as handle:
                handle.write(text)
            print('wrote %s' % path)
        else:
            print('same  %s' % path)
    if stale:
        print('%d of %d fixtures stale -- run: python3 src/avionics/golden/generate.py' % (stale, len(selected)))
        return 1
    print('%s: %d of %d' % ('in sync' if arguments.check else 'generated', len(selected), len(known)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
