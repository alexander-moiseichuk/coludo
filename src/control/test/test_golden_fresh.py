"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test: the golden fixtures (src/avionics/golden/*.json) are IN SYNC with plan B (src/glider) --
`generate.py --check` must pass. The Rust port is tested against these files, so a plan B change that moves
an answer fails CI here until the fixtures are regenerated on purpose, instead of the port silently chasing a
stale oracle. The generator's own rules are checked both ways: the staleness rule (a moved value is caught, a
moved meta.commit, which changes with every commit, is not), the builder failures it must report rather than
write, and the provenance refusals (the wrong interpreter, an unknown tag, plan B moved past the tag). The
uncommitted-src/glider refusal is not exercised: that would mean editing the frozen tree.

The shims and the cc_dispatch builder's own guards run in a child interpreter: shims.install() is process-wide
and would leave this one on MicroPython's clock.
"""

import contextlib
import importlib.util
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile
import types

HERE: str = os.path.dirname(os.path.abspath(__file__))
ROOT: str = os.path.join(HERE, '..', '..', '..')
GOLDEN: str = os.path.join(ROOT, 'src', 'avionics', 'golden')
GENERATE: str = os.path.join(GOLDEN, 'generate.py')

sys.path.insert(0, GOLDEN)
import generate  # noqa: E402 -- the golden directory is not a package; it joins the path just above

"""Throwaway builders, one per failure generate.py must report: {fixture name: builder source}."""
BROKEN_BUILDERS: dict = {
    'raises': 'def build():\n    raise RuntimeError("boom in the builder")\n',
    'silent': 'import sys\n\n\ndef build():\n    sys.exit(0)\n',
    'extra_meta': 'def build():\n    return {"meta": {"source": "x", "tag": "mine"}, "cases": []}\n',
}


def test_fixtures_in_sync() -> None:
    """Every fixture on disk is what plan B generates today."""
    result = subprocess.run([sys.executable, GENERATE, '--check'], capture_output=True, text=True)
    assert result.returncode == 0, (
        'src/avionics/golden fixtures are STALE -- run: python3 src/avionics/golden/generate.py\n'
        + result.stdout + result.stderr)
    assert 'in sync' in result.stdout, result.stdout  # a run that checked nothing must not pass


def test_staleness_rule() -> None:
    """A changed value is stale; a changed meta.commit alone is not; a missing file or one with no meta is."""
    with open(os.path.join(GOLDEN, 'config_default.json')) as handle:
        text = handle.read()
    document = json.loads(text)
    with tempfile.TemporaryDirectory() as scratch:
        path = os.path.join(scratch, 'config_default.json')
        document['meta']['commit'] = '0' * 40
        with open(path, 'w') as handle:
            handle.write(generate._render(document))
        assert generate._stale(path, text) == [], 'meta.commit alone must not make a fixture stale'
        document['config_id'] = '000000000000'
        with open(path, 'w') as handle:
            handle.write(generate._render(document))
        assert any('000000000000' in line for line in generate._stale(path, text)), 'a moved value must be stale'
        assert generate._stale(os.path.join(scratch, 'absent.json'), text)[0].endswith('missing')
        del document['meta']
        with open(path, 'w') as handle:
            handle.write(generate._render(document))
        assert generate._stale(path, text) == ['%s: not a fixture (no meta.commit)' % path]
        with open(path, 'w') as handle:
            handle.write('not json')
        assert generate._stale(path, text) == ['%s: not a fixture (no meta.commit)' % path]


def _copy_with_builders(scratch: str) -> types.ModuleType:
    """
    A copy of generate.py and shims.py in `scratch`, next to the BROKEN_BUILDERS, imported as a module.

    The copy's children run the copy, so they find the throwaway builders and never touch the real fixtures.

    Args:
        scratch - an empty directory.

    Returns:
        The imported copy of generate.py.
    """
    for name in ('generate.py', 'shims.py'):
        shutil.copy(os.path.join(GOLDEN, name), scratch)
    for fixture, source in BROKEN_BUILDERS.items():
        with open(os.path.join(scratch, 'build_%s.py' % fixture), 'w') as handle:
            handle.write(source)
    specification = importlib.util.spec_from_file_location('generate_copy', os.path.join(scratch, 'generate.py'))
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _failure(copy: types.ModuleType, fixture: str) -> str:
    """
    The message _generate() stops with for a builder that must fail.

    Args:
        copy - the generate.py copy holding the builder.
        fixture - the builder's fixture name.

    Returns:
        The SystemExit message.
    """
    try:
        copy._generate(fixture)
    except SystemExit as stop:
        return str(stop.code)
    raise AssertionError('%s: the builder failure went unreported' % fixture)


def test_builder_failures_are_reported() -> None:
    """A builder that raises, prints nothing, or writes provenance of its own stops the run with its reason."""
    with tempfile.TemporaryDirectory() as scratch:
        copy = _copy_with_builders(scratch)
        assert copy._fixtures() == sorted(BROKEN_BUILDERS), copy._fixtures()
        message = _failure(copy, 'raises')
        assert message.startswith('raises: the builder failed (exit 1)'), message
        assert 'RuntimeError: boom in the builder' in message, message
        message = _failure(copy, 'silent')
        assert message.startswith('silent: the builder failed (exit 0)'), message
        message = _failure(copy, 'extra_meta')
        assert message.startswith('extra_meta: the builder failed (exit 1)'), message
        assert 'build_extra_meta.py: meta must hold exactly `source`' in message, message


def _refusal_with(**settings: str) -> str:
    """
    generate._refusal() with some of its module settings replaced for the call.

    Args:
        settings - generate module attributes (_PYTHON, _TAG) and the values to try.

    Returns:
        The refusal reason; '' when none.
    """
    saved = {name: getattr(generate, name) for name in settings}
    try:
        for name, value in settings.items():
            setattr(generate, name, value)
        return generate._refusal()
    finally:
        for name, value in saved.items():
            setattr(generate, name, value)


def test_provenance_refusals() -> None:
    """The real checkout passes; another interpreter, an unknown tag, or a tag plan B has moved past is refused."""
    assert _refusal_with() == '', _refusal_with()
    assert _refusal_with(_PYTHON='3.99') == 'CPython 3.99 is the oracle, this is %d.%d' % sys.version_info[:2]
    assert _refusal_with(_TAG='no_such_tag') == 'no tag no_such_tag in this checkout (fetch the tags)'
    first = subprocess.run(['git', '-C', GOLDEN, 'rev-list', '--max-parents=0', 'HEAD'], capture_output=True,
                           text=True, check=True).stdout.split()[0]
    reason = _refusal_with(_TAG=first)
    assert reason.startswith('src/glider is not plan B as %s froze it (src/glider/' % first), reason
    assert 'src/glider/test/' not in reason, 'the tag does not pin the tests: ' + reason


def test_refusal_stops_main() -> None:
    """A refused run exits 2 and generates nothing, in --check mode too."""
    saved = (generate._PYTHON, sys.argv)
    errors = io.StringIO()
    try:
        generate._PYTHON = '3.99'
        sys.argv = ['generate.py', '--check']
        with contextlib.redirect_stderr(errors):
            assert generate.main() == 2
    finally:
        generate._PYTHON, sys.argv = saved
    assert errors.getvalue().startswith('error: CPython 3.99 is the oracle'), errors.getvalue()


"""The shims' MicroPython semantics and refusals, checked in a child interpreter (see the module docstring)."""
SHIMS_CHECK: str = """
import sys, time
import shims
sys.modules['server'] = None
try:
    shims.install()
    raise AssertionError('install() ran beside the hub')
except RuntimeError as error:
    assert 'run plan B in a subprocess' in str(error), error
del sys.modules['server']
shims.install()
import esp32, machine
assert isinstance(esp32.mcu_temperature(), float)
assert time.ticks_ms() == 10000 and time.ticks_us() == 10000000, 'the board starts 10 s after boot'
assert time.time() == 10, 'a cold board counts from 2000'
assert time.ticks_add(5, (1 << 29) - 1) == (1 << 29) + 4
for delta in (1 << 29, -(1 << 29), 10 ** 20):
    try:
        time.ticks_add(0, delta)
        raise AssertionError('ticks_add took %d' % delta)
    except OverflowError as error:
        assert str(error) == 'ticks interval overflow', error
assert time.ticks_diff(5, (1 << 30) - 5) == 10, 'a diff across the wrap'
time.sleep_ms(250)
shims.advance(750)
assert time.ticks_ms() == 11000 and time.time() == 11
try:
    shims.advance(-1)
    raise AssertionError('the clock went back')
except ValueError:
    pass
machine.RTC().datetime((2026, 10, 6, 1, 12, 30, 15, 0))
assert time.time() == 1791289815 - 946684800 and time.localtime()[:6] == (2026, 10, 6, 12, 30, 15)
shims.advance(2000)
assert time.time() == 1791289817 - 946684800, 'the set clock runs on'
try:
    machine.Pin(5)
    raise AssertionError('a Pin was built on the host')
except OSError:
    pass
try:
    from machine import I2C  # noqa: F401
    raise AssertionError('machine.I2C exists on the host')
except ImportError:
    pass
print('shims ok')
"""

"""The cc_dispatch builder refuses declarations and replies it cannot record, checked in a child interpreter."""
BUILDER_CHECK: str = """
import shims
shims.install()
import build_cc_dispatch as builder
assert builder._json_keys('{"temp": null}', {'temp': 'float|null'}) == {'temp': 'float|null'}
assert builder._json_keys('taster', {}) == 'taster'
for argument, types, why in (('{"temp": 41.5}', {'temp': 'int|null'}, 'plan B sent float'),
                             ('{"temp": 41.5}', {'agl': 'float|null'}, 'keys plan B did not send')):
    try:
        builder._json_keys(argument, types)
        raise AssertionError(why)
    except ValueError as error:
        assert why in str(error), error
for case, reply, why in (
        ({'name': 'a', 'request': 'x', 'arguments_kind': 'roughly'}, 'ok', 'unknown arguments_kind'),
        ({'name': 'a', 'request': 'x', 'arguments_kind': 'json_keys', 'types': {'t': 'decimal'}}, 'ok', 'unknown type'),
        ({'name': 'a', 'request': 'x', 'arguments_kind': 'any'}, 'err badargs', 'has 1 arguments'),
        ({'name': 'a', 'request': 'x', 'arguments_kind': 'exact'}, 'ok key=value', 'named parameters'),
        ({'name': 'a', 'request': 'x', 'arguments_kind': 'exact',
          'owed': {'status': 'ok', 'arguments_kind': 'nearly', 'expected': []}}, 'ok', 'unknown owed')):
    try:
        builder._record(case, reply)
        raise AssertionError(why)
    except ValueError as error:
        assert why in str(error), error
assert builder._record({'name': 'a', 'request': 'x', 'arguments_kind': 'any'}, 'err badargs text')['expected'] == [
    'badargs']
print('builder ok')
"""


def _child(source: str) -> str:
    """
    Run `source` in a fresh interpreter beside the golden scripts.

    Args:
        source - the Python program.

    Returns:
        Its stdout.
    """
    result = subprocess.run([sys.executable, '-c', source], capture_output=True, text=True, cwd=GOLDEN,
                            env=dict(os.environ, PYTHONPATH=GOLDEN))
    assert result.returncode == 0, result.stdout + result.stderr
    return result.stdout


def test_shims() -> None:
    """The virtual clock, ticks_add's range, the RTC, the stubs, and the refusal to share a process with the hub."""
    assert _child(SHIMS_CHECK).endswith('shims ok\n')


def test_builder_guards() -> None:
    """The cc_dispatch builder records what it can pin and refuses the rest."""
    assert _child(BUILDER_CHECK).endswith('builder ok\n')


def test_unknown_fixture_refused() -> None:
    """Naming a fixture with no builder is an error, not an empty success."""
    result = subprocess.run([sys.executable, GENERATE, '--check', 'nosuch'], capture_output=True, text=True)
    assert result.returncode == 2, (result.returncode, result.stdout, result.stderr)
    assert 'no fixture nosuch' in result.stderr, result.stderr


test_fixtures_in_sync()
test_staleness_rule()
test_builder_failures_are_reported()
test_provenance_refusals()
test_refusal_stops_main()
test_shims()
test_builder_guards()
test_unknown_fixture_refused()
print('ok: golden generate --check -- the fixtures match plan B; staleness, builder failures, provenance '
      'refusals, the shims and the builder guards hold')
