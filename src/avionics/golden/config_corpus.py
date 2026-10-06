"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The board-config corpus the config_* builders share (config_id, config_validate, config_load, config_wifi), and
how a fixture names a document of it.

A fixture input names its document by one of three forms, plus optional edits:
  {"file": "<repository path>"}   a committed board file, read as it is (the 5 launches/20261003/*/*.config);
                                  the Rust test compiles the same files in (include_str!), so the fixture does
                                  not copy 10 KB per file;
  {"base": "<name>"}              a document plan B (or a tool over it) builds, written once into the
                                  fixture's `bases` member: see BASES;
  {"document": <value>}           the document itself, for small ones.
  "edits": [{"path": [...], "value": v} | {"path": [...], "delete": true}, ...]
                                  applied in order: set (or append, at an index one past a list's end) or delete
                                  the member at `path` (object keys and list indices).

The edits are plain data moves, so applying them on the Rust side derives nothing the port computes. After the
edits a document's keys are SORTED (a JSON round trip with sort_keys) before plan B sees it: plan B walks a dict
in insertion order and the Rust port (serde_json's default map) in sorted order, so a sorted document is the one
both read alike. Every file plan B and the tools write is sorted already.

The corpus reads two inputs outside src/glider, which generate.py's provenance guard does not cover:
tools/launch_config.py (the flight_ready_* bases) and the launches/20261003 board files. A fixture stamps them with
tag micropython_plan_B and commit HEAD all the same, so importing this module refuses (RuntimeError) while either
has an uncommitted or untracked change, or differs from what the tag holds: the fixture would name inputs nobody
can check out at the tag, and --check would then blame the committed fixture.

Imported by the builders, which generate.py runs with the shims installed: the imports below need src/glider on
sys.path.
"""

import copy
import glob
import importlib.util
import json
import os
import subprocess

import config_default
import config_hitl

_ROOT: str = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))

_TAG: str = 'micropython_plan_B'  # the tag a fixture names (generate.py's _TAG): the inputs must be as it holds them

"""The corpus inputs outside src/glider, as repository paths (see the module docstring)."""
_INPUTS: tuple = ('tools/launch_config.py', 'launches/20261003')


def _git(*arguments: str) -> str:
    """One git query about this checkout, run from the repository root: its output, stripped."""
    return subprocess.run(['git', '-C', _ROOT, *arguments], capture_output=True, text=True, check=True).stdout.strip()


def _refuse_unfrozen_inputs() -> None:
    """
    Refuse a corpus whose inputs outside src/glider are not what the fixture's provenance names.

    Args:
        (none)

    Returns:
        None when every input is committed and the same as at the tag.

    Raises:
        RuntimeError - an input has an uncommitted or untracked change, or HEAD moved it past the tag.
    """
    changed = _git('status', '--porcelain', '--untracked-files=all', '--', *_INPUTS)
    if changed:
        raise RuntimeError('corpus inputs have uncommitted changes, so the fixture would name a tree nobody can '
                           'check out: %s' % ', '.join(line.split(None, 1)[1] for line in changed.splitlines()))
    moved = _git('diff', '--name-only', _TAG, 'HEAD', '--', *_INPUTS)
    if moved:
        raise RuntimeError('corpus inputs are not as %s holds them (%s): move the tag on purpose first'
                           % (_TAG, ', '.join(moved.split())))


_refuse_unfrozen_inputs()

"""The committed board files, repository paths: every launches/20261003/*/*.config."""
LAUNCH_FILES: tuple = tuple(sorted(os.path.relpath(path, _ROOT) for path in
                                   glob.glob(os.path.join(_ROOT, 'launches', '20261003', '*', '*.config'))))

"""The 7F board file, which the taster copy and the load proof start from."""
TMS7F_FILE: str = 'launches/20261003/TMS-7F/tms7f.config'

"""The tool that makes a flight-ready config from an airframe's own (tools/launch_config.py --base)."""
_LAUNCH_CONFIG_TOOL: str = os.path.join(_ROOT, 'tools', 'launch_config.py')

"""The plan B sources and tools the corpus reads, for a fixture's meta.source."""
SOURCES: str = ('src/glider/config.py, src/glider/config_default.py, src/glider/config_hitl.py, '
                'tools/launch_config.py, launches/20261003/*/*.config')


def sorted_keys(value: object) -> object:
    """A JSON value with every object's keys sorted, as the Rust port's map holds it (a JSON round trip)."""
    return json.loads(json.dumps(value, sort_keys=True))


def read_file(path: str) -> dict:
    """
    A committed board file's document.

    Args:
        path - the repository path, one of LAUNCH_FILES.

    Returns:
        The parsed document.

    Raises:
        ValueError - the path is not one of LAUNCH_FILES (the Rust test compiles in only those).
    """
    if path not in LAUNCH_FILES:
        raise ValueError('%s is not a corpus file: %s' % (path, ', '.join(LAUNCH_FILES)))
    with open(os.path.join(_ROOT, path), encoding='utf-8') as handle:
        return json.load(handle)


def read_text(path: str) -> str:
    """A committed board file's exact text (the Rust test compiles in the same bytes)."""
    read_file(path)  # refuses a path outside the corpus
    with open(os.path.join(_ROOT, path), encoding='utf-8', newline='') as handle:
        return handle.read()


def _launch_config() -> object:
    """tools/launch_config.py as a module (it is a script, not on sys.path)."""
    specification = importlib.util.spec_from_file_location('launch_config', _LAUNCH_CONFIG_TOOL)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def _flight_ready_new() -> dict:
    """tools/launch_config.py --board-id TMS-7G: a new airframe's flight-ready config, from config_default."""
    document = config_default.default()
    document['board']['id'] = 'TMS-7G'
    return _launch_config().flight_ready(document)


"""
The generated documents a fixture may name as a base: name -> how plan B (or a tool over it) builds it. The
default's firmware_version is the frozen 'dev' (the shims block version.py).
"""
BASES: dict = {
    'default': config_default.default,
    'hitl': config_hitl.default,
    'hitl_e16_noisy': lambda: config_hitl.default('E16', noise=0.5, spike=True, wind=3.5, wind_dir=270.0,
                                                  inject_hz=10, gnss_drift=0.3, gnss_drift_dir=45.0,
                                                  pad_dwell_s=30.0),
    'flight_ready_tms7f': lambda: _launch_config().flight_ready(read_file(TMS7F_FILE)),
    'flight_ready_new': _flight_ready_new,
}


def bases(names: list) -> dict:
    """
    The fixture's `bases` member: the named generated documents, keys sorted.

    Args:
        names - names from BASES.

    Returns:
        {name: document}.
    """
    return {name: sorted_keys(BASES[name]()) for name in names}


def _set(document: object, path: list, value: object) -> None:
    """Set (or, at a list's end, append) the member at `path`."""
    parent = document
    for key in path[:-1]:
        parent = parent[key]
    last = path[-1]
    if isinstance(parent, list) and last == len(parent):
        parent.append(value)
    else:
        parent[last] = value


def _delete(document: object, path: list) -> None:
    """Delete the member at `path`."""
    parent = document
    for key in path[:-1]:
        parent = parent[key]
    del parent[path[-1]]


def resolve(specification: dict, known: dict) -> object:
    """
    The document a fixture input names, edited and with sorted keys: what plan B is shown.

    Args:
        specification - {file | base | document, edits?} (see the module docstring).
        known - the fixture's bases.

    Returns:
        The document.

    Raises:
        ValueError - the specification names no document, or two.
    """
    forms = [form for form in ('file', 'base', 'document') if form in specification]
    if len(forms) != 1:
        raise ValueError('a document is named by exactly one of file, base, document: %r' % specification)
    if 'file' in specification:
        document = read_file(specification['file'])
    elif 'base' in specification:
        document = copy.deepcopy(known[specification['base']])
    else:
        document = copy.deepcopy(specification['document'])
    for edit in specification.get('edits', []):
        if edit.get('delete'):
            _delete(document, edit['path'])
        else:
            _set(document, edit['path'], edit['value'])
    return sorted_keys(document)


def index_of(document: dict, section: str, name: str) -> int:
    """
    Where the device `name` sits in `section` ('sensors' / 'components'), so an edit can address it.

    Raises:
        ValueError - no such device.
    """
    for index, device in enumerate(document[section]):
        if isinstance(device, dict) and device.get('name') == name:
            return index
    raise ValueError('no %s device %s' % (section, name))
