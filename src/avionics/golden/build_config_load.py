"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture config_load.json: plan B's config.load() -- which document a boot runs, the source text whoami
reports and the messages -- over stored texts and defaults: nothing stored, text that is not JSON, documents
validate() refuses, a document of the firmware's schema version, older and unversioned ones (the source text
exact), and the firmware_version restamp.

Members:
  * bases -- the generated documents the cases name (config_corpus.py);
  * cases -- {name, input: {stored, defaults}, output: {document, source, errors}}. `stored` is the board.config
    text (null for no file), or {"file": path}: a committed board file's exact bytes, which the Rust test compiles
    in. `defaults` names the firmware's default document as config_corpus.py describes.

plan B reads board.config from a file; the builder writes each stored text to a scratch directory and loads it
from there (no board.config lying in the working directory is read). Texts plan B parses but serde_json does not
(NaN, Infinity, a lone surrogate) are left to the Rust unit tests: the crate documents that difference.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import json
import os
import tempfile

import config as config_module
import config_corpus

_BASE_NAMES: list = ['default']  # the generated documents this fixture's cases may name (config_corpus.BASES)

"""Small defaults: the running firmware is schema 20261005, build 2026.10.05.running."""
_RUNNING: dict = {'version': '20261005', 'buses': {}, 'pins': {},
                  'board': {'id': 'taster', 'mcu': 'esp32p4', 'firmware_version': '2026.10.05.running'}}

"""A stored document of the running schema, saved by an older build."""
_SAVED: dict = {'version': '20261005', 'buses': {'i2c': {'0': {'scl': 8, 'sda': 7}}}, 'pins': {'servo_yaw': 26},
                'board': {'id': 'restamp-board', 'mcu': 'esp32p4', 'firmware_version': '2026.08.31.whenSaved',
                          'setup_retries': 7}}


def _edited(document: dict, *edits: dict) -> dict:
    """A copy of `document` with config_corpus edits applied ({path, value} sets, {path, delete} deletes)."""
    return config_corpus.resolve({'document': document, 'edits': list(edits)}, {})


def _set(path: list, value: object) -> dict:
    """An edit setting the member at `path`."""
    return {'path': path, 'value': value}


def _delete(path: list) -> dict:
    """An edit deleting the member at `path`."""
    return {'path': path, 'delete': True}


def _text(document: object) -> str:
    """A stored text for `document`, keys sorted, as the tools and the Rust writer store one."""
    return json.dumps(document, indent=1, sort_keys=True) + '\n'


def _cases() -> list:
    """The load cases: (name, stored text or {file} or None, defaults specification)."""
    running = {'document': _RUNNING}
    unversioned = {'document': _edited(_RUNNING, _delete(['version']))}
    unstamped = {'document': _edited(_RUNNING, _delete(['board', 'firmware_version']))}
    stamped_null = {'document': _edited(_RUNNING, _set(['board', 'firmware_version'], None))}
    taster = config_corpus.resolve({'file': config_corpus.TMS7F_FILE,
                                    'edits': [{'path': ['board', 'id'], 'value': 'taster'}]}, {})
    return [
        ('nothing_stored', None, running),
        ('nothing_stored_invalid_defaults', None, {'document': {}}),
        ('nothing_stored_firmware_default', None, {'base': 'default'}),
        ('empty_text', '', running),
        ('corrupt_text', '{ not json ', running),
        ('trailing_garbage', '{} x', running),
        ('trailing_comma', '{"version": "20261005",}', running),
        ('byte_order_mark', chr(0xFEFF) + json.dumps(_SAVED), running),
        ('json_null', 'null', running),
        ('json_list', '[1, 2]', running),
        ('json_text', '"board"', running),
        ('json_empty_object', '{}', running),
        ('invalid_pin_collision', _text(_edited(_SAVED, _set(['pins', 'servo_yaw'], 7))), running),
        ('valid_current', _text(_SAVED), running),
        ('valid_current_compact', json.dumps(_SAVED), running),
        ('valid_current_padded', '\n  ' + json.dumps(_SAVED) + '  \n\n', running),
        ('valid_older', _text(_edited(_SAVED, _set(['version'], '19700101'))), running),
        ('valid_unversioned', _text(_edited(_SAVED, _delete(['version']))), running),
        ('valid_version_empty', _text(_edited(_SAVED, _set(['version'], ''))), running),
        ('valid_version_null', _text(_edited(_SAVED, _set(['version'], None))), running),
        ('valid_version_empty_list', _text(_edited(_SAVED, _set(['version'], []))), running),
        ('valid_version_zero', _text(_edited(_SAVED, _set(['version'], 0))), running),
        ('valid_version_an_int', _text(_edited(_SAVED, _set(['version'], 20261005))), running),
        ('valid_newer', _text(_edited(_SAVED, _set(['version'], '20991231'))), running),
        ('unversioned_defaults_unversioned_config', _text(_edited(_SAVED, _delete(['version']))), unversioned),
        ('unversioned_defaults_versioned_config', _text(_SAVED), unversioned),
        ('stored_without_firmware_version', _text(_edited(_SAVED, _delete(['board', 'firmware_version']))), running),
        ('defaults_without_firmware_version', _text(_SAVED), unstamped),
        ('defaults_firmware_version_null', _text(_SAVED), stamped_null),
        ('defaults_without_board', _text(_SAVED), {'document': {'version': '20261005'}}),
        ('defaults_board_not_an_object', _text(_SAVED), {'document': {'version': '20261005', 'board': 'x'}}),
        ('duplicate_keys_last_wins', '{"version": "19700101", "version": "20261005", "buses": {}, "pins": {},'
                                     ' "board": {"id": "a", "id": "dup", "mcu": "esp32p4"}}', running),
        ('tms7f_file_on_the_firmware_default', {'file': config_corpus.TMS7F_FILE}, {'base': 'default'}),
        ('taster_copy_on_the_firmware_default', _text(taster), {'base': 'default'}),
    ]


def _load(stored: object, defaults: dict) -> dict:
    """
    plan B's load() of one stored text.

    Args:
        stored - the board.config text, {'file': path} for a committed file's bytes, or None for no file.
        defaults - the firmware default document (load() may return it, so the caller passes a copy).

    Returns:
        {document, source, errors}.
    """
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, 'board.config')
        if isinstance(stored, dict):
            stored = config_corpus.read_text(stored['file'])
        if stored is not None:
            with open(path, 'w', encoding='utf-8', newline='') as handle:
                handle.write(stored)
        document, source, errors = config_module.load(path, defaults=defaults)
    return {'document': document, 'source': source, 'errors': errors}


def build() -> dict:
    """
    The config_load fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, bases, cases}.
    """
    known = config_corpus.bases(_BASE_NAMES)
    cases = []
    for name, stored, defaults in _cases():
        output = _load(stored, config_corpus.resolve(defaults, known))
        cases.append({'name': name, 'input': {'stored': stored, 'defaults': defaults}, 'output': output})
    return {
        'meta': {'source': 'src/glider/config.py (load, outdated, schema_version, validate); ' + config_corpus.SOURCES},
        'compare': {'kind': 'exact'},
        'bases': known,
        'cases': cases,
    }
