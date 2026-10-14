"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture on_backup.json: plan B's answer to "is this channel flying on a backup?", cc_client._on_backup()
(src/glider/cc_client.py:225-260), the health flag `attitude-backup`. Asked over the board-config corpus -- the five
launches/20261003 board files, config_default and config_hitl -- and over copies of config_default with the attitude
providers fitted, disabled, re-ranked and re-shaped, plus the documents plan B cannot read as a config.

Members:
  * bases -- the generated documents the cases name (config_corpus.BASES);
  * cases -- {name, config, sources, on_backup}. `config` names a document as config_corpus.py describes. `sources`
    lists every fused winner asked about: each device name in the document, enabled or not, then `hitl` (the HITL
    sim's source, which no config declares) and null (nothing fresh). `on_backup` maps each channel asked about --
    every quantity a device of the document provides, `attitude`, and `nothing`, which nobody provides -- to the
    sources plan B answered True for, in the order of `sources`. Every other (channel, source) pair answered False.

plan B reads the config through the client context's controller (`ctx.controller.config`); the builder hands it a
context holding a controller holding the document, as the board's does.

Two kinds of document stay out, refused by the builder: a device that provides something with no name (plan B's walk
counts it among the providers; the port's providers are the bring-up's, which never brings it up) and a priority that
is no int within i32 (the port ranks it as its databoard registration does: a float as 0, a huge int saturated).
validate() refuses both; the Rust unit tests pin the port's answers there.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import types

import cc_client
import config_corpus

"""The generated documents this fixture's cases may name as a base (config_corpus.BASES)."""
_BASE_NAMES: list = ['default', 'hitl']

"""The HITL sim's source name: tasks/hitl.py provides its quantities under its component's name, outside `provides`."""
_SIM: str = 'hitl'

"""A channel no device provides."""
_NOBODY: str = 'nothing'

"""The device lists plan B walks, in its order."""
_SECTIONS: tuple = ('sensors', 'components')

"""The bounds of the port's rank, an i32."""
_RANK_LOW: int = -(2**31)
_RANK_HIGH: int = 2**31 - 1


def _set(path: list, value: object) -> dict:
    """An edit setting the member at `path`."""
    return {'path': path, 'value': value}


def _delete(path: list) -> dict:
    """An edit deleting the member at `path`."""
    return {'path': path, 'delete': True}


def _device(document: dict, name: str) -> list:
    """
    The edit path of the device `name` in `document`, in whichever list holds it.

    Raises:
        ValueError - no such device in either list.
    """
    for section in _SECTIONS:
        for index, device in enumerate(document.get(section, [])):
            if isinstance(device, dict) and device.get('name') == name:
                return [section, index]
    raise ValueError('no device %s' % name)


def _corpus() -> list:
    """Every corpus document as it is: (name, config)."""
    cases = [(path.rsplit('/', 1)[1][: -len('.config')], {'file': path}) for path in config_corpus.LAUNCH_FILES]
    return cases + [(name, {'base': name}) for name in _BASE_NAMES]


def _attitude(default: dict) -> list:
    """
    The attitude providers of config_default edited: the SEN0253's BNO055 fitted (as the layout fits it), the filter
    disabled or gone, the two re-ranked, their `enabled`, `priority` and `provides` in each shape plan B reads.

    Args:
        default - the default base.

    Returns:
        [(name, config)].
    """
    backup = _device(default, 'imu_bno055')
    primary = _device(default, 'attitude')
    fitted = _set(backup + ['enabled'], True)
    backup_rank = backup + ['provides', 'attitude', 'priority']
    primary_rank = primary + ['provides', 'attitude', 'priority']
    """A second, enabled `imu_bno055` above the filter, appended after the first: plan B's walk takes its rank."""
    twin = dict(default[backup[0]][backup[1]], enabled=True, provides={'attitude': {'priority': 0, 'timeout_ms': 40}})

    def default_with(*edits: dict) -> dict:
        """The default base with these edits."""
        return {'base': 'default', 'edits': list(edits)}

    cases = [
        ('backup_fitted', default_with(fitted)),
        ('backup_fitted_filter_disabled', default_with(fitted, _set(primary + ['enabled'], False))),
        ('backup_fitted_filter_absent', default_with(fitted, _delete(primary))),
        ('backup_outranks_the_filter', default_with(fitted, _set(backup_rank, 0))),
        ('backup_ties_the_filter', default_with(fitted, _set(backup_rank, 1))),
        ('filter_priority_absent', default_with(fitted, _delete(primary_rank))),
        ('filter_priority_true', default_with(fitted, _set(primary_rank, True))),
        ('filter_priority_false', default_with(fitted, _set(primary_rank, False))),
        ('filter_priority_negative', default_with(fitted, _set(primary_rank, -3))),
        ('filter_priority_below_the_backup', default_with(fitted, _set(primary_rank, 7))),
        ('filter_priority_at_the_rank_ceiling', default_with(fitted, _set(primary_rank, _RANK_HIGH),
                                                             _set(backup_rank, _RANK_LOW))),
        ('filter_provides_null', default_with(fitted, _set(primary + ['provides'], None))),
        ('filter_provides_empty', default_with(fitted, _set(primary + ['provides'], {}))),
        ('filter_attitude_not_an_object', default_with(fitted, _set(primary + ['provides', 'attitude'], 1))),
        ('filter_attitude_empty', default_with(fitted, _set(primary + ['provides', 'attitude'], {}))),
        ('backup_named_twice', default_with(fitted, _set(['sensors', len(default['sensors'])], twin))),
        ('sensors_absent', default_with(fitted, _delete(['sensors']))),
        ('components_absent', default_with(fitted, _delete(['components']))),
    ]
    enabled = backup + ['enabled']
    for label, value in [('zero', 0), ('one', 1), ('null', None), ('empty_text', ''), ('text', 'no'),
                         ('empty_list', []), ('list', [0]), ('empty_object', {}), ('float_zero', 0.0),
                         ('float', 0.5)]:
        cases.append(('backup_enabled_%s' % label, default_with(_set(enabled, value))))
    cases.append(('backup_enabled_absent', default_with(_delete(enabled))))
    return cases


def _documents() -> list:
    """Small documents: one with no device lists, and ones plan B cannot read as a config at all: (name, config)."""
    return [
        ('no_devices', {'document': {'board': {'id': 'taster'}}}),
        ('config_null', {'document': None}),
        ('config_a_list', {'document': [{'name': 'attitude', 'provides': {'attitude': {'priority': 1}}}]}),
        ('config_text', {'document': 'board.config'}),
    ]


def _devices(document: object) -> list:
    """Every device entry of `document` that is an object, in plan B's walk order; none when it is no config."""
    if not isinstance(document, dict):
        return []
    return [device for section in _SECTIONS for device in document.get(section, []) if isinstance(device, dict)]


def _refuse_divergent(name: str, document: object) -> None:
    """
    Refuse a document plan B and the port answer differently by design (see the module notes).

    Args:
        name - the case name, for the message.
        document - the case's document as plan B sees it.

    Returns:
        None when every providing device has a name and every priority is an int within i32.

    Raises:
        ValueError - a providing device has no name, or a priority is no int within i32.
    """
    for device in _devices(document):
        provides = device.get('provides') or {}
        named = isinstance(device.get('name'), str) and device.get('name') != ''
        if provides and not named:
            raise ValueError('case %s: a device provides %s with no name' % (name, sorted(provides)))
        for quantity, specification in provides.items():
            if not isinstance(specification, dict) or 'priority' not in specification:
                continue
            priority = specification['priority']
            if not isinstance(priority, int) or not _RANK_LOW <= priority <= _RANK_HIGH:
                raise ValueError('case %s: %s.provides.%s.priority %r is no int within i32'
                                 % (name, device.get('name'), quantity, priority))


def _answers(name: str, config: dict, known: dict) -> dict:
    """
    Plan B's answers for one case.

    Args:
        name - the case name.
        config - the case's document, named as config_corpus.py describes.
        known - the fixture's bases.

    Returns:
        The case {name, config, sources, on_backup}.

    Raises:
        ValueError - the document is one the fixture refuses (see _refuse_divergent).
    """
    document = config_corpus.resolve(config, known)
    _refuse_divergent(name, document)
    names = sorted({device['name'] for device in _devices(document) if isinstance(device.get('name'), str)})
    sources = names + ([] if _SIM in names else [_SIM]) + [None]  # config_hitl names its sim task `hitl` too
    provided = {quantity for device in _devices(document) for quantity in (device.get('provides') or {})}
    channels = sorted(provided | {'attitude', _NOBODY})
    context = types.SimpleNamespace(controller=types.SimpleNamespace(config=document))
    return {
        'name': name,
        'config': config,
        'sources': sources,
        'on_backup': {channel: [source for source in sources if cc_client._on_backup(context, channel, source)]
                      for channel in channels},
    }


def build() -> dict:
    """
    The on_backup fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, bases, cases}.

    Raises:
        ValueError - a case name is used twice, or a case is refused (see _answers).
    """
    known = config_corpus.bases(_BASE_NAMES)
    named = _corpus() + _attitude(known['default']) + _documents()
    names = [name for name, _ in named]
    if len(set(names)) != len(names):
        raise ValueError('a case name is used twice')
    return {
        'meta': {'source': 'src/glider/cc_client.py (_on_backup), src/glider/config_default.py, '
                           'src/glider/config_hitl.py, launches/20261003/*/*.config'},
        'compare': {'kind': 'exact'},
        'bases': known,
        'cases': [_answers(name, config, known) for name, config in named],
    }
