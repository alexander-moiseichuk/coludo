"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture config_id.json: plan B's config.config_id() over the board-config corpus, and config._canon(), the
Python-repr text it hashes, over a seeded sweep of the values a config can hold.

Members:
  * bases -- the generated documents the cases name (config_corpus.py explains the input forms);
  * cases -- {name, input: a document named as config_corpus.py describes, output: the 12-digit id}: every
    launches/20261003/*/*.config as it is, config_default.default() (firmware_version the frozen 'dev', which the
    id leaves out anyway), {}, the taster copy of tms7f.config (board.id 'taster': what the M2 proof pushes to the
    taster), config_hitl variants, tools/launch_config.py outputs, firmware_version moves and non-objects. The
    ids the port plan publishes (doc/specs/avionics-port.md M2) are checked here, so a corpus that drifts from
    them stops the builder instead of moving a fixture;
  * canon -- {name, input: a value, output: _canon(value)}: floats over every magnitude (explicit edges, random
    bit patterns, random short decimals; each exact tie and narrow power of two also alone, as float_tie_<n>
    and float_narrow_<n>), ints to the u64 / i64 ends, strings with quotes, backslashes, controls
    and unicode, nested containers with keys that sort tellingly, True/False/None and the negative zero;
  * unicode -- the same shape: strings made of the neighbours of every edge of CPython's printable set (each
    first non-printable character after a printable run, and the one before it, and the same at each run's
    end), so the port's printable table is held to CPython 3.12's at every edge.

Values the Rust side cannot receive are left out: lone surrogates (not JSON for serde_json) and integers past
the u64 / i64 range (serde_json reads them as floats).

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import random
import struct
import unicodedata

import config as config_module
import config_corpus

_SEED: int = 20261006  # every sweep's random.Random seed
_RANDOM_BIT_FLOATS: int = 300  # finite floats from random 64-bit patterns
_RANDOM_DECIMALS: int = 300  # floats from random short decimals over -30..30 decades
_RANDOM_STRINGS: int = 80  # random strings from the character pool
_PER_CASE: int = 10  # floats per canon case (one list each)
_UNICODE_PER_CASE: int = 32  # characters per unicode case
_SURROGATES: range = range(0xD800, 0xE000)  # not JSON text for serde_json, and no Rust char

"""The ids doc/specs/avionics-port.md (M2) publishes: the builder refuses a corpus that disagrees."""
_PUBLISHED: dict = {
    'tms7c': 'e95f341166d4',
    'tms7d': '7f06b2f496a9',
    'tms7d_control': '04b9c8644717',
    'tms7e': '2492734e035f',
    'tms7f': 'f80fcbb3231e',
    'default': '95e1af1d7b1d',
    'empty': '44136fa355b3',
    'taster_copy_of_tms7f': '38052780281d',
}

"""The generated documents this fixture's cases may name as a base (config_corpus.BASES)."""
_BASE_NAMES: list = ['default', 'hitl', 'hitl_e16_noisy', 'flight_ready_tms7f', 'flight_ready_new']

"""
Floats that are an exact tie between their two closest shortest spellings: 1371010358211522.25 is as near
...522.2 as ...522.3. CPython's dtoa rounds such a tie half to even; Rust's `{:e}` rounds it half up.
"""
_TIE_FLOATS: list = [1371010358211522.2, 259189143840433.12, 23219187932159.812, 2.9802322387695312e-08]

"""
Powers of two whose closest digits at the shortest length do not read back (the rounding interval below a
power of two is half as wide as the one above): the shortest digits that do read back are printed instead.
"""
_NARROW_FLOATS: list = [5.960464477539063e-08, 5.282945311356653e+269]


def _corpus() -> list:
    """
    The config_id cases' inputs.

    Returns:
        [(name, document specification)].
    """
    cases = [(path.rsplit('/', 1)[1][:-len('.config')], {'file': path}) for path in config_corpus.LAUNCH_FILES]
    firmware = ['board', 'firmware_version']
    cases += [
        ('default', {'base': 'default'}),
        ('empty', {'document': {}}),
        ('taster_copy_of_tms7f', {'file': config_corpus.TMS7F_FILE,
                                  'edits': [{'path': ['board', 'id'], 'value': 'taster'}]}),
        ('default_restamped', {'base': 'default', 'edits': [{'path': firmware, 'value': '2026.10.05.0123456789ab'}]}),
        ('default_unstamped', {'base': 'default', 'edits': [{'path': firmware, 'delete': True}]}),
        ('default_rev_99', {'base': 'default', 'edits': [{'path': ['board', 'rev'], 'value': 99}]}),
        ('tms7c_restamped', {'file': config_corpus.LAUNCH_FILES[0], 'edits': [{'path': firmware, 'value': 'dev'}]}),
        ('hitl', {'base': 'hitl'}),
        ('hitl_e16_noisy', {'base': 'hitl_e16_noisy'}),
        ('flight_ready_tms7f', {'base': 'flight_ready_tms7f'}),
        ('flight_ready_new', {'base': 'flight_ready_new'}),
        ('board_not_an_object', {'document': {'board': 'TMS-7F', 'version': '20261005'}}),
        ('firmware_version_outside_board', {'document': {'board': {'id': 'a'}, 'firmware_version': 'x'}}),
        ('only_a_firmware_version', {'document': {'board': {'firmware_version': 'x'}}}),
        ('null', {'document': None}),
        ('list', {'document': [1, 2]}),
        ('text', {'document': 'board'}),
        ('integer', {'document': 5}),
        ('float', {'document': 2.5}),
        ('true', {'document': True}),
    ]
    return cases


def _random_bit_float(generator: random.Random) -> float:
    """A finite float from a random 64-bit pattern: every exponent and mantissa shape, subnormals included."""
    while True:
        value = struct.unpack('<d', generator.getrandbits(64).to_bytes(8, 'little'))[0]
        if value == value and abs(value) != float('inf'):
            return value


def _random_decimal(generator: random.Random) -> float:
    """A float read from a random decimal of 1..17 significant digits and a decade in -30..30: the texts people type."""
    digits = generator.randint(1, 17)
    mantissa = generator.randrange(10 ** (digits - 1), 10 ** digits)
    sign = '-' if generator.random() < 0.3 else ''
    return float('%s%de%d' % (sign, mantissa, generator.randint(-30, 30) - digits + 1))


def _edge_floats() -> list:
    """The floats where CPython's repr changes layout or is easy to get wrong."""
    edges = [0.0, -0.0, 5.0, 0.1, 0.2, 0.30000000000000004, 1.5, -1.5e-07, 0.001, 0.0001, 0.00011,
             9.999999999999999e-05, 1e-05, 1e+15, 1e+16, 9999999999999998.0, 1234567890123456.0,
             12345678901234568.0, 123456789.123, 1e+22, 1e+23, 5e-324, 2.2250738585072014e-308,
             1.7976931348623157e+308, -1.7976931348623157e+308, 4.35, 0.5, 2.675, 100.0, 1e+100, 1.18, 28.0]
    edges += _TIE_FLOATS + _NARROW_FLOATS
    for decade in range(-30, 31):
        edges += [10.0 ** decade, 1.5 * 10.0 ** decade, -9.87654321 * 10.0 ** decade]
    return edges


def _float_cases(generator: random.Random) -> list:
    """Every float sweep, in lists of _PER_CASE: (name, list)."""
    sweeps = {'float_edges': _edge_floats(),
              'float_bits': [_random_bit_float(generator) for _ in range(_RANDOM_BIT_FLOATS)],
              'float_decimals': [_random_decimal(generator) for _ in range(_RANDOM_DECIMALS)]}
    cases = []
    for name, values in sweeps.items():
        for start in range(0, len(values), _PER_CASE):
            cases.append(('%s_%03d' % (name, start // _PER_CASE), values[start:start + _PER_CASE]))
    return cases


def _character_pool() -> list:
    """The characters random strings are drawn from: ASCII, quotes, backslash, controls, Latin-1, wider unicode."""
    pool = [chr(code) for code in range(0x20, 0x7F)] + ['\'', '"', '\\'] * 8 + [chr(code) for code in range(0x20)]
    pool += [chr(0x7F), chr(0x85), chr(0xA0), chr(0xAD), chr(0xE9), chr(0xDF), chr(0x416), chr(0x3042), chr(0x4E2D),
             chr(0x200B), chr(0x2028), chr(0x301), chr(0xFE0F), chr(0xE000), chr(0xFFFD), chr(0x1F600),
             chr(0xE0001), chr(0x10FFFF), chr(0x378), chr(0x2EBF0)]
    return pool


def _random_text(generator: random.Random, pool: list) -> str:
    """A random string of 0..20 characters: mostly from the pool, sometimes any code point outside the surrogates."""
    characters = []
    for _ in range(generator.randint(0, 20)):
        if generator.random() < 0.2:
            code = generator.choice([generator.randrange(0x80, 0x800), generator.randrange(0x800, 0x10000),
                                     generator.randrange(0x10000, 0x110000)])
            characters.append(chr(code) if code not in _SURROGATES else 'x')
        else:
            characters.append(generator.choice(pool))
    return ''.join(characters)


def _scalar_and_container_cases(generator: random.Random) -> list:
    """Ints, strings, and nested containers: (name, value)."""
    integers = [0, 1, -1, 2 ** 31, -2 ** 31, 2 ** 53 + 1, 2 ** 63 - 1, -2 ** 63, 2 ** 64 - 1]
    integers += [generator.randrange(-2 ** 63, 2 ** 64) for _ in range(20)]
    cases = [('integers', integers), ('bools_and_none', [True, False, None])]
    cases += [('float_tie_%02d' % index, value) for index, value in enumerate(_TIE_FLOATS)]
    cases += [('float_narrow_%02d' % index, value) for index, value in enumerate(_NARROW_FLOATS)]
    texts = ['', 'taster', "it's", 'say "hi"', 'it\'s "both"', '\'', '"', '\\', 'a\\b\\', 'tab\tnew\nreturn\r',
             ''.join(chr(code) for code in range(0x20)) + chr(0x7F), 'Pr' + chr(0xE9) + 's', chr(0xA0) + 'nbsp',
             chr(0x1F600) + ' ' + chr(0xE0001), 'GPIO7 used by both i2c:0.sda and pins.servo_yaw']
    cases += [('text_%02d' % index, text) for index, text in enumerate(texts)]
    pool = _character_pool()
    cases += [('text_random_%02d' % index, _random_text(generator, pool)) for index in range(_RANDOM_STRINGS)]
    cases += [
        ('empty_containers', [[], {}, [[]], {'a': {}}]),
        ('keys_sort_by_code_point', {'b': 1, 'a': 2, 'B': 3, chr(0xE9): 4, '10': 5, '9': 6, '': 7, 'a b': 8,
                                     'z': 9, chr(0x1F600): 10, chr(0xFFFD): 11}),
        ('keys_with_quotes', {"it's": 1, 'say "x"': 2, 'both\'"': 3, 'back\\slash': 4}),
        ('nested', {'list': [1, [2.5, [None, [True, {'deep': -0.0}]]]], 'mixed': [{'b': [], 'a': 'x'}, 'y', 0]}),
        ('negative_zero', -0.0),
        ('board_like', {'board': {'id': 'TMS-7F', 'mcu': 'esp32p4', 'rev': 1}, 'pins': {'servo_yaw': 26,
                                                                                        'laser_int': None}}),
    ]
    return cases


def _unicode_cases() -> list:
    """
    Strings of the neighbours of every edge of CPython's printable set from U+0080 on: (name, text).

    The edges are those of the port's table (src/printable.rs): U+0080, where the first run starts, and every
    code point whose printability differs from the one before it. Each edge and the code point below it are
    taken (U+007F and U+10FFFF included). Surrogates are skipped (no JSON text carries one alone).
    """
    edges, inside = [], False
    for code in range(0x80, 0x110000):
        if chr(code).isprintable() == inside:
            edges.append(code)
            inside = not inside
    edges.append(0x110000)  # the last run's end: its neighbour below, U+10FFFF, is taken
    characters = []
    for edge in edges:
        for code in (edge - 1, edge):
            if code < 0x110000 and code not in _SURROGATES and (not characters or characters[-1] != chr(code)):
                characters.append(chr(code))
    return [('unicode_%03d' % (start // _UNICODE_PER_CASE), ''.join(characters[start:start + _UNICODE_PER_CASE]))
            for start in range(0, len(characters), _UNICODE_PER_CASE)]


def build() -> dict:
    """
    The config_id fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, bases, cases, canon, unicode}.

    Raises:
        ValueError - plan B's id for a corpus document differs from the one the port plan publishes.
    """
    known = config_corpus.bases(_BASE_NAMES)
    cases = []
    for name, specification in _corpus():
        identifier = config_module.config_id(config_corpus.resolve(specification, known))
        if name in _PUBLISHED and identifier != _PUBLISHED[name]:
            raise ValueError('%s: plan B says %s, the port plan publishes %s' % (name, identifier, _PUBLISHED[name]))
        cases.append({'name': name, 'input': specification, 'output': identifier})
    generator = random.Random(_SEED)
    sweep = _float_cases(generator) + _scalar_and_container_cases(generator)
    return {
        'meta': {'source': 'src/glider/config.py (config_id, _canon); ' + config_corpus.SOURCES},
        'compare': {'kind': 'exact'},
        'unicode_version': unicodedata.unidata_version,
        'bases': known,
        'cases': cases,
        'canon': [{'name': name, 'input': value, 'output': config_module._canon(value)} for name, value in sweep],
        'unicode': [{'name': name, 'input': text, 'output': config_module._canon(text)}
                    for name, text in _unicode_cases()],
    }
