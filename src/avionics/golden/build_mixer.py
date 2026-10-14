"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture mixer.json: plan B's control-surface mixer (src/glider/mixer.py) as the Rust port (crates/control,
module mixer) must answer it -- the whole-degree angle each fin is commanded.

The fins take WHOLE degrees in both firmwares. Plan B's mixer hands each fin the raw sum and the sg90 driver's
_clamp rounds it with Python's round(), half to even; the Rust mixer rounds where it forms the sum, the same way
(round_ties_even). So each expected angle here is plan B's mix() output passed through sg90's own _clamp
(drivers/sg90.py, called on a stand-in whose range never binds, so only its rounding acts): exactly what plan B's
servo would have been set to.

Compare EXACT, which holds because every input is DYADIC: the axis commands are multiples of 1/8 degree and the
config gains multiples of 1/4, so every product, sum and neutral offset is exact in f32 and in f64 alike, and a
tie is a real tie on both sides (the sweep and the explicit cases include them, and the parity that decides one is
the angle's, not the deflection's: 90.5 rounds to 90, 91.5 to 92). Plan B's flight loop only ever sends integer
commands (it floors each PID output to whole degrees first); the fractional ones here pin the rounding the port
does where plan B's driver did it.

Two sections, each a golden fixture of its own ({compare, note, cases}):
  * mix -- {config, command [roll, pitch, yaw]} -> {surface name: angle}. config is the board's fins.mixer block
    as plan B reads it (null for the defaults).
  * actuate -- {config, bound [surface names with a fin], commands [[roll, pitch, yaw], ...]} -> {missing, angles}:
    after bind() and each actuate(), the surfaces with no fin (sorted: plan B lists them in config order, the Rust
    port in its own surface order) and plan B's angles(), the operator view: the last angle each bound fin's
    driver stored, None for one never commanded.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import random

import mixer
from drivers import sg90

_SEED: int = 20261006  # the sweep's random.Random seed: the fixture is a function of the code alone
_SWEEP: int = 160  # seeded mix cases
_EIGHTHS: int = 8  # sweep commands are multiples of 1/8 degree
_COMMAND_RANGE: int = 60  # sweep commands span +/- this many degrees, past every limit used


class _Range:
    """A stand-in for an SG90 instance: only the range sg90._clamp reads, wide enough never to bind."""

    _min_deg: int = -100000
    _max_deg: int = 100000


class _Fin:
    """A fin the mixer binds: records what sg90 would have set (its _clamp of the command), as test_mixer's _Fin."""

    def __init__(self):
        self.angle = None

    def set_angle(self, angle) -> None:
        self.angle = sg90.SG90._clamp(_Range(), angle)

    def settle(self) -> None:
        pass


"""The configs the cases mix under: plan B's defaults, test_mixer's tight one, and dyadic custom gains."""
_TIGHT: dict = {'neutral_deg': 90, 'limit_deg': 30, 'surfaces': {'servo_yaw': {'yaw': 1}}}
_CUSTOM: dict = {'neutral_deg': 85, 'limit_deg': 20,
                 'surfaces': {'servo_eleron_left': {'roll': 0.5, 'pitch': 1.25},
                              'servo_eleron_right': {'roll': -0.5, 'pitch': 1.25},
                              'servo_yaw': {'yaw': -2, 'roll': 0.25},
                              'servo_canard': {'pitch': -0.75, 'yaw': 0.5, 'roll': 1}}}
_ODD_NEUTRAL: dict = {'neutral_deg': 91}
_FRACTIONAL_NEUTRAL: dict = {'neutral_deg': 90.7, 'limit_deg': 10}  # plan B's base is int(90.7): 90

"""Explicit mix cases: (name, config, roll, pitch, yaw) -- test_mixer.py, bench_cases.py, then the rounding."""
_MIXES: tuple = (
    ('neutral', None, 0, 0, 0),
    ('pitch_moves_both_elevons', None, 0, 10, 0),
    ('roll_moves_the_elevons_differentially', None, 10, 0, 0),
    ('yaw_moves_the_rudder_only', None, 0, 0, 15),
    ('roll_and_pitch_combine', None, 5, 10, 0),
    ('integer_angles_out', None, 3, 7, 2),
    ('pitch_past_the_limit', None, 0, 100, 0),
    ('yaw_past_the_limit', None, 0, 0, -100),
    ('a_combined_demand_clamps_as_a_whole', None, 40, 40, 0),
    ('tight_neutral', _TIGHT, 0, 0, 0),
    ('tight_yaw_up', _TIGHT, 0, 0, 100),
    ('tight_yaw_down', _TIGHT, 0, 0, -100),
    ('bench_cases_mixer_angles', None, 50, -30, 10),
    ('a_half_rounds_to_the_even_angle_below', None, 0, 0.5, 0),
    ('a_half_rounds_to_the_even_angle_above', None, 0, 1.5, 0),
    ('a_negative_half', None, 0, -0.5, 0),
    ('a_negative_half_and_a_half', None, 0, -1.5, 0),
    ('quarters_cancel_on_one_fin', None, 0.25, 0.25, 0),
    ('an_odd_neutral_flips_the_half', _ODD_NEUTRAL, 0, 0.5, 0),
    ('an_odd_neutral_flips_the_half_and_a_half', _ODD_NEUTRAL, 0, 1.5, 0),
    ('a_fractional_neutral_truncates', _FRACTIONAL_NEUTRAL, 0, 3, 0),
    ('a_fractional_neutral_clamped', _FRACTIONAL_NEUTRAL, 0, -40, 0),
    ('custom_gains_neutral', _CUSTOM, 0, 0, 0),
    ('custom_gains_roll', _CUSTOM, 6, 0, 0),
    ('custom_gains_mixed', _CUSTOM, -3.5, 7.25, 2.125),
    ('custom_gains_clamped', _CUSTOM, 30, -30, 30),
    ('the_limit_exactly', None, 0, 45, 0),
    ('just_past_the_limit_by_an_eighth', None, 0, -45.125, 0),
)

"""Explicit actuate cases: (name, config, bound surface names, commands) -- test_mixer.py's bind/actuate."""
_ACTUATES: tuple = (
    ('all_bound', None, ('servo_yaw', 'servo_eleron_left', 'servo_eleron_right'),
     ((10, 0, 0), (0, 0, 15), (0, 100, 0))),
    ('a_fin_missing_is_skipped', None, ('servo_eleron_left',), ((0, 10, 0),)),
    ('nothing_bound', None, (), ((5, 5, 5),)),
    ('the_last_command_stands', None, ('servo_yaw', 'servo_eleron_right'), ((1.5, -2.5, 3.5), (-7, 4, 0.5))),
    ('custom_partial', _CUSTOM, ('servo_canard', 'servo_yaw'), ((3, 4, -5), (2.5, 0, 0))),
    ('bound_but_never_commanded', None, ('servo_yaw', 'servo_eleron_left'), ()),  # angles() reads None
)


def _angles(config: dict | None, roll: float, pitch: float, yaw: float) -> dict:
    """
    What plan B commands each fin for these axis commands: mix(), then sg90's rounding.

    Returns:
        {surface name: whole-degree angle}.
    """
    out = mixer.Mixer(config).mix(roll, pitch, yaw)
    return {name: sg90.SG90._clamp(_Range(), angle) for name, angle in out.items()}


def _mix_case(name: str, config: dict | None, roll: float, pitch: float, yaw: float) -> dict:
    """One mix case."""
    return {'name': name, 'input': {'config': config, 'command': [roll, pitch, yaw]},
            'output': _angles(config, roll, pitch, yaw)}


def _actuate_case(name: str, config: dict | None, bound: tuple, commands: tuple) -> dict:
    """
    One actuate case: bind() the named surfaces (the rest have no fin), actuate() each command.

    Returns:
        The case: {missing (sorted), angles (plan B's angles(): each bound fin's stored angle, None if never set)}.
    """
    unit = mixer.Mixer(config)
    fins = {name: _Fin() for name in bound}
    unit.bind(fins)
    for roll, pitch, yaw in commands:
        unit.actuate(roll, pitch, yaw)
    return {'name': name,
            'input': {'config': config, 'bound': list(bound), 'commands': [list(command) for command in commands]},
            'output': {'missing': sorted(unit.missing), 'angles': unit.angles()}}


def _command(generator: random.Random) -> float:
    """A sweep command: a multiple of 1/8 degree within +/- _COMMAND_RANGE, half of them small."""
    span = _COMMAND_RANGE if generator.random() < 0.5 else 4
    return generator.randint(-span * _EIGHTHS, span * _EIGHTHS) / _EIGHTHS


def build() -> dict:
    """
    The mixer fixture document.

    Args:
        (none)

    Returns:
        {meta, mix, actuate}: each section a {compare, note, cases} golden fixture of its own.
    """
    generator = random.Random(_SEED)
    mixes = [_mix_case(*case) for case in _MIXES]
    configs = (None, _TIGHT, _CUSTOM, _ODD_NEUTRAL, _FRACTIONAL_NEUTRAL)
    for index in range(_SWEEP):
        config = configs[index % len(configs)]
        mixes.append(_mix_case('sweep_%03d' % index, config, _command(generator), _command(generator),
                               _command(generator)))
    return {
        'meta': {'source': 'src/glider/mixer.py, src/glider/drivers/sg90.py (_clamp), src/glider/commons.py'},
        'mix': {
            'compare': {'kind': 'exact'},
            'note': ('whole degrees per fin: plan B mix() then sg90 round() (half to even); dyadic inputs, so every '
                     'sum is exact in f32 and f64 and a tie is a tie on both sides'),
            'cases': mixes,
        },
        'actuate': {
            'compare': {'kind': 'exact'},
            'note': "bind() then actuate(): the surfaces with no fin (sorted) and angles(), each bound fin's angle",
            'cases': [_actuate_case(*case) for case in _ACTUATES],
        },
    }
