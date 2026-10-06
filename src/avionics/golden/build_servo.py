"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture servo.json: plan B's fin servos (src/glider/drivers/sg90.py and its thin subclass mg90s.py) as the
Rust port (crates/drivers, modules sg90 and mg90s) must answer them, compared exactly: the degree-to-pulse map is
integer arithmetic in plan B and in the port.

  * clamp     -- _clamp(angle): Python's round(), a half to EVEN (sg90.py:251: 90.5 -> 90, 91.5 -> 92), then
                 commons.clamp_int to [min_deg, max_deg], over explicit ties and seeded floats;
  * scenarios -- stateful: a servo set up from a component block (its neutral, its first command, its trim), then
                 operator updates, the flight loop's compare-and-set set_angle, move()'s travel estimate and
                 travel_ms itself, step by step. Inputs are test_sg90.py's and test_mg90s.py's (the 0..180 SG90,
                 the -30..30 elevon, the +9 / +4 / fractional trims, the centred 360-degree MG90S), the corners
                 (an upside-down range, a zero span, a trim past the pulse range) and seeded servos.

The builder runs plan B's own setup(), update(), set_angle(), move() and travel_ms() with a stand-in PWM (the host
has no machine.PWM) and a telemetry stand-in that counts the rows a command writes; move()'s settle wait is
recorded instead of slept, so its length is plan B's travel estimate. The coalescing branch of set_angle is off by
default and not ported (avionics-port.md §6), so no scenario turns it on.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import random

import config_default
import servo
import shims
from drivers import mg90s, sg90

_SEED: int = 20261006  # the seeded servos' random.Random seed: the fixture is a function of the code alone
_SEEDED: int = 16  # seeded scenarios
_TYPES: dict = {'sg90': sg90.SG90, 'mg90s': mg90s.MG90S}


class _Controller:
    """The board the servo reads its pin and the fins block from: plan B's default config."""

    config: dict = config_default.default()


class _Pwm:
    """machine.PWM as sg90 uses it: constructed on a pin, duty_u16() written, deinit()."""

    def __init__(self, pin, freq: int = 50, duty_u16: int = 0) -> None:
        self.duty = duty_u16

    def duty_u16(self, duty: int) -> None:
        self.duty = duty

    def deinit(self) -> None:
        pass


class _Rows:
    """A telemetry stream stand-in: keeps the rows pushed."""

    def __init__(self) -> None:
        self.rows: list = []

    def push(self, row: tuple) -> None:
        self.rows.append(list(row))


class _Scenario:
    """One servo on plan B: set up from its block, then driven step by step."""

    def __init__(self, name: str, kind: str, config: dict) -> None:
        self.name = name
        block = dict(config, pin='servo_yaw')
        self.servo = _TYPES[kind]('servo_yaw', block, _Controller())
        servo.Gate.reset()
        if not asyncio.run(self.servo.setup()):
            raise ValueError('servo: scenario %s failed setup' % name)
        self.servo._telemetry = _Rows()
        self.init = {'type': kind, 'config': config}
        self.setup = self.state()
        self.steps: list = []

    def state(self) -> dict:
        """What the servo holds: the command, the pulse, the trim, the neutral."""
        return {'angle': self.servo.angle, 'pulse_us': self.servo._pulse_us, 'trim': self.servo._trim_deg,
                'neutral': self.servo._neutral}

    def _rows(self) -> int:
        return len(self.servo._telemetry.rows)

    def update(self, properties: dict) -> None:
        """An operator override: the keys applied and whether a command was written."""
        before = self._rows()
        applied = self.servo.update(dict(properties))
        self.steps.append({'in': {'action': 'update', 'properties': properties},
                           'out': dict(self.state(), applied=applied, written=self._rows() > before)})

    def set_angle(self, angle) -> None:
        """The flight loop's compare-and-set: the clamped angle returned and whether it wrote."""
        before = self._rows()
        returned = self.servo.set_angle(angle)
        self.steps.append({'in': {'action': 'set_angle', 'angle': angle},
                           'out': dict(self.state(), returned=returned, written=self._rows() > before)})

    def move(self, angle) -> None:
        """A gated move: its target and the settle wait plan B slept (its travel estimate)."""
        waits = []

        async def sleep_ms(milliseconds: int) -> None:
            waits.append(milliseconds)
            shims.advance(milliseconds)

        original = asyncio.sleep_ms
        asyncio.sleep_ms = sleep_ms
        try:
            target = asyncio.run(self.servo.move(angle))
        finally:
            asyncio.sleep_ms = original
        rows = self.servo._telemetry.rows[-2:]
        self.steps.append({'in': {'action': 'move', 'angle': angle},
                           'out': dict(self.state(), target=target, travel_ms=waits[0], rows=rows)})

    def travel(self, degrees: int, settle: bool) -> None:
        """travel_ms(degrees, settle)."""
        self.steps.append({'in': {'action': 'travel', 'degrees': degrees, 'settle': settle},
                           'out': dict(self.state(), travel_ms=self.servo.travel_ms(degrees, settle))})

    def document(self) -> dict:
        return {'name': self.name, 'init': self.init, 'setup': self.setup, 'steps': self.steps}


def _clamp_case(minimum: int, maximum: int, angle) -> dict:
    """One clamp case: plan B's _clamp on a servo with this range."""
    driver = sg90.SG90('servo_yaw', {}, _Controller())
    driver._min_deg, driver._max_deg = minimum, maximum
    return {'name': 'range_%d_%d_angle_%r' % (minimum, maximum, angle),
            'input': {'min_deg': minimum, 'max_deg': maximum, 'angle': angle}, 'output': driver._clamp(angle)}


def _clamps() -> dict:
    """The clamp section."""
    explicit = [(0, 180, value) for value in (90.5, 91.5, -0.5, 0.5, 1.5, 2.5, 179.5, 180.5, -0.49, 89.5000001,
                                              45, 0, 180, 999, -50, 1e9, -1e9)]
    explicit += [(-30, 30, value) for value in (29.5, 30.5, -29.5, -30.5, 0.5, -0.5)]
    explicit += [(-90, 270, value) for value in (135, 269.5, 270.5, -90.5, -89.5)]
    explicit += [(180, 0, value) for value in (90, 200, -10)]  # a range set upside down
    generator = random.Random(_SEED)
    for _ in range(60):
        whole = generator.randint(-200, 300)
        explicit.append((0, 180, whole + generator.choice((0.5, -0.5, 0.25, 0.75, 0.0))))
    return {'compare': {'kind': 'exact'}, 'note': '_clamp(angle) with _min_deg / _max_deg: round half even, then '
                                                  'clamp_int', 'cases': [_clamp_case(*case) for case in explicit]}


def _explicit() -> list:
    """test_sg90.py's and test_mg90s.py's servos, then the corners."""
    scenarios = []

    def scenario(name: str, kind: str = 'sg90', **config) -> _Scenario:
        built = _Scenario(name, kind, config)
        scenarios.append(built)
        return built

    fin = scenario('test_sg90_default_range')
    fin.update({'angle': 45})
    fin.update({'angle': 999})
    fin.update({'angle': -50})
    fin.update({})
    fin.update({'angle': 90})
    for angle in (45, 45, 999, 180, 179.5, 178.5):
        fin.set_angle(angle)
    fin.travel(60, True)
    fin.travel(60, False)
    fin.travel(-60, False)
    fin.travel(3, False)
    geared = scenario('test_sg90_limited_throw', min_deg=-30, max_deg=30, angle=0)
    geared.update({'angle': 90})
    geared.set_angle(-12.5)
    trimmed = scenario('test_sg90_trim_9', trim=9)
    trimmed.update({'trim': 0})
    trimmed.update({'trim': 4, 'angle': 90})
    fractional = scenario('test_sg90_fractional_trims', trim=2.6)
    for trim in (-1.4, 2.5, -2.5, 3.5, 0.5):
        fractional.update({'trim': trim})
    moved = scenario('test_sg90_move')
    moved.move(120)
    moved.move(999)
    moved.move(119)
    centred = scenario('test_mg90s_centred_360', 'mg90s', min_deg=-90, max_deg=270)
    for angle in (135, 45, 999, -999):
        centred.update({'angle': angle})
    centred.update({'angle': 90, 'trim': 9})
    centred.travel(60, True)
    stock = scenario('test_mg90s_stock', 'mg90s')
    stock.move(135)
    stock.travel(45, True)
    scenario('a_setup_angle_of_a_half', angle=45.5)
    upside = scenario('an_upside_down_range', min_deg=180, max_deg=0)
    upside.update({'angle': 90})
    upside.update({'angle': 200})
    upside.set_angle(-10)
    flat = scenario('a_zero_span', min_deg=90, max_deg=90)
    flat.update({'angle': 30})
    flat.update({'trim': 5})
    overtrimmed = scenario('a_trim_past_the_pulse_range', trim=-100)
    overtrimmed.update({'angle': 0})
    overtrimmed.update({'trim': 100})
    narrow = scenario('custom_pulse_range', min_us=1000, max_us=2000, min_deg=-45, max_deg=45, trim=-3)
    narrow.update({'angle': 7})
    narrow.set_angle(-44.5)
    narrow.move(10)
    return scenarios


def _seeded(generator: random.Random, index: int) -> _Scenario:
    """One seeded servo: a random range, pulse span and trim, then a random mix of commands."""
    minimum = generator.choice((0, -30, -45, -90))
    maximum = minimum + generator.choice((60, 90, 180, 360))
    config = {'min_deg': minimum, 'max_deg': maximum, 'min_us': generator.randint(400, 700),
              'max_us': generator.randint(2300, 2600), 'trim': generator.randint(-20, 20) / 2}
    built = _Scenario('seeded_%02d' % index, generator.choice(('sg90', 'mg90s')), config)
    for _ in range(12):
        draw = generator.random()
        angle = generator.randint(minimum - 20, maximum + 20) + generator.choice((0, 0.5, -0.5, 0.3))
        if draw < 0.45:
            built.set_angle(angle)
        elif draw < 0.7:
            built.update({'angle': angle})
        elif draw < 0.8:
            built.update({'trim': generator.randint(-20, 20) / 2})
        elif draw < 0.9:
            built.move(angle)
        else:
            built.travel(generator.randint(-400, 400), generator.random() < 0.5)
    return built


def build() -> dict:
    """
    The servo fixture document.

    Returns:
        {meta, clamp, scenarios}.
    """
    scenarios = _explicit()
    generator = random.Random(_SEED + 1)
    scenarios += [_seeded(generator, index) for index in range(_SEEDED)]
    sg90.PWM, sg90.Pin = None, None  # nothing past build() constructs one
    return {
        'meta': {'source': ('src/glider/drivers/sg90.py, src/glider/drivers/mg90s.py, src/glider/commons.py, '
                            'src/glider/test/test_sg90.py, src/glider/test/test_mg90s.py')},
        'clamp': _clamps(),
        'scenarios': {
            'compare': {'kind': 'exact'},
            'note': ('stateful: init {type, config}, setup (the state after setup()), then steps: update(properties) '
                     '-> applied and written, set_angle(angle) -> returned and written, move(angle) -> target, the '
                     'travel_ms it waited and its two rows [angle, pulse_us, done], travel(degrees, settle) -> '
                     'travel_ms; every out carries angle, pulse_us, trim and neutral after the step'),
            'cases': [scenario.document() for scenario in scenarios],
        },
    }


sg90.PWM, sg90.Pin = _Pwm, (lambda gpio: gpio)  # the host has no machine.PWM: the stand-in for setup()
