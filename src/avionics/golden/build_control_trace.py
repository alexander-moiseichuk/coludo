"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture control_trace.json: plan B's flight task (src/glider/tasks/flight.py) as the Rust port (crates/control,
module flight) must answer it, replayed OPEN-LOOP on the inputs of a closed-loop flight.

tools/virtual_flight.py flies the closed loop on the host (sim_model + plan B's guidance/governor/PID/mixer) and,
through its TRACE hook, hands over every control tick's inputs: the stage, |a|, the attitude, the gyro rates, and
the GNSS speed, position and course, the pitot, the laser and the baro elevation as the handles hold them. The
builder records them, then replays them through plan B's OWN Flight task -- setup() and _tick() as the board runs
them -- with:
  * the databoard parameters as stand-ins whose read() answers the tick's input (a None input has no source), the
    course's stamp() advancing on each new GNSS sample (the wind estimator's feed);
  * a stub Controller holding the tick's stage, armed, the config's fins and three fin stubs the mixer writes;
  * virtual_flight's Mission stand-in on the scenario's zone (no launch point: tier-1 navigation on the live fix);
  * the shims' virtual clock moved one control slice per tick.
After every tick the fins, the cap and the airspeed are taken, and every 10th tick the flight.csv state (setpoints,
per-axis demands, the wind, active, reckoning).

Scenarios: a clean F15 to touchdown, and a noisy (5 %) windy (6 m/s from 210) F15 seeded, cut at 50 s.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import os
import sys

import config_hitl
import databoard
import fixed
import inspector
import recorder
import shims
import sim_model
from tasks import flight

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'tools'))
import virtual_flight  # noqa: E402

_HZ: int = 100  # the control rate virtual_flight flies (its --hz default) and the replay steps at
_SAMPLE_EVERY: int = 10  # the flight.csv state's sampling: 10 Hz, as flight.csv records it
_GLIDER_G: int = 285  # the collectors' glider mass (VF_GLIDER_G), the mass the Rust board flies in HITL


class _Channel:
    """A databoard parameter stand-in: read() answers the tick's input, value() the same, stamp() the sample count."""

    def __init__(self, window_us: int) -> None:
        self.value_now = None
        self.window_us: int = window_us
        self.samples: int = 0

    def read(self) -> tuple:
        return (self.value_now, None if self.value_now is None else 'sim', 0 if self.value_now is not None else None)

    def value(self):
        return self.value_now

    def stamp(self):
        return self.samples if self.samples else None


class _Fin:
    """A servo stand-in: the mixer's set_angle() lands here."""

    def __init__(self) -> None:
        self.angle: int = 90

    def set_angle(self, angle: int) -> None:
        self.angle = angle

    def settle(self) -> None:
        """A folded mid-slew command is the driver's business (sg90): the replay keeps the commanded angle."""


class _Controller:
    """The stage, the arming, the config's fins and the fin stubs plan B's flight task reads."""

    def __init__(self, config: dict) -> None:
        self.config = config
        self.stage: int = 1
        self.armed: bool = True
        self.fins = {name: _Fin() for name in config['fins']['mixer']['surfaces']}

    def find(self, names: list) -> list:
        return [self.fins.get(name) for name in names]


class _Telemetry:
    """flight.csv: never due -- the replay reads the task's state itself, on its own sampling."""

    def __init__(self, *_arguments, **_keywords) -> None:
        pass

    def due(self, _now: int) -> bool:
        return False


def _record(motor: str, noise: float, wind: float, wind_dir: float, seconds: float, seed: int) -> list:
    """virtual_flight's closed loop, its per-tick inputs as the TRACE hook hands them."""
    ticks = []
    os.environ['VF_SEED'] = str(seed)
    os.environ['VF_GLIDER_G'] = str(_GLIDER_G)
    virtual_flight.TRACE = ticks.append
    try:
        virtual_flight.fly(motor, noise, False, _HZ, seconds, wind, wind_dir)
    finally:
        virtual_flight.TRACE = None
    return ticks


_HEADINGS: dict = {0: 'auto', 1: 'o', 2: 'oo', 3: 'ov'}  # plan B guidance.Heading ids -> their config names

_INPUTS: tuple = ('stage', 'accel', 'attitude', 'rate', 'speed', 'position', 'course', 'pitot', 'agl', 'elevation')


def _pack(values) -> str:
    """One tick as one text: its values joined by ';', a vector's components by ',', a missing value empty -- a
    fixture of one number per line would run to megabytes."""
    def text(value) -> str:
        if value is None:
            return ''
        if isinstance(value, (list, tuple)):
            return ','.join(text(item) for item in value)
        return repr(value)
    return ';'.join(text(value) for value in values)


def _rounded(value, places: int = 4):
    """A float input rounded for the fixture (tuples to lists); None stays None. Both replays read the rounded."""
    if value is None:
        return None
    if isinstance(value, (list, tuple)):
        return [round(item, places) for item in value]
    return round(value, places)


def _replay(name: str, motor: str, ticks: list) -> dict:
    """Plan B's flight task over `ticks`: the case."""
    config = config_hitl.default(motor=motor, glider_g=_GLIDER_G)
    entry = next(component for component in config['components'] if component['name'] == 'flight')
    zone = sim_model.HPRC['zone']
    mission = virtual_flight._Mission(zone)
    channels = {quantity: _Channel(window) for quantity, window in (
        ('attitude', 40000), ('rate', 20000), ('accel', 20000), ('speed', 500000), ('airspeed', 100000),
        ('course', 500000), ('position', 400000), ('agl', 100000), ('elevation', 200000))}
    stub = _Controller(config)
    originals = (databoard.Databoard.parameter, inspector.Inspector.get, inspector.Inspector.register,
                 recorder.Telemetry, recorder.Recorder.log)
    databoard.Databoard.parameter = staticmethod(lambda quantity: channels[quantity])
    inspector.Inspector.get = staticmethod(lambda wanted: mission if wanted == 'mission' else None)
    inspector.Inspector.register = staticmethod(lambda _object: None)
    recorder.Telemetry = _Telemetry
    recorder.Recorder.log = staticmethod(lambda _descriptor, _message: None)
    inputs, outputs, samples = [], [], []
    try:
        task = flight.Flight('flight', dict(entry), stub)
        asyncio.run(task.setup())
        course = None
        for index, tick in enumerate(ticks):
            row = {'stage': tick['stage'], 'accel': _rounded(tick['accel']),
                   'attitude': _rounded([tick['heading'], tick['roll'], tick['pitch']]),
                   'rate': _rounded(list(tick['rate'])), 'speed': _rounded(tick['speed']),
                   'position': _rounded(tick['position'], 7), 'course': _rounded(tick['course']),
                   'pitot': _rounded(tick['pitot']), 'agl': _rounded(tick['agl']),
                   'elevation': _rounded(tick['elevation'])}
            inputs.append(_pack(row[key] for key in _INPUTS))
            heading, roll, pitch = row['attitude']
            channels['attitude'].value_now = (heading, fixed.from_float(roll), fixed.from_float(pitch))
            channels['rate'].value_now = tuple(fixed.from_float(rate) for rate in row['rate'])
            channels['accel'].value_now = (0.0, 0.0, row['accel'])
            channels['speed'].value_now = row['speed']
            channels['airspeed'].value_now = row['pitot']
            channels['position'].value_now = None if row['position'] is None else tuple(row['position'])
            channels['agl'].value_now = row['agl']
            channels['elevation'].value_now = row['elevation']
            if row['course'] is not None:  # a new GNSS sample: the course is fresh and its stamp moves
                course = row['course']
                channels['course'].samples += 1
            channels['course'].value_now = course if row['speed'] is not None else None
            stub.stage = row['stage']
            shims.advance(1000 // _HZ)
            task._tick()
            law = task._guidance
            outputs.append(_pack(([stub.fins[surface].angle for surface in sorted(stub.fins)], task._mixer.limit,
                                  round(task._governor.airspeed(), 4))))
            if index % _SAMPLE_EVERY == 0:
                samples.append(_pack((index, int(task._active), law.roll_setpoint / fixed.SCALE,
                                      law.pitch_setpoint / fixed.SCALE, law.heading_error,
                                      [task._roll_cmd, task._pitch_cmd, task._yaw_cmd],
                                      [round(task._wind.speed(), 4), round(task._wind.direction(), 3)],
                                      int(bool(law.reckoning)))))
    finally:
        (databoard.Databoard.parameter, inspector.Inspector.get, inspector.Inspector.register, recorder.Telemetry,
         recorder.Recorder.log) = originals
    return {'name': name,
            'input': {'hz': _HZ, 'zone': [list(corner) for corner in zone],
                      'endgame_heading': _HEADINGS[mission.endgame_heading()], 'flight': entry, 'fins': config['fins'],
                      'columns': list(_INPUTS), 'ticks': inputs},
            'output': {'columns': ['fins', 'cap', 'airspeed'], 'ticks': outputs,
                       'sample_columns': ['tick', 'active', 'roll_sp', 'pitch_sp', 'heading_err', 'commands', 'wind',
                                          'reckoning'],
                       'samples': samples}}


def build() -> dict:
    """
    The control_trace fixture document.

    Returns:
        {meta, compare, note, cases}.
    """
    cases = [_replay('f15_clean', 'F15', _record('F15', 0.0, 0.0, 0.0, 240.0, 1)),
             _replay('f15_noise05_wind6', 'F15', _record('F15', 0.05, 6.0, 210.0, 50.0, 20261011))]
    return {
        'meta': {'source': 'src/glider/tasks/flight.py, tools/virtual_flight.py'},
        'compare': {'kind': 'abs', 'tol': 1.0},
        'note': ('per case: input {hz, zone, endgame_heading, flight (the entry), fins (the config section), columns, '
                 'ticks: one text per tick, its columns joined by ";" and a vector\'s components by ",", empty = no '
                 'source -- stage id, accel |a| g, attitude heading,roll,pitch deg, rate roll,pitch,yaw deg/s, speed '
                 'm/s, position lat,lon, course deg (only on a tick with a new GNSS sample), pitot m/s, agl m, '
                 'elevation m}; output {columns, ticks: one text per tick -- fins per surface sorted by name, cap, '
                 'airspeed m/s; samples: every 10th tick, packed the same -- tick, active, roll_sp, pitch_sp deg, '
                 'heading_err, commands roll,pitch,yaw whole deg, wind speed,from, reckoning}. Fins, cap, '
                 'setpoints and commands within 1 deg; the test sets its own tolerance for airspeed and wind'),
        'cases': cases,
    }
