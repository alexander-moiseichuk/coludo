#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Plan B's SITL (M14): plan B's own Controller, stage machine (tasks/sequencer.py) and flight task (tasks/flight.py),
under the golden shims (src/avionics/golden/shims.py), behind the same UDP sim-link the Rust board and the Rust SITL
serve, lockstep. tools/hitl_host.py `--sitl tools/planb_sitl.py` flies it exactly as it flies the Rust SITL, with
the same frames, so the two stacks meet the same sensors and only the flight code differs.

tools/virtual_flight.py is no such reference: its loop feeds no wind estimator and hands its law the GNSS's total
speed, clean, where plan B's on-board HITL (tasks/hitl.py) and hitl_host publish the ground speed and the track,
noised. Here the tasks read what a board's databoard would hand them:
  * the stage machine's accel, elevation and agl as plan B's Parameter.read() answers them, [value, 'hitl', 0], or
    no reading while the frame carries none (its 1 s window: the laser out of range drops out at once, as the hitl
    entry's agl does after its timeout on a board);
  * the flight task's channels as build_control_trace.py's replay sets them (the attitude's roll and pitch and the
    rates as centidegree fixnums), the course's stamp moving with each frame that carries it;
  * a mission stand-in on the scenario's zone and launch point (the payload hitl_host seeds a board with);
  * three fin stubs the mixer writes, answered as the FINS frame's fins.
Lockstep as the Rust SITL: the clock is the frame's physics time after a 10 s boot; the control ticks due before a
frame run on what the earlier frames left, then the frame is applied, then the tick at its time, then the answer.

  planb_sitl.py --config <board config JSON> [--mission <launch JSON>] [--port 1236]
"""

import argparse
import asyncio
import json
import os
import socket
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'src', 'avionics', 'golden'))
import shims  # noqa: E402

shims.install()  # before anything from src/glider: MicroPython semantics, the virtual clock
sys.path.insert(0, _HERE)
import controller as controller_module  # noqa: E402
import databoard  # noqa: E402
import fixed  # noqa: E402
import inspector  # noqa: E402
import recorder  # noqa: E402
import sim_link_codec  # noqa: E402
import virtual_flight  # noqa: E402
from tasks import flight, sequencer  # noqa: E402

_BOOT_MS: int = 10_000  # the clock at physics time zero: the shims' boot uptime, the Rust SITL's
_HITL_WINDOW_MS: int = 1000  # the hitl source's timeout (plan B hitl.py `provided`)
_SURFACES: tuple = ('servo_eleron_left', 'servo_eleron_right', 'servo_yaw')
_SETTING = 1
_TICK_MS: int = 10  # the control tick: the board's 100 Hz


class _Reading:
    """A stage-machine parameter stand-in: plan B's read() of the last frame's value inside the hitl window."""

    def __init__(self) -> None:
        self.value = None
        self.at_ms: int | None = None
        self.now_ms: int = 0

    def read(self) -> list:
        if self.value is None or self.at_ms is None or self.now_ms - self.at_ms > _HITL_WINDOW_MS:
            return [None, None, None]
        return [self.value, 'hitl', self.now_ms - self.at_ms]

    def value_now(self):
        return self.read()[0]


class _Channel:
    """A flight-task channel stand-in (build_control_trace.py's): read() and value() answer the current input."""

    def __init__(self) -> None:
        self.value_now = None
        self.samples: int = 0
        self.window_us: int = _HITL_WINDOW_MS * 1000  # every HITL quantity is the hitl source's, at its timeout

    def read(self) -> tuple:
        return (self.value_now, None if self.value_now is None else 'hitl', 0 if self.value_now is not None else None)

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
        """A folded mid-slew command is the driver's business: the SITL keeps the commanded angle."""


class _Rows:
    """A telemetry stream stand-in: nothing written, never due."""

    def __init__(self, *_arguments, **_keywords) -> None:
        pass

    def push(self, _row: tuple) -> None:
        pass

    def due(self, _now: int) -> bool:
        return False


class _Mission(virtual_flight._Mission):
    """virtual_flight's mission stand-in with the launch point the host seeds a board with."""

    def __init__(self, zone, launch) -> None:
        super().__init__(zone)
        self.launch = launch

    def launch_point(self):
        return self.launch

    def freeze_launch(self):
        """The launch point is set: arming keeps it."""


class _Stack:
    """Plan B's flight code for one board config and mission."""

    def __init__(self, config: dict, mission: dict) -> None:
        components = {component['name']: component for component in config['components']
                      if component.get('enabled', True)}
        entry = lambda activity: next(component for component in components.values()  # noqa: E731
                                      if component.get('activity') == activity)
        zone = mission.get('zone')
        launch = (mission['latitude'], mission['longitude']) if 'latitude' in mission else None
        self.mission = _Mission(zone, launch)
        self.fins = {name: _Fin() for name in config['fins']['mixer']['surfaces']}
        self.channels = {quantity: _Channel() for quantity in (
            'attitude', 'rate', 'accel', 'speed', 'airspeed', 'course', 'position', 'agl', 'elevation')}
        # any quantity a task asks for: the frame fills those it carries (apply), the rest never read fresh
        databoard.Databoard.parameter = staticmethod(lambda quantity: self.channels.setdefault(quantity, _Channel()))
        inspector.Inspector.get = staticmethod(lambda wanted: self.mission if wanted == 'mission' else None)
        inspector.Inspector.register = staticmethod(lambda _object: None)
        recorder.Telemetry = _Rows
        recorder.Recorder.log = staticmethod(lambda _descriptor, _message: None)
        self.controller = controller_module.Controller(config, log=lambda *_arguments: None)
        self.controller.find = lambda names: [self.fins.get(name) for name in names]
        sequencer_entry = entry('sequencer')
        self.sequencer = sequencer.Sequencer('sequencer', dict(sequencer_entry, disable_gc_flight=False),
                                             self.controller)
        if not asyncio.run(self.sequencer.setup()):
            raise SystemExit('planb_sitl: the sequencer refused its config')
        self.sequencer._telemetry = _Rows()
        self.readings = {quantity: _Reading() for quantity in ('accel', 'elevation', 'agl')}
        self.sequencer._accel = self.readings['accel']
        self.sequencer._elevation = self.readings['elevation']
        self.sequencer._agl = self.readings['agl']
        self.clock_ms: int = _BOOT_MS  # the shims' virtual clock, which starts at their boot uptime
        self.stage_period_ms: int = self.sequencer._period_ms
        self.stage_due_ms: int = 0
        self.flight = flight.Flight('flight', dict(entry('flight')), self.controller)
        asyncio.run(self.flight.setup())
        self.controller.arm()  # armed on the pad, as the host arms a board

    def apply(self, fields: dict, now_ms: int) -> None:
        """One frame's fields as the stage machine's readings and the flight task's channels."""
        for quantity in ('accel', 'elevation', 'agl'):
            value = fields.get(quantity)
            if value is not None:
                self.readings[quantity].value = tuple(value) if isinstance(value, list) else value
                self.readings[quantity].at_ms = now_ms
        channels = self.channels
        if 'attitude' in fields:
            heading, roll, pitch = fields['attitude']
            channels['attitude'].value_now = (heading, fixed.from_float(roll), fixed.from_float(pitch))
        if 'rate' in fields:
            # the frame's rate is the marks frame (x forward, y left, z up); plan B's channel is (roll, pitch, yaw)
            x, y, z = fields['rate']
            channels['rate'].value_now = tuple(fixed.from_float(rate) for rate in (x, -y, -z))
        if 'accel' in fields:
            channels['accel'].value_now = tuple(fields['accel'])
        channels['speed'].value_now = fields.get('speed')
        channels['airspeed'].value_now = fields.get('airspeed')
        position = fields.get('position')
        channels['position'].value_now = None if position is None else (position[0] * 1e-7, position[1] * 1e-7)
        for quantity in ('agl', 'elevation', 'altitude', 'dynamic_pressure', 'mag'):
            value = fields.get(quantity)
            channels.setdefault(quantity, _Channel()).value_now = tuple(value) if isinstance(value, list) else value
        if fields.get('course') is not None:
            channels['course'].value_now = fields['course']
            channels['course'].samples += 1
        elif fields.get('speed') is None:
            channels['course'].value_now = None

    def tick(self, now_ms: int) -> None:
        """One control tick at `now_ms`: the stage machine when its step is due, then the flight task."""
        if now_ms > self.clock_ms:
            shims.advance(now_ms - self.clock_ms)
            self.clock_ms = now_ms
        for reading in self.readings.values():
            reading.now_ms = now_ms
        if now_ms >= self.stage_due_ms:
            self.stage_due_ms = max(self.stage_due_ms + self.stage_period_ms, now_ms + self.stage_period_ms // 2)
            self.sequencer._tick(now_ms)
        self.flight._tick()

    def answer(self, seq: int, now_ms: int) -> bytes:
        """The FINS answer: the stage, the arming, and the fins the mixer commanded."""
        stage = self.controller.stage
        return sim_link_codec.encode_fins(seq, now_ms, stage, self.controller.armed, stage > _SETTING,
                                          [self.fins[name].angle for name in _SURFACES])


def main() -> int:
    """Serve forever."""
    parser = argparse.ArgumentParser(description="plan B's flight code behind the HITL sim-link, lockstep")
    parser.add_argument('--config', required=True)
    parser.add_argument('--mission')
    parser.add_argument('--port', type=int, default=1236)
    args = parser.parse_args()
    with open(args.config) as handle:
        config = json.load(handle)
    mission = {}
    if args.mission:
        with open(args.mission) as handle:
            mission = json.load(handle)
    stack = _Stack(config, mission)
    tick_ms = _TICK_MS
    endpoint = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    endpoint.bind(('0.0.0.0', args.port))
    sys.stderr.write('planb_sitl :: sim-link on udp:%d, a tick every %d ms\n' % (args.port, tick_ms))
    next_tick, last_seq = _BOOT_MS, None
    while True:
        datagram, sender = endpoint.recvfrom(256)
        frame = sim_link_codec.decode_sensors(datagram)
        if frame is None or (last_seq is not None and (frame['seq'] - last_seq) & 0xFFFFFFFF >= 1 << 31):
            continue
        last_seq = frame['seq']
        now_ms = _BOOT_MS + frame['sim_time_ms']
        while next_tick < now_ms:
            stack.tick(next_tick)
            next_tick += tick_ms
        stack.apply(frame['fields'], now_ms)
        if next_tick == now_ms:
            stack.tick(next_tick)
            next_tick += tick_ms
        endpoint.sendto(stack.answer(frame['seq'], now_ms), sender)


if __name__ == '__main__':
    sys.exit(main())
