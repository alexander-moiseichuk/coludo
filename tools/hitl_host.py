#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

HITL with the physics on the host (doc/specs/sim-link.md, M12): a Rust board flies its real stage machine against
src/glider/sim_model.py, the one physics implementation, over the UDP sim-link.

The run, one scenario:
  1. CC (through the hub with --hub, else the USB console): the board's saved config and mission are kept; a HITL
     copy is installed -- the board's own config, the entries it cannot run yet removed (verify's "not ported
     yet"), separation, watchdog, bluetooth and field off, its flight loop on with plan B's HITL test gains
     (config_hitl, M13: the glide is flown), and an enabled `hitl` entry carrying the scenario (the board masks the
     simulated sensors itself, after its layout); the mission gets the scenario's pad and zone (plan B hitl.py did
     the same at setup); reboot.
  2. The sim-link gate: frames until the board answers one (it answers only once its Wi-Fi joined and its socket
     is bound; frames before that are lost).
  3. The pad: the body held at rest, published at inject_hz, while CC arms the board; then the pad dwell.
  4. The flight: plan B hitl.py's realtime accumulator (fixed physics sub-steps covering the measured wall time, a
     stall's catch-up capped at 0.5 s), boost while the board's stage is SETTING or BOOSTING, the glide from
     GLIDING, the body stopped at DONE; the fins the board sends (neutral while it sends none).
  5. The board's saved config and mission restored, and a reboot (unless --keep-config).

It prints the lines tools/hitl_collect.sh parses from plan B's hitl_run: `SESSION <session> <motor> noise <n> wind
<w>`, `STAGE <id> <name>` per change, a trace every 5 s of glide, `DONE` or `TIMEOUT <stage>`, `FINS <n> of <answers>
...` (how many answers carried fins, and each fin's angle range), `RUN_END`; and the physics' score, as the SITL
prints it: `LANDED miss=<m> inside=<bool> aloft=<s> apogee=<m>`, and with --boost 6dof `BOOST path_min=<deg>
tilt_max=<deg> looped=<bool> burnout=<m/s>` (tools/boost_sweep.py parses both).

  tools/hitl_host.py --ip 192.168.102.152 --hub taster                  # the taster, F15, clean
  tools/hitl_host.py --ip 192.168.102.162 --hub TMS-7F --noise 0.05 --wind 6 --wind-dir 210
  tools/hitl_host.py --ip 192.168.102.152 --hub taster --serial /dev/ttyACM0 --reboot

Prefer --hub: the console interleaves the board's log lines with a long reply (get-config's 9.5 KB take ~0.8 s at
115200 baud), so on a board that logs while it answers the reply arrives in pieces; the hub's link frames each reply.

--reboot is plan B's mid-glide reboot made real: 1-6 s into the glide the console (--serial) panics the board
(`panic kind=rust confirm`) while the sim flies on; the board warm-starts from its crumb, joins the Wi-Fi again (a
HITL board may, the sim rides it) and flies on. It prints `REBOOT`, `WARM GATE: <restored> <the checkpoint's
gate>` (the collectors fail a reboot scenario on `WARM GATE: False`) and `REBOOTED <session> boot <id>`. The panic
goes through the console (`panic` is a console command), the rest through --hub when given.
"""
import argparse
import copy
import json
import math
import os
import random
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time

_ROOT: str = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, 'src', 'glider'))
sys.path.insert(0, os.path.join(_ROOT, 'src', 'control'))
import board_clean  # noqa: E402
import cc_protocol as cc  # noqa: E402
import config_hitl  # noqa: E402
import sim_model  # noqa: E402
import stack_dynamics  # noqa: E402
from contract import transports  # noqa: E402
from sim_link_codec import decode_fins, decode_sensors, encode_fins, encode_sensors  # noqa: E402,F401

SIM_PORT: int = 1236  # the board's sim-link port (the hitl entry's sim_port default)
_STAGES: dict = {1: 'setting', 2: 'boosting', 3: 'gliding', 4: 'landing', 5: 'done'}
_SETTING, _BOOSTING, _GLIDING, _DONE = 1, 2, 3, 5

_BARO_NOISE_SCALE: float = 0.05  # plan B hitl.py: the baro is ~20x more precise than the IMU/GNSS
_PITOT_RAIL_MS: float = (2.0 * 546.0 / sim_model.RHO) ** 0.5  # the SDP810's +-546 Pa rail, ~29.9 m/s
_MAG_UNIT: float = 1000.0  # a calibrated BMM350's horizontal scale
_MAG_DOWN: float = 700.0  # a plausible vertical component
_MAG_DECLINATION_DEG: float = 17.0  # declination + mounting as one constant, the lump attitude learns
_SERVO_NEUTRAL_DEG: float = 90.0
_MAX_CATCHUP_S: float = 0.5  # a scheduling stall's catch-up cap (plan B hitl.py)
_TIMEOUT_S: float = 150.0  # from ignition, plan B hitl_run's cap
_HANDOVER_S: float = 0.5  # --no-attitude: the separation's window, the true attitude handed to the filter
_SITL_PAD_S: float = 2.0  # the SITL's pad before ignition, physics seconds: the stage machine's windows fill
_FILTER_PAD_S: float = 15.0  # ...with --no-attitude: the board's filter converges on the rod, as minutes there do
_LOCKSTEP_WAIT_S: float = 2.0  # how long a lockstep frame waits for its answer before the run stalls
_GATE_S: float = 120.0  # how long the board gets to join Wi-Fi and answer a frame
_QUIET_S: float = 3.0  # after an answer that is no reply: the board finishes a long one (get-config, ~0.8 s)
# the board's hitl quantities hold their last sample for their timeout (1000 ms, firmware/src/devices.rs): a held
# launch-sized |a| fires the stage machine's launch dwell again after `stage setting` (7F, noise100, 2026-10-13), so
# the restore waits them out first, and makes sure the board is in SETTING before it writes the configs
_HELD_S: float = 1.5
_SETTING_TRIES: int = 3
_OFF: tuple = ('separation', 'watchdog', 'bluetooth', 'field')  # plan B's _OFF less the radios the link rides
_SERVO_DRIVERS: tuple = ('sg90', 'mg90s')  # the fins' drivers (firmware/src/fins.rs SERVO_DRIVERS)


def _plan_b_gains() -> dict:
    """The test gains plan B's HITL flew (config_hitl.default's flight entry), the one source of them."""
    return next(entry for entry in config_hitl.default()['components'] if entry['name'] == 'flight')['gains']


def hitl_config(saved: dict, not_ported: set, args: argparse.Namespace) -> dict:
    """
    The board's HITL copy of its own saved config.

    Args:
        saved - the board's saved config (get-config saved).
        not_ported - the entry names verify reports as not ported yet: removed, not disabled (the layout re-enables
            a fitted part), so arm passes (avionics-port.md M12).
        args - the scenario.

    Returns:
        The config to install: plan B config_hitl.default's changes over the board's own config.

    Raises:
        ValueError - the config has no flight entry, or more than one: HITL flies exactly one loop.
    """
    hitl = copy.deepcopy(saved)
    flights = [entry for entry in hitl.get('components', []) if entry.get('activity') == 'flight']
    if len(flights) != 1:
        raise ValueError('%d flight entries in the config: HITL flies exactly one' % len(flights))
    flights[0]['enabled'] = True
    flights[0]['gains'] = _plan_b_gains()
    # BOOSTING's law (doc/specs/boost-control.md): passive leaves BOOSTING out of the control stages, hold is plan B's
    # entry-attitude hold, lean the BC3 law; `config` flies whatever the board's config says
    law = getattr(args, 'boost_law', 'config')
    if law != 'config':
        stages = flights[0].setdefault('stages', {})
        flights[0].pop('boost_law', None)
        if law == 'passive':
            stages.pop('boosting', None)
        else:
            stages.setdefault('boosting', {})
            if law == 'lean':
                flights[0]['boost_law'] = 'lean'
    # BOOSTING's own ceiling on the fin cap (the flight entry's boost_limit_deg); None keeps the config's
    if getattr(args, 'boost_limit', None) is not None:
        flights[0]['boost_limit_deg'] = args.boost_limit
    # the law plans with the airframe the physics flies: its reach (glide_ratio) and the endgame's stall floor
    frame = sim_model.AIRFRAMES[getattr(args, 'airframe', 'tms7')]
    flights[0]['glide_ratio'] = frame['air_quality']
    flights[0]['stall_speed_1g'] = frame['stall_ms']
    for key in ('sensors', 'components'):
        kept = []
        for entry in hitl.get(key, []):
            if entry.get('name') in not_ported or entry.get('activity') == 'hitl':
                continue
            if entry.get('name') in _OFF:
                entry['enabled'] = False
            # a board whose servo rail is off (7F in its airframe, the operator 2026-10-11): the fins are still
            # commanded and recorded (degrees, steps), but with no power reading arm's probe passes open-loop
            if getattr(args, 'no_power', False) and entry.get('driver') == 'ina226':
                entry['enabled'] = False
            # the fins' step (tools/servo_step_sweep.py): every fin's grid, its own setting otherwise
            if getattr(args, 'servo_step', None) is not None and entry.get('driver') in _SERVO_DRIVERS:
                entry['step_deg'] = args.servo_step
            kept.append(entry)
        if key in hitl:
            hitl[key] = kept
    liftoff_g = config_hitl._BOOSTER_G.get(args.motor, 193) + args.glider_g  # the boost carries booster + glider
    entry = {'name': 'hitl', 'activity': 'hitl', 'enabled': True, 'sim_port': SIM_PORT,
             'sim_hz': args.sim_hz, 'motor': args.motor, 'noise': args.noise, 'spike': args.spike,
             'liftoff_g': liftoff_g, 'glider_g': args.glider_g, 'wind': args.wind, 'wind_dir': args.wind_dir,
             'boost_axis': args.boost_axis, 'gnss_drift': args.gnss_drift, 'gnss_drift_dir': args.gnss_drift_dir,
             'pad_dwell_s': args.pad_dwell_s}
    if args.inject_hz:
        entry['inject_hz'] = args.inject_hz
    hitl.setdefault('components', []).append(entry)
    return hitl


class _Board:
    """The board over CC: one request line, one parsed reply."""

    def __init__(self, transport: object) -> None:
        self._transport = transport

    def raw(self, line: str) -> str:
        """
        Send `line` once and return its reply line as the board sent it, named params included (`ask` keeps only the
        last argument, and a Rust board answers its figures as named params).

        Raises:
            transports.NoAnswer - no reply line came.
        """
        for text in self._transport.exchange(line):
            if text.startswith(('ok', 'err')):
                return text
        raise transports.NoAnswer('no reply to %r' % line)

    def ask(self, line: str, attempts: int = 15) -> tuple:
        """
        Send `line`, retrying past a reboot's silence and the console's noise.

        A board log line cut off at its start (the transport discards what was pending) passes for a reply, and the
        real reply then arrives late; after any answer that is no reply the board gets _QUIET_S to finish sending,
        which the next exchange discards.

        Returns:
            (status, payload): the reply's command (ok / err / iam) and its last argument, JSON-decoded when it is
            JSON.

        Raises:
            SystemExit - no answer after every attempt.
        """
        for _ in range(attempts):
            try:
                replies = self._transport.exchange(line)
            except (transports.NoAnswer, OSError):
                time.sleep(_QUIET_S)
                continue
            for text in replies:
                try:
                    reply = cc.parse(text)
                except ValueError:
                    continue
                if reply.command in ('ok', 'err', 'iam'):
                    argument = reply.args[-1] if reply.args else ''
                    try:
                        return reply.command, json.loads(argument)
                    except (TypeError, ValueError):
                        return reply.command, argument
            time.sleep(_QUIET_S)
        raise SystemExit('no answer to %s' % line.split(' ', 1)[0])


    def reboot(self) -> dict:
        """Reboot and wait for the new boot's whoami; returns it."""
        self.ask('reboot')
        self._transport.await_reboot()
        time.sleep(8)
        return self.ask('whoami')[1]


class _Sim:
    """
    The body and its realtime loop, on its own thread: publishes SENSORS frames at inject_hz and keeps the board's
    latest FINS answer.
    """

    def __init__(self, args: argparse.Namespace) -> None:
        scenario = sim_model.HPRC
        liftoff_g = config_hitl._BOOSTER_G.get(args.motor, 193) + args.glider_g
        self.thrust, self.burn_s = sim_model.MOTORS.get(args.motor, sim_model.MOTORS['F15'])
        self.body = sim_model.Body(liftoff_g / 1000.0, tuple(scenario['launch']), scenario['elevation_m'],
                                   scenario['heading_deg'], glide_mass=args.glider_g / 1000.0,
                                   airframe=args.airframe)
        self.body.wind_e = args.wind * math.sin(math.radians(args.wind_dir))
        self.body.wind_n = args.wind * math.cos(math.radians(args.wind_dir))
        self.body.gnss_drift_e = args.gnss_drift * math.sin(math.radians(args.gnss_drift_dir))
        self.body.gnss_drift_n = args.gnss_drift * math.cos(math.radians(args.gnss_drift_dir))
        if getattr(args, 'boost_cda', None) is not None:
            self.body.boost_cda = args.boost_cda  # a drag sweep: the airframe's boost CdA overridden
        # the 6-DoF boost (--boost 6dof, doc/specs/boost-control.md BC1): it flies ignition to separation and keeps the
        # body where it is, so the glide takes over from there; its dorsal side faces the wind's direction by default
        # (the 10-03 replays: the crosswind against the folded wings)
        self.stack = None
        # the burn as the BOOST line reports it: its path minimum, the most the nose leaned off vertical, whether the
        # nose went below the horizon with the motor burning (a loop), the speed at burnout
        self.burn = {'path_min': 90.0, 'tilt_max': 0.0, 'looped': False, 'burnout_ms': None}
        if getattr(args, 'boost', 'vertical') == '6dof':
            facing = args.wind_dir if args.stack_azimuth is None else args.stack_azimuth
            profile = dict(stack_dynamics.STACKS[args.stack])
            if args.stack_margin is not None:  # a sweep's margin, both planes (the y plane 0.15 cal stiffer, as fitted)
                profile['margin_cal'] = (args.stack_margin, args.stack_margin + 0.15)
            self.stack = stack_dynamics.Stack(profile, self.body.wind_e, self.body.wind_n, facing)
        self.apogee = 0.0  # the highest the body got (publish), for the study's LANDED line
        self.args = args
        self.axis = {'x': 0, 'y': 1, 'z': 2}[args.boost_axis]
        self.faults = sim_model.Faults(args.faults)
        self.address = (args.ip, SIM_PORT)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.seq = 0
        self.answer: dict | None = None  # the board's latest FINS
        self.answers = 0
        self.with_fins = 0  # answers carrying fins
        self.fin_range: list = []  # [low, high] per fin over those answers
        self.ignited = False  # set by the main thread once armed and the pad dwell is over
        self.sim_time = 0.0
        self.clock: float | None = None  # lockstep: the frames' clock (pad included), else the physics time
        self.rows: dict = {}  # lockstep --record: stream -> [(t_s, cells...)]
        self.wall_start: float | None = None
        # --no-attitude: the sim never publishes the attitude, so the board's own filter flies from power-up on the
        # sim's gyro, accelerometer, magnetometer and GNSS track (with the sim's attitude at priority 0 the filter only
        # mirrors it, which hid a filter that never read the GNSS course until 2026-10-14)
        self.drop_attitude = getattr(args, 'no_attitude', False)
        # ...and its sensors then read as a real airframe's (the filter flies on them alone): the pad's and the
        # vertical boost's 1 g on the nose axis and the gyro's BODY rates. The sim's attitude JUMPS at the separation
        # (sim_model begin_glide), which no gyro rate can follow: a real gyro sees that turn as it happens, so the
        # filter is handed the true attitude for _HANDOVER_S (it mirrors it, a perfect gyro through the separation),
        # then flies alone again
        self.physical = self.drop_attitude
        self.handover_until: float | None = None  # the separation's window, sim time
        self.drop_gnss = False
        self.running = True

    def gate(self) -> bool:
        """Frames at rest until the board answers one, or _GATE_S passes."""
        deadline = time.monotonic() + _GATE_S
        while time.monotonic() < deadline:
            self.publish()
            time.sleep(0.25)
            self.drain()
            if self.answer is not None:
                return True
        return False

    def drain(self) -> None:
        """Take every FINS answer waiting; keep the newest."""
        while True:
            try:
                frame, _ = self.socket.recvfrom(64)
            except (BlockingIOError, InterruptedError):
                return
            self.take(frame)

    def take(self, frame: bytes) -> dict | None:
        """One FINS datagram counted and kept when newest; the answer, or None when it is no FINS frame."""
        fins = decode_fins(frame)
        if fins is not None:
            self.answers += 1
            if fins['fins'] is not None:
                self.with_fins += 1
                self.fin_range = [[min(low, angle), max(high, angle)] for (low, high), angle in
                                  zip(self.fin_range or [[angle, angle] for angle in fins['fins']], fins['fins'])]
            if self.answer is None or (fins['seq'] - self.answer['seq']) & 0xFFFFFFFF < 1 << 31:
                self.answer = fins
        return fins

    def stage(self) -> int:
        """The board's stage by its latest answer; SETTING before any."""
        return self.answer['stage'] if self.answer is not None else _SETTING

    def fins(self) -> tuple:
        """The commanded (roll, pitch, yaw) deflections from neutral; zero while the board sends no fins."""
        fins = self.answer['fins'] if self.answer is not None else None
        if fins is None:
            return 0.0, 0.0, 0.0
        left, right, yaw = fins
        return (left - right) / 2.0, (left + right) / 2.0 - _SERVO_NEUTRAL_DEG, yaw - _SERVO_NEUTRAL_DEG

    def boost(self, fixed: float, fins: tuple) -> None:
        """
        One boost step of `fixed` seconds under the board's `fins` (roll, pitch, yaw): the 6-DoF stack's, which then
        puts the body where it is, or sim_model's vertical climb.
        """
        if self.stack is not None:
            self.stack.step(fixed, fins)
            self.stack.hand_over(self.body)
            burnout = stack_dynamics.F15[-1][0]
            if 0.4 < self.stack.seconds < burnout and not self.stack.on_rod:
                self.burn['path_min'] = min(self.burn['path_min'], self.stack.path_angle())
                self.burn['tilt_max'] = max(self.burn['tilt_max'], 90.0 - self.stack.nose_elevation())
                self.burn['looped'] = self.burn['looped'] or self.stack.nose_elevation() < 0.0
            if self.burn['burnout_ms'] is None and self.stack.seconds >= burnout:
                self.burn['burnout_ms'] = math.sqrt(sum(component * component for component in self.stack.velocity))
            return
        roll, pitch, _yaw = fins
        thrust = self.thrust if self.sim_time < self.burn_s else 0.0
        self.body.boost_step(fixed, thrust, pitch, roll)

    def run(self) -> None:
        """Plan B hitl.py's run loop on the host clock: the pad hold, then the fixed-step accumulator."""
        inject_hz = self.args.inject_hz or self.args.sim_hz
        period = 1.0 / inject_hz
        fixed = 1.0 / self.args.sim_hz
        accumulator = 0.0
        last = time.monotonic()
        while self.running:
            time.sleep(period)
            now = time.monotonic()
            elapsed = now - last
            last = now
            self.drain()
            stage = self.stage()
            if not self.ignited:  # the pad: held at rest, no backlog
                self.body.accel_g = 1.0
                accumulator = 0.0
                self.publish()
                continue
            if self.wall_start is None:
                self.wall_start = now
            if stage >= _DONE:  # the flight is over: stop flying it
                accumulator = 0.0
                continue
            accumulator += min(elapsed, _MAX_CATCHUP_S)
            boosting = stage < _GLIDING
            roll, pitch, yaw = self.fins()
            if not boosting and not self.body.gliding:
                self.body.begin_glide()  # BOOSTING -> GLIDING: deploy and glide
            while accumulator >= fixed:
                if boosting:
                    self.boost(fixed, (roll, pitch, yaw))
                else:
                    spiked = roll * 2.0 if (self.args.spike and random.random() < fixed / 3.0) else roll
                    self.body.glide_step(fixed, spiked, pitch, yaw)
                accumulator -= fixed
                self.sim_time += fixed
            self.publish()

    def separating(self, t: float) -> bool:
        """Whether --no-attitude hands the filter the true attitude now: the separation's window (see __init__)."""
        return self.handover_until is not None and t < self.handover_until

    def body_rates(self, t: float) -> list:
        """
        The gyro's reading in the marks frame as a real one measures it: the BODY rates of the sim's Euler rates
        (p = roll' - heading' sin(pitch), q = pitch' cos(roll) + heading' cos(pitch) sin(roll),
        r = -pitch' sin(roll) + heading' cos(pitch) cos(roll)), not the Euler rates themselves.
        """
        body = self.body
        if body.gliding and self.handover_until is None:
            self.handover_until = t + _HANDOVER_S
        roll, pitch = math.radians(body.roll), math.radians(body.pitch)
        heading_rate = body.yaw_rate
        p = body.roll_rate - heading_rate * math.sin(pitch)
        q = body.pitch_rate * math.cos(roll) + heading_rate * math.cos(pitch) * math.sin(roll)
        r = -body.pitch_rate * math.sin(roll) + heading_rate * math.cos(pitch) * math.cos(roll)
        return [p, -q, -r]

    def publish(self) -> None:
        """The noised sensors as one SENSORS frame (plan B hitl.py _publish), in the Rust databoard's units."""
        body, noise, t = self.body, self.args.noise, self.sim_time
        self.apogee = max(self.apogee, body.alt)
        noisy = sim_model.noisy
        # the marks frame (x forward, y left, z up), as the IMU drivers publish it. On the pad and through the boost
        # the |a| rides one axis (plan B's magnitude parity, `--boost-axis`); in the glide the specific force is the
        # load along z with gravity's share on x, (sin pitch, 0, n cos pitch), so the attitude filter's gravity
        # vector agrees with the attitude when it flies alone (an attitude drop)
        accel = [0.0, 0.0, 0.0]
        if body.gliding:
            pitch_rad = math.radians(body.pitch)
            accel = [noisy(math.sin(pitch_rad), noise, -200.0, 200.0), 0.0,
                     noisy(body.accel_g * math.cos(pitch_rad), noise, -200.0, 200.0)]
        else:
            # the nose axis (the marks' x) when the filter flies alone: on the rod a real IMU reads 1 g along it
            accel[0 if self.physical else self.axis] = noisy(body.accel_g, noise, -200.0, 200.0)
        heading = noisy(body.heading % 360.0, noise, 0.0, 360.0, sim_model.HEADING_NOISE_REF)
        roll = noisy(body.roll, noise, -180.0, 180.0)
        pitch = noisy(body.pitch, noise, -180.0, 180.0)
        # the marks frame too, by the right-hand rule: about y left the nose goes DOWN, about z up it goes LEFT
        # (crates/control attitude::rate_from_marks maps it back to roll, pitch, yaw)
        rates = [body.roll_rate, -body.pitch_rate, -body.yaw_rate]
        if self.physical:
            rates = self.body_rates(t)
        stack = self.stack if self.stack is not None and not body.gliding else None
        if stack is not None:
            # the 6-DoF's own: the specific force and the body rates are the marks frame already; the Euler angles
            # degenerate near a vertical nose as the board's do
            accel = [noisy(component / stack_dynamics.G, noise, -200.0, 200.0) for component in stack.specific]
            rates = [math.degrees(rate) for rate in stack.rates]
            heading, roll, pitch = stack.euler()
            heading = noisy(heading, noise, 0.0, 360.0, sim_model.HEADING_NOISE_REF)
            roll, pitch = noisy(roll, noise, -180.0, 180.0), noisy(pitch, noise, -180.0, 180.0)
        baro_noise = noise * _BARO_NOISE_SCALE
        altitude = noisy(body.elev0 + body.alt, baro_noise, -100.0, 10000.0)
        elevation = noisy(body.alt, baro_noise, -100.0, 10000.0)
        agl_clean = max(0.0, body.alt)
        fields = {
            'accel': self.faults.apply('accel', accel, t),
            'rate': [noisy(rate, noise, -2000.0, 2000.0) for rate in rates],
            'attitude': None if self.drop_attitude and not self.separating(t)
            else self.faults.apply('attitude', [heading, roll, pitch], t),
            'altitude': self.faults.apply('baro', altitude, t),
            'elevation': self.faults.apply('baro', elevation, t),
        }
        if not self.args.no_mag:
            magnetic = math.radians(heading + _MAG_DECLINATION_DEG)
            fields['mag'] = self.faults.apply(
                'mag', [_MAG_UNIT * math.cos(magnetic), -_MAG_UNIT * math.sin(magnetic), _MAG_DOWN], t)
        if agl_clean <= self.args.laser_range_m:  # the laser sees the ground only within its range
            fields['agl'] = self.faults.apply('laser', noisy(agl_clean, noise, 0.0, 1000.0), t)
        if not self.drop_gnss and not self.faults.active('gnss', t):
            latitude, longitude = sim_model.noisy_position(body.position(), noise)
            fields['position'] = [round(latitude * 1e7), round(longitude * 1e7)]
            fields['speed'] = noisy(body.ground_speed(), noise, 0.0, 200.0)
            fields['course'] = noisy(body.track(), noise, 0.0, 360.0, sim_model.HEADING_NOISE_REF)
            if stack is not None:  # the stack moves over the ground: its velocity plus the receiver's drift
                east, north = stack.velocity[0] + body.gnss_drift_e, stack.velocity[1] + body.gnss_drift_n
                fields['speed'] = noisy(math.hypot(east, north), noise, 0.0, 200.0)
                fields['course'] = noisy(math.degrees(math.atan2(east, north)) % 360.0, noise, 0.0, 360.0,
                                         sim_model.HEADING_NOISE_REF)
        pitot = min((body.speed * body.speed + body.vu * body.vu) ** 0.5, _PITOT_RAIL_MS)
        if stack is not None:  # the stack's speed through the air
            air = [stack.velocity[index] - stack.wind[index] for index in range(3)]
            pitot = min(math.sqrt(sum(component * component for component in air)), _PITOT_RAIL_MS)
        pitot = self.faults.apply('pitot', noisy(pitot, noise, 0.0, _PITOT_RAIL_MS), t)
        if pitot is not None:
            fields['airspeed'] = pitot
            fields['dynamic_pressure'] = 0.5 * sim_model.RHO * pitot * pitot
        self.seq += 1
        clock = self.clock if self.clock is not None else t
        try:
            self.socket.sendto(encode_sensors(self.seq, int(clock * 1000), fields), self.address)
        except OSError:
            pass  # a refused send (no route yet) is a lost frame, as the link allows
        return fields

    def lockstep(self, pad_s: float) -> int:
        """
        The flight frame by frame (M14): each fixed physics step is one frame, and the next waits for its answer, so
        the run is the same every time and as fast as the flight code answers. The pad holds `pad_s`, then the boost
        or the glide follows the answered stage as the realtime loop's does. Prints the realtime run's lines.

        Returns:
            0 on DONE, 1 on a timeout or a stalled answer.
        """
        fixed = 1.0 / self.args.sim_hz
        self.socket.setblocking(True)
        self.socket.settimeout(_LOCKSTEP_WAIT_S)
        self.clock = 0.0
        pad_steps, steps, last_stage = round(pad_s / fixed), 0, None
        # the realtime run's drops, timed in sim time from GLIDING (M14): the attitude's for good, the GNSS's for
        # gnss_drop_s from 2-6 s in (seeded); a reboot needs a warm start and stays realtime-only
        drop_attitude_at = drop_gnss_at = recover_gnss_at = None
        while True:
            stage = self.stage()
            if stage != last_stage:
                print('STAGE', stage, _STAGES.get(stage))
                self.row('sequencer', _STAGES.get(stage, str(stage)))
                if stage == _GLIDING and self.args.attitude_drop_s > 0:
                    drop_attitude_at = self.sim_time + self.args.attitude_drop_s
                if stage == _GLIDING and self.args.gnss_drop_s > 0:
                    drop_gnss_at = self.sim_time + 2.0 + random.random() * 4.0
                last_stage = stage
            if drop_attitude_at is not None and self.sim_time >= drop_attitude_at and not self.drop_attitude:
                self.drop_attitude = True
                print('ATTITUDE DROP: sim attitude off, the attitude filter carries the glide')
            if drop_gnss_at is not None and self.sim_time >= drop_gnss_at and recover_gnss_at is None:
                self.drop_gnss = True
                recover_gnss_at = self.sim_time + self.args.gnss_drop_s
                print('GNSS DROP: position/speed/course out for %.1fs' % self.args.gnss_drop_s)
            if recover_gnss_at is not None and self.drop_gnss and self.sim_time >= recover_gnss_at:
                self.drop_gnss = False
                print('GNSS RECOVER: fix reacquired')
            if stage >= _DONE:
                print('DONE')
                return 0
            if self.sim_time > _TIMEOUT_S:
                print('TIMEOUT', stage)
                return 1
            if not self.ignited and steps >= pad_steps:
                self.ignited = True
            if not self.ignited:
                self.body.accel_g = 1.0
            else:
                boosting = stage < _GLIDING
                roll, pitch, yaw = self.fins()
                if not boosting and not self.body.gliding:
                    self.body.begin_glide()
                if boosting:
                    self.boost(fixed, (roll, pitch, yaw))
                else:
                    spiked = roll * 2.0 if (self.args.spike and random.random() < fixed / 3.0) else roll
                    self.body.glide_step(fixed, spiked, pitch, yaw)
                self.sim_time += fixed
            self.clock += fixed
            fields = self.publish()
            if not self.await_answer(self.seq):
                print('LOCKSTEP STALL: no answer to frame %d in %.0f s' % (self.seq, _LOCKSTEP_WAIT_S))
                return 1
            self.record(fields)
            steps += 1

    def await_answer(self, seq: int) -> bool:
        """Waits for the answer to frame `seq`; whether it came."""
        while True:
            try:
                frame, _ = self.socket.recvfrom(64)
            except (socket.timeout, OSError):
                return False
            answer = self.take(frame)
            if answer is not None and answer['seq'] == seq:
                return True

    def row(self, stream: str, *cells: object) -> None:
        """One `--record` row of `stream` at the frames' clock."""
        self.rows.setdefault(stream, []).append((self.clock or 0.0, *cells))

    def record(self, fields: dict) -> None:
        """The `--record` rows of one answered frame: what it carried, and the left fin the answer commanded."""
        if 'elevation' in fields and fields['elevation'] is not None:
            self.row('baro_icp10111', fields['elevation'])
        if fields.get('accel') is not None:
            self.row('imu_lsm6dso32', *fields['accel'])
        if fields.get('airspeed') is not None:
            self.row('airspeed_sdp810', fields['airspeed'] * 100.0)
        if fields.get('position') is not None:
            self.row('gnss', fields['position'][0] * 1e-7, fields['position'][1] * 1e-7)
        if self.answer is not None and self.answer['fins'] is not None:
            self.row('servo_eleron_left', self.answer['fins'][0])

    def write_record(self, folder: str) -> None:
        """The `--record` rows as a Rust cut's streams (`<folder>/flight/<stream>.csv`, t_s first, ';')."""
        columns = {'sequencer': ('stage',), 'baro_icp10111': ('elevation',), 'imu_lsm6dso32': ('ax', 'ay', 'az'),
                   'airspeed_sdp810': ('airspeed_cms',), 'gnss': ('lat', 'lon'), 'servo_eleron_left': ('angle',)}
        os.makedirs(os.path.join(folder, 'flight'), exist_ok=True)
        for stream, header in columns.items():
            with open(os.path.join(folder, 'flight', stream + '.csv'), 'w') as handle:
                handle.write(';'.join(('t_s',) + header) + '\n')
                for t, *cells in self.rows.get(stream, []):
                    handle.write(';'.join(['%.3f' % t] + [str(cell) for cell in cells]) + '\n')


def _panic(port: str) -> None:
    """The console's `panic kind=rust confirm`: the board resets before it could reply, so no answer is awaited."""
    console = transports.Serial(port)
    try:
        console.exchange('panic kind=rust confirm')
    except (transports.NoAnswer, OSError):
        pass
    finally:
        console.close()


def _arguments() -> argparse.Namespace:
    """The command line."""
    parser = argparse.ArgumentParser(description='HITL with the physics on the host (doc/specs/sim-link.md)')
    parser.add_argument('--ip', help="the board's address on the panda net (not with --sitl)")
    parser.add_argument('--serial', default='/dev/ttyACM0', help='the board console (default /dev/ttyACM0): CC '
                        'without --hub, and the panic of --reboot')
    parser.add_argument('--hub', help="drive CC through the hub by this board id (preferred: see the notes)")
    parser.add_argument('--motor', default='F15', choices=sorted(sim_model.MOTORS))
    parser.add_argument('--noise', type=float, default=0.0)
    parser.add_argument('--wind', type=float, default=0.0)
    parser.add_argument('--wind-dir', type=float, default=0.0)
    parser.add_argument('--spike', action='store_true')
    parser.add_argument('--glider-g', type=int, default=285)
    parser.add_argument('--boost-cda', type=float, help="override the airframe's boost CdA, m^2 (a drag sweep)")
    parser.add_argument('--airframe', default='tms7', choices=sorted(sim_model.AIRFRAMES),
                        help="the physics' airframe (sim_model.AIRFRAMES); the board's flight entry gets its "
                             'glide_ratio and stall_speed_1g')
    parser.add_argument('--sim-hz', type=int, default=50)
    parser.add_argument('--inject-hz', type=int, default=0, help='the publish rate; 0 = sim_hz')
    parser.add_argument('--boost-axis', default='z', choices=('x', 'y', 'z'))
    parser.add_argument('--boost', default='vertical', choices=('vertical', '6dof'),
                        help="the boost's physics: sim_model's vertical climb, or the 6-DoF stack (F15 only; "
                             'doc/specs/boost-control.md BC1)')
    parser.add_argument('--stack', default='tms7d', choices=sorted(stack_dynamics.STACKS),
                        help='--boost 6dof: the stack (tools/stack_dynamics.STACKS)')
    parser.add_argument('--stack-margin', type=float, help='--boost 6dof: the stack\'s static margin in calibers '
                        '(its profile\'s otherwise; a sweep)')
    parser.add_argument('--boost-law', default='config', choices=('config', 'passive', 'hold', 'lean'),
                        help="BOOSTING's law over the board's config: passive (not a control stage), hold (plan B's), "
                             "lean (BC3); config (default) flies the config's")
    parser.add_argument('--boost-limit', type=int, help="BOOSTING's own ceiling on the fin cap, whole degrees "
                        "(the flight entry's boost_limit_deg); default: the config's")
    parser.add_argument('--stack-azimuth', type=float, help='--boost 6dof: the compass direction the stack\'s '
                        'dorsal side faces on the rod (default: the wind\'s, against the folded wings)')
    parser.add_argument('--laser-range-m', type=float, default=4.0)
    parser.add_argument('--pad-dwell-s', type=float, default=0.0)
    parser.add_argument('--gnss-drift', type=float, default=0.0)
    parser.add_argument('--gnss-drift-dir', type=float, default=0.0)
    parser.add_argument('--attitude-drop-s', type=float, default=0.0)
    parser.add_argument('--no-attitude', action='store_true',
                        help="publish no attitude at all: the board's filter flies from power-up")
    parser.add_argument('--gnss-drop-s', type=float, default=0.0)
    parser.add_argument('--no-mag', action='store_true')
    parser.add_argument('--faults', default='', help="sim_model.Faults: 'gnss@30,pitot@45'")
    parser.add_argument('--reboot', action='store_true', help='a real panic 1-6 s into the glide (the console)')
    parser.add_argument('--keep-config', action='store_true', help='leave the HITL config installed')
    parser.add_argument('--clean', action='store_true', help='clear the card after the reboot: every session but '
                        'the run\'s own goes, for good (tools/board_clean.py)')
    parser.add_argument('--video-segment', type=int, help='--video: its segment length in s (the board\'s 300 '
                        'when not given); a short one on a --pad-dwell-s pad shows the cuts stop at the launch')
    parser.add_argument('--no-power', action='store_true', help='disable the INA226: the fins are commanded and '
                        'recorded as ever, arm\'s servo probe passes open-loop (a board whose servo rail is off)')
    parser.add_argument('--video', action='store_true', help='record video through the flight: `video start` '
                        'before the arm (the rod trigger never fires in HITL: no dwell, the sim\'s boost axis)')
    parser.add_argument('--sitl', help='fly the SITL binary at this path instead of a board: lockstep, no CC')
    parser.add_argument('--config', help='--sitl: the board config JSON it runs (a board\'s saved config)')
    parser.add_argument('--sitl-port', type=int, default=SIM_PORT + 10, help='--sitl: its UDP port on localhost')
    parser.add_argument('--seed', type=int, help='seed the noise and the spikes (a lockstep run then repeats)')
    parser.add_argument('--record', help='--sitl: write the flown streams here as a Rust cut (<dir>/flight/)')
    parser.add_argument('--servo-step', type=int, help="every fin's step_deg, whole degrees: a grid about the neutral "
                        '(over 1), any change (1), every command (0); the config\'s own when not given')
    return parser.parse_args()


def landing(position: tuple, zone: list) -> tuple:
    """
    Where the body came down against the landing zone, by the physics itself (no receiver noise): (metres from the
    zone's centre, inside the zone's rectangle).

    Args:
        position - the body's (lat, lon) at DONE.
        zone - the scenario's zone, [[lat, lon] top-left, [lat, lon] bottom-right].
    """
    (top, left), (bottom, right) = zone
    centre_lat, centre_lon = (top + bottom) / 2.0, (left + right) / 2.0
    north = (position[0] - centre_lat) * sim_model.commons.M_PER_DEG
    east = (position[1] - centre_lon) * sim_model.commons.M_PER_DEG * math.cos(math.radians(centre_lat))
    inside = bottom <= position[0] <= top and left <= position[1] <= right
    return math.hypot(north, east), inside


def _sitl(args: argparse.Namespace) -> int:
    """
    One scenario against the SITL (M14): the board config made HITL as a board's is, the mission the host seeds a
    board with, the binary started on localhost, flown lockstep; 0 on DONE, 1 otherwise.
    """
    if args.reboot:
        raise SystemExit('--sitl flies no reboot: it needs a warm start of the binary (a realtime event)')
    with open(args.config) as handle:
        saved = json.load(handle)
    scenario = sim_model.HPRC
    folder = tempfile.mkdtemp(prefix='sitl_')
    config, mission = os.path.join(folder, 'board.json'), os.path.join(folder, 'launch.json')
    with open(config, 'w') as handle:
        json.dump(hitl_config(saved, set(), args), handle)
    with open(mission, 'w') as handle:
        json.dump({'latitude': scenario['launch'][0], 'longitude': scenario['launch'][1],
                   'altitude': scenario['elevation_m'], 'zone': scenario['zone']}, handle)
    with open(os.path.join(folder, 'sitl.log'), 'w') as log:
        process = subprocess.Popen([args.sitl, '--config', config, '--mission', mission,
                                    '--port', str(args.sitl_port)], stdout=log, stderr=log)
    try:
        args.ip = '127.0.0.1'
        sim = _Sim(args)
        sim.address = (args.ip, args.sitl_port)
        time.sleep(0.5)  # the binary binds its socket
        print('SESSION sitl', args.motor, 'noise', args.noise, 'wind', args.wind)
        pad_s = _FILTER_PAD_S if args.no_attitude else _SITL_PAD_S
        outcome = sim.lockstep(pad_s)
        print('CLOCK sim=%.2f frames=%d answers=%d' % (sim.sim_time, sim.seq, sim.answers))
        # the scenario's score by the physics: the miss, the zone, the time from ignition to DONE
        miss, inside = landing(sim.body.position(), scenario['zone'])
        print('LANDED miss=%.1f inside=%s aloft=%.1f apogee=%.0f' % (miss, inside, sim.sim_time - pad_s,
                                                                      sim.apogee))
        if sim.stack is not None:
            burn = sim.burn
            print('BOOST path_min=%.1f tilt_max=%.1f looped=%s burnout=%.1f' % (
                burn['path_min'], burn['tilt_max'], burn['looped'],
                burn['burnout_ms'] if burn['burnout_ms'] is not None else -1.0))
        print('FINS %d of %d %s' % (sim.with_fins, sim.answers, ' '.join(
            '%s=%d..%d' % (name, low, high) for name, (low, high) in zip(('left', 'right', 'yaw'), sim.fin_range))))
        # the SITL's own tally at DONE (logged before its DONE answer): each fin's writes, the moves among them and
        # the degrees they moved it
        with open(os.path.join(folder, 'sitl.log')) as handle:
            tally = re.search(r'sitl :: servos \(writes/moves/degrees\) (.*)$', handle.read(), re.M)
        if tally:
            print('SERVOS %s' % tally.group(1))
        if args.record:
            sim.write_record(args.record)
        print('RUN_END')
        return outcome
    finally:
        process.terminate()
        process.wait()


def main() -> int:
    """One scenario; 0 on DONE, 1 otherwise."""
    args = _arguments()
    if args.boost == '6dof' and args.motor != 'F15':
        raise SystemExit('--boost 6dof flies the measured F15 only (tools/stack_dynamics.F15)')
    if args.seed is not None:
        random.seed(args.seed)
    if args.sitl:
        return _sitl(args)
    if not args.ip:
        raise SystemExit('--ip names the board (or fly the SITL with --sitl)')
    transport = transports.Hub(args.hub) if args.hub else transports.Serial(args.serial)
    board = _Board(transport)
    saved = board.ask('get-config saved')[1]
    launch = board.ask('get-config launch')[1]
    # the board's own configs, on the host until the restore: a run that dies before it (a board that does not come
    # back from the HITL reboot) leaves the HITL copy saved, and the next run takes the originals from here
    backup = os.path.join(tempfile.gettempdir(), 'coludo_hitl_saved_%s.json' % (args.hub or args.ip))
    if any(entry.get('activity') == 'hitl' for entry in saved.get('components', [])):
        if not os.path.exists(backup):
            raise SystemExit('the saved config is a HITL copy (a run never restored it) and %s is missing' % backup)
        with open(backup) as stream:
            saved, launch = (json.load(stream)[key] for key in ('board', 'launch'))
        print('RESTORING the configs a run left unrestored, from %s' % backup)
    else:
        with open(backup, 'w') as stream:
            json.dump({'board': saved, 'launch': launch}, stream)
    problems = board.ask('verify')[1].get('problems', {})
    not_ported = {name for name, line in problems.items() if 'not ported yet' in str(line)}
    status, answer = board.ask(cc.build('set-config', ['board', json.dumps(hitl_config(saved, not_ported, args))]))
    if status != 'ok':
        raise SystemExit('the HITL config was refused: %s' % answer)
    scenario = sim_model.HPRC  # the nav target, as plan B hitl.py seeded the mission
    board.ask(cc.build('update', ['mission', json.dumps({'latitude': scenario['launch'][0],
                                                         'longitude': scenario['launch'][1],
                                                         'altitude': scenario['elevation_m'],
                                                         'zone': scenario['zone']})]))
    outcome = 1
    sim = None
    try:
        whoami = board.reboot()
        sim = _Sim(args)
        if not sim.gate():
            print('GATE FAILED: no answer on udp %s:%d in %.0f s' % (args.ip, SIM_PORT, _GATE_S))
            return 1
        print('SESSION', whoami.get('session'), args.motor, 'noise', args.noise, 'wind', args.wind)
        if args.clean:
            removed = board_clean.clean(board, whoami.get('session'))
            print('CLEANED %d %s' % (len(removed), board.raw('df')))
        if args.video:
            segment = '' if args.video_segment is None else ' segment=%d' % args.video_segment
            print('VIDEO START %s' % board.raw('video start' + segment))
        flyer = threading.Thread(target=sim.run, daemon=True)
        flyer.start()
        status, armed = board.ask('arm')
        if status != 'ok':
            print('ARM REFUSED:', json.dumps(armed))
            sim.running = False
            return 1
        time.sleep(args.pad_dwell_s)
        sim.ignited = True
        started = time.monotonic()
        last_stage, last_trace = None, started
        drop_attitude_at = drop_gnss_at = recover_gnss_at = reboot_at = None
        rebooted = False
        while True:
            stage = sim.stage()
            now = time.monotonic()
            if stage != last_stage:
                print('STAGE', stage, _STAGES.get(stage))
                if stage == _GLIDING and args.attitude_drop_s > 0:
                    drop_attitude_at = now + args.attitude_drop_s
                if stage == _GLIDING and args.gnss_drop_s > 0:
                    drop_gnss_at = now + 2.0 + random.random() * 4.0
                if stage == _GLIDING and args.reboot and reboot_at is None:
                    reboot_at = now + 1.0 + random.random() * 5.0  # early-to-mid glide, as plan B's
                last_stage = stage
            if drop_attitude_at is not None and now >= drop_attitude_at and not sim.drop_attitude:
                sim.drop_attitude = True
                print('ATTITUDE DROP: sim attitude off, the attitude filter carries the glide')
            if drop_gnss_at is not None and now >= drop_gnss_at and recover_gnss_at is None:
                sim.drop_gnss = True
                recover_gnss_at = now + args.gnss_drop_s
                print('GNSS DROP: position/speed/course out for %.1fs' % args.gnss_drop_s)
            if recover_gnss_at is not None and sim.drop_gnss and now >= recover_gnss_at:
                sim.drop_gnss = False
                print('GNSS RECOVER: fix reacquired')
            if reboot_at is not None and not rebooted and now >= reboot_at and stage == _GLIDING:
                rebooted = True
                print('REBOOT: a real panic mid-glide (the console); the sim flies on')
                _panic(args.serial)
                whoami = board.ask('whoami')[1]
                gate = board.ask('inspect checkpoint')[1].get('gate')
                health = board.ask('health')[1]
                restored = health.get('stage') in ('gliding', 'landing') and bool(health.get('armed'))
                print('WARM GATE:', restored, gate)
                print('REBOOTED', whoami.get('session'), 'boot', whoami.get('boot_id'))
                continue
            if stage >= _DONE:
                print('DONE')
                outcome = 0
                break
            if now - started > _TIMEOUT_S:
                print('TIMEOUT', stage)
                break
            if stage == _GLIDING and now - last_trace >= 5.0:
                last_trace = now
                body = sim.body
                print('  t=%ds alt=%.0f roll=%.0f pitch=%.0f hdg=%.0f dropgnss=%s answers=%d'
                      % (now - started, body.alt, body.roll, body.pitch, body.heading, sim.drop_gnss, sim.answers))
            time.sleep(0.2)
        time.sleep(1.2)  # the recorder's tail
        sim.running = False
        wall = time.monotonic() - (sim.wall_start or started)
        print('CLOCK sim=%.2f wall=%.2f lag=%.2f frames=%d answers=%d'
              % (sim.sim_time, wall, wall - sim.sim_time, sim.seq, sim.answers))
        print('FINS %d of %d %s' % (sim.with_fins, sim.answers, ' '.join(
            '%s=%d..%d' % (name, low, high) for name, (low, high) in zip(('left', 'right', 'yaw'), sim.fin_range))))
        # the physics' score, as the SITL prints it: realtime, the sim's clock runs from ignition and stops at DONE
        miss, inside = landing(sim.body.position(), scenario['zone'])
        print('LANDED miss=%.1f inside=%s aloft=%.1f apogee=%.0f' % (miss, inside, sim.sim_time, sim.apogee))
        if sim.stack is not None:
            burn = sim.burn
            print('BOOST path_min=%.1f tilt_max=%.1f looped=%s burnout=%.1f' % (
                burn['path_min'], burn['tilt_max'], burn['looped'],
                burn['burnout_ms'] if burn['burnout_ms'] is not None else -1.0))
        # the control tick's figures over the flight (its counters live until the restore's reboot below): rate,
        # overruns, the largest lateness and work -- how fast the board reacts; and the card's: the video writer's,
        # the ring_saver's and the free space (the partition A/B, avionics-port.md "SD and flash file layout")
        for label, line in (('CONTROL', 'control'), ('VIDEO', 'video stats'), ('RING', 'ring'), ('DF', 'df')):
            try:
                print('%s %s' % (label, board.raw(line)))
            except (transports.NoAnswer, OSError) as error:
                print('%s none (%s)' % (label, error))
        print('RUN_END')
    finally:
        if not args.keep_config:
            if sim is not None:
                sim.running = False
            time.sleep(_HELD_S)
            for _ in range(_SETTING_TRIES):
                board.ask('disarm')
                board.ask('stage setting')  # set-config is SETTING-only: back from DONE, or from a timeout's stage
                if board.ask('health')[1].get('stage') == 'setting':
                    break
                time.sleep(_HELD_S)
            board.ask(cc.build('set-config', ['board', json.dumps(saved)]))
            board.ask(cc.build('set-config', ['launch', json.dumps(launch)]))
            board.reboot()
    return outcome


if __name__ == '__main__':
    sys.exit(main())
