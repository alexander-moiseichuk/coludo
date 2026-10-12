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
...` (how many answers carried fins, and each fin's angle range), `RUN_END`.

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
import socket
import subprocess
import sys
import tempfile
import threading
import time

_ROOT: str = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_ROOT, 'src', 'glider'))
sys.path.insert(0, os.path.join(_ROOT, 'src', 'control'))
import cc_protocol as cc  # noqa: E402
import config_hitl  # noqa: E402
import sim_model  # noqa: E402
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
_SITL_PAD_S: float = 2.0  # the SITL's pad before ignition, physics seconds: the stage machine's windows fill
_LOCKSTEP_WAIT_S: float = 2.0  # how long a lockstep frame waits for its answer before the run stalls
_GATE_S: float = 120.0  # how long the board gets to join Wi-Fi and answer a frame
_QUIET_S: float = 3.0  # after an answer that is no reply: the board finishes a long one (get-config, ~0.8 s)
_OFF: tuple = ('separation', 'watchdog', 'bluetooth', 'field')  # plan B's _OFF less the radios the link rides


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
        self.drop_attitude = False
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
                    thrust = self.thrust if self.sim_time < self.burn_s else 0.0
                    self.body.boost_step(fixed, thrust, pitch, roll)
                else:
                    spiked = roll * 2.0 if (self.args.spike and random.random() < fixed / 3.0) else roll
                    self.body.glide_step(fixed, spiked, pitch, yaw)
                accumulator -= fixed
                self.sim_time += fixed
            self.publish()

    def publish(self) -> None:
        """The noised sensors as one SENSORS frame (plan B hitl.py _publish), in the Rust databoard's units."""
        body, noise, t = self.body, self.args.noise, self.sim_time
        noisy = sim_model.noisy
        accel = [0.0, 0.0, 0.0]
        accel[self.axis] = noisy(body.accel_g, noise, -200.0, 200.0)
        heading = noisy(body.heading % 360.0, noise, 0.0, 360.0, sim_model.HEADING_NOISE_REF)
        roll = noisy(body.roll, noise, -180.0, 180.0)
        pitch = noisy(body.pitch, noise, -180.0, 180.0)
        baro_noise = noise * _BARO_NOISE_SCALE
        altitude = noisy(body.elev0 + body.alt, baro_noise, -100.0, 10000.0)
        elevation = noisy(body.alt, baro_noise, -100.0, 10000.0)
        agl_clean = max(0.0, body.alt)
        fields = {
            'accel': self.faults.apply('accel', accel, t),
            'rate': [noisy(body.roll_rate, noise, -2000.0, 2000.0), noisy(body.pitch_rate, noise, -2000.0, 2000.0),
                     noisy(body.yaw_rate, noise, -2000.0, 2000.0)],
            'attitude': None if self.drop_attitude else self.faults.apply('attitude', [heading, roll, pitch], t),
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
        pitot = min((body.speed * body.speed + body.vu * body.vu) ** 0.5, _PITOT_RAIL_MS)
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
        while True:
            stage = self.stage()
            if stage != last_stage:
                print('STAGE', stage, _STAGES.get(stage))
                self.row('sequencer', _STAGES.get(stage, str(stage)))
                last_stage = stage
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
                    thrust = self.thrust if self.sim_time < self.burn_s else 0.0
                    self.body.boost_step(fixed, thrust, pitch, roll)
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
    parser.add_argument('--airframe', default='tms7', choices=sorted(sim_model.AIRFRAMES),
                        help="the physics' airframe (sim_model.AIRFRAMES); the board's flight entry gets its "
                             'glide_ratio and stall_speed_1g')
    parser.add_argument('--sim-hz', type=int, default=50)
    parser.add_argument('--inject-hz', type=int, default=0, help='the publish rate; 0 = sim_hz')
    parser.add_argument('--boost-axis', default='z', choices=('x', 'y', 'z'))
    parser.add_argument('--laser-range-m', type=float, default=4.0)
    parser.add_argument('--pad-dwell-s', type=float, default=0.0)
    parser.add_argument('--gnss-drift', type=float, default=0.0)
    parser.add_argument('--gnss-drift-dir', type=float, default=0.0)
    parser.add_argument('--attitude-drop-s', type=float, default=0.0)
    parser.add_argument('--gnss-drop-s', type=float, default=0.0)
    parser.add_argument('--no-mag', action='store_true')
    parser.add_argument('--faults', default='', help="sim_model.Faults: 'gnss@30,pitot@45'")
    parser.add_argument('--reboot', action='store_true', help='a real panic 1-6 s into the glide (the console)')
    parser.add_argument('--keep-config', action='store_true', help='leave the HITL config installed')
    parser.add_argument('--sitl', help='fly the SITL binary at this path instead of a board: lockstep, no CC')
    parser.add_argument('--config', help='--sitl: the board config JSON it runs (a board\'s saved config)')
    parser.add_argument('--sitl-port', type=int, default=SIM_PORT + 10, help='--sitl: its UDP port on localhost')
    parser.add_argument('--seed', type=int, help='seed the noise and the spikes (a lockstep run then repeats)')
    parser.add_argument('--record', help='--sitl: write the flown streams here as a Rust cut (<dir>/flight/)')
    return parser.parse_args()


def _sitl(args: argparse.Namespace) -> int:
    """
    One scenario against the SITL (M14): the board config made HITL as a board's is, the mission the host seeds a
    board with, the binary started on localhost, flown lockstep; 0 on DONE, 1 otherwise.
    """
    if args.reboot or args.attitude_drop_s or args.gnss_drop_s:
        raise SystemExit('--sitl flies neither a reboot nor a timed drop: they are realtime events')
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
        outcome = sim.lockstep(_SITL_PAD_S)
        print('CLOCK sim=%.2f frames=%d answers=%d' % (sim.sim_time, sim.seq, sim.answers))
        print('FINS %d of %d %s' % (sim.with_fins, sim.answers, ' '.join(
            '%s=%d..%d' % (name, low, high) for name, (low, high) in zip(('left', 'right', 'yaw'), sim.fin_range))))
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
    try:
        whoami = board.reboot()
        sim = _Sim(args)
        if not sim.gate():
            print('GATE FAILED: no answer on udp %s:%d in %.0f s' % (args.ip, SIM_PORT, _GATE_S))
            return 1
        print('SESSION', whoami.get('session'), args.motor, 'noise', args.noise, 'wind', args.wind)
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
        # the control tick's figures over the flight (its counters live until the restore's reboot below): rate,
        # overruns, the largest lateness and work -- how fast the board reacts
        try:
            print('CONTROL %s' % board.raw('control'))
        except (transports.NoAnswer, OSError) as error:
            print('CONTROL none (%s)' % error)
        print('RUN_END')
    finally:
        if not args.keep_config:
            board.ask('disarm')
            board.ask('stage setting')  # set-config is SETTING-only: back from DONE, or from a timeout's stage
            board.ask(cc.build('set-config', ['board', json.dumps(saved)]))
            board.ask(cc.build('set-config', ['launch', json.dumps(launch)]))
            board.reboot()
    return outcome


if __name__ == '__main__':
    sys.exit(main())
