# virtual_flight.py — fly a complete Coludo mission on the HOST and emit a recorder capture (6/23
# global4). It runs the SAME closed loop the board runs in HITL, but in CPython: the shared flight model
# (src/glider/sim_model.Body) is driven by the REAL control code (navigation + pid + mixer) under the
# REAL config (config_hitl), through the same stage machine thresholds the sequencer uses. Each control
# tick reads NOISE-degraded attitude/accel (the `--noise` knob, same sim_model.noisy as the board) so
# you can see how the loop holds the zone when the sensors are clean (5 %) vs ratty (50 %). The output is
# the exact wire format flight_telemetry.parse() reads, so it renders with flight_report.py -- a virtual
# flight movie before any real one. The trajectory (position) is the body's TRUE path, which already
# reflects the noisy control, so a degraded run visibly wanders / spirals over the zone.
#
#   python3 virtual_flight.py --motor F15 --noise 0.05 -o clean.txt
#   python3 virtual_flight.py --motor F15 --noise 0.50 -o ratty.txt
#   python3 flight_report.py clean.txt -o clean.html      # pip install plotly

import argparse
import math
import os
import sys

_GLIDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'glider')
sys.path.insert(0, _GLIDER)

import config_hitl  # noqa: E402  -- the SAME board config the on-board HITL uses (host-importable)
import mixer  # noqa: E402
import navigation  # noqa: E402
import pid  # noqa: E402
import sim_model  # noqa: E402

_FINS = ('servo_eleron_left', 'servo_eleron_right', 'servo_yaw')


def _heading_error(target: float, current: float) -> int:
    """Mirror flight.Flight._heading_error: shortest signed heading error, integer degrees."""
    error = int(target - current)
    return error if -180 <= error <= 180 else (error + 180) % 360 - 180


def _component(cfg: dict, name: str) -> dict:
    return next(c for c in cfg['components'] if c['name'] == name)


def fly(motor: str, noise: float, spike: bool, sim_hz: int, seconds: float,
        wind: float = 0.0, wind_dir: float = 0.0) -> str:
    """Run the closed loop and return a recorder capture (text). Reuses config_hitl so the gains, mixer,
    sequencer thresholds and scenario are byte-for-byte what the board flies."""
    cfg = config_hitl.default(motor=motor, noise=noise, spike=spike)
    flight_c = _component(cfg, 'flight')
    seq_c = _component(cfg, 'sequencer')
    hitl_c = _component(cfg, 'hitl')

    scenario = dict(sim_model.HPRC)
    scenario.update(hitl_c.get('scenario', {}))
    zone = scenario['zone']
    launch_g = seq_c.get('launch_g', 3.0)
    launch_ms = seq_c.get('launch_ms', 100)
    boost_timeout_ms = seq_c.get('boost_timeout_ms', 6000)
    land_agl_m = seq_c.get('land_agl_m', 5.0)
    land_ms = seq_c.get('land_ms', 300)
    laser_range_m = hitl_c.get('laser_range_m', 4.0)
    thrust, burn_s = sim_model.MOTORS[motor]

    body = sim_model.Body(hitl_c.get('liftoff_g', 430) / 1000.0,
                          tuple(scenario['launch']), scenario['elevation_m'], scenario['heading_deg'])
    body.wind_e = wind * math.sin(math.radians(wind_dir))   # steady wind the glider must crab against
    body.wind_n = wind * math.cos(math.radians(wind_dir))
    mix = mixer.Mixer(cfg.get('mixer', {}))
    gains = flight_c.get('gains', {})
    stages = flight_c.get('stages', {'gliding': {'roll': 0.0, 'pitch': 0.0}})
    bank_gain = flight_c.get('nav_bank_gain', 1.5)   # bank-to-turn (mirror tasks/flight.py)
    bank_limit = flight_c.get('bank_limit', 30)
    pids = {axis: pid.Pid(output_limit=mix.limit, integral_limit=mix.limit, **gains.get(axis, {}))
            for axis in ('roll', 'pitch', 'yaw')}

    dt = 1.0 / sim_hz
    stage = 'setting'
    since = 0.0          # time the current sustained-detect window started
    rows = _Capture()
    rows.header()

    t = 0.0
    while t < seconds:
        sensors = body.sensors()
        # NOISE-degraded readings -- what the control loop and the recorder actually see (board parity:
        # accel/attitude/altitude/agl are noised; GNSS position is not -- see tasks/hitl._publish).
        accel_m = sim_model.noisy(sensors['accel'], noise, -200.0, 200.0)
        heading_m = sim_model.noisy(sensors['heading'], noise, 0.0, 360.0)
        roll_m = sim_model.noisy(sensors['roll'], noise, -180.0, 180.0)
        pitch_m = sim_model.noisy(sensors['pitch'], noise, -180.0, 180.0)
        altitude_m = sim_model.noisy(sensors['altitude'], noise, -100.0, 10000.0)
        agl = sensors['agl']

        # --- stage machine (mirrors tasks/sequencer.py; separation off -> boost timeout drives glide) ---
        if stage == 'setting':
            if accel_m > launch_g:
                if (t - since) * 1000.0 >= launch_ms:
                    stage, since = 'boosting', t
                    rows.event(t, 'controller :: stage -> boosting')
            else:
                since = t
        elif stage == 'boosting':
            if (t - since) * 1000.0 >= boost_timeout_ms:
                stage, since = 'gliding', t
                body.begin_glide()
                for controller in pids.values():
                    controller.reset()
                rows.event(t, 'controller :: stage -> gliding')
        elif stage == 'gliding':
            if agl < land_agl_m:
                if (t - since) * 1000.0 >= land_ms:
                    stage, since = 'landing', t
                    rows.event(t, 'controller :: stage -> landing')
            else:
                since = t
        elif stage == 'landing':
            if abs(accel_m - 1.0) < 0.3 and (t - since) * 1000.0 >= seq_c.get('ground_ms', 3000):
                rows.event(t, 'controller :: stage -> done')
                break

        # --- control law (mirrors flight._step): only the configured control stages actuate ---
        setpoint = stages.get(stage)
        fins = (mix.neutral, mix.neutral, mix.neutral)   # commanded (left, right, yaw) -- neutral off-control
        if stage in ('setting', 'boosting'):
            body.boost_step(dt, thrust if t < burn_s else 0.0)
        elif setpoint is not None:                       # a control stage (gliding) -> PID -> mixer -> fins
            target = navigation.steer(sensors['position'], zone[0], zone[1])[0]  # tier-1: live fix
            heading_error = _heading_error(target, heading_m)
            roll_setpoint = setpoint.get('roll', 0.0)
            if bank_gain and stage == 'gliding':         # bank-to-turn toward the zone
                roll_setpoint = navigation.bank_demand(heading_error, bank_gain, bank_limit)
            roll_cmd = pids['roll'].step(roll_setpoint - roll_m, dt)
            pitch_cmd = pids['pitch'].step(setpoint.get('pitch', 0.0) - pitch_m, dt)
            yaw_cmd = pids['yaw'].step(heading_error, dt)
            angles = mix.mix(roll=round(roll_cmd), pitch=round(pitch_cmd), yaw=round(yaw_cmd))
            fins = tuple(angles[f] for f in _FINS)
            body.glide_step(dt, (fins[0] - fins[1]) / 2.0, (fins[0] + fins[1]) / 2.0 - 90.0, fins[2] - 90.0)
        else:                                            # non-control stage -> coast, fins neutral
            body.glide_step(dt, 0.0, 0.0, 0.0)

        rows.sample(t, accel_m, altitude_m, sensors['altitude'] - body.elev0, heading_m, roll_m, pitch_m,
                    sensors['position'], agl, laser_range_m, body.speed, fins)
        rows.health(t, stage)
        if body.gliding and body.alt <= 0.0:             # touched down
            rows.event(t, 'controller :: stage -> done')
            break
        t += dt
    return rows.text()


class _Capture:
    """Accumulate recorder telemetry lines in the @<session>_<file>@<row> wire format (one stream per
    real sensor file, matching tasks/hitl + the live drivers so flight_report keys on the same fields)."""

    _SESSION = '20260623_120000_000'

    def __init__(self):
        self._lines = []
        self._last_gnss = -1.0
        self._last_health = -1.0

    def _tlm(self, file: str, row: str) -> None:
        self._lines.append('@%s_%s@%s' % (self._SESSION, file, row))

    def header(self) -> None:
        self._tlm('accel_adxl375.csv', 'uptime;ax;ay;az')
        self._tlm('baro_icp10111.csv', 'uptime;altitude;temperature;pressure;elevation')
        self._tlm('imu_bno055.csv', 'uptime;heading;roll;pitch')
        self._tlm('gnss.csv', 'uptime;lat;lon;speed_kn;course')
        self._tlm('laser_agl.csv', 'uptime;agl')
        self._tlm('fins.csv', 'uptime;eleron_left;eleron_right;yaw')  # commanded servo angles (deg)
        self._tlm('health.csv', 'uptime;temp;mem_free;load')          # board vitals (board_health.py)

    def sample(self, t, accel, altitude, elevation, heading, roll, pitch, position, agl, laser_range, speed, fins):
        microseconds = int(t * 1e6)
        self._tlm('accel_adxl375.csv', '%u;0.000;0.000;%.3f' % (microseconds, accel))
        self._tlm('baro_icp10111.csv', '%u;%.2f;21.0;100000;%.2f' % (microseconds, altitude, elevation))
        self._tlm('imu_bno055.csv', '%u;%.1f;%.1f;%.1f' % (microseconds, heading, roll, pitch))
        self._tlm('fins.csv', '%u;%d;%d;%d' % (microseconds, fins[0], fins[1], fins[2]))
        if t - self._last_gnss >= 0.1:                   # GNSS ~10 Hz
            self._last_gnss = t
            self._tlm('gnss.csv', '%u;%.6f;%.6f;%.1f;%.1f'    # speed in knots (GPS convention)
                      % (microseconds, position[0], position[1], speed * 1.94384, heading))
        if agl <= laser_range:                           # the laser only resolves the last few metres
            self._tlm('laser_agl.csv', '%u;%.3f' % (microseconds, agl))

    def health(self, t, stage):
        """A 1 Hz board-vitals row (board_health.csv fields). SYNTHETIC + phase-modeled -- the host has no
        real MCU -- but shaped like the board would read: load tracks the work per stage (idle on the rod,
        high under boost sampling, steady in the glide loop, highest while the laser hammers I2C on
        landing); temperature drifts up under load; free memory stays consistent (the firmware
        pre-allocates and avoids churn, so GC is gentle -- only a shallow sawtooth around ~4 MB)."""
        if t - self._last_health < 1.0:
            return
        self._last_health = t
        load = {'setting': 5, 'boosting': 45, 'gliding': 30, 'landing': 60}.get(stage, 8)
        load = max(0, min(100, load + int(6 * math.sin(t * 2.5))))
        temp = min(63.0, 45.0 + 0.18 * t + (4.0 if stage == 'landing' else 0.0))
        mem_free = 4_190_000 - int((t * 1000) % 30000)   # ~4 MB, a shallow GC sawtooth (memory is steady)
        self._tlm('health.csv', '%u;%.1f;%d;%d' % (int(t * 1e6), temp, mem_free, load))

    def event(self, t, line: str) -> None:
        self._lines.append('%u %s' % (int(t * 1e6), line))

    def text(self) -> str:
        return '\n'.join(self._lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description='Fly a virtual Coludo mission and emit a recorder capture.')
    parser.add_argument('--motor', default='F15', choices=sorted(sim_model.MOTORS), help='motor (default F15)')
    parser.add_argument('--noise', type=float, default=0.05, help='sensor noise fraction N (default 0.05)')
    parser.add_argument('--spike', action='store_true', help='inject occasional 2x roll spikes (g16)')
    parser.add_argument('--wind', type=float, default=0.0, help='steady wind speed m/s (default 0)')
    parser.add_argument('--wind-dir', type=float, default=0.0, help='wind blows TOWARD this heading deg (default 0=N)')
    parser.add_argument('--hz', type=int, default=50, help='simulation rate (default 50)')
    parser.add_argument('--seconds', type=float, default=240.0, help='max flight time (default 240)')
    parser.add_argument('-o', '--out', help='write capture here (default stdout)')
    args = parser.parse_args()

    capture = fly(args.motor, args.noise, args.spike, args.hz, args.seconds, args.wind, args.wind_dir)
    if args.out:
        with open(args.out, 'w') as handle:
            handle.write(capture)
        sys.stderr.write('wrote %s (%d lines, %s @ noise %.0f%%, wind %.0f m/s)\n'
                         % (args.out, capture.count('\n'), args.motor, args.noise * 100, args.wind))
    else:
        sys.stdout.write(capture)


if __name__ == '__main__':
    main()
