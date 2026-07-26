"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Fly a complete Coludo mission on the HOST and emit a recorder capture. It runs the SAME closed loop the
board runs in HITL, but in CPython: the shared flight model (src/glider/sim_model.Body) is driven by the
REAL control code -- guidance.Guidance (per-stage law, GPS tiers, boost hold, final approach),
governor.Governor (estimated-airspeed fin-authority cap + adaptive throttle), pid.Pid and the fused
mixer.actuate() -- under the REAL config (config_hitl). The board's databoard/mission are stood in by
tiny injected handles; ONLY the sequencer's stage machine and the physics are mirrored here (the
sequencer is genuinely board-bound: databoard, recorder, gc policy). The old hand-mirrored control law +
`_run_pid` copy are GONE -- a control-law change lands in this tool automatically.

Each control tick reads NOISE-degraded attitude/accel (the `--noise` knob, same sim_model.noisy as the
board) so you can see how the loop holds the zone when the sensors are clean (5 %) vs ratty (50 %). Note
the governor now flies the ESTIMATED airspeed (accel backbone + GNSS corrector) like the board -- not the
sim's true airspeed. The output is the exact wire format flight_telemetry.parse() reads, so it renders
with flight_report.py -- a virtual flight movie before any real one.

python3 virtual_flight.py --motor F15 --noise 0.05 -o clean.txt
python3 virtual_flight.py --motor F15 --noise 0.50 -o ratty.txt
python3 flight_report.py clean.txt -o clean.html # pip install plotly
"""

import argparse
import math
import os
import random
import sys

_GLIDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'glider')
sys.path.insert(0, _GLIDER)

import config_hitl  # noqa: E402 -- the SAME board config the on-board HITL uses (host-importable)
import controller as controller_mod  # noqa: E402 -- Stage ids (host-importable)
import fixed  # noqa: E402 -- fixed-point convention: PID error/output in centidegree fixnum (board parity)
import governor  # noqa: E402 -- the REAL fin-authority governor (estimated airspeed + throttle)
import guidance  # noqa: E402 -- the REAL per-stage guidance law
import mixer  # noqa: E402
import navigation  # noqa: E402 -- zone geometry for the _Mission stub (memoized, mirrors mission.Mission)
import pid  # noqa: E402
import preflight  # noqa: E402 -- install + data-consistency gates, run before any flying
import sim_model  # noqa: E402

_STAGE = controller_mod.Stage
_FINS = ('servo_eleron_left', 'servo_eleron_right', 'servo_yaw')
_SPIKE_S = 3.0  # a transient 2x sensor glitch fires once every this many seconds (within 2-5 s)
_GNSS_S = 0.1  # GNSS fix cadence (~10 Hz), for both the injected handles and the capture rows
_LEAK_BPS = 15000.0   # measured GC-off control-path leak (B/s) -- doc/sims/TMS-7-guiding_refactoring
_FREE_AT_BOOT = 33_000_000  # MEASURED on the ESP32-P4 (gc.mem_free() = 33.09 MB of PSRAM). The old
# 4.19 MB guess was ~8x low, which -- with the sawtooth -- is why the report's time-to-OOM printed
# absurdities. 33 MB at 15 KB/s = ~37 min, matching the real board's measured time-to-OOM.
_SERVO_HOLD_MW = 41   # measured rail draw with the fins holding (INA226, 2026-07-25)
_SERVO_MOVE_MW = 1400  # measured MEAN draw of one servo in travel (peak 3925 mW = 0.79 A)
_SERVO_SLEW_S_PER_DEG = 0.1 / 60.0  # MG90S: ~0.1 s per 60 deg at max slew


class _Fin:
    """set_angle() stand-in for an sg90 driver: the fused mixer.actuate() writes the commanded angle
    here and the sim reads it back (same role the servo task plays on the board)."""

    def __init__(self, neutral: int):
        self.angle = neutral

    def set_angle(self, angle):
        self.angle = angle
        return angle


class _Handle:
    """Databoard-parameter stand-in: the sim publishes the live reading, the real governor/guidance
    consume it through the same value()/read() surface the board's Parameter offers."""

    def __init__(self):
        self.value_now = None
        self.source = None

    def value(self):
        return self.value_now

    def read(self):
        return (self.value_now, self.source, 0)  # age 0: the sim publishes fresh (board: databoard ages)


class _Mission:
    """Mission stand-in: the landing zone from the HITL scenario; no CC-set launch point, so the
    guidance tiers exercise tier 1 (live fix) and tier 3 (blind) exactly as a real flight would. The
    zone geometry getters mirror the real mission.Mission (memoized by zone identity)."""

    def __init__(self, zone):
        self.zone = zone
        self._zone_key = None
        self._zone_points = None
        self._zone_aspect = 1.0

    def launch_point(self):
        return None

    def zone_points(self):
        if self.zone is None:
            return None
        if self.zone is not self._zone_key:  # first call / zone replaced -> resolve + cache
            self._zone_key = self.zone
            self._zone_points = navigation.zone(self.zone[0], self.zone[1])
            self._zone_aspect = navigation.zone_aspect(self.zone[0], self.zone[1])
        return self._zone_points

    def zone_aspect(self):
        return 1.0 if self.zone_points() is None else self._zone_aspect

    def endgame_heading(self):
        aspect = self.zone_aspect()
        if aspect >= guidance.Heading.OO_ASPECT:
            return guidance.Heading.FIG_OO
        return guidance.Heading.FIG_OVAL if aspect >= guidance.Heading.OVAL_ASPECT else guidance.Heading.FIG_O


def _component(cfg: dict, name: str) -> dict:
    return next(c for c in cfg['components'] if c['name'] == name)


def fly(motor: str, noise: float, spike: bool, sim_hz: int, seconds: float,
        wind: float = 0.0, wind_dir: float = 0.0, final_agl_override: float = None,
        imbalance_pitch: float = 0.0, imbalance_roll: float = 0.0,
        endgame_alt_override: float = None) -> str:
    """
    Run the closed loop and return a recorder capture (text).

    Reuses config_hitl so the gains, mixer, sequencer thresholds and scenario are byte-for-byte what the
    board flies. Set VF_SEED to seed the sensor noise deterministically (A/B a control change on the
    SAME noise realisation).
    """
    _seed = os.environ.get('VF_SEED')
    if _seed is not None:
        random.seed(int(_seed))
    # glider (glide) mass in grams: 270 = the full build, 215 = the light build -- the two weight
    # variants every campaign flies. Env rather than an arg so a sweep script sets it per case.
    glider_g = int(os.environ.get('VF_GLIDER_G', 0)) or None
    cfg = (config_hitl.default(motor=motor, noise=noise, spike=spike, glider_g=glider_g) if glider_g
           else config_hitl.default(motor=motor, noise=noise, spike=spike))
    flight_c = _component(cfg, 'flight')
    _endgame = os.environ.get('VF_ENDGAME')
    if _endgame:
        flight_c['endgame_pattern'] = _endgame  # A/B the endgame pattern (o / oo / auto) from the shell
    seq_c = _component(cfg, 'sequencer')
    hitl_c = _component(cfg, 'hitl')

    scenario = dict(sim_model.HPRC)
    scenario.update(hitl_c.get('scenario', {}))
    zone = scenario['zone']
    launch_g = seq_c.get('launch_g', 3.0)
    launch_ms = seq_c.get('launch_ms', 100)
    boost_timeout_ms = seq_c.get('boost_timeout_ms', 6000)
    apogee_arm_ms = seq_c.get('apogee_arm_ms', 4000)  # the detector is blind through the burn
    apogee_drop_m = seq_c.get('apogee_drop_m', 5.0)
    apogee_max = None  # baro peak tracking (one boost per run, no reset needed)
    apogee_since = None
    land_agl_m = seq_c.get('land_agl_m', 5.0)
    land_ms = seq_c.get('land_ms', 300)
    laser_range_m = hitl_c.get('laser_range_m', 4.0)
    thrust, burn_s = sim_model.MOTORS[motor]

    body = sim_model.Body(hitl_c.get('liftoff_g', 430) / 1000.0,
                          tuple(scenario['launch']), scenario['elevation_m'], scenario['heading_deg'])
    body.trim_sink = 14.0 / float(os.environ.get('VF_QUALITY', 2.0))  # air-quality (L/D) sink: 2 = worst-case floor
    # robustness knobs (findings §27.20/§27.21); both default OFF so existing studies reproduce exactly
    body.gust = float(os.environ.get('VF_GUST', 0.0))          # 1-sigma gust amplitude (m/s)
    body.gust_tau = float(os.environ.get('VF_GUST_TAU', 3.0))  # gust correlation time (s)
    faults = sim_model.Faults(os.environ.get('VF_FAULT', ''))  # e.g. 'gnss@30,pitot@45'
    pitot_on = os.environ.get('VF_PITOT', '1') != '0'  # feed the SDP810 direct airspeed to the fusion (default on)
    pitot_rail = (2.0 * 546.0 / 1.225) ** 0.5  # the ±500 Pa sensor rails ~29.85 m/s -> boost/dive fall back to accel
    body.imbalance_pitch = imbalance_pitch  # weight-imbalance torque during burn (deg/s^2)
    body.imbalance_roll = imbalance_roll
    body.wind_e = wind * math.sin(math.radians(wind_dir))   # steady wind the glider must crab against
    body.wind_n = wind * math.cos(math.radians(wind_dir))

    # the REAL control stack, exactly as tasks/flight.py builds it -- mixer with bound fins, the
    # governor and guidance over injected handles, one fixed-point PID per axis.
    mix = mixer.Mixer(cfg.get('mixer', {}))
    fins_by_name = {name: _Fin(mix.neutral) for name in _FINS}
    mix.bind(fins_by_name)
    accel_handle, speed_handle, pitot_handle, position_handle, agl_handle, elevation_handle = (
        _Handle(), _Handle(), _Handle(), _Handle(), _Handle(), _Handle())
    fin_governor = governor.Governor(governor.GovernorConfig(flight_c), mix, accel_handle, speed_handle,
                                     pitot_handle, cfg.get('fin_limit_multiplier', 1.0))
    law = guidance.Guidance(guidance.GuidanceConfig(flight_c, int(_GNSS_S * 2000)), _Mission(zone),
                            fin_governor, position_handle, agl_handle, elevation_handle)
    if final_agl_override is not None:
        law._config.final_agl = final_agl_override
    if endgame_alt_override is not None:
        law._config.endgame_alt_m = endgame_alt_override
    gains = flight_c.get('gains', {})
    pids = {axis: pid.Pid(output_limit=mix.limit, integral_limit=mix.limit, **gains.get(axis, {}))
            for axis in ('roll', 'pitch', 'yaw')}

    dt = 1.0 / sim_hz
    dt_ms = max(1, int(round(dt * 1000)))   # integer-ms slice the fixed-point PID expects (board parity)
    stage = 'setting'
    since = 0.0          # time the current sustained-detect window started
    active = False       # in a control stage (mirrors flight._active: enter() + PID reset on entry)
    last_gnss = -1.0     # last GNSS publish into the injected handles (~10 Hz, board cadence)
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
        # gyro angular rates the PID D term reads -- noised deg/s (recorded to imu_lsm6dso32, board parity),
        # converted once to centideg/s fixnum for the PID (same unit + mapping as tasks/hitl + the driver)
        roll_rate_dps = sim_model.noisy(sensors['roll_rate'], noise, -2000.0, 2000.0)
        pitch_rate_dps = sim_model.noisy(sensors['pitch_rate'], noise, -2000.0, 2000.0)
        yaw_rate_dps = sim_model.noisy(sensors['yaw_rate'], noise, -2000.0, 2000.0)
        roll_rate = fixed.from_float(roll_rate_dps)
        pitch_rate = fixed.from_float(pitch_rate_dps)
        yaw_rate = fixed.from_float(yaw_rate_dps)

        """
        inject a transient 2x glitch on the attitude + accel for ONE tick every _SPIKE_S seconds
        (deterministic schedule so the stored corner-case traces reproduce). Exercises the control
        loop's rejection of a sudden bad sample -- the fin trace shows the kick, the trajectory should
        barely move.
        """
        if spike and int(t / _SPIKE_S) != int((t - dt) / _SPIKE_S):
            roll_m *= 2.0
            pitch_m *= 2.0
            accel_m *= 2.0

        # --- stage machine (mirrors tasks/sequencer.py; separation off -> boost timeout drives glide) ---
        if stage == 'setting':
            if accel_m > launch_g:
                if (t - since) * 1000.0 >= launch_ms:
                    stage, since = 'boosting', t
                    rows.event(t, 'controller :: stage -> boosting')
            else:
                since = t
        elif stage == 'boosting':
            """
            APOGEE detect (mirror of sequencer._detect_apogee -- the mirror had DRIFTED to
            timeout-only deploy, and a low arc could be back underground by the timeout): blind
            for apogee_arm_ms after entry (burn pressure wave), then track the noised baro peak
            and deploy once it falls apogee_drop_m below, sustained; the timeout stays fallback.
            """
            elevation_now = altitude_m - body.elev0
            if (t - since) * 1000.0 >= apogee_arm_ms:
                if apogee_max is None or elevation_now > apogee_max:
                    apogee_max, apogee_since = elevation_now, None
                elif elevation_now < apogee_max - apogee_drop_m:
                    apogee_since = t if apogee_since is None else apogee_since
                    if (t - apogee_since) * 1000.0 >= launch_ms:
                        stage, since = 'gliding', t
                        body.begin_glide()
                        rows.event(t, 'controller :: stage -> gliding')
                else:
                    apogee_since = None
            if stage == 'boosting' and (t - since) * 1000.0 >= boost_timeout_ms:
                stage, since = 'gliding', t
                body.begin_glide()
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

        # --- publish the sim readings into the injected handles (what the databoard does on-board) ---
        accel_handle.value_now = (0.0, 0.0, accel_m)   # boost-axis |a| in g (magnitude parity)
        accel_handle.source = 'sim'
        if pitot_on:  # SDP810 DIRECT airspeed (m/s) each tick, clamped at the rail so boost/dive saturates -> accel
            true_pitot = min((body.speed * body.speed + body.vu * body.vu) ** 0.5, pitot_rail)
            pitot_handle.value_now = faults.apply('pitot', true_pitot, t)
            pitot_handle.source = None if pitot_handle.value_now is None else 'sim'
        agl_handle.value_now = faults.apply('laser', agl, t)
        agl_handle.source = None if agl_handle.value_now is None else 'sim'
        elevation_handle.value_now = faults.apply('baro', altitude_m - body.elev0, t)  # noised baro elevation
        elevation_handle.source = None if elevation_handle.value_now is None else 'sim'
        if t - last_gnss >= _GNSS_S:                    # GNSS ~10 Hz, the board's fix cadence
            last_gnss = t
            # total speed (vertical + horizontal), matching what tasks/hitl publishes on 'speed' --
            # the estimator's corrector sees the same signal the on-board HITL feeds it
            true_speed = (body.vu * body.vu + body.speed * body.speed) ** 0.5
            speed_handle.value_now = faults.apply('gnss', true_speed, t)
            speed_handle.source = None if speed_handle.value_now is None else 'gnss'
            position_handle.value_now = faults.apply('gnss', sensors['position'], t)
            position_handle.source = None if position_handle.value_now is None else 'gnss'

        # --- the REAL control pipeline (mirrors flight._step): governor -> gate -> guidance -> PID ---
        stage_id = _STAGE.NAMES[stage]
        roll_deg = pitch_deg = yaw_deg = 0  # per-axis demands for flight.csv (0 whenever fins are neutral)
        roll_cd = fixed.from_float(roll_m)
        pitch_cd = fixed.from_float(pitch_m)
        fin_governor.step(dt, stage_id < _STAGE.GLIDING, pitch_cd)
        setpoint = law.setpoint(stage_id)
        if setpoint is None:                            # non-control stage -> fins neutral
            if active:
                active = False
            mix.actuate(0, 0, 0)
        else:
            if not active:                              # entering control: capture holds, reset PIDs
                active = True
                law.enter(heading_m, roll_cd, pitch_cd)
                for axis_pid in pids.values():
                    axis_pid.reset()
            if law.compute(stage_id, setpoint, heading_m, int(t * 1e6)):
                cap = mix.limit  # PID clamps track the governor's live cap (mirrors flight._run_pid, finding 23.5)
                for axis_pid in pids.values():
                    axis_pid.set_limit(cap)
                roll_cmd = pids['roll'].step(law.roll_setpoint - roll_cd, dt_ms, roll_rate)
                pitch_cmd = pids['pitch'].step(law.pitch_setpoint - pitch_cd, dt_ms, pitch_rate)
                yaw_cmd = pids['yaw'].step(law.heading_error * fixed.SCALE, dt_ms, yaw_rate)
                roll_deg = roll_cmd // fixed.SCALE
                pitch_deg = pitch_cmd // fixed.SCALE
                yaw_deg = yaw_cmd // fixed.SCALE
                mix.actuate(roll_deg, pitch_deg, yaw_deg)
            else:                                       # boost still on the rod -> neutral
                mix.actuate(0, 0, 0)
        fins = tuple(fins_by_name[name].angle for name in _FINS)

        # --- physics: fly the body with the commanded fins (left, right, yaw) ---
        if stage in ('setting', 'boosting'):
            burn = thrust if t < burn_s else 0.0
            body.boost_step(dt, burn, (fins[0] + fins[1]) / 2.0 - 90.0, (fins[0] - fins[1]) / 2.0)
        elif setpoint is not None:                      # a control stage (gliding/landing)
            body.glide_step(dt, (fins[0] - fins[1]) / 2.0, (fins[0] + fins[1]) / 2.0 - 90.0, fins[2] - 90.0)
        else:                                           # non-control stage -> coast, fins neutral
            body.glide_step(dt, 0.0, 0.0, 0.0)

        wind_speed = (body.wind_e ** 2 + body.wind_n ** 2) ** 0.5
        wind_from = int(math.degrees(math.atan2(-body.wind_e, -body.wind_n))) % 360 if wind_speed else 0
        rows.sample(t, accel_m, altitude_m, sensors['altitude'] - body.elev0, heading_m, roll_m, pitch_m,
                    sensors['position'], agl, laser_range_m, body.speed, fins,
                    (roll_rate_dps, pitch_rate_dps, yaw_rate_dps),
                    dt=dt,
                    pitot=(pitot_handle.value_now if pitot_on else None),
                    control=(stage_id, 1 if active else 0, int(fin_governor.airspeed() * 100), mix.limit,
                             law.roll_setpoint, law.pitch_setpoint, law.heading_error,
                             roll_deg, pitch_deg, yaw_deg, int(wind_speed * 100), wind_from))
        if stage == 'boosting':
            rows.leak_starts(t)  # GC goes off at BOOSTING on the board -> the leak clock starts
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
        self._leak_from = None   # when GC went off (BOOSTING) -- the modelled leak's origin
        self._fins_prev = None   # previous commanded fin angles -> which servos MOVED this tick

    def _tlm(self, file: str, row: str) -> None:
        self._lines.append('@%s_%s@%s' % (self._SESSION, file, row))

    def header(self) -> None:
        self._tlm('accel_adxl375.csv', 'uptime;ax;ay;az')
        self._tlm('baro_icp10111.csv', 'uptime;altitude;temperature;pressure;elevation')
        self._tlm('imu_bno055.csv', 'uptime;heading;roll;pitch')
        self._tlm('imu_lsm6dso32.csv', 'uptime;ax;ay;az;gx;gy;gz')   # low-g accel + gyro rate (deg/s)
        self._tlm('gnss.csv', 'uptime;lat;lon;speed_kn;course')
        self._tlm('laser_agl.csv', 'uptime;agl')
        self._tlm('fins.csv', 'uptime;eleron_left;eleron_right;yaw')  # commanded servo angles (deg)
        self._tlm('health.csv', 'uptime;temp;mem_free;load')          # board vitals (board_health.py)
        # flight.csv: the CONTROL STATE, byte-identical in shape to the board's (tasks/flight.py) so a
        # sim capture exercises the same report panels a real capture will (findings §27.1/§27.2)
        self._tlm('flight.csv', 'uptime;stage;active;airspeed_cms;fin_cap;roll_sp;pitch_sp;heading_err;'
                                'roll_cmd;pitch_cmd;yaw_cmd;wind_cms;wind_from')
        # the SDP810 pitot as the board's driver records it (Pa fixnum + derived m/s)
        self._tlm('airspeed_sdp810.csv', 'uptime;dynamic_pressure;airspeed_cms;temperature')
        # servo-rail power as the INA226 records it -- MODELLED from the measured MG90S figures, so
        # the report's engine panel and flight_kpi's servo-energy metric are not blank on a sim run
        self._tlm('power_ina226.csv', 'uptime;voltage_mv;current_ma;power_mw;alerts')

    def sample(self, t, accel, altitude, elevation, heading, roll, pitch, position, agl, laser_range, speed,
               fins, rate, control=None, pitot=None, dt=0.02):
        microseconds = int(t * 1e6)
        self._tlm('accel_adxl375.csv', '%u;0.000;0.000;%.3f' % (microseconds, accel))
        self._tlm('baro_icp10111.csv', '%u;%.2f;21.0;100000;%.2f' % (microseconds, altitude, elevation))
        self._tlm('imu_bno055.csv', '%u;%.1f;%.1f;%.1f' % (microseconds, heading, roll, pitch))
        # imu_lsm6dso32: the boost-axis low-g accel + the gyro rate (deg/s) the PID reads (board parity)
        self._tlm('imu_lsm6dso32.csv', '%u;0.000;0.000;%.3f;%.1f;%.1f;%.1f'
                  % (microseconds, accel, rate[0], rate[1], rate[2]))
        self._tlm('fins.csv', '%u;%d;%d;%d' % (microseconds, fins[0], fins[1], fins[2]))
        if t - self._last_gnss >= _GNSS_S:               # GNSS ~10 Hz
            self._last_gnss = t
            self._tlm('gnss.csv', '%u;%.6f;%.6f;%.1f;%.1f'    # speed in knots (GPS convention)
                      % (microseconds, position[0], position[1], speed * 1.94384, heading))
        if agl <= laser_range:                           # the laser only resolves the last few metres
            self._tlm('laser_agl.csv', '%u;%.3f' % (microseconds, agl))
        if pitot is not None:                            # SDP810: q = 0.5*rho*v^2, Pa as a x100 fixnum
            self._tlm('airspeed_sdp810.csv', '%u;%d;%d;2500'
                      % (microseconds, int(0.5 * 1.225 * pitot * pitot * 100), int(pitot * 100)))
        """
        SERVO POWER: a fin that CHANGED angle is travelling; at max slew it needs
        |delta| * _SERVO_SLEW_S_PER_DEG seconds, during which the measured mean draw is
        _SERVO_MOVE_MW. Spread that energy over this tick (a move shorter than dt averages down),
        add the measured holding draw, and report it as the INA226 would.
        """
        moving_mw = 0.0
        if self._fins_prev is not None and dt > 0:
            for now_deg, was_deg in zip(fins, self._fins_prev):
                travel_s = abs(now_deg - was_deg) * _SERVO_SLEW_S_PER_DEG
                moving_mw += _SERVO_MOVE_MW * min(travel_s / dt, 1.0)
        self._fins_prev = tuple(fins)
        power_mw = int(_SERVO_HOLD_MW + moving_mw)
        self._tlm('power_ina226.csv', '%u;5000;%d;%d;0' % (microseconds, power_mw // 5, power_mw))
        if control is not None:                          # the control state the board's flight task records
            self._tlm('flight.csv', '%u;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d;%d'
                      % ((microseconds,) + control))

    def leak_starts(self, t) -> None:
        """Mark when GC went off (BOOSTING) so the modelled leak runs from there, as on the board."""
        if self._leak_from is None:
            self._leak_from = t

    def health(self, t, stage):
        """
        A 1 Hz board-vitals row (board_health.csv fields).

        SYNTHETIC + phase-modeled -- the host has no real MCU -- but shaped like the board would read:
        load tracks the work per stage (idle on the rod, high under boost sampling, steady in the glide
        loop, highest while the laser hammers I2C on landing); temperature drifts up under load.

        mem_free follows the MEASURED GC-off leak (~15 KB/s settled at 100 Hz, doc/sims/
        TMS-7-guiding_refactoring), declining monotonically once airborne. It used to be a 30 s
        SAWTOOTH, which left a near-zero net slope over BOOSTING->DONE -- so flight_report's
        time-to-OOM headline divided by ~0 and printed an absurd ~15000 s on every sim study. A
        modelled number that matches the real board (~36 min to OOM) is honest; a fabricated flat one
        that reads as a measurement is not.
        """
        if t - self._last_health < 1.0:
            return
        self._last_health = t
        load = {'setting': 5, 'boosting': 45, 'gliding': 30, 'landing': 60}.get(stage, 8)
        load = max(0, min(100, load + int(6 * math.sin(t * 2.5))))
        temp = min(63.0, 45.0 + 0.18 * t + (4.0 if stage == 'landing' else 0.0))
        airborne = max(0.0, t - self._leak_from) if self._leak_from else 0.0
        mem_free = int(_FREE_AT_BOOT - _LEAK_BPS * airborne)  # GC is OFF airborne -> monotonic decline
        self._tlm('health.csv', '%u;%.1f;%d;%d' % (int(t * 1e6), temp, mem_free, load))

    def event(self, t, line: str) -> None:
        self._lines.append('%u %s' % (int(t * 1e6), line))

    def text(self) -> str:
        return '\n'.join(self._lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description='Fly a virtual Coludo mission and emit a recorder capture.')
    parser.add_argument('--motor', default='F15', choices=sorted(sim_model.MOTORS), help='motor (default F15)')
    parser.add_argument('--noise', type=float, default=0.05, help='sensor noise fraction N (default 0.05)')
    parser.add_argument('--spike', action='store_true',
                        help='inject a transient 2x attitude+accel glitch every ~3 s')
    parser.add_argument('--wind', type=float, default=0.0, help='steady wind speed m/s (default 0)')
    parser.add_argument('--wind-dir', type=float, default=0.0, help='wind blows TOWARD this heading deg (default 0=N)')
    parser.add_argument('--endgame-alt', type=float, default=None,
                        help='endgame band elevation override (m; 0 = off)')
    parser.add_argument('--imbalance-pitch', type=float, default=0.0,
                        help='weight-imbalance torque on the PITCH axis during burn (deg/s^2)')
    parser.add_argument('--imbalance-roll', type=float, default=0.0,
                        help='weight-imbalance torque on the ROLL axis during burn (deg/s^2)')
    parser.add_argument('--final-agl', type=float, default=None,
                        help=' final-approach trigger AGL override (0 = disabled / old blind flare)')
    parser.add_argument('--hz', type=int, default=50, help='simulation rate (default 50)')
    parser.add_argument('--seconds', type=float, default=240.0, help='max flight time (default 240)')
    parser.add_argument('-o', '--out', help='write capture here (default stdout)')
    parser.add_argument('--no-preflight', action='store_true',
                        help='skip the install/data-consistency gates (not recommended)')
    args = parser.parse_args()
    if not args.no_preflight:
        # gate BEFORE flying: a missing dep or a sim/board schema drift should cost a second here,
        # not a whole sweep that produces captures the renderers cannot read (findings §27.6)
        preflight.gate('simulation')

    capture = fly(args.motor, args.noise, args.spike, args.hz, args.seconds, args.wind, args.wind_dir,
                  args.final_agl, args.imbalance_pitch, args.imbalance_roll, args.endgame_alt)
    if args.out:
        with open(args.out, 'w') as handle:
            handle.write(capture)
        sys.stderr.write('wrote %s (%d lines, %s @ noise %.0f%%, wind %.0f m/s)\n'
                         % (args.out, capture.count('\n'), args.motor, args.noise * 100, args.wind))
    else:
        sys.stdout.write(capture)


if __name__ == '__main__':
    main()
