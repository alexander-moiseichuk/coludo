# config_hitl.py — a HITL board config derived from config_default (). The real sensor drivers
# are turned OFF and the `hitl` task supplies accel/attitude/agl/altitude/elevation/position at priority
# 0, so the control code reads the simulation. flight is enabled with test gains, the watchdog and the
# radios are off (self-contained sim), and separation is off (the boost-timeout drives BOOSTING ->
# GLIDING). Servos stay on so the sim can read the commanded fin angles. `default()` returns a fresh
# dict -- mutate freely. Run it instead of config_default for a simulation; the flight config is untouched.

import config_default
import sim_model

_SIM_SENSORS = ('accel_adxl375', 'imu_lsm6dso32', 'imu_bno055', 'baro_icp10111', 'baro_bmp280',
                'laser_agl', 'gnss')
_OFF = ('separation', 'watchdog', 'wifi', 'cc', 'bluetooth')

# TMS-7 v2 masses (g). The BOOSTER (motor + casing) ejects at separation, so the boost phase carries the
# whole stack (booster + glider) and the glide carries the glider alone. The glider is the airframe +
# electronics: 300 g today, ~150 g the weight-optimisation target (lighter carbon wings + a single C25
# 1-cell battery). Whole-stack liftoff = booster + glider: E16 500/350 g, F15 517/370 g (full/half glider).
_BOOSTER_G = {'E16': 200, 'F15': 217}
_GLIDER_G = 300  # full glider; pass glider_g=150 for the half-weight (optimised) build
# Heavier v2 stacks read a LOWER boost |a| (specific force = thrust/mass): F15 at 517 g ≈ 2.84 g, so the
# stock 3.0 g launch threshold would miss it. Trip at 2.0 g -- safely above the ~1 g rest + noise, below
# every config's boost. (config_default's real launch_g likely wants the same review for the v2 stack.)
_LAUNCH_G = 2.0


def default(motor: str = 'F15', noise: float = 0.0, spike: bool = False, wind: float = 0.0,
            wind_dir: float = 0.0, eject_delay_s: float = 4.0, boost_axis: str = 'z',
            glider_g: int = _GLIDER_G, inject_hz: int = 0) -> dict:
    """Build a HITL config. `eject_delay_s` is the motor's ejection delay (the '-4' in F15-4/E16-4 ~=
    4 s after burnout, near apogee); since separation is off in HITL, the sequencer's boost->glide
    timeout stands in for that ejection charge, so set it to burn + delay (otherwise a generic timeout
    glides from the wrong altitude). `wind`/`wind_dir` set a steady cross-wind (m/s, toward deg) the
    glide must crab against. `boost_axis` picks which accel axis carries the boost |a|. `glider_g` is the
    glider (glide) mass in grams (default 300, the full build; 150 = the half-weight optimisation target)
    -- the booster adds to it for the boost phase, then ejects at separation so the glide runs on
    `glider_g` alone (a lighter glider -> a longer, slower glide, the worst case for the GC-off leak).
    `inject_hz` > 0 sets the sensor publish rate (default 0 -> the sim's sim_hz); lower it (e.g. 10) to
    slim the sim's own heap churn so an on-board HITL leak reflects real flight -- physics stay at sim_hz."""
    cfg = config_default.default()
    for sensor in cfg['sensors']:
        if sensor['name'] in _SIM_SENSORS:
            sensor['enabled'] = False  # the sim provides these instead
    by_name = {c['name']: c for c in cfg['components']}
    for name in _OFF:
        if name in by_name:
            by_name[name]['enabled'] = False
    flight = by_name['flight']
    flight['enabled'] = True  # the loop under test
    flight['gains'] = {'roll': {'kp': 2.0, 'kd': 0.2}, 'pitch': {'kp': 1.5}, 'yaw': {'kp': 1.5, 'kd': 0.1}}
    # separation is off here, so the boost->glide FALLBACK timeout emulates the ejection charge firing
    # ~eject_delay_s after burnout (near apogee). Tie it to the motor burn so HITL deploys at apogee, not
    # at a generic 6 s. Needs hitl.py's wall-clock sim for the wall-clock timeout to line up.
    burn_s = sim_model.MOTORS.get(motor, sim_model.MOTORS['F15'])[1]
    sequencer = by_name.get('sequencer')
    if sequencer is not None:
        sequencer['boost_timeout_ms'] = round((burn_s + eject_delay_s) * 1000)
        sequencer['launch_g'] = _LAUNCH_G  # heavier v2 stacks boost below the stock 3.0 g threshold
    liftoff_g = _BOOSTER_G.get(motor, 217) + glider_g  # boost mass = booster + glider; glide = glider alone
    hitl = {
        'name': 'hitl', 'activity': 'hitl', 'enabled': True,
        'sim_hz': 50, 'motor': motor, 'noise': noise, 'spike': spike,
        'liftoff_g': liftoff_g, 'glider_g': glider_g,  # boost then glide masses (booster ejects at apogee)
        'wind': wind, 'wind_dir': wind_dir, 'boost_axis': boost_axis,
    }
    if inject_hz:  # 0 -> omit -> hitl.py defaults the publish rate to sim_hz (avoid a 0 loop period)
        hitl['inject_hz'] = inject_hz
    cfg['components'].append(hitl)
    return cfg
