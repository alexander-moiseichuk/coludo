# config_hitl.py — a HITL board config derived from config_default (). The real sensor drivers
# are turned OFF and the `hitl` task supplies accel/attitude/agl/altitude/elevation/position at priority
# 0, so the control code reads the simulation. flight is enabled with test gains, the watchdog and the
# radios are off (self-contained sim), and separation is off (the boost-timeout drives BOOSTING ->
# GLIDING). Servos stay on so the sim can read the commanded fin angles. `default()` returns a fresh
# dict -- mutate freely. Run it instead of config_default for a simulation; the flight config is untouched.

import config_default

_SIM_SENSORS = ('accel_adxl375', 'imu_lsm6dso32', 'imu_bno055', 'baro_icp10111', 'baro_bmp280',
                'laser_agl', 'gnss')
_OFF = ('separation', 'watchdog', 'wifi', 'cc', 'bluetooth')

# TMS-7 v3 masses (g), MEASURED on the printed airframe (models/TMS-7 README, 7/03). The BOOSTER
# (motor + casing) ejects at separation, so the boost phase carries the whole stack and the glide
# carries the glider alone. Booster WITH engine: E16 165.1 g, F15 181.5 g. The glider construction
# weighs 134.8 g bare; electronics add ~100-150 g -> 285 g full (150 g electronics) / 235 g light
# (the 100 g floor). Whole-stack liftoff: E16 450/400 g, F15 467/417 g (full/light glider). The
# static-burn test glider (partial electronics) measured 194.4 g -- between bare and light.
_BOOSTER_G = {'E16': 165, 'F15': 182}
_GLIDER_G = 285  # full glider (150 g electronics); pass glider_g=235 for the light build


def default(motor: str = 'F15', noise: float = 0.0, spike: bool = False, wind: float = 0.0,
            wind_dir: float = 0.0, boost_axis: str = 'z',
            glider_g: int = _GLIDER_G, inject_hz: int = 0,
            gnss_drift: float = 0.0, gnss_drift_dir: float = 0.0, pad_dwell_s: float = 0.0) -> dict:
    """Build a HITL config. Separation is off here, so boost->glide deploy rides the sequencer's baro
    APOGEE detect (mass/motor-independent -- the top of the arc), with config_default's long boost_timeout
    as the last-resort fallback; the sim's reduced baro noise keeps the peak-detect clean. `wind`/`wind_dir`
    set a steady cross-wind (m/s, toward deg) the glide must crab against. `boost_axis` picks which accel
    axis carries the boost |a|. `glider_g` is the glider (glide) mass in grams (default 300, the full build;
    150 = the half-weight optimisation target) -- the booster adds to it for the boost phase, then ejects
    at separation so the glide runs on `glider_g` alone (a lighter glider -> a longer glide, the worst case
    for the GC-off leak). `inject_hz` > 0 sets the sensor publish rate (default 0 -> the sim's sim_hz);
    lower it (e.g. 10) to slim the sim's own heap churn so an on-board HITL leak reflects real flight."""
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
        # launch_g / launch_alt_m are inherited from config_default (2.5 g + the 10 m baro backup) so HITL
        # exercises the REAL launch thresholds against the v2 boost profiles.
    liftoff_g = _BOOSTER_G.get(motor, 217) + glider_g  # boost mass = booster + glider; glide = glider alone
    hitl = {
        'name': 'hitl', 'activity': 'hitl', 'enabled': True,
        # 50 Hz physics: the memory leak was localized to the CONTROL path (the nav distance/bearing float
        # work, throttled airspeed estimator), not the sim's step rate, so full-fidelity 50 Hz is restored
        # (25 Hz was a diagnostic stopgap). A memory-measurement run slims the sim's own churn via
        # inject_hz instead (the PUBLISH rate), leaving the physics honest -- see fly()/soak(inject_hz).
        'sim_hz': 50, 'motor': motor, 'noise': noise, 'spike': spike,
        'liftoff_g': liftoff_g, 'glider_g': glider_g,  # boost then glide masses (booster ejects at apogee)
        'wind': wind, 'wind_dir': wind_dir, 'boost_axis': boost_axis,
        'gnss_drift': gnss_drift, 'gnss_drift_dir': gnss_drift_dir, 'pad_dwell_s': pad_dwell_s,  # (#2)
    }
    if inject_hz:  # 0 -> omit -> hitl.py defaults the publish rate to sim_hz (avoid a 0 loop period)
        hitl['inject_hz'] = inject_hz
    cfg['components'].append(hitl)
    return cfg
