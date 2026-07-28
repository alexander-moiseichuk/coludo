"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

A HITL board config derived from config_default(). The real sensor drivers are turned OFF and the
`hitl` task supplies accel / attitude / agl / altitude / elevation / position at priority 0, so the
control code reads the simulation. flight is enabled with test gains, the watchdog and the radios are
off (self-contained sim), and separation is off (the boost-timeout drives BOOSTING -> GLIDING).
Servos stay on so the sim can read the commanded fin angles. `default()` returns a fresh dict --
mutate freely. Run it instead of config_default for a simulation; the flight config is untouched.
"""

import config_default

_SIM_SENSORS = ('accel_adxl375', 'imu_lsm6dso32', 'imu_bno055', 'baro_icp10111', 'baro_bmp280',
                'laser_agl', 'gnss', 'airspeed_sdp810')
"""
`airspeed_sdp810` masked (7/27): the sim now publishes `airspeed`/`dynamic_pressure` itself. Leaving
the real part enabled meant a bench SDP810 in still air was the ONLY publisher of the fused airspeed
channel on every board HITL flight -- the findings §28 bistability. That was patched at the consumer
(`pitot_min_ms`), which is correct for a blocked tube in flight but left board HITL never exercising
the pitot path while the host sim did. Masking it here closes the harness side.
"""
_OFF = ('separation', 'watchdog', 'wifi', 'cc', 'bluetooth')

"""
TMS-7 v4 masses (g), MEASURED on the printed airframe (models/TMS-7 README, 7/13). The BOOSTER (motor +
casing) ejects at separation, so the boost phase carries the whole stack and the glide carries the glider
alone. Booster WITH engine: E16 184.5 g, F15 200.9 g (booster body 102.0 g + engine 82.5 / 98.9 g). The
glider structure weighs 115.3 g; electronics add ~100-155 g -> 270 g full (154.9 g electronics) / 215 g
light (the ~100 g floor). Whole-stack liftoff: E16 455/400 g, F15 471/416 g (full/light glider).
"""
_BOOSTER_G = {'E16': 185, 'F15': 201}
_GLIDER_G = 270  # full glider (154.9 g electronics); pass glider_g=215 for the light build


def default(motor: str = 'F15', noise: float = 0.0, spike: bool = False, wind: float = 0.0,
            wind_dir: float = 0.0, boost_axis: str = 'z',
            glider_g: int = _GLIDER_G, inject_hz: int = 0,
            gnss_drift: float = 0.0, gnss_drift_dir: float = 0.0, pad_dwell_s: float = 0.0) -> dict:
    """
    Build a HITL config from config_default(), the real sensors off and the `hitl` sim task added.

    Separation is off here, so the boost->glide deploy rides the sequencer's baro APOGEE detect
    (mass / motor-independent -- the top of the arc), with config_default's long boost_timeout as the
    last-resort fallback; the sim's reduced baro noise keeps the peak-detect clean. The booster adds to
    the glide mass for the boost phase then ejects at separation, so the glide runs on `glider_g` alone
    -- a lighter glider glides LONGER, the worst case for the GC-off leak.

    Args:
        motor - the booster motor ('E16' / 'F15'); its mass sets the boost-phase liftoff mass.
        noise - sensor-noise level fed to the sim (0.0 = clean).
        spike - inject accel spikes when True (a robustness stressor).
        wind - steady cross-wind speed (m/s) the glide must crab against.
        wind_dir - the wind's toward-bearing (deg).
        boost_axis - which accel axis ('x' / 'y' / 'z') carries the boost |a|.
        glider_g - the glider (glide) mass in grams (default _GLIDER_G, the full build; pass the
            light-build mass for the half-weight optimisation target).
        inject_hz - the sensor publish rate; 0 -> the sim's sim_hz. Lower it (e.g. 10) to slim the
            sim's own heap churn so an on-board HITL leak reflects real flight.
        gnss_drift - steady GNSS ground-velocity drift (m/s) the pad-drift calibration must measure out.
        gnss_drift_dir - the drift's bearing (deg).
        pad_dwell_s - seconds held stationary on the pad before launch (lets the drift calibration
            gather samples).

    Returns:
        A fresh HITL config dict (the real sensors disabled, flight + the `hitl` task enabled).
    """
    cfg = config_default.default()
    for sensor in cfg['sensors']:
        if sensor['name'] in _SIM_SENSORS:
            sensor['enabled'] = False  # the sim provides these instead
    by_name = {comp['name']: comp for comp in cfg['components']}
    for name in _OFF:
        if name in by_name:
            by_name[name]['enabled'] = False
    flight = by_name['flight']
    flight['enabled'] = True  # the loop under test
    flight['gains'] = {'roll': {'kp': 2.0, 'kd': 0.2}, 'pitch': {'kp': 1.5}, 'yaw': {'kp': 1.5, 'kd': 0.1}}
        # launch_g / launch_alt_m are inherited from config_default (2.5 g + the 10 m baro backup) so HITL
        # exercises the REAL launch thresholds against the v2 boost profiles.
    liftoff_g = _BOOSTER_G.get(motor, 193) + glider_g  # boost mass = booster + glider; glide = glider alone
    hitl = {
        'name': 'hitl', 'activity': 'hitl', 'enabled': True,
        # 50 Hz physics: the memory leak was localized to the CONTROL path (the nav distance/bearing float
        # work, throttled airspeed estimator), not the sim's step rate, so full-fidelity 50 Hz is restored
        # (25 Hz was a diagnostic stopgap). A memory-measurement run slims the sim's own churn via
        # inject_hz instead (the PUBLISH rate), leaving the physics honest -- see fly()/soak(inject_hz).
        'sim_hz': 50, 'motor': motor, 'noise': noise, 'spike': spike,
        'liftoff_g': liftoff_g, 'glider_g': glider_g,  # boost then glide masses (booster ejects at apogee)
        'wind': wind, 'wind_dir': wind_dir, 'boost_axis': boost_axis,
        'gnss_drift': gnss_drift, 'gnss_drift_dir': gnss_drift_dir, 'pad_dwell_s': pad_dwell_s,
    }
    if inject_hz:  # 0 -> omit -> hitl.py defaults the publish rate to sim_hz (avoid a 0 loop period)
        hitl['inject_hz'] = inject_hz
    cfg['components'].append(hitl)
    return cfg
