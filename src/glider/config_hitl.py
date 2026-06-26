# config_hitl.py — a HITL board config derived from config_default (). The real sensor drivers
# are turned OFF and the `hitl` task supplies accel/attitude/agl/altitude/elevation/position at priority
# 0, so the control code reads the simulation. flight is enabled with test gains, the watchdog and the
# radios are off (self-contained sim), and separation is off (the boost-timeout drives BOOSTING ->
# GLIDING). Servos stay on so the sim can read the commanded fin angles. `default()` returns a fresh
# dict -- mutate freely. Run it instead of config_default for a simulation; the flight config is untouched.

import config_default
import sim_model

_SIM_SENSORS = ('accel_adxl375', 'imu_bno055', 'baro_icp10111', 'baro_bmp280', 'laser_agl', 'gnss')
_OFF = ('separation', 'watchdog', 'wifi', 'cc', 'bluetooth')


def default(motor: str = 'F15', noise: float = 0.0, spike: bool = False, wind: float = 0.0,
            wind_dir: float = 0.0, eject_delay_s: float = 4.0, boost_axis: str = 'z') -> dict:
    """Build a HITL config. `eject_delay_s` is the motor's ejection delay (the '-4' in F15-4/E16-4 ~=
    4 s after burnout, near apogee); since separation is off in HITL, the sequencer's boost->glide
    timeout stands in for that ejection charge, so set it to burn + delay (otherwise a generic timeout
    glides from the wrong altitude). `wind`/`wind_dir` set a steady cross-wind (m/s, toward deg) the
    glide must crab against. `boost_axis` picks which accel axis carries the boost |a|."""
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
    cfg['components'].append({
        'name': 'hitl', 'activity': 'hitl', 'enabled': True,
        'sim_hz': 50, 'motor': motor, 'noise': noise, 'spike': spike, 'liftoff_g': 430,
        'wind': wind, 'wind_dir': wind_dir, 'boost_axis': boost_axis,
    })
    return cfg
