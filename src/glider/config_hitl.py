# config_hitl.py — a HITL board config derived from config_default (6/23 g15). The real sensor drivers
# are turned OFF and the `hitl` task supplies accel/attitude/agl/altitude/elevation/position at priority
# 0, so the control code reads the simulation. flight is enabled with test gains, the watchdog and the
# radios are off (self-contained sim), and separation is off (the boost-timeout drives BOOSTING ->
# GLIDING). Servos stay on so the sim can read the commanded fin angles. `default()` returns a fresh
# dict -- mutate freely. Run it instead of config_default for a simulation; the flight config is untouched.

import config_default

_SIM_SENSORS = ('accel_adxl375', 'imu_bno055', 'baro_icp10111', 'baro_bmp280', 'laser_agl', 'gnss')
_OFF = ('separation', 'watchdog', 'wifi', 'cc', 'bluetooth')


def default(motor: str = 'F15', noise: float = 0.0, spike: bool = False) -> dict:
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
    cfg['components'].append({
        'name': 'hitl', 'activity': 'hitl', 'enabled': True,
        'sim_hz': 50, 'motor': motor, 'noise': noise, 'spike': spike, 'liftoff_g': 430,
    })
    return cfg
