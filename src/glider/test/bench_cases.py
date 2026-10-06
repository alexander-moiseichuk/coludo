"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Our own flight code, case by case, timed in MicroPython: the twin of src/avionics/firmware/src/bin/cases.rs,
same case names, same inputs, but calling the REAL deployed modules (commons, fixed, pid, mixer, navigation,
recorder), so the two columns of doc/benches/avionics-20261005 compare languages on identical work.

Prints `check;<name>;<value>` (the Rust side prints the same lines -- proof the port is equivalent) and
`cases;<name>;<nanoseconds per call>;ns`, best of 3 runs, loop and call included (a call's overhead is part
of what a call costs here). Needs the firmware deployed (tools/deploy.sh). Run on-board:
    mpremote connect PORT run src/glider/test/bench_cases.py
"""

import gc
import struct
import time

import commons
import fixed
import mixer
import navigation
import pid
import recorder


class _FakeFin:
    """A servo that only remembers its angle (bench_flight's FakeFin)."""

    def __init__(self):
        self.angle = 90

    def set_angle(self, angle):
        self.angle = angle

    def settle(self):
        pass


def _case(name, count, function):
    """Print the best of 3 runs of `count` calls of function(), in ns per call."""
    best = None
    for _ in range(3):
        gc.collect()
        start = time.ticks_us()
        for _ in range(count):
            function()
        elapsed = time.ticks_diff(time.ticks_us(), start)
        best = elapsed if best is None else min(best, elapsed)
    print('cases;%s;%d;ns' % (name, best * 1000 // count))


_TOP_LEFT = (48.1010, 11.5000)  # a made-up zone (not a real site): 200 m x 100 m, the glider 600 m out
_BOTTOM_RIGHT = (48.1000, 11.5027)
_POSITION = (48.1050, 11.4950)
_FIELDS = tuple(field * 137 - 900 for field in range(19))


def _mixer():
    """A Mixer bound to three fake fins: (mixer, [yaw, left, right] fins)."""
    fins = [_FakeFin(), _FakeFin(), _FakeFin()]
    mix = mixer.Mixer()
    mix.bind({'servo_yaw': fins[0], 'servo_eleron_left': fins[1], 'servo_eleron_right': fins[2]})
    return mix, fins


def _row_format():
    """recorder.Telemetry.push's precomputed line format, for 19 fields after the uptime."""
    return '@%s_%s@%s\n' % ('000135', 'flight', '%u;' + ';'.join('%s' for _ in _FIELDS))


def checks():
    def check(name, value):
        print('check;%s;%s' % (name, value))

    check('clamp_int', commons.clamp_int(0, 5, 10))
    check('wrap180', commons.wrap180(200))
    check('bank_demand', '%.3f' % commons.bank_demand(90, 1.5, 30.0))
    check('sensirion_crc8', commons.sensirion_crc8(0xBE, 0xEF))
    check('atan2_cd', fixed.atan2_cd(-3000, -4000))
    check('to_str', fixed.to_str(-12345))
    roll = pid.Pid(kp=1.5, ki=0.2, kd=0.1, integral_limit=20, output_limit=30)
    check('pid_steps', ','.join(str(roll.step(500 + step * 900, 10, -300)) for step in range(5)))
    mix, fins = _mixer()
    mix.actuate(50, -30, 10)
    check('mixer_angles', ','.join(str(fin.angle) for fin in fins))
    check('fin_limits', ','.join(str(commons.fin_deflection_limit(speed)) for speed in (0.0, 5.0, 15.0, 30.0, 90.0)))
    bearing, waypoint, leg = navigation.steer(_POSITION, _TOP_LEFT, _BOTTOM_RIGHT)
    check('steer_f32', '%.2f,%.5f,%.5f,%d' % (bearing, waypoint[0], waypoint[1], leg == navigation.TARGET))
    zone = navigation.zone(_TOP_LEFT, _BOTTOM_RIGHT)
    check('approach_f32', '%.2f' % navigation.approach_to(_POSITION, *zone, 120.0, 0.5, 30.0))
    record = (_row_format() % ((123456789,) + _FIELDS)).encode()
    line = bytearray(320)
    size = recorder._wrap(record, len(record), line, len(line), recorder._crc_table(), b'0123456789abcdef')
    check('recorder_wrap', bytes(line[: size - 1]).decode())


def primitives():
    _case('clamp_int', 30000, lambda: commons.clamp_int(0, 5, 10))
    _case('wrap180', 30000, lambda: commons.wrap180(200))
    _case('between', 30000, lambda: commons.between(0.0, 5.0, 10.0))
    _case('magnitude_sq', 30000, lambda: commons.magnitude_sq(3.0, 4.0, 12.0))
    _case('bank_demand', 30000, lambda: commons.bank_demand(90, 1.5, 30.0))
    _case('sensirion_crc8', 30000, lambda: commons.sensirion_crc8(0xBE, 0xEF))
    _case('atan2_cd', 10000, lambda: fixed.atan2_cd(-3000, -4000))
    _case('to_str', 10000, lambda: fixed.to_str(-12345))


def control():
    roll = pid.Pid(kp=1.5, ki=0.2, kd=0.1, integral_limit=20, output_limit=30)
    pitch = pid.Pid(kp=1.5, ki=0.2, kd=0.1, integral_limit=20, output_limit=30)
    yaw = pid.Pid(kp=1.0, ki=0.0, kd=0.05, integral_limit=20, output_limit=30)
    _case('pid_step', 10000, lambda: roll.step(500, 10, -300))

    def three():
        roll.step(500, 10, -300)
        pitch.step(-250, 10, 120)
        yaw.step(80, 10, -40)

    _case('pid_step_x3', 5000, three)
    mix, _fins = _mixer()
    _case('mixer_actuate', 10000, lambda: mix.actuate(5, -3, 1))

    def step():
        # one glide step: airspeed governor, the bank demand, three PIDs, the fused mixer
        speed = commons.magnitude_sq(18.0, 2.5, -1.2) ** 0.5
        mix.limit = commons.fin_deflection_limit(speed)
        bank = commons.bank_demand(40, 1.5, 30.0)
        roll_out = roll.step(fixed.from_float(bank) - 1250, 10, -300)
        pitch_out = pitch.step(-250, 10, 120)
        yaw_out = yaw.step(80, 10, -40)
        mix.actuate(roll_out // 100, pitch_out // 100, yaw_out // 100)

    _case('control_step', 5000, step)


def navigation_cases():
    _case('steer_f32', 1000, lambda: navigation.steer(_POSITION, _TOP_LEFT, _BOTTOM_RIGHT))
    zone = navigation.zone(_TOP_LEFT, _BOTTOM_RIGHT)
    _case('approach_f32', 1000, lambda: navigation.approach_to(_POSITION, *zone, 120.0, 0.5, 30.0))


def recorder_cases():
    row_format = _row_format()
    stamped = (123456789,) + _FIELDS
    _case('telemetry_row_format', 5000, lambda: (row_format % stamped).encode())
    record = (row_format % stamped).encode()
    line = bytearray(320)
    table = recorder._crc_table()
    digits = b'0123456789abcdef'
    _case('recorder_wrap', 5000, lambda: recorder._wrap(record, len(record), line, len(line), table, digits))


def infrastructure():
    _case('ticks_us', 30000, time.ticks_us)
    _case('alloc_bytes256', 10000, lambda: bytearray(256))
    buffer = bytearray(64)
    _case('pack_into_16B', 10000, lambda: struct.pack_into('<IiiI', buffer, 16, 123456, -42, 7, 0xDEADBEEF))


checks()
primitives()
control()
navigation_cases()
recorder_cases()
infrastructure()
print('cases;done')
