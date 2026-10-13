"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for the 6-DoF boost (tools/stack_dynamics.py, doc/specs/boost-control.md BC1): the measured
motor, the rod, the frames and signs the HITL host publishes in, a stable stack's straight climb, and the 10-03
replays (tools/stack_replay.py) -- 7C departs, 7D bends and recovers, TMS-7 climbs to its apogee. Run by
`make test` / `make check`.
"""

import math
import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_ROOT, 'tools'))
import stack_dynamics  # noqa: E402
import stack_replay  # noqa: E402

_CALM_STRAIGHT: dict = dict(stack_dynamics.STACKS['tms7'])  # a stable stack with no asymmetry


def _fly(stack, seconds: float, fins: tuple = (0.0, 0.0, 0.0)) -> None:
    """`seconds` of flight in the host's 10 ms steps."""
    for _ in range(int(round(seconds / 0.01))):
        stack.step(0.01, fins)


def test_the_motor_is_the_measured_f15():
    assert 40.5 < stack_dynamics.impulse_of() < 42.5, stack_dynamics.impulse_of()
    assert stack_dynamics.thrust_at(0.30) > 30.0, 'the 32 N spike'
    assert 12.0 < stack_dynamics.thrust_at(2.0) < 15.0, 'the 13-15 N sustain'
    # NEGATIVE: nothing before ignition or after burnout
    assert stack_dynamics.thrust_at(-0.1) == 0.0 and stack_dynamics.thrust_at(2.68) == 0.0
    assert stack_dynamics.thrust_at(3.5) == 0.0


def test_the_rod_holds_the_stack_and_lets_it_go_at_its_length():
    stack = stack_dynamics.Stack(stack_dynamics.STACKS['tms7c'], 4.5, 0.0, 90.0)
    # on the pad, before the thrust beats the weight: 1 g along the nose, no motion
    assert abs(stack.specific[0] - stack_dynamics.G) < 1e-9 and stack.specific[1:] == (0.0, 0.0)
    _fly(stack, 0.25)
    assert stack.on_rod and stack.rates == (0.0, 0.0, 0.0), 'the rod takes the crosswind: no rotation'
    assert abs(stack.position[0]) < 1e-9 and abs(stack.position[1]) < 1e-9
    _fly(stack, 0.15)
    assert not stack.on_rod
    speed = math.sqrt(sum(component * component for component in stack.velocity))
    assert 10.0 < speed < 16.0, speed  # 7C left its 1 m rod at 10.6 m/s


def test_euler_angles_follow_the_aerospace_convention():
    stack = stack_dynamics.Stack(_CALM_STRAIGHT)
    heading, roll, pitch = stack.euler()
    assert abs(pitch - 90.0) < 1e-6, 'the nose straight up'
    # level, nose north, dorsal up: heading 0, roll 0, pitch 0
    stack.attitude = stack_dynamics._quaternion_of(((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)))
    assert all(abs(angle) < 1e-6 for angle in stack.euler()), stack.euler()
    # level, nose east: heading 90; the right wing down 30 degrees: roll +30
    right_down = math.radians(30.0)
    nose, left, dorsal = (1.0, 0.0, 0.0), (0.0, math.cos(right_down), math.sin(right_down)), \
        (0.0, -math.sin(right_down), math.cos(right_down))
    columns = (nose, left, dorsal)
    stack.attitude = stack_dynamics._quaternion_of(tuple(tuple(columns[c][r] for c in range(3)) for r in range(3)))
    heading, roll, pitch = stack.euler()
    assert abs(heading - 90.0) < 1e-6 and abs(roll - 30.0) < 1e-6 and abs(pitch) < 1e-6, (heading, roll, pitch)


def test_each_fin_command_turns_the_stack_its_way():
    """A positive pitch raises the nose toward the dorsal side (-y rate), roll rolls the right wing down (+x),
    yaw turns the nose right (-z): the host's (roll, pitch, yaw) as sim_model reads them."""
    for fins, axis, sign in (((0.0, 5.0, 0.0), 1, -1), ((5.0, 0.0, 0.0), 0, +1), ((0.0, 0.0, 5.0), 2, -1)):
        profile = dict(_CALM_STRAIGHT, fin_moment=(1.0e-4, 1.0e-3, 1.0e-3))
        stack = stack_dynamics.Stack(profile)
        _fly(stack, 0.5)  # off the rod, fast enough to bite
        before = stack.rates[axis]
        _fly(stack, 0.05, fins)
        assert sign * (stack.rates[axis] - before) > 0.05, (fins, stack.rates)
    # NEGATIVE: neutral fins on a symmetric stack in calm air rotate nothing
    stack = stack_dynamics.Stack(_CALM_STRAIGHT)
    _fly(stack, 2.0)
    assert all(abs(rate) < 1e-9 for rate in stack.rates), stack.rates


def test_a_stable_stack_climbs_straight_and_weathercocks_into_the_wind():
    stack = stack_dynamics.Stack(_CALM_STRAIGHT)
    _fly(stack, 2.6)
    assert stack.nose_elevation() > 89.9 and math.hypot(stack.position[0], stack.position[1]) < 1e-6
    speed = math.sqrt(sum(component * component for component in stack.velocity))
    assert 115.0 < speed < 135.0, speed  # TMS-7 burned out at 125 m/s
    # a crosswind toward east: the stable stack turns its nose INTO it (west), and drifts upwind
    stack = stack_dynamics.Stack(_CALM_STRAIGHT, 4.5, 0.0)
    _fly(stack, 2.0)
    assert stack.velocity[0] < 0.0 and stack.position[0] < 0.0, (stack.velocity, stack.position)
    assert 70.0 < stack.nose_elevation() < 89.0, stack.nose_elevation()


def test_the_roll_asymmetry_spins_the_stack_with_the_speed():
    stack = stack_dynamics.Stack(dict(_CALM_STRAIGHT, roll_forcing=-0.34, roll_damping=0.5))
    _fly(stack, 2.5)
    speed = math.sqrt(sum(component * component for component in stack.velocity))
    per_speed = math.degrees(stack.rates[0]) / speed
    # forcing x rho / (2 x damping): the steady roll per m/s, ~24 deg/s (7D coasted at ~20)
    assert -30.0 < per_speed < -18.0, per_speed


def test_the_hand_over_puts_the_glide_where_the_stack_is():
    class Glide:
        pe = pn = alt = vu = 0.0

    stack = stack_dynamics.Stack(_CALM_STRAIGHT, 4.5, 0.0)
    _fly(stack, 3.0)
    glide = Glide()
    stack.hand_over(glide)
    assert (glide.pe, glide.pn, glide.alt) == stack.position and glide.vu == stack.velocity[2]


def test_7c_departs_off_the_rod_and_loops_under_thrust():
    """launches/20261003/TMS-7C: the nose past horizontal at ~1.06 s still burning, a 9.3 g side load, the roll at
    -372 deg/s by 0.47 s."""
    events = stack_replay.replay('tms7c')
    assert events['horizontal'] is not None and events['horizontal'] < 1.3 and events['burning'], events
    assert events['side_g'] > 6.0, events['side_g']
    assert -450.0 < events['roll_at_half'] < -200.0, events['roll_at_half']


def test_7d_bends_over_then_recovers_as_its_cg_moves_forward():
    """launches/20261003/TMS-7D: 17-19 deg of angle of attack early, 2 deg by 2.2 s; the path's minimum 32 deg; burnout
    53.5 m/s, 54 m up, 48 m out; apogee 91-98 m, ~145 m out; the coast's roll ~20 deg/s per m/s."""
    events = stack_replay.replay('tms7d')
    assert 10.0 < events['early_angle'] < 22.0 and events['late_angle'] < 5.0, events
    assert 25.0 < events['path_minimum'] < 45.0, events['path_minimum']
    speed, height, distance = events['burnout']
    assert 45.0 < speed < 62.0 and 40.0 < height < 80.0 and 30.0 < distance < 70.0, events['burnout']
    assert 80.0 < events['apogee'] < 125.0 and 110.0 < events['apogee_range'] < 190.0, events
    assert events['horizontal'] is None or not events['burning'], 'no loop under thrust'
    assert -28.0 < events['roll_per_speed'] < -15.0, events['roll_per_speed']


def test_tms7_climbs_to_its_apogee_and_a_thin_margin_does_not():
    events = stack_replay.replay('tms7')
    assert 430.0 < events['apogee'] < 495.0 and 115.0 < events['burnout'][0] < 135.0, events
    assert events['path_minimum'] > 70.0, events['path_minimum']  # ~12 deg of tilt through the burn
    # NEGATIVE: 7D's stack with 7C's margin loops under thrust in the same wind
    thin = dict(stack_dynamics.STACKS['tms7d'], margin_cal=(0.24, 0.39))
    events = stack_replay.replay('tms7d', thin)
    assert events['horizontal'] is not None and events['horizontal'] < 2.6 and events['burning'], events


test_the_motor_is_the_measured_f15()
test_the_rod_holds_the_stack_and_lets_it_go_at_its_length()
test_euler_angles_follow_the_aerospace_convention()
test_each_fin_command_turns_the_stack_its_way()
test_a_stable_stack_climbs_straight_and_weathercocks_into_the_wind()
test_the_roll_asymmetry_spins_the_stack_with_the_speed()
test_the_hand_over_puts_the_glide_where_the_stack_is()
test_7c_departs_off_the_rod_and_loops_under_thrust()
test_7d_bends_over_then_recovers_as_its_cg_moves_forward()
test_tms7_climbs_to_its_apogee_and_a_thin_margin_does_not()
print('ok: the 6-DoF boost: the measured F15, the rod, the frames, the fin signs, the roll, and the 10-03 replays')
