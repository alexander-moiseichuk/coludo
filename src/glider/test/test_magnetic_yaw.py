"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

attitude.py's MAGNETIC yaw: learn the offset while the GNSS track is good, use it when the track is gone.

This is the path that exists for the GNSS dropout at boost, so it is tested against SYNTHETIC fields
rather than the bench magnetometer: a board sitting still reports one constant heading, which cannot
exercise a learn-then-use sequence. HITL cannot exercise it either -- the sim flies while the board does
not -- so this file is the only thing standing behind the logic until a real flight.

Run by `make test`.
"""

import fixed
from tasks import attitude


class _Channel:
    """A databoard channel stub: read() returns (value, source, age)."""

    def __init__(self, value=None, present=True):
        self.value_, self.present = value, present

    def read(self):
        return (self.value_, 'stub' if self.present else None, 0)


class _Task:
    """The parts of Attitude the magnetic path touches, without a Controller or a databoard."""

    _magnetic_heading = attitude.Attitude._magnetic_heading
    _magnetic_yaw = attitude.Attitude._magnetic_yaw

    def __init__(self):
        self._mag = _Channel((1000, 0, 0))     # field along +x -> heading 0
        self._roll_cd = self._pitch_cd = 0
        self._yaw_cd = 0
        self._mag_offset_cd = 0
        self._mag_known = False
        self._mag_shift, self._mag_offset_shift = 2, 1  # fast, so a few steps converge in a test
        self._mag_level_cd = 15 * fixed.SCALE


def _degrees(centidegrees):
    return ((centidegrees + 18000) % 36000 - 18000) / 100.0


def test_heading_comes_from_the_horizontal_axes():
    """A field along +x reads 0 deg; along +y reads -90; the level gate refuses a banked airframe."""
    unit = _Task()
    # not an exact 0: atan2_cd is an integer CORDIC and documents ~0.05 deg of error, so this asserts
    # the angle rather than the arithmetic
    assert abs(_degrees(unit._magnetic_heading())) < 0.1, unit._magnetic_heading()
    unit._mag = _Channel((0, 1000, 0))
    assert abs(_degrees(unit._magnetic_heading()) - -90.0) < 1.0, unit._magnetic_heading()

    """
    NEGATIVE, and the reason the gate exists: past the level limit a flat atan2 is not a heading, so it
    must return NOTHING rather than a plausible number. Tilt compensation would need sin/cos, which the
    integer `fixed` module does not carry.
    """
    unit._roll_cd = 30 * fixed.SCALE
    assert unit._magnetic_heading() is None, 'a banked airframe must not yield a heading'
    unit._roll_cd, unit._pitch_cd = 0, 40 * fixed.SCALE
    assert unit._magnetic_heading() is None, 'a pitched airframe must not yield a heading'
    unit._pitch_cd = 0
    unit._mag = _Channel(None, present=False)
    assert unit._magnetic_heading() is None, 'no fresh mag -> no heading'


def test_offset_is_learned_from_the_track_then_used_without_it():
    """The whole point: the track teaches the offset, and the mag carries yaw once the track is gone."""
    unit = _Task()
    unit._mag = _Channel((1000, 0, 0))  # magnetic heading 0 throughout

    # the airframe is really tracking 90 deg while the mag says 0 -> the offset is 90
    for _ in range(40):
        unit._magnetic_yaw(True, 90.0)
    assert unit._mag_known, 'a usable track must teach the offset'
    assert abs(_degrees(unit._mag_offset_cd) - 90.0) < 2.0, _degrees(unit._mag_offset_cd)

    """
    Now the GNSS drops. Yaw starts wrong (0) and must be pulled to the magnetic heading PLUS the learned
    offset -- 90 deg -- with no track available at all. This is the boost-dropout case.
    """
    unit._yaw_cd = 0
    for _ in range(40):
        unit._magnetic_yaw(False, None)
    assert abs(_degrees(unit._yaw_cd) - 90.0) < 2.0, \
        'yaw should converge on the magnetic reference, got %r' % _degrees(unit._yaw_cd)


def test_an_unlearned_offset_never_steers():
    """
    NEGATIVE: with no track ever seen, the mag must NOT touch yaw.

    An unlearned offset is not a reference -- it is zero, which would assert that magnetic north is the
    ground track. A freely drifting gyro is better than a confidently wrong pull, and this is the state
    every flight starts in.
    """
    unit = _Task()
    unit._yaw_cd = 12345
    for _ in range(20):
        unit._magnetic_yaw(False, None)
    assert unit._yaw_cd == 12345, 'an unlearned mag must leave yaw alone, got %r' % unit._yaw_cd
    assert not unit._mag_known


def test_the_track_wins_while_it_is_available():
    """While tracking, the mag is measured -- never used to steer. Two references fighting is worse than one."""
    unit = _Task()
    unit._mag_known = True
    unit._mag_offset_cd = 0
    unit._yaw_cd = 9000  # 90 deg
    before = unit._yaw_cd
    unit._magnetic_yaw(True, 0.0)  # tracking: this step may only teach
    assert unit._yaw_cd == before, 'the mag must not nudge yaw while the track is in charge'


test_heading_comes_from_the_horizontal_axes()
test_offset_is_learned_from_the_track_then_used_without_it()
test_an_unlearned_offset_never_steers()
test_the_track_wins_while_it_is_available()
print('ok: magnetic yaw -- heading from horizontal axes, level gate, offset learned from the track, '
      'carried without it, unlearned never steers, track takes precedence')
