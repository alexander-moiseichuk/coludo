"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

MG90S metal-gear positional fin servo. @task.driver('mg90s'). Electrically IDENTICAL to the SG90 (same
50 Hz frame, ~500..2500 us pulse -> angle, open-loop, no feedback), so this is a THIN SG90 subclass --
the angle->pulse math, per-fin `trim`, the shared slew gate, update()/move() and the probe() self-test
are all inherited. What differs is MECHANICAL: metal gears give higher holding torque, so aerodynamic
load cannot back-drive the horn (an SG90's plastic train lets wind slide the fin off its commanded
angle). Preferred on the yaw fin (the rudder sees the most steady wind pressure).

TRAVEL is per instance via the component's min_deg/max_deg, whose SPAN must equal the servo's
mechanical travel so one command-degree maps to one degree of rotation. The 180deg variant uses the
default (0..180) and drops in exactly where an SG90 was. The 360deg-travel MG90S wants its 360deg
CENTRED on the mixer neutral (90): `min_deg: -90, max_deg: 270` -> command 90 = servo mid = 1500 us =
fin centre (so boot / probe / failsafe all sit centred), and +/-45 command = +/-45deg of fin. A fin
only uses neutral +/- the mixer limit, so the spare range is unused and the ~0.18deg pulse step is
still plenty. MIXED fleets are fine -- each fin's `driver` is independent (e.g. mg90s yaw + sg90
elevons); the mixer commands angles by name and is servo-type-blind.

Slew is a touch quicker (metal gear) and the probe draw window is shifted up (more current). Both are
datasheet-approximate TYPE defaults -- tune per built rig via the component's slew / engine_*_mw config.
"""

import task

from drivers.sg90 import SG90


@task.driver('mg90s')
class MG90S(SG90):
    """
    MG90S metal-gear fin servo -- the SG90 protocol + logic with higher holding torque.

    Center the 360deg-travel instance on the mixer neutral with `min_deg: -90, max_deg: 270`; the
    180deg variant and the default match the SG90 elevons. Everything else (trim, clamp, slew gate,
    probe, open-loop reporting) is SG90's.
    """

    _SLEW_MS_PER_60: int = 100   # ~0.1 s / 60deg (metal gear, a touch quicker than SG90's 150)
    _ENGINE_MIN_MW: int = 600    # metal gear draws more -> probe floor shifted up from SG90's 500
    _ENGINE_MAX_MW: int = 5000   # higher stall current -> HIGH-draw ceiling raised from SG90's 3500
