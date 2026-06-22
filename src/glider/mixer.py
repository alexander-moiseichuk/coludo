# mixer.py — control-surface mixer (sibling of servo.py / gnss.py). Maps the control axes (roll,
# pitch, yaw -- each a deflection command in degrees) to per-fin servo angles for the airframe's
# mixing: ELEVONS (the two elerons move together for pitch, differentially for roll) + a RUDDER (the
# yaw fin). Per-fin trim (mechanical neutral alignment) and a hard +/- limit on control deflection.
# Pure integer math, no hardware -- the flight control task (Phase 3) feeds it axis commands and
# applies the angles to the sg90 drivers; the per-driver clamp still guards the physical range.
#
# Signs are config (`surfaces` gains + `trim`), set during bench alignment: if a surface deflects the
# wrong way, flip its gain sign; if its neutral is off, set its trim.

_DEFAULT_SURFACES: dict = {
    'servo_yaw': {'yaw': 1},                          # rudder
    'servo_eleron_left': {'pitch': 1, 'roll': 1},     # elevon
    'servo_eleron_right': {'pitch': 1, 'roll': -1},   # elevon (roll is differential)
}


class Mixer:
    """Mix (roll, pitch, yaw) deflection commands -> {fin_name: integer angle}:
    angle = neutral + trim + clamp(sum(gain * axis), +/- limit)."""

    def __init__(self, config: dict = None):
        config = config or {}
        self.neutral: int = config.get('neutral_deg', 90)
        self.limit: int = config.get('limit_deg', 45)  # max control deflection from neutral, per surface
        self.surfaces: dict = config.get('surfaces', _DEFAULT_SURFACES)
        self.trim: dict = config.get('trim', {})  # per-fin neutral offset (deg)

    def mix(self, roll: int = 0, pitch: int = 0, yaw: int = 0) -> dict:
        """Per-fin integer angle for the given axis deflections (degrees)."""
        axes = {'roll': roll, 'pitch': pitch, 'yaw': yaw}
        angles = {}
        for name, gains in self.surfaces.items():
            deflection = 0
            for axis, gain in gains.items():
                deflection += gain * axes.get(axis, 0)
            if deflection > self.limit:  # clamp the control deflection (authority), not the trim
                deflection = self.limit
            elif deflection < -self.limit:
                deflection = -self.limit
            angles[name] = int(self.neutral + self.trim.get(name, 0) + deflection)
        return angles

    def neutralise(self) -> dict:
        """The neutral (zero-deflection) angle per fin -- the safe / control-disabled output."""
        return self.mix(0, 0, 0)
