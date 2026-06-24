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
        # g3 (zero-alloc hot path): pre-resolve each surface to (name, base, roll_gain, pitch_gain,
        # yaw_gain) where base = neutral + trim, and pre-allocate the output dict ONCE. mix() then runs
        # at 100 Hz with NO per-call allocation -- it rewrites the shared `_out` in place (the nested
        # axis-dict + per-axis `.get()` of the old version are gone). Under the GC-disabled-in-flight
        # policy (coludo.md) this keeps the glide from churning the heap.
        self._surfaces: list = [(name, int(self.neutral + self.trim.get(name, 0)),
                                 gains.get('roll', 0), gains.get('pitch', 0), gains.get('yaw', 0))
                                for name, gains in self.surfaces.items()]
        self._out: dict = {name: base for name, base, _r, _p, _y in self._surfaces}

    def mix(self, roll: int = 0, pitch: int = 0, yaw: int = 0) -> dict:
        """Per-fin integer angle for the given axis deflections (degrees). Returns a SHARED dict REUSED
        on every call (zero-alloc) -- apply it immediately, do not retain it across mix() calls."""
        limit = self.limit
        for name, base, roll_gain, pitch_gain, yaw_gain in self._surfaces:
            deflection = roll_gain * roll + pitch_gain * pitch + yaw_gain * yaw
            if deflection > limit:  # clamp the control deflection (authority), not the trim
                deflection = limit
            elif deflection < -limit:
                deflection = -limit
            self._out[name] = base + deflection
        return self._out

    def neutralise(self) -> dict:
        """The neutral (zero-deflection) angle per fin -- the safe / control-disabled output (shared dict)."""
        return self.mix(0, 0, 0)
