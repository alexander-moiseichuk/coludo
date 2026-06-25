# mixer.py — control-surface mixer (sibling of servo.py / gnss.py). Maps the control axes (roll,
# pitch, yaw -- each a deflection command in degrees) to per-fin servo angles for the airframe's
# mixing: ELEVONS (the two elerons move together for pitch, differentially for roll) + a RUDDER (the
# yaw fin). Per-fin trim (mechanical neutral alignment) and a hard +/- limit on control deflection.
# Pure integer math, no hardware -- the flight control task (Phase 3) feeds it axis commands and
# applies the angles to the sg90 drivers; the per-driver clamp still guards the physical range.
#
# Signs are config (`surfaces` gains + `trim`), set during bench alignment: if a surface deflects the
# wrong way, flip its gain sign; if its neutral is off, set its trim.

from commons import between

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
        on every call (zero-alloc) -- apply it immediately, do not retain it across mix() calls;
        flight._apply consumes it synchronously in the same step (g1).

        This clamps only the CONTROL AUTHORITY to +/-limit. The absolute PHYSICAL endpoint is enforced
        per-fin by sg90's `[min_deg, max_deg]` clamp (g2) -- that is the correct layer: the safe travel
        is per-linkage, not a single global bound, so set each fin's min_deg/max_deg to its mechanical
        range in board.config (e.g. a horn that binds at 135 deg -> max_deg=135) and the trim+deflection
        sum can never drive it past that. Inputs are already integers (flight rounds the PID output) so
        there is no per-call float cast (g3)."""
        limit = self.limit
        out = self._out  # g3: hoist the attribute lookup out of the per-fin loop
        for name, base, roll_gain, pitch_gain, yaw_gain in self._surfaces:
            # clamp the CONTROL deflection (authority) to +/-limit, then add to base (neutral+trim);
            # the absolute physical end-stop is sg90's per-fin [min_deg, max_deg] (g2, g13)
            deflection = between(-limit, roll_gain * roll + pitch_gain * pitch + yaw_gain * yaw, limit)
            out[name] = base + deflection
        return out

    def neutralise(self) -> dict:
        """The neutral (zero-deflection) angle per fin -- the safe / control-disabled output (shared dict)."""
        return self.mix(0, 0, 0)
