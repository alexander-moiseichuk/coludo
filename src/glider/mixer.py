"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Control-surface mixer (sibling of servo.py / gnss.py). Maps the control axes (roll, pitch, yaw --
each a deflection command in degrees) to per-fin servo angles for the airframe's mixing: ELEVONS (the
two elerons move together for pitch, differentially for roll) + a RUDDER (the yaw fin). A hard +/-
limit on control deflection. Pure integer math, no hardware -- the flight control task (Phase 3) binds
the resolved fin driver objects once (bind()) and then drives them straight from the mixing loop
(actuate()); the per-driver clamp still guards the physical range. mix() keeps the dict form for
tests/host tools.

Signs are config (`surfaces` gains), set during bench alignment: if a surface deflects the wrong way,
flip its gain sign. MECHANICAL NEUTRAL (each fin's true zero) is NOT here -- it lives in the servo
driver (sg90 `trim`, per fin), so it applies to boot / failsafe / control alike; the mixer commands a
COMMON neutral and the driver offsets each fin to its centre.
"""

from commons import SERVO_NEUTRAL_DEG, between

_DEFAULT_SURFACES: dict = {
    'servo_yaw': {'yaw': 1},                          # rudder
    'servo_eleron_left': {'pitch': 1, 'roll': 1},     # elevon
    'servo_eleron_right': {'pitch': 1, 'roll': -1},   # elevon (roll is differential)
}


class Mixer:
    """
    Mix (roll, pitch, yaw) deflection commands -> {fin_name: integer angle}.

    angle = neutral + clamp(sum(gain * axis), +/- limit).  Each fin's mechanical zero is the servo
    driver's `trim`, applied downstream -- the mixer only commands the COMMON neutral.
    """

    def __init__(self, config: dict = None):
        config = config or {}
        self.neutral: int = config.get('neutral_deg', SERVO_NEUTRAL_DEG)
        """
        max control deflection from neutral, per surface. OWNERSHIP: config only seeds the pre-flight
        value -- in flight the Governor OWNS it (rewrites it every update, ∝ 1/v² of airspeed) and
        mix()/actuate() read it. Nothing else may write it: a "configurable limit" would silently
        fight the dynamic-pressure governor.
        """
        self.limit: int = config.get('limit_deg', 45)
        self.surfaces: dict = config.get('surfaces', _DEFAULT_SURFACES)
        """
        (zero-alloc hot path): pre-resolve each surface to (name, base, roll_gain, pitch_gain,
        yaw_gain) where base = neutral (the COMMON zero; each fin's mechanical offset is the servo
        driver's `trim`), and pre-allocate the output dict ONCE. mix() then runs at 100 Hz with NO
        per-call allocation -- it rewrites the shared `_out` in place (the nested axis-dict + per-axis
        `.get()` of the old version are gone). Under the GC-disabled-in-flight policy (coludo.md) this
        keeps the glide from churning the heap.
        """
        self._surfaces: list = [(name, int(self.neutral),
                                 gains.get('roll', 0), gains.get('pitch', 0), gains.get('yaw', 0))
                                for name, gains in self.surfaces.items()]
        self._out: dict = {name: base for name, base, _r, _p, _y in self._surfaces}
        self.bound: bool = False  # bind() resolved the fin drivers -> actuate() is armed
        self._fins: list = []  # (fin, base, roll_gain, pitch_gain, yaw_gain) per bound surface
        self._names: list = []  # the bound surface names, parallel to _fins (operator view only)

    def mix(self, roll: int = 0, pitch: int = 0, yaw: int = 0) -> dict:
        """
        Per-fin integer angle for the given axis deflections (degrees).

        This clamps only the CONTROL AUTHORITY to +/-limit. The absolute PHYSICAL endpoint is enforced
        per-fin by sg90's `[min_deg, max_deg]` clamp -- that is the correct layer: the safe travel is
        per-linkage, not a single global bound, so set each fin's min_deg/max_deg to its mechanical
        range in board.config (e.g. a horn that binds at 135 deg -> max_deg=135) and the deflection
        sum can never drive it past that. Inputs are already integers (flight rounds the PID output) so
        there is no per-call float cast.

        Args:
            roll - roll-axis deflection command (degrees).
            pitch - pitch-axis deflection command (degrees).
            yaw - yaw-axis deflection command (degrees).

        Returns:
            A SHARED dict {fin_name: integer angle}, REUSED on every call (zero-alloc) -- apply it
            immediately, do not retain it across mix() calls; flight._apply consumes it synchronously
            in the same step.
        """
        limit = self.limit
        out = self._out  # hoist the attribute lookup out of the per-fin loop
        for name, base, roll_gain, pitch_gain, yaw_gain in self._surfaces:
            # clamp the CONTROL deflection (authority) to +/-limit, then add to base (the common neutral;
            # each fin's mechanical zero is the sg90 driver's trim). Physical end-stop = sg90 [min,max].
            deflection = between(-limit, roll_gain * roll + pitch_gain * pitch + yaw_gain * yaw, limit)
            out[name] = base + deflection
        return out

    def neutralise(self) -> dict:
        """The neutral (zero-deflection) angle per fin -- the safe / control-disabled output (shared dict)."""
        return self.mix(0, 0, 0)

    def bind(self, fins: dict) -> None:
        """
        Fuse the resolved fin driver objects into the surface table.

        So actuate() drives the servos straight from the mixing loop -- no intermediate dict, no
        per-step name lookup (the old flight._apply items()/get() pair, the biggest single hot-slice in
        the bench_flight breakdown). A surface with no resolved fin is skipped (same tolerance the old
        apply had): it is simply never written.

        Args:
            fins - {surface name: object with set_angle(angle)}; the resolved fin drivers.

        Returns:
            None -- populates self._fins and sets self.bound.
        """
        self._fins = [(fins[name], base, roll_gain, pitch_gain, yaw_gain)
                      for name, base, roll_gain, pitch_gain, yaw_gain in self._surfaces
                      if fins.get(name) is not None]
        self._names = [name for name, _b, _r, _p, _y in self._surfaces if fins.get(name) is not None]
        self.bound = True

    def angles(self) -> dict:
        """
        {surface name: the angle currently commanded to its fin}.

        For the OPERATOR view only -- it allocates a dict, so it is called at the ~0.5 Hz vitals rate,
        never from actuate(). Reads each driver's stored command rather than re-deriving the mix, so
        what the panel shows is literally what the servo was told (findings §27.19).

        Args:
            (none)

        Returns:
            The per-surface commanded angles; empty before bind().
        """
        return {name: getattr(entry[0], 'angle', None)
                for name, entry in zip(self._names, self._fins)}

    def actuate(self, roll: int, pitch: int, yaw: int) -> None:
        """
        mix() fused with the servo write: clamp and set_angle() each bound fin in one loop.

        Clamps each surface's control deflection to +/- limit and set_angle()s the bound fin in the
        same loop (zero-alloc, no output dict). bind() first. Positional args on purpose -- the 100 Hz
        caller must not build a kwargs dict. A held fin does no PWM write (sg90.set_angle
        compare-and-sets).

        Args:
            roll - roll-axis deflection command (degrees).
            pitch - pitch-axis deflection command (degrees).
            yaw - yaw-axis deflection command (degrees).

        Returns:
            None -- writes each bound fin's servo angle directly.
        """
        limit = self.limit
        for fin, base, roll_gain, pitch_gain, yaw_gain in self._fins:
            fin.set_angle(base + between(-limit, roll_gain * roll + pitch_gain * pitch + yaw_gain * yaw,
                                         limit))
