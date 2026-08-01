"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The stage-dependent guidance law, sibling of pid.py / mixer.py / navigation.py. Turns (stage,
heading) into the attitude setpoints + heading error the PIDs chase: the boost rod-vertical hold,
bank-to-turn toward the landing zone, the three GPS-degrading heading tiers, and the
low-final-approach centreline tracker. Extracted from tasks/flight.py so the control law is
unit-testable without a Flight task; per-stage laws dispatch through a table (the proven
sequencer._detect pattern), so a new stage is one entry + one method, not a branch in a growing
if/elif.

Host-runnable by construction (tools/virtual_flight.py drives the REAL law): dependencies are
INJECTED -- the mission (zone/launch_point), the governor (airspeed for the boost rod gate), and
databoard-style handles (`position.read()` -> ((lat, lon), source, age_ms), `agl.value()` -> m or
None). Timing comes in as `now_us` from the caller; only commons.ticks_diff touches ticks.

Results land in the roll_setpoint/pitch_setpoint (centidegree fixnum) + heading_error (int degrees)
INSTANCE SLOTS rather than a returned tuple -- decomposed WITHOUT adding a per-step heap allocation
(GC is off in flight).
"""

import math

import commons
import controller as controller_mod
import fixed
import navigation
from fixed import fixnum

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const

_STAGE = controller_mod.Stage
_G: float = 9.81  # gravity (m/s^2) -> the coordinated-turn radius R = v^2 / (g * tan(bank))
# 'oo' endgame lobe: which side of the centre the current lobe sits, as a SIGN used arithmetically
# (lobe centre = leg_dir * r along the long axis); the two flip by negation at each centre crossing.
_LOBE_B: int = 1   # the gate_b-side lobe
_LOBE_A: int = -1  # the gate_a-side lobe


_RECKON_MAX_STEP_US: int = const(500000)  # dead-reckoning integration step ceiling (0.5 s). A stage hop,
# a GC pause or a first call after a long gap would otherwise integrate one huge step from a heading
# sampled at the far end of it; skipping the advance loses a little travel and never invents any.


class Heading:
    """
    The endgame HOLDING pattern, self-contained like controller.Stage.

    Int ids (cheap to compare/store on MicroPython) + the `PATTERNS` id->name mapping and `NAMES`
    reverse. Config names the pattern by string; resolve() turns 'auto'/'o'/'ov'/'oo'/'o-o'
    (case-insensitive; 'o-o' aliases 'oo') into an id once. AUTO defers to the Mission
    (mission.endgame_heading), which picks by the zone's long/short aspect k: k < OVAL_ASPECT -> 'o'
    (single circle), OVAL_ASPECT <= k < OO_ASPECT -> 'ov' (centreline oval), k >= OO_ASPECT -> 'oo'.
    """

    AUTO = const(0)    # the Mission decides o vs oo from the zone shape
    FIG_O = const(1)   # a single circle
    FIG_OO = const(2)  # two lobes along the long axis (an elongated strip)
    FIG_OVAL = const(3)  # a centreline oval / racetrack for a MODERATELY elongated strip
    OVAL_ASPECT: float = 2.0  # long/short aspect >= this flies the oval ('ov'); below it a single circle ('o')
    OO_ASPECT: float = 6.0    # ...and >= this flies the two-lobe 'oo' (the converging oval wins up to here)
    PATTERNS: dict = {AUTO: 'auto', FIG_O: 'o', FIG_OO: 'oo', FIG_OVAL: 'ov'}
    NAMES: dict = {name: pattern_id for pattern_id, name in PATTERNS.items()}

    @classmethod
    def resolve(cls, name) -> int:
        """
        A config string -> the pattern id.

        'auto'/'o'/'ov'/'oo'/'o-o', case-insensitive, 'o-o' aliasing 'oo'. An unknown value falls back
        to AUTO (the Mission decides).

        Args:
            name - the config pattern name (any case; 'o-o' aliases 'oo').

        Returns:
            The Heading id (AUTO/FIG_O/FIG_OO); AUTO for an unrecognised name.
        """
        return cls.NAMES.get(str(name).lower().replace('-', ''), cls.AUTO)


def heading_error(target: float, current: float) -> int:
    """
    Shortest signed heading error (deg), wrapped to [-180, 180] so 350 -> 10 is +20, not -340.

    Integer degrees -- sub-degree precision is irrelevant to a servo and lets one modulo replace the
    wrap loop (commons.wrap180, viper bundle).

    Args:
        target - the desired heading (degrees).
        current - the current heading (degrees).

    Returns:
        The signed error target - current, wrapped to [-180, 180] (integer degrees).
    """
    return commons.wrap180(int(target - current))


class GuidanceConfig:
    """
    The guidance knobs, resolved from the flight task's config dict ONCE.

    Typed config: one place for defaults + doc-in-code; the keys keep their board.config names.
    `position_window_ms` is the caller's default tier-1 freshness gate -- the GNSS channels' own
    databoard windows -- so it tracks the GNSS rate instead of a magic number; config sets
    position_age_max_ms TIGHTER to distrust GNSS sooner (looser is a no-op: the source is already None
    past the window).
    """

    def __init__(self, config: dict, position_window_ms: int):
        # which flight stages are CONTROL stages and their attitude setpoint; resolved to Stage INT
        # keys ONCE so the hot loop compares integers, never strings. Stages not listed hold neutral.
        self.stages: dict = {_STAGE.NAMES[name]: setpoint
                             for name, setpoint in config.get('stages', {'gliding': {}}).items()
                             if name in _STAGE.NAMES}
        """
        the configured roll/pitch setpoints as centidegree fixnums, resolved ONCE here (they are
        per-flight constants). _steer/_hold read these instead of `fixed.from_float(setpoint.get(...))`
        every tick -- that boxed a float on the 100 Hz GC-off path, and the roll box was discarded
        whenever bank-to-turn overwrote it.
        """
        self.setpoints_fx: dict = {stage_id: (fixed.from_float(setpoint.get('roll', 0.0)),
                                              fixed.from_float(setpoint.get('pitch', 0.0)))
                                   for stage_id, setpoint in self.stages.items()}
        """
        bank-to-turn: in GLIDING the roll SETPOINT comes from the heading error, so the glider
        banks into the turn (tight, ~v²/(g·tan(bank))) instead of skidding flat on the rudder
        (which over-ranges a small zone). gain 0 -> rudder-only steering.
        """
        self.bank_gain: float = config.get('nav_bank_gain', 1.5)
        self.bank_limit: float = config.get('bank_limit', 30)
        # final approach / landing: track the strip CENTRELINE with the FULL fin authority to crab
        # the crosswind out -- keep it gliding, not rolling-and-dropping. final_agl 0 -> disabled.
        self.land_bank_gain: float = config.get('land_bank_gain', 1.5)
        self.land_bank_limit: float = config.get('land_bank_limit', 45)
        """
        the ENDGAME band (fly-long objectives, coludo.md "Gliding"): below this ELEVATION the
        glide steering opens the full land-bank authority, halving the turn radius so the last
        seconds spiral tightly around the zone instead of racetracking past it. High up the
        gentler bank_limit preserves the fly-long objective (a tight bank costs sink ~load^1.5); 0 -> off.
        """
        self.endgame_alt_m: float = config.get('endgame_alt_m', 50)
        """
        ENDGAME airspeed-gated bank (doc/specs/coludo.md "Turn-radius limit"): instead of the fixed
        land_bank_limit, the endgame spiral banks as steep as the LIVE airspeed safely allows and no
        steeper. A coordinated turn at bank phi pulls load n=1/cos(phi), which raises the stall speed to
        stall_speed_1g*sqrt(n); holding airspeed >= stall_margin*that bounds phi -- steeper than 45 deg
        when there is energy (a tighter R_min -> closer touchdown), collapsing to level as speed decays.
        Capped at endgame_max_bank (structural). stall_speed_1g 0 -> off (fall back to land_bank_limit).
        """
        self.stall_speed_1g: float = config.get('stall_speed_1g', 9.0)  # 1-g stall speed (m/s)
        self.stall_margin: float = config.get('stall_margin', 1.2)      # airspeed cushion above stall
        self.endgame_max_bank: float = config.get('endgame_max_bank', 60)  # structural bank ceiling (deg)
        """
        the LOITER orbit (the "orbit the target to bleed altitude" the docs always intended):
        within loiter_capture_m of the zone centre the heading command becomes the CIRCLE TANGENT
        plus an inward correction (bearing + 90 - gain*(distance - radius)) -- the glider CAPTURES
        a constant-radius orbit instead of bang-banging between overfly and U-turn (measured: the
        point-steer law swung 184 m racetrack legs and landed on phase luck). R=40 m at 14 m/s
        needs only ~26 deg of bank -- inside bank_limit, sustainable for the whole descent. R BELOW
        the cruise-bank minimum (~34 m at 30 deg) destabilizes the pre-endgame orbit: do not shrink.
        """
        self.loiter_radius_m: float = config.get('loiter_radius_m', 30)
        self.loiter_capture_m: float = config.get('loiter_capture_m', 120)
        self.loiter_gain: float = config.get('loiter_gain', 3.0)  # deg of inward cut per m off-circle
        """
        ENDGAME holding pattern: 'o' a single circle, 'oo'/'o-o' two lobes along the long axis (cover an
        elongated strip), or 'auto' (default) -- the Mission decides from the zone shape. Resolved to a
        Heading id ONCE (case-insensitive; the aspect threshold is Heading.OO_ASPECT, a fixed rule).
        """
        self.endgame_pattern: int = Heading.resolve(config.get('endgame_pattern', 'auto'))
        self.final_agl: float = config.get('final_approach_agl', 8)
        self.final_cross_gain: float = config.get('final_cross_gain', 3.0)  # deg intercept per m off
        self.final_intercept: float = config.get('final_intercept_deg', 45)  # max intercept angle
        # boost: engage only PAST the rod (airspeed > boost_engage) -- below it the fins have no q to
        # bite and heading is ill-defined near vertical.
        self.boost_engage: float = config.get('boost_engage_speed', 15.0)
        """
        steering noise filter: an integer EMA on the heading ERROR (the measured heading jitters
        at the control rate under sensor noise; the nav target is already 10 Hz-cached), shift =
        the EMA divisor power (3 -> alpha 1/8, tau ~80 ms at 100 Hz). Kills the bank flapping
        that wobbled the endgame spiral at >=25 % noise. 0 -> off. All-int -> zero alloc (GC-off).
        """
        self.steer_filter_shift: int = config.get('steer_filter_shift', 3)
        # navigation.steer()/approach() are float trig; the GNSS fixes at ~10 Hz, so the target
        # heading is cached and recomputed at most every nav_period_ms (see _target_heading).
        self.nav_period_us: int = config.get('nav_period_ms', 100) * 1000
        self.position_age_max_ms: int = config.get('position_age_max_ms', position_window_ms)


class Guidance:
    """
    The per-stage control law.

    setpoint(stage) gates control stages; enter() captures the holds on entering control; compute()
    dispatches the stage's law and fills the setpoint slots.
    """

    def __init__(self, config: GuidanceConfig, mission, governor, position, agl, elevation=None,
                 wind=None):
        self._config: GuidanceConfig = config
        self._mission = mission  # the landing zone + launch point live here (may be None)
        self._governor = governor  # airspeed estimate -> the boost rod gate
        self._position = position  # injected handle: read() -> ((lat, lon), source, age_ms)
        self._agl = agl  # injected handle: value() -> height above ground (m) or None
        self._elevation = elevation  # baro height above the pad (m) -> the endgame band (optional)
        self._wind = wind  # injected WindEstimator: components() -> (east, north) m/s (optional)
        self._reckoned = None  # the last real fix (lat, lon) -- the dead-reckoning SEED, never mutated
        self._reckon_east: float = 0.0  # metres travelled east of that seed since the fix was lost
        self._reckon_north: float = 0.0  # metres travelled north of it (see _reckon: metres, not degrees)
        self._reckoned_us: int = 0  # when the reckoned offset was last advanced (ticks_us)
        self.reckoning: bool = False  # True while navigating on dead reckoning -- telemetry reads it
        """
        per-stage law table (the sequencer._detect pattern): dispatch is O(1) and a new
        stage's law is one entry + one method. GLIDING and LANDING share the steering law (it
        branches on the bank gains internally); anything else configured as a control stage falls
        back to _hold (configured setpoints + the captured heading).
        """
        self._laws: dict = {_STAGE.BOOSTING: self._boost, _STAGE.GLIDING: self._steer,
                            _STAGE.LANDING: self._steer}
        self.roll_setpoint: fixnum = 0  # compute() writes, the PID caller reads (instance slots,
        self.pitch_setpoint: fixnum = 0  # not a per-step tuple -> no hot-path allocation)
        self.heading_error: int = 0
        self._heading_hold = None  # captured on entering a control stage -> the blind fallback
        self._roll_hold: fixnum = 0  # captured rod-vertical attitude (set on entering control,
        self._pitch_hold: fixnum = 0  # held through the boost climb)
        self._nav_heading = None  # cached target heading (None -> recompute on the next compute)
        self._nav_updated_us: int = 0
        self._error_filtered16 = None  # the heading-error EMA state, x16 fixed (None -> seed next)
        self._leg_dir: int = _LOBE_B  # 'oo' endgame: current lobe (_LOBE_B/_LOBE_A sign), flips at a crossing
        self._was_near = False  # 'oo' hysteresis: glider was within a lobe radius of the centre last tick

    def setpoint(self, stage: int):
        """
        The configured attitude setpoint dict for a stage, or None when it is not a CONTROL stage.

        SETTING/BOOSTING/DONE are not control stages by default -- no actuation under thrust / on the
        ground.

        Args:
            stage - the flight stage (Stage int id).

        Returns:
            The stage's attitude setpoint dict, or None when `stage` is not a control stage.
        """
        return self._config.stages.get(stage)

    def reachability(self, glide_ratio: float, wind_e: float = 0.0, wind_n: float = 0.0,
                     airspeed: float = 0.0):
        """
        Can the glider still glide to the zone from here?

        The still-air reach = elevation × glide_ratio (how far it travels spending the current height at
        a nominal L/D). The wind component ALONG the bearing to target scales the GROUND range --
        ground_range = reach × (1 + w_along/airspeed): a tailwind toward the zone extends it, a headwind
        shrinks it. Compared to the distance from the live fix to the zone target. The operator's EARLY
        warning of an unreachable zone (flight panel) -- and the groundwork for a deliberate land-short
        decision instead of a doomed stretch.

        Args:
            glide_ratio - the nominal L/D used to turn height into still-air range.
            wind_e - eastward wind component (m/s); 0 skips the wind projection.
            wind_n - northward wind component (m/s).
            airspeed - the airspeed (m/s) the ground-range scaling divides by; 0 skips the projection.

        Returns:
            {reachable, margin_m, distance_m, headwind_mps}, or None with no fix / zone / elevation.
        """
        if self._mission is None or not self._mission.zone or self._elevation is None:
            return None
        # FRESH baro only -- a stale scalar channel extrapolates without bound, and this figure tells
        # the operator whether the zone is still reachable (see sequencer._detect_launch)
        elevation, elevation_source, _elevation_age = self._elevation.read()
        if elevation_source is None:
            elevation = None
        position, source, _age = self._position.read()
        geometry = self._mission.geometry()
        if elevation is None or position is None or source is None or geometry is None:
            return None
        target = geometry['target']
        distance, target_bearing = navigation.range_bearing(position[0], position[1], target[0], target[1])
        reach = elevation * glide_ratio
        headwind = 0.0
        if airspeed > 0.0:  # project the wind onto the bearing to target (+ = tailwind, extends the reach)
            bearing_r = math.radians(target_bearing)  # reuse the fused bearing -- no 2nd geographic pass
            along = wind_e * math.sin(bearing_r) + wind_n * math.cos(bearing_r)
            reach *= max(0.0, 1.0 + along / airspeed)
            headwind = -along                        # report the HEADWIND (opposing) component, signed
        return {'reachable': reach >= distance, 'margin_m': round(reach - distance),
                'distance_m': round(distance), 'headwind_mps': round(headwind, 1)}

    def min_turn_radius(self, bank_deg: float) -> float:
        """
        The tightest coordinated turn the airframe can HOLD at `bank_deg` and its LIVE airspeed.

        `R = v^2 / (g * tan(bank))`. A physical restriction, not a tuning knob -- the guidance clamps
        the commanded loiter/endgame radius to this so it never asks for a turn the airframe cannot fly
        (at the 45deg land-bank limit and ~14 m/s trim that floor is ~20 m, and it BOUNDS landing
        accuracy; see doc/specs/coludo.md 'Turn-radius limit').

        Args:
            bank_deg - the bank angle to evaluate the radius at (degrees).

        Returns:
            The minimum turn radius in metres; 0 when airspeed/bank are unusable.
        """
        airspeed = self._governor.airspeed()
        tan_bank = math.tan(math.radians(bank_deg))
        if airspeed <= 0.0 or tan_bank <= 0.0:
            return 0.0
        return airspeed * airspeed / (_G * tan_bank)

    def endgame_bank(self) -> float:
        """
        The steepest bank the ENDGAME spiral may hold at the LIVE airspeed, stall-margin bounded.

        A coordinated turn at bank phi pulls load n = 1/cos(phi), which raises the stall speed to
        stall_speed_1g*sqrt(n); requiring airspeed >= stall_margin*stall_speed_1g*sqrt(n) bounds phi to
        acos((stall_margin*stall_speed_1g / airspeed)^2). Steeper than the fixed land_bank_limit when
        the airspeed allows (a tighter R_min -> a closer touchdown), collapsing toward wings-level as
        speed decays toward stall. Capped at the structural endgame_max_bank.

        Args:
            (none -- reads self._governor.airspeed() + the config)

        Returns:
            Bank angle in degrees; 0.0 when too slow to bank; land_bank_limit when disabled
            (stall_speed_1g 0) or the airspeed estimate is unusable.
        """
        config = self._config
        floor = config.stall_speed_1g * config.stall_margin  # min airspeed to hold a 1-g turn with margin
        if floor <= 0.0:
            return config.land_bank_limit  # gate disabled -> the fixed conservative limit
        airspeed = self._governor.airspeed()
        if airspeed <= floor:
            return 0.0  # too slow to bank without stalling -> hold the wings level until speed recovers
        return min(config.endgame_max_bank, math.degrees(math.acos((floor / airspeed) ** 2)))

    def landing_turn_radius(self):
        """
        The endgame turn-radius floor at the bank the endgame will ACTUALLY hold -- the precision bound
        reported for a land-short-vs-stretch decision (flight-panel telemetry).

        Off `endgame_bank()`, not the fixed `land_bank_limit`: since the airspeed-gated bank landed
        (#5.2) the spiral banks as steep as the live airspeed safely allows, up to endgame_max_bank
        (60 deg), so quoting the 45 deg figure UNDER-states what the glider can do -- 20.0 m against
        11.5 m at 14 m/s. An operator deciding whether the zone is still reachable was reading a bound
        the aircraft had already beaten.

        Args:
            (none -- reads the live gated bank)

        Returns:
            The minimum turn radius in metres; None when the glider is too slow to bank at all, where
            the radius is UNBOUNDED. That case used to return 0.0 -- a radius of zero reads as perfect
            precision on the panel, the exact inverse of the truth.
        """
        bank = self.endgame_bank()
        return self.min_turn_radius(bank) if bank > 0.0 else None

    def enter(self, heading: float, roll: fixnum, pitch: fixnum) -> None:
        """
        Entering a control stage (from a non-control one): capture the holds and reset the nav cache.

        Captures the heading to hold blind and the rod-vertical attitude for the boost hold;
        invalidates the nav cache so the first controlled step steers fresh.

        Args:
            heading - the heading to hold blind (degrees).
            roll - the rod-vertical roll attitude to hold through boost (centidegree fixnum).
            pitch - the rod-vertical pitch attitude to hold through boost (centidegree fixnum).

        Returns:
            None -- captures the holds and resets the nav-cache / filter / endgame-lobe state.
        """
        self._heading_hold = heading
        self._roll_hold = roll
        self._pitch_hold = pitch
        self._nav_heading = None
        self._error_filtered16 = None  # a fresh control entry seeds the error filter anew
        self._leg_dir = _LOBE_B  # the 'oo' endgame starts fresh -- a stale near-latch would drop a crossing
        self._was_near = False

    def compute(self, stage: int, setpoint: dict, heading: float, now_us: int) -> bool:
        """
        Run a stage's law: fill the setpoint slots and report whether the fins may actuate.

        Dispatches through the per-stage law table (falling back to _hold), filling roll_setpoint /
        pitch_setpoint / heading_error.

        Args:
            stage - the flight stage (Stage int id) whose law to run.
            setpoint - the stage's configured setpoint dict (passed through to the law).
            heading - the current heading (degrees).
            now_us - the caller's timestamp (microseconds) for the nav cache.

        Returns:
            True when the setpoint slots are filled and the fins may actuate; False when the fins must
            hold neutral (boost still on the rod -- no q to bite).
        """
        law = self._laws.get(stage, self._hold)
        return law(stage, setpoint, heading, now_us)

    def _boost(self, _unused_stage: int, _unused_setpoint: dict, _unused_heading: float, _unused_now_us: int) -> bool:
        """
        BOOSTING: hold the rod-vertical attitude captured at entry.

        Engages only PAST the rod (airspeed > boost_engage); below that the caller neutrals (the rod
        holds it vertical). No nav/yaw steering near vertical, where heading is ill-defined.

        Args:
            stage, setpoint, heading, now_us - the uniform dispatch signature (see compute()); _boost
                reads only the captured hold + the governor's airspeed gate.

        Returns:
            True once past the rod (setpoints filled); False below boost_engage (the caller holds
            neutral).
        """
        if self._governor.airspeed() < self._config.boost_engage:
            return False
        self.roll_setpoint = self._roll_hold  # centidegree fixnum (captured from the centideg roll)
        self.pitch_setpoint = self._pitch_hold
        self.heading_error = 0  # no nav/yaw steering near vertical (heading is ill-defined)
        return True

    def _steer(self, stage: int, _unused_setpoint: dict, heading: float, now_us: int) -> bool:
        """
        GLIDING / LANDING: steer to the landing zone, banking into the turn.

        Steers through the three GPS-degrading tiers; low on FINAL approach the target switches to the
        strip centreline. Setpoints go to centidegree fixnum HERE (from_float once, at this boundary)
        so the PID subtract is plain int.

        Args:
            stage, setpoint, heading, now_us - the uniform dispatch signature (see compute()); _steer
                reads the stage, heading and now_us (setpoint is unused -- setpoints come from the
                config table).

        Returns:
            True -- the setpoint slots are always filled in GLIDING / LANDING.
        """
        # FRESH agl only: the laser reaches ~4 m, so out of range `value()` projects stale samples
        # without bound and a bogus low reading would hold FINAL APPROACH on for the whole glide.
        # See sequencer._detect_landing -- the same staleness ended a flight at apogee.
        agl, agl_source, _agl_age = self._agl.read()
        config = self._config
        final = config.final_agl and agl_source is not None and agl < config.final_agl  # low on final
        """
        the ENDGAME band: elevation below endgame_alt_m -> full land-bank authority (the turn
        radius halves, the last seconds spiral around the zone). Costs sink only briefly at the
        bottom, so the fly-long objective is untouched up high.
        """
        # FRESH baro only: this opens the FULL land-bank authority, so an extrapolated
        # elevation would unlock the endgame bank at the wrong height (or never)
        elevation = None
        if self._elevation is not None:
            elevation, elevation_source, _elevation_age = self._elevation.read()
            if elevation_source is None:
                elevation = None
        # endgame = the remaining-altitude FRACTION of the band (None above it): the loiter radius
        # shrinks with it, so the orbit SPIRALS IN onto the centre as the energy runs out.
        endgame = None
        if config.endgame_alt_m and elevation is not None and elevation < config.endgame_alt_m:
            endgame = max(0.0, elevation / config.endgame_alt_m)
        raw_error = heading_error(self._target_heading(heading, final, now_us, endgame), heading)
        self.heading_error = self._filter_error(raw_error)
        self.pitch_setpoint = config.setpoints_fx[stage][1]  # cached fixnum (per-flight constant)
        if config.land_bank_gain and (final or endgame is not None or stage == _STAGE.LANDING):
            """
            endgame / final approach / landing: FULL fin authority -- spiral tight, crab the crosswind
            out; the residual at strong wind is airframe-bound, not a control gap. In the ENDGAME the
            ceiling is the airspeed-gated bank (steeper than land_bank_limit when energy allows);
            final/LANDING keep the fixed land_bank_limit (a straight crabbed approach, not a spiral).
            """
            limit = self.endgame_bank() if endgame is not None else config.land_bank_limit
            self.roll_setpoint = fixed.from_float(commons.bank_demand(
                self.heading_error, config.land_bank_gain, limit))
        elif config.bank_gain and stage == _STAGE.GLIDING:  # bank-to-turn toward the zone (vs skid)
            self.roll_setpoint = fixed.from_float(commons.bank_demand(
                self.heading_error, config.bank_gain, config.bank_limit))
        else:  # no bank-to-turn configured -> the CONFIGURED roll setpoint (cached fixnum)
            self.roll_setpoint = config.setpoints_fx[stage][0]
        return True

    def _hold(self, stage: int, _unused_setpoint: dict, heading: float, _unused_now_us: int) -> bool:
        """
        Any other configured control stage (ground-test configs): hold the configured setpoints.

        Holds the heading captured at entry -- no navigation.

        Args:
            stage, setpoint, heading, now_us - the uniform dispatch signature (see compute()); _hold
                reads the stage + heading (setpoint and now_us are unused).

        Returns:
            True -- the setpoint slots are always filled.
        """
        self.heading_error = heading_error(self._heading_hold, heading)
        self.roll_setpoint, self.pitch_setpoint = self._config.setpoints_fx[stage]  # cached fixnums
        return True

    def _filter_error(self, error: int) -> int:
        """
        The steering noise filter: an all-integer EMA on the heading error.

        State is x16 fixed, alpha = 1/2^shift. A GENUINE target change (|jump| > 90 deg: an overfly
        flip, a law handover) resets the state so steering follows at once -- only per-step sensor
        jitter is smoothed. Zero heap allocation (GC is off in flight); shift 0 disables.

        Args:
            error - the raw heading error this step (integer degrees).

        Returns:
            The filtered heading error (integer degrees); the raw error unchanged when the filter is
            off or on a genuine target change.
        """
        shift = self._config.steer_filter_shift
        if not shift:
            return error
        error16 = error * 16
        if self._error_filtered16 is None or abs(error16 - self._error_filtered16) > 90 * 16:
            self._error_filtered16 = error16  # seed / follow a real change immediately
            return error
        self._error_filtered16 += (error16 - self._error_filtered16) >> shift
        return self._error_filtered16 // 16

    def _target_heading(self, heading: float, final: bool, now_us: int,
                        endgame=None) -> float:
        """
        The heading to steer in GLIDING / LANDING.

        High on the glide it homes to the zone (navigation.steer: gate -> centre); in the ENDGAME band
        it heads for the zone CENTRE unconditionally -- the gates are doorways for a glider with
        altitude to spend, but the last seconds must spiral down ON the target, not around the nearest
        doorway (the zone's long axis puts a gate ~90 m from the centre); low on FINAL approach it
        TRACKS the strip centreline (navigation.approach), so a crosswind is crabbed out before the
        narrow touchdown. Tiers when homing:
          1. a FRESH fix (< position_age_max_ms) -> steer from the current position (closed-loop);
          2. no fix but a CC-set launch point -> hold the launch->gate bearing (open-loop fallback);
          3. neither -> the captured glide heading (blind).
        Float trig (steer()/approach()) is recomputed at most every nav_period_us; between refreshes
        the cached float is returned (the final-approach value rides the same cache -- the position
        only moves at the GPS rate anyway).

        Args:
            heading - the current heading (degrees), the blind fallback and the along-strip direction.
            final - True low on final approach (track the strip centreline instead of homing).
            now_us - the caller's timestamp (microseconds) for the nav-refresh cache.
            endgame - the remaining-altitude fraction of the endgame band, or None above it.

        Returns:
            The heading to steer (degrees).
        """
        if self._mission is None or not self._mission.zone:
            return self._heading_hold
        if self._nav_heading is not None and \
                commons.ticks_diff(now_us, self._nav_updated_us) < self._config.nav_period_us:
            return self._nav_heading  # cached -- skip the trig this step
        self._nav_updated_us = now_us
        zone = self._mission.zone
        target, gate_a, gate_b = self._mission.zone_points()  # per-flight-constant geometry, memoized
        config = self._config
        position, source, age_ms = self._position.read()
        if source is not None and position is not None and age_ms < config.position_age_max_ms:
            # keep the dead-reckoning seed fresh, so losing the fix starts from the last real one
            self._reckoned, self._reckoned_us, self.reckoning = position, now_us, False
            self._reckon_east = self._reckon_north = 0.0  # re-seeded: the offset restarts at the fix
            if final:
                self._nav_heading = navigation.approach_to(position, target, gate_a, gate_b, heading,
                                                           config.final_cross_gain, config.final_intercept)
            else:
                east, north = navigation.offset(position[0], position[1], target[0], target[1])
                span = math.sqrt(east * east + north * north)
                if endgame is not None or span <= config.loiter_capture_m:
                    """
                    ENDGAME holding pattern (Heading id) -- FIG_O (a single circle) or FIG_OO (two lobes
                    along the long axis, for an elongated strip). AUTO defers to the Mission, which picks
                    from the zone shape.
                    """
                    pattern = config.endgame_pattern
                    if pattern == Heading.AUTO:
                        pattern = self._mission.endgame_heading()
                    if pattern == Heading.FIG_OO:
                        self._nav_heading = self._oo_heading(position, target, gate_b, endgame)
                    elif pattern == Heading.FIG_OVAL:
                        self._nav_heading = self._oval_heading(position, target, gate_b, endgame)
                    else:
                        self._nav_heading = self._circle_heading(east, north, span, endgame)
                else:  # far out: travel to the zone through the nearer gate, as always
                    self._nav_heading = navigation.steer_to(position, zone[0], zone[1],
                                                            target, gate_a, gate_b)[0]
        else:
            """
            No usable fix. GNSS is EXPECTED to be missing for much of a flight -- the boost is a
            high-g, high-vibration, antenna-shadowed few seconds and a receiver that loses lock there
            can take tens of seconds to reacquire, which on a <60 s flight can mean it never returns.
            So this branch is a design case, not an error path.

            Tier 2 is DEAD RECKONING: advance the last known position by the velocity the glider can
            still measure without GNSS -- airspeed along the current heading, plus the last wind
            estimate. It degrades gradually (heading error and stale wind integrate) where the old
            fallback did not degrade at all: it steered the pad->target bearing forever, ignoring that
            the glider had moved, so it flew over the target and kept going. Every input here survives
            a dead GNSS: airspeed is the pitot/governor, heading is the fused compass.

            The wind estimate itself needs GNSS, so it necessarily freezes at its last value. That is
            the dominant DR error and it is bounded and known -- a frozen 5 m/s estimate that is wrong
            by 2 m/s costs ~80 m over 40 s, against the ~720 m a frozen position costs.
            """
            self.reckoning = self._reckoned is not None
            self._nav_heading = self._reckon(now_us, heading, zone, target, gate_a, gate_b)
        return self._nav_heading

    def _reckon(self, now_us: int, heading: float, zone: tuple, target: tuple,
                gate_a: tuple, gate_b: tuple) -> float:
        """
        Navigate with no usable GNSS fix: dead-reckon the position, else fall back to the launch point.

        Seeds from the last GNSS fix the moment one goes stale (`_reckoned` is refreshed on every
        tier-1 pass), then integrates air velocity + wind per call. Falls through to the launch point
        (tier 3) and finally the captured heading (tier 4) when there has never been a fix at all --
        the pad IS a known position, so a GNSS that never locks still steers the right way initially.

        Args:
            now_us - the current time (ticks_us), for the integration step.
            heading - the current fused heading (deg) the air velocity points along.
            zone, target, gate_a, gate_b - the landing-zone geometry from the Mission.

        Returns:
            The heading to steer (deg).
        """
        if self._reckoned is None:
            launch = self._mission.launch_point()  # tier 3: open-loop from the launch point (CC-set)
            return navigation.steer_to(launch, zone[0], zone[1], target, gate_a, gate_b)[0] \
                if launch is not None else self._heading_hold  # tier 4: blind
        step_us = commons.ticks_diff(now_us, self._reckoned_us)
        self._reckoned_us = now_us
        if 0 < step_us < _RECKON_MAX_STEP_US:  # a stage hop / long gap leaves the position untouched
            step_s = step_us / 1000000.0
            airspeed = self._governor.airspeed()
            radians = math.radians(heading)
            east = airspeed * math.sin(radians)
            north = airspeed * math.cos(radians)
            if self._wind is not None:
                wind_east, wind_north = self._wind.components()
                east += wind_east
                north += wind_north
            """
            Accumulate the displacement in METRES from the seed fix, and convert ONCE below -- do NOT
            advance the latitude/longitude per step.

            MEASURED on this board: MicroPython here is single-precision, so a latitude near 48 deg has
            an ULP of ~0.4 m. Adding a 0.2 m step (2 m/s over a 100 ms tick) to it changes NOTHING --
            `(48.01 + 0.2/M_PER_DEG - 48.01) * M_PER_DEG` returns exactly 0.0 -- and even a 4 m step
            lands 4.5 % short at 3.82 m. Per-step integration in degrees therefore quantizes away
            precisely when the glider is slow, which is exactly when dead reckoning has to work; it
            would have looked correct and silently held position. Metres accumulate near zero where
            float32 has resolution to spare, and the single conversion applies the quantization once to
            the whole offset instead of once per tick.
            """
            self._reckon_east += east * step_s
            self._reckon_north += north * step_s
        reckoned = navigation.advance(self._reckoned, self._reckon_east, self._reckon_north)
        return navigation.steer_to(reckoned, zone[0], zone[1], target, gate_a, gate_b)[0]

    def _circle_heading(self, east: float, north: float, span: float, endgame) -> float:
        """
        'o' endgame: a single constant-radius orbit around the centre.

        The tangent heading (bearing + 90) corrected inward/outward by the radius error. One fixed
        orbit direction, so noise never flips the turn into S-hunting; in the ENDGAME the radius
        shrinks with the altitude (a spiral collapsing toward the centre), CLAMPED to the physical min
        turn radius at the phase's bank.

        Args:
            east, north - the offset from the glider to the centre (metres).
            span - the length of that offset (metres).
            endgame - the remaining-altitude fraction of the endgame band, or None (fixed radius).

        Returns:
            The heading to fly (degrees).
        """
        bank = self.endgame_bank() if endgame is not None else self._config.bank_limit
        radius = max(self._config.loiter_radius_m * (endgame if endgame is not None else 1.0),
                     self.min_turn_radius(bank))
        correction = commons.between(-60.0, self._config.loiter_gain * (span - radius), 60.0)
        return (navigation.compass(east, north) + 90.0 - correction) % 360.0

    def _oo_heading(self, position: tuple, target: tuple, gate_b: tuple, endgame) -> float:
        """
        'oo' endgame: two lobes offset along the long axis, covering an elongated strip.

        Two orbits rather than one central circle. Each lobe is a circle of radius r offset r along the
        axis (so it passes through the centre); the lobe FLIPS at each centre crossing but the turn
        SENSE stays fixed (+90) -- both loops turn the same way, so the transition is SMOOTH (the sharp
        sense-reversal of a true figure-8 diverged). r stays bounded (~loiter_radius) and shrinks with
        the endgame altitude, so the pair collapses toward the centre. KNOWN LIMITATION: on a strip
        narrower than ~2*r the lobes reach outside the short edges -- lands out of zone in HITL on the
        ~40 m HPRC strip; fitting r to the width needs the steep bank whose sink then drops it out
        anyway. 'o' stays the reliable in-zone default; 'oo' needs more geometry work.

        Args:
            position - the glider position (lat, lon), decimal degrees.
            target - the zone centre (lat, lon).
            gate_b - the gate that sets the long-axis direction (lat, lon).
            endgame - the remaining-altitude fraction of the endgame band, or None (fixed radius).

        Returns:
            The heading to fly (degrees).
        """
        centre_e, centre_n = navigation.offset(target[0], target[1], gate_b[0], gate_b[1])  # centre -> gate_b
        half_len = math.sqrt(centre_e * centre_e + centre_n * centre_n) or 1.0
        unit_e, unit_n = centre_e / half_len, centre_n / half_len  # unit vector along the long axis
        bank = self.endgame_bank() if endgame is not None else self._config.bank_limit
        radius = max(self.min_turn_radius(bank),
                     self._config.loiter_radius_m * (endgame if endgame is not None else 1.0))
        glider_e, glider_n = navigation.offset(target[0], target[1], position[0], position[1])  # glider vs centre
        if glider_e * glider_e + glider_n * glider_n < radius * radius:  # within a lobe radius -> a crossing
            if not self._was_near:
                # switch to the other lobe (turn sense unchanged)
                self._leg_dir = _LOBE_A if self._leg_dir == _LOBE_B else _LOBE_B
            self._was_near = True
        else:
            self._was_near = False
        # tangent to the current lobe's circle (FIXED +90 sense) + an inward/outward cut to hold the radius
        delta_e = self._leg_dir * radius * unit_e - glider_e  # glider -> lobe centre
        delta_n = self._leg_dir * radius * unit_n - glider_n
        dist = math.sqrt(delta_e * delta_e + delta_n * delta_n) or 1.0
        cut = commons.between(-60.0, self._config.loiter_gain * (dist - radius), 60.0)
        return (navigation.compass(delta_e, delta_n) + 90.0 - cut) % 360.0

    def _oval_heading(self, position: tuple, target: tuple, gate_b: tuple, endgame) -> float:
        """
        'ov' endgame: a CONVERGING centreline oval / racetrack for a moderately elongated strip.

        The 'oo' failure is round lobes bulging +/-r across the SHORT axis; a straight leg has ZERO
        perpendicular extent, so this flies ALONG the long axis holding the centreline (the proven
        approach_to intercept -- proportional to the cross-track offset + capped, so it tracks without
        the S-hunt limit cycle of a raw pursuit) and reverses at each leg end. The KEY is CONVERGENCE:
        the reversal point shrinks with the endgame altitude, so the whole racetrack COLLAPSES onto the
        centre (like the 'o' spiral, but oval) and lands ON the midline near the centre (in-zone).

        Args:
            position - the glider position (lat, lon), decimal degrees.
            target - the zone centre (lat, lon).
            gate_b - the gate that sets the long-axis direction (lat, lon).
            endgame - the remaining-altitude fraction of the endgame band, or None (no collapse).

        Returns:
            The heading to fly (degrees).
        """
        centre_e, centre_n = navigation.offset(target[0], target[1], gate_b[0], gate_b[1])  # centre -> gate_b
        half_len = math.sqrt(centre_e * centre_e + centre_n * centre_n) or 1.0
        unit_e, unit_n = centre_e / half_len, centre_n / half_len  # along-axis unit (toward gate_b)
        glider_e, glider_n = navigation.offset(target[0], target[1], position[0], position[1])  # glider vs centre
        along = glider_e * unit_e + glider_n * unit_n  # signed position along the axis (0 = centre)
        # CONVERGE: the leg reversal point shrinks with the endgame altitude -> the racetrack COLLAPSES
        # onto the centre (like the 'o' spiral, but oval); floored so a small end-to-end oscillation
        # persists down to touchdown rather than a dead stop on the line
        reach = half_len * max(0.15, min(1.0, endgame if endgame is not None else 1.0))
        if self._leg_dir == _LOBE_B and along >= reach:
            self._leg_dir = _LOBE_A
        elif self._leg_dir == _LOBE_A and along <= -reach:
            self._leg_dir = _LOBE_B
        # fly the current leg direction, TRACKING the centreline with the damped approach_to intercept
        # (proportional to the cross-track offset + capped) -- no S-hunt limit cycle
        leg_bearing = navigation.compass(self._leg_dir * unit_e, self._leg_dir * unit_n)
        intercept = commons.between(-self._config.final_intercept, -self._config.final_cross_gain
                                    * navigation.cross_track(position, target, leg_bearing),
                                    self._config.final_intercept)
        return (leg_bearing + intercept) % 360.0
