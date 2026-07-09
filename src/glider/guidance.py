# guidance.py — the stage-dependent guidance law, sibling of pid.py / mixer.py / navigation.py.
# Turns (stage, heading) into the attitude setpoints + heading error the PIDs chase: the boost
# rod-vertical hold, bank-to-turn toward the landing zone, the three GPS-degrading heading tiers,
# and the low-final-approach centreline tracker. Extracted from tasks/flight.py (doc/plan.md
# structural roadmap #1, findings §20 S03) so the control law is unit-testable without a Flight
# task; per-stage laws dispatch through a table (S04 — the proven sequencer._detect pattern), so a
# new stage is one entry + one method, not a branch in a growing if/elif.
#
# Host-runnable by construction (tools/virtual_flight.py drives the REAL law): dependencies are
# INJECTED — the mission (zone/launch_point), the governor (airspeed for the boost rod gate), and
# databoard-style handles (`position.read()` -> ((lat, lon), source, age_ms), `agl.value()` -> m or
# None). Timing comes in as `now_us` from the caller; only commons.ticks_diff touches ticks.
#
# Results land in the roll_setpoint/pitch_setpoint (centidegree fixnum) + heading_error (int
# degrees) INSTANCE SLOTS rather than a returned tuple — decomposed WITHOUT adding a per-step heap
# allocation (GC is off in flight).

import commons
import controller as controller_mod
import fixed
import navigation
from fixed import fixnum

_STAGE = controller_mod.Stage


def heading_error(target: float, current: float) -> int:
    """Shortest signed heading error (deg), wrapped to [-180, 180] so 350 -> 10 is +20, not -340.
    Integer degrees — sub-degree precision is irrelevant to a servo and lets one modulo replace the
    wrap loop (commons.wrap180, viper bundle)."""
    return commons.wrap180(int(target - current))


class GuidanceConfig:
    """The guidance knobs, resolved from the flight task's config dict ONCE (typed config: one place
    for defaults + doc-in-code; the keys keep their board.config names). `position_window_ms` is the
    caller's default tier-1 freshness gate — the GNSS channels' own databoard windows — so it tracks
    the GNSS rate instead of a magic number; config sets position_age_max_ms TIGHTER to distrust
    GNSS sooner (looser is a no-op: the source is already None past the window)."""

    def __init__(self, config: dict, position_window_ms: int):
        # which flight stages are CONTROL stages and their attitude setpoint; resolved to Stage INT
        # keys ONCE so the hot loop compares integers, never strings. Stages not listed hold neutral.
        self.stages: dict = {_STAGE.NAMES[name]: setpoint
                             for name, setpoint in config.get('stages', {'gliding': {}}).items()
                             if name in _STAGE.NAMES}
        # bank-to-turn: in GLIDING the roll SETPOINT comes from the heading error, so the glider
        # banks into the turn (tight, ~v²/(g·tan(bank))) instead of skidding flat on the rudder
        # (which over-ranges a small zone). gain 0 -> rudder-only steering.
        self.bank_gain: float = config.get('nav_bank_gain', 1.5)
        self.bank_limit: float = config.get('bank_limit', 30)
        # final approach / landing: track the strip CENTRELINE with the FULL fin authority to crab
        # the crosswind out — keep it gliding, not rolling-and-dropping. final_agl 0 -> disabled.
        self.land_bank_gain: float = config.get('land_bank_gain', 1.5)
        self.land_bank_limit: float = config.get('land_bank_limit', 45)
        # the ENDGAME band (fly-long objectives, coludo.md "Gliding"): below this ELEVATION the
        # glide steering opens the full land-bank authority, halving the turn radius so the last
        # seconds spiral tightly around the zone instead of racetracking past it. High up the
        # gentler bank_limit preserves objective #1 (a tight bank costs sink ~load^1.5); 0 -> off.
        self.endgame_alt_m: float = config.get('endgame_alt_m', 50)
        # the LOITER orbit (the "orbit the target to bleed altitude" the docs always intended):
        # within loiter_capture_m of the zone centre the heading command becomes the CIRCLE TANGENT
        # plus an inward correction (bearing + 90 - gain*(distance - radius)) -- the glider CAPTURES
        # a constant-radius orbit instead of bang-banging between overfly and U-turn (measured: the
        # point-steer law swung 184 m racetrack legs and landed on phase luck). R=40 m at 14 m/s
        # needs only ~26 deg of bank -- inside bank_limit, sustainable for the whole descent. R BELOW
        # the cruise-bank minimum (~34 m at 30 deg) destabilizes the pre-endgame orbit: do not shrink.
        self.loiter_radius_m: float = config.get('loiter_radius_m', 30)
        self.loiter_capture_m: float = config.get('loiter_capture_m', 120)
        self.loiter_gain: float = config.get('loiter_gain', 3.0)  # deg of inward cut per m off-circle
        self.final_agl: float = config.get('final_approach_agl', 8)
        self.final_cross_gain: float = config.get('final_cross_gain', 3.0)  # deg intercept per m off
        self.final_intercept: float = config.get('final_intercept_deg', 45)  # max intercept angle
        # boost: engage only PAST the rod (airspeed > boost_engage) — below it the fins have no q to
        # bite and heading is ill-defined near vertical.
        self.boost_engage: float = config.get('boost_engage_speed', 15.0)
        # steering noise filter: an integer EMA on the heading ERROR (the measured heading jitters
        # at the control rate under sensor noise; the nav target is already 10 Hz-cached), shift =
        # the EMA divisor power (3 -> alpha 1/8, tau ~80 ms at 100 Hz). Kills the bank flapping
        # that wobbled the endgame spiral at >=25 % noise. 0 -> off. All-int -> zero alloc (GC-off).
        self.steer_filter_shift: int = config.get('steer_filter_shift', 3)
        # navigation.steer()/approach() are float trig; the GNSS fixes at ~10 Hz, so the target
        # heading is cached and recomputed at most every nav_period_ms (see _target_heading).
        self.nav_period_us: int = config.get('nav_period_ms', 100) * 1000
        self.position_age_max_ms: int = config.get('position_age_max_ms', position_window_ms)


class Guidance:
    """The per-stage control law: setpoint(stage) gates control stages; enter() captures the holds
    on entering control; compute() dispatches the stage's law and fills the setpoint slots."""

    def __init__(self, config: GuidanceConfig, mission, governor, position, agl, elevation=None):
        self._config: GuidanceConfig = config
        self._mission = mission  # the landing zone + launch point live here (may be None)
        self._governor = governor  # airspeed estimate -> the boost rod gate
        self._position = position  # injected handle: read() -> ((lat, lon), source, age_ms)
        self._agl = agl  # injected handle: value() -> height above ground (m) or None
        self._elevation = elevation  # baro height above the pad (m) -> the endgame band (optional)
        # per-stage law table (S04, the sequencer._detect pattern): dispatch is O(1) and a new
        # stage's law is one entry + one method. GLIDING and LANDING share the steering law (it
        # branches on the bank gains internally); anything else configured as a control stage falls
        # back to _hold (configured setpoints + the captured heading).
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

    def setpoint(self, stage: int):
        """The configured attitude setpoint dict for `stage`, or None when it is not a CONTROL stage
        (SETTING/BOOSTING/DONE by default — no actuation under thrust / on the ground)."""
        return self._config.stages.get(stage)

    def reachability(self, glide_ratio: float):
        """Can the glider still glide to the zone from here? The reach = elevation × glide_ratio (how
        far it can travel spending the current height at a nominal L/D); compare to the distance from
        the live fix to the zone target. Returns {reachable, margin_m, distance_m}, or None with no
        fix / zone / elevation. The operator's EARLY warning of an unreachable zone (flight panel) —
        and the groundwork for a deliberate land-short decision instead of a doomed stretch."""
        if self._mission is None or not self._mission.zone or self._elevation is None:
            return None
        elevation = self._elevation.value()          # baro height above the pad (m)
        position, source, _age = self._position.read()
        geometry = self._mission.geometry()
        if elevation is None or position is None or source is None or geometry is None:
            return None
        target = geometry['target']
        distance = navigation.distance(position[0], position[1], target[0], target[1])
        reach = elevation * glide_ratio
        return {'reachable': reach >= distance, 'margin_m': round(reach - distance),
                'distance_m': round(distance)}

    def enter(self, heading: float, roll: fixnum, pitch: fixnum) -> None:
        """Entering a control stage (from a non-control one): capture the heading to hold blind and
        the rod-vertical attitude for the boost hold; invalidate the nav cache so the first
        controlled step steers fresh."""
        self._heading_hold = heading
        self._roll_hold = roll
        self._pitch_hold = pitch
        self._nav_heading = None
        self._error_filtered16 = None  # a fresh control entry seeds the error filter anew

    def compute(self, stage: int, setpoint: dict, heading: float, now_us: int) -> bool:
        """Run `stage`'s law: fill roll_setpoint/pitch_setpoint/heading_error and return True, or
        return False when the fins must hold neutral (boost still on the rod — no q to bite)."""
        law = self._laws.get(stage, self._hold)
        return law(stage, setpoint, heading, now_us)

    def _boost(self, stage: int, setpoint: dict, heading: float, now_us: int) -> bool:
        """BOOSTING: hold the rod-vertical attitude captured at entry, engaging only PAST the rod
        (airspeed > boost_engage); below that the caller neutrals (the rod holds it vertical)."""
        if self._governor.airspeed() < self._config.boost_engage:
            return False
        self.roll_setpoint = self._roll_hold  # centidegree fixnum (captured from the centideg roll)
        self.pitch_setpoint = self._pitch_hold
        self.heading_error = 0  # no nav/yaw steering near vertical (heading is ill-defined)
        return True

    def _steer(self, stage: int, setpoint: dict, heading: float, now_us: int) -> bool:
        """GLIDING / LANDING: steer to the landing zone (three GPS-degrading tiers), banking into
        the turn; low on FINAL approach the target switches to the strip centreline. Setpoints go to
        centidegree fixnum HERE (from_float once, at this boundary) so the PID subtract is plain int."""
        agl = self._agl.value()
        config = self._config
        final = config.final_agl and agl is not None and agl < config.final_agl  # low on final
        # the ENDGAME band: elevation below endgame_alt_m -> full land-bank authority (the turn
        # radius halves, the last seconds spiral around the zone). Costs sink only briefly at the
        # bottom, so objective #1 (fly long) is untouched up high.
        elevation = self._elevation.value() if self._elevation is not None else None
        # endgame = the remaining-altitude FRACTION of the band (None above it): the loiter radius
        # shrinks with it, so the orbit SPIRALS IN onto the centre as the energy runs out.
        endgame = None
        if config.endgame_alt_m and elevation is not None and elevation < config.endgame_alt_m:
            endgame = max(0.0, elevation / config.endgame_alt_m)
        raw_error = heading_error(self._target_heading(heading, final, now_us, endgame), heading)
        self.heading_error = self._filter_error(raw_error)
        self.roll_setpoint = fixed.from_float(setpoint.get('roll', 0.0))
        self.pitch_setpoint = fixed.from_float(setpoint.get('pitch', 0.0))
        if config.land_bank_gain and (final or endgame is not None or stage == _STAGE.LANDING):
            # endgame / final approach / landing: FULL fin authority -- spiral tight, crab the
            # crosswind out; the residual at strong wind is airframe-bound, not a control gap.
            self.roll_setpoint = fixed.from_float(commons.bank_demand(
                self.heading_error, config.land_bank_gain, config.land_bank_limit))
        elif config.bank_gain and stage == _STAGE.GLIDING:  # bank-to-turn toward the zone (vs skid)
            self.roll_setpoint = fixed.from_float(commons.bank_demand(
                self.heading_error, config.bank_gain, config.bank_limit))
        return True

    def _hold(self, stage: int, setpoint: dict, heading: float, now_us: int) -> bool:
        """Any other configured control stage (ground-test configs): hold the configured setpoints
        and the heading captured at entry — no navigation."""
        self.heading_error = heading_error(self._heading_hold, heading)
        self.roll_setpoint = fixed.from_float(setpoint.get('roll', 0.0))
        self.pitch_setpoint = fixed.from_float(setpoint.get('pitch', 0.0))
        return True

    def _filter_error(self, error: int) -> int:
        """The steering noise filter: an all-integer EMA (state x16 fixed, alpha = 1/2^shift) on the
        heading error. A GENUINE target change (|jump| > 90 deg: an overfly flip, a law handover)
        resets the state so steering follows at once -- only per-step sensor jitter is smoothed.
        Zero heap allocation (GC is off in flight); shift 0 disables."""
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
        """The heading to steer in GLIDING / LANDING. High on the glide it homes to the zone
        (navigation.steer: gate -> centre); in the ENDGAME band it heads for the zone CENTRE
        unconditionally -- the gates are doorways for a glider with altitude to spend, but the last
        seconds must spiral down ON the target, not around the nearest doorway (the zone's long
        axis puts a gate ~90 m from the centre); low on FINAL approach it TRACKS the strip
        centreline (navigation.approach), so a crosswind is crabbed out before the narrow
        touchdown. Tiers when homing:
          1. a FRESH fix (< position_age_max_ms) -> steer from the current position (closed-loop);
          2. no fix but a CC-set launch point -> hold the launch->gate bearing (open-loop fallback);
          3. neither -> the captured glide heading (blind).
        (CPU): steer()/approach() are float trig recomputed at most every nav_period_us; between
        refreshes the cached float is returned (the final-approach value rides the same cache — the
        position only moves at the GPS rate anyway)."""
        if self._mission is None or not self._mission.zone:
            return self._heading_hold
        if self._nav_heading is not None and \
                commons.ticks_diff(now_us, self._nav_updated_us) < self._config.nav_period_us:
            return self._nav_heading  # cached — skip the trig this step
        self._nav_updated_us = now_us
        zone = self._mission.zone
        config = self._config
        position, source, age_ms = self._position.read()
        if source is not None and position is not None and age_ms < config.position_age_max_ms:
            if final:
                self._nav_heading = navigation.approach(position, zone[0], zone[1], heading,
                                                        config.final_cross_gain, config.final_intercept)
            else:
                target, _gate_a, _gate_b = navigation.zone(zone[0], zone[1])
                span = navigation.distance(position[0], position[1], target[0], target[1])
                if endgame is not None or span <= config.loiter_capture_m:
                    # LOITER: hold the constant-radius orbit around the centre -- the tangent
                    # heading corrected inward/outward by the radius error. One fixed orbit
                    # direction (+90), so noise never flips the turn into S-hunting. In the
                    # ENDGAME the radius shrinks with the remaining altitude -> a spiral that
                    # collapses onto the centre exactly as the energy runs out.
                    radius = config.loiter_radius_m * (endgame if endgame is not None else 1.0)
                    correction = commons.between(
                        -60.0, config.loiter_gain * (span - radius), 60.0)
                    bearing_centre = navigation.bearing(position[0], position[1],
                                                        target[0], target[1])
                    self._nav_heading = (bearing_centre + 90.0 - correction) % 360.0
                else:  # far out: travel to the zone through the nearer gate, as always
                    self._nav_heading = navigation.steer(position, zone[0], zone[1])[0]
        else:
            launch = self._mission.launch_point()  # tier 2: open-loop from the launch point (CC-set)
            self._nav_heading = navigation.steer(launch, zone[0], zone[1])[0] if launch is not None \
                else self._heading_hold  # tier 3: blind
        return self._nav_heading
