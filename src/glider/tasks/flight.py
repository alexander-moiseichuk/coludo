# tasks/flight.py — Phase 3 stabilization loop. @task.activity('flight'). At `schedule_hz` it reads the
# IMU 'attitude' (heading, roll, pitch), runs a PID per axis to the current stage's setpoint (+ heading
# hold), mixes the result to the fins (mixer.py) and writes them via sg90.update(). Per-stage: the
# `stages` config names the CONTROL stages and their setpoint (GLIDING = wings-level + steer to the
# landing zone, LANDING = its own flare setpoint, straight-and-level); any other stage
# (SETTING/BOOSTING/DONE) holds the fins neutral. In GLIDING the yaw heading setpoint comes from navigation.py
# in three GPS-degrading tiers (_target_heading): live fix -> steer from the current position; no fix
# but a CC-set launch point -> hold the launch->gate bearing (open-loop); neither -> the captured glide
# heading. LANDING locks the heading. Degraded: stale/absent attitude -> neutral.
#
# Scheduling: schedule_hz > 0 -> a machine.Timer ticks the step, so the control law gets a regular slice
# independent of what other asyncio tasks are doing (deterministic, e.g. while the laser hammers I2C in
# landing). schedule_hz == 0 -> a plain asyncio loop at period_ms (reconfigure/debug; subject to the ~10 ms
# asyncio floor). Default 100 Hz timer = ~1 m per control step at 100 m/s. Gains default to 0 and the
# task is disabled by default -- it cannot move a surface until enabled + tuned on the airframe.

import asyncio
import time

import airspeed
import commons
import controller as controller_mod
import databoard
import inspector
import mixer
import navigation
import pid
import task

_STAGE = controller_mod.Stage


@task.activity('flight')
class Flight(task.Task):
    """Attitude-hold stabilization: GLIDING-gated, timer- or asyncio-scheduled, fail-safe to neutral."""

    _AXES: tuple = ('roll', 'pitch', 'yaw')

    @staticmethod
    def _heading_error(target: float, current: float) -> int:
        """Shortest signed heading error (deg), wrapped to [-180, 180] so 350 -> 10 is +20, not -340.
        Integer degrees -- sub-degree precision is irrelevant to a servo and lets one modulo replace the
        wrap loop. The wrap itself is the shared commons.wrap180 (viper bundle)."""
        return commons.wrap180(int(target - current))

    async def setup(self) -> bool:
        board = self.controller.config
        self._mixer = mixer.Mixer(board.get('mixer', {}))
        self._schedule_hz: int = self.config.get('schedule_hz', 100)  # > 0 -> timer; 0 -> asyncio at period_ms
        self._period_ms: int = self.config.get('period_ms', 20)
        self._dt: float = (1.0 / self._schedule_hz) if self._schedule_hz > 0 else (self._period_ms / 1000.0)
        self._dt_us: int = int(self._dt * 1000000)  # nominal slice in µs -> float dt + integer dt_ms fallback
        gains = self.config.get('gains', {})
        limit = self._mixer.limit
        self._pid = {axis: pid.Pid(output_limit=limit,
                                   integral_limit=self.config.get('integral_limit', limit),
                                   **gains.get(axis, {})) for axis in self._AXES}
        # per-stage behaviour: which flight stages are CONTROL stages and their attitude setpoint.
        # Stages not listed hold the fins neutral (SETTING/BOOSTING/DONE -- no actuation under thrust /
        # on the ground). The config names stages by string; resolve to Stage INT keys ONCE (via
        # Stage.NAMES) so the hot loop compares integers, never strings (Stage exists for exactly this).
        self._stages: dict = {_STAGE.NAMES[name]: setpoint
                              for name, setpoint in self.config.get('stages', {'gliding': {}}).items()
                              if name in _STAGE.NAMES}
        # bank-to-turn: in GLIDING the roll SETPOINT comes from the heading error (navigation.bank_demand),
        # so the glider banks into the turn toward the zone instead of skidding flat on the rudder (which
        # over-ranges a small zone). gain 0 -> rudder-only (the old wings-level steering).
        self._bank_gain: float = self.config.get('nav_bank_gain', 1.5)
        self._bank_limit: float = self.config.get('bank_limit', 30)
        # final approach: below final_approach_agl the loop stops homing to the centre POINT and
        # TRACKS the strip CENTRELINE (navigation.approach) using the FULL fin authority (45 deg) to crab
        # the crosswind out -- keep it gliding, not rolling-and-dropping. The crosswind envelope is
        # airframe-bound (~8 m/s onto the strip; >10 m/s is beyond a 14 m/s glider). final_approach_agl
        # 0 -> disabled.
        self._land_bank_gain: float = self.config.get('land_bank_gain', 1.5)
        self._land_bank_limit: float = self.config.get('land_bank_limit', 45)
        self._final_agl: float = self.config.get('final_approach_agl', 8)
        self._final_cross_gain: float = self.config.get('final_cross_gain', 3.0)  # deg intercept per m off
        self._final_intercept: float = self.config.get('final_intercept_deg', 45)  # max intercept angle
        self._agl = databoard.Databoard.parameter('agl')  # height above ground -> final-approach trigger
        self._attitude = databoard.Databoard.parameter('attitude')  # (heading, roll, pitch)
        self._position = databoard.Databoard.parameter('position')  # (lat, lon) for landing-zone navigation
        # dynamic-pressure fin governor (coludo.md "Fin authority"): cap fin control deflection by
        # airspeed (torque ∝ v²). Airspeed is fused from the accel backbone + the GNSS speed corrector;
        # the multiplier (board.config, default 1.0) scales the whole 1/v² schedule -- the safety dial.
        self._accel = databoard.Databoard.parameter('accel')  # (x, y, z) in g -> airspeed integration
        self._gnss_speed = databoard.Databoard.parameter('speed')  # GNSS ground speed (m/s) corrector
        self._airspeed = airspeed.AirspeedEstimator()
        self._fin_limit_multiplier: float = board.get('fin_limit_multiplier', 1.0)
        # boost stage: hold the rod-vertical attitude captured at BOOSTING entry, engaging only PAST
        # the rod (airspeed > boost_engage_speed); below that the fins stay neutral (the rod holds it).
        self._boost_engage: float = self.config.get('boost_engage_speed', 15.0)
        self._roll_hold: float = 0.0  # captured rod-vertical roll/pitch (set on entering BOOSTING)
        self._pitch_hold: float = 0.0
        self._mission = inspector.Inspector.get('mission')  # the landing zone lives here (may be None)
        # throttle navigation.steer() (sin/cos/atan2) to GPS cadence -- recompute the target heading
        # every nav_period_ms, cache the float, and read the cache at schedule_hz (see _target_heading).
        self._nav_period_us: int = self.config.get('nav_period_ms', 100) * 1000
        # navigation steers only from a position fix THIS fresh. Default to the GNSS channels' own
        # freshness windows (max of the position + speed timeout_ms -- the same point the databoard drops
        # `source` to None), so it tracks the GNSS rate instead of a magic number; set TIGHTER in config to
        # distrust GNSS sooner than the databoard does. (Looser than the window is a no-op: source is
        # already None past it.)
        self._position_age_max_ms: int = self.config.get(
            'position_age_max_ms', max(self._position.window_us, self._gnss_speed.window_us) // 1000)
        self._nav_heading = None  # cached target heading (None -> recompute on the next step)
        self._nav_updated_us: int = 0
        self._heading_hold = None  # captured on entering a control stage -> hold that heading
        self._active: bool = False  # in a control stage (PID engaged)
        self._stage = None  # the current control-stage name (for inspect)
        self._steps: int = 0  # control steps run (self-timing for load characterization)
        self._max_step_us: int = 0
        self._last_step_us: int = 0  # ticks_us of the previous control step -> actual dt (finding 1.14.2)
        self._fins = None  # resolved fin objects, cached on the first apply
        self._timer = None
        self._ok = True
        return True

    def _step(self) -> None:
        """One control update (sync, no await -> runs whole in a timer slice). The airspeed estimate + fin
        governor run EVERY step (even when NOT in a control stage) so the deflection cap is warm the instant
        control begins -- e.g. straight off a fast, off-vertical boost. Then gate -> attitude -> PID -> mix
        -> apply. Self-times for the load sweep."""
        start = time.ticks_us()
        # ACTUAL elapsed since the previous step -- every step now, so always fresh (finding 1.14.2 /):
        # a GC pause or a delayed slice makes the real interval longer and the PID I/D + airspeed integral
        # must use it. A long gap (>0.5 s) or the first step falls back to the nominal slice.
        dt_us = time.ticks_diff(start, self._last_step_us)
        self._last_step_us = start
        if dt_us <= 0 or dt_us > 500000:  # first step / long gap (GC pause, delayed slice) -> nominal slice
            dt_us = self._dt_us
        dt = dt_us / 1000000.0  # float seconds: the airspeed integrator (an isolated float, off the PID path)
        dt_ms = dt_us // 1000  # integer ms: the fixed-point PID (no float box)
        # dynamic-pressure fin governor: integrate airspeed (accel backbone) + blend a sane GNSS fix,
        # then cap the mixer's control authority by it (commons.fin_deflection_limit ∝ 1/v², × the safety
        # multiplier). Runs unconditionally so boost speed carries into the glide cap (coludo.md).
        accel = self._accel.value()
        if accel is not None:
            self._airspeed.predict((commons.magnitude_sq(accel[0], accel[1], accel[2]) ** 0.5 - 1.0) * 9.81, dt)
        speed, speed_source, _speed_age = self._gnss_speed.read()
        self._airspeed.correct(speed if speed is not None else 0.0, speed_source is not None)
        cap = commons.fin_deflection_limit(self._airspeed.value()) * self._fin_limit_multiplier
        self._mixer.limit = max(1, int(cap))

        setpoint = self._stages.get(self.controller.stage)  # int key -> None if not a control stage
        if setpoint is None or not self.controller.armed:  # not a control stage, or disarmed -> neutral
            if self._active:  # left the control stages (or disarmed) -> centre the fins
                self._neutral()
                self._active = False
                self._stage = None
            return
        value, source, _age = self._attitude.read()
        if source is None or value is None:  # stale / absent attitude -> degraded -> neutral
            self._neutral()
            return
        heading, roll, pitch = value
        if not self._active:  # entering control (from a non-control stage): capture heading, reset PIDs
            self._active = True
            self._heading_hold = heading
            self._roll_hold = roll  # boost: the rod-vertical attitude to hold through the climb
            self._pitch_hold = pitch
            self._nav_heading = None  # force a fresh steer() on the first controlled step (cache)
            for controller in self._pid.values():
                controller.reset()
        self._stage = self.controller.stage  # Stage id; may switch between control stages
        agl = self._agl.value()
        final = self._final_agl and agl is not None and agl < self._final_agl  # low on final approach
        if self._stage == _STAGE.BOOSTING:
            # boost: hold the captured rod-vertical attitude; engage ONLY past the rod (airspeed >
            # boost_engage) -- below that the fins have no q to bite and the 3-point rod holds it vertical.
            # Heading is ill-defined near vertical -> no nav/yaw steering; the speed governor caps the throw.
            if self._airspeed.value() < self._boost_engage:
                self._neutral()  # still on/near the rod -> no actuation
                return
            roll_setpoint = self._roll_hold
            pitch_setpoint = self._pitch_hold
            heading_error = 0
        else:
            heading_error = self._heading_error(self._target_heading(heading, final), heading)
            roll_setpoint = setpoint.get('roll', 0.0)
            pitch_setpoint = setpoint.get('pitch', 0.0)
            if self._land_bank_gain and (final or self._stage == _STAGE.LANDING):
                # final approach / landing: track the strip centreline (set up in _target_heading) with
                # the FULL fin authority (45 deg) to crab the crosswind out -- keep it gliding, not
                # rolling-and-dropping. The residual at strong wind is airframe-bound, not a control gap.
                roll_setpoint = commons.bank_demand(heading_error, self._land_bank_gain, self._land_bank_limit)
            elif self._bank_gain and self._stage == _STAGE.GLIDING:  # bank-to-turn toward the zone (vs skid)
                roll_setpoint = commons.bank_demand(heading_error, self._bank_gain, self._bank_limit)
        # fixed-point PID: errors -> integer millidegrees at the sensor boundary (the sole boxed float on
        # this path), output millidegrees -> integer degrees for the mixer. heading_error is already an int
        # (deg), so ×1000 stays a small int (no box); roll/pitch cost one float subtract+multiply per axis.
        roll_cmd = self._pid['roll'].step(int((roll_setpoint - roll) * 1000), dt_ms)
        pitch_cmd = self._pid['pitch'].step(int((pitch_setpoint - pitch) * 1000), dt_ms)
        yaw_cmd = self._pid['yaw'].step(heading_error * 1000, dt_ms)  # rudder coordinates the turn (0 in boost)
        # positional (not roll=...) so no kwargs dict is built on the hot path
        self._apply(self._mixer.mix(roll_cmd // 1000, pitch_cmd // 1000, yaw_cmd // 1000))
        self._steps += 1
        elapsed = time.ticks_diff(time.ticks_us(), start)
        if elapsed > self._max_step_us:
            self._max_step_us = elapsed

    def _target_heading(self, heading: float, final: bool) -> float:
        """The heading to steer in GLIDING / LANDING (non-control stages just hold). High on the glide it
        homes to the zone (steer: gate -> centre, three GPS-degrading tiers below); low on FINAL approach
        it instead TRACKS the strip centreline (approach), so a crosswind is crabbed out before
        the narrow touchdown. Tiers when homing:
          1. a FRESH fix (< position_age_max_ms) -> steer from the current position (closed-loop,
             corrects wind drift);
          2. no fix but a launch point (CC-set) -> hold the launch->gate bearing (open-loop fallback);
          3. neither -> the captured glide heading (blind).

        (CPU): navigation.steer()/approach() are float trig (sin/cos/atan2 x several). The GNSS fixes
        at ~10 Hz, so recomputing every 100 Hz step is wasted work that inflates max_step_us. The result
        is CACHED and refreshed at most every nav_period_ms (the loop reads the cached float between);
        the final-approach value rides the same cache (position only moves at the GPS rate anyway)."""
        if self.controller.stage not in (_STAGE.GLIDING, _STAGE.LANDING) or self._mission is None \
                or not self._mission.zone:
            return self._heading_hold
        now = time.ticks_us()
        if self._nav_heading is not None and time.ticks_diff(now, self._nav_updated_us) < self._nav_period_us:
            return self._nav_heading  # cached -- skip the trig this step
        self._nav_updated_us = now
        zone = self._mission.zone
        position, source, age_ms = self._position.read()
        if source is not None and position is not None and age_ms < self._position_age_max_ms:  # tier 1: fresh fix
            self._nav_heading = (navigation.approach(position, zone[0], zone[1], heading,
                                                     self._final_cross_gain, self._final_intercept)
                                 if final else navigation.steer(position, zone[0], zone[1])[0])
        else:
            launch = self._mission.launch_point()  # tier 2: open-loop from the launch point (CC-set)
            self._nav_heading = navigation.steer(launch, zone[0], zone[1])[0] if launch is not None \
                else self._heading_hold  # tier 3: blind
        return self._nav_heading

    def _apply(self, angles: dict) -> None:
        # Resolve the fin objects ONCE and cache them (finding.A): controller.find() is a dict
        # search, and doing it per fin per step (100 Hz) is pure overhead. By the first apply all servo
        # tasks are up (bring-up finishes before any run loop), so the lookup is stable.
        if self._fins is None:
            self._fins = {name: self.controller.find([name])[0] for name in angles}
        for name, angle in angles.items():
            fin = self._fins.get(name)
            if fin is not None:
                fin.set_angle(angle)  # no per-fin dict + compare-and-set: a held fin does no write

    def _neutral(self) -> None:
        self._apply(self._mixer.neutralise())

    async def run(self) -> None:
        if self._schedule_hz > 0:
            await self._run_timer()
        else:
            await self._run_asyncio()

    async def _run_timer(self) -> None:
        """A machine.Timer ticks a ThreadSafeFlag at schedule_hz; the step runs in this task (not the ISR).
        A regular slice regardless of other tasks. The flag coalesces, so an overrun runs the latest
        step (no backlog)."""
        from machine import Timer

        flag = asyncio.ThreadSafeFlag()
        self._timer = Timer(self.config.get('timer_id', 0))
        self._timer.init(freq=self._schedule_hz, mode=Timer.PERIODIC, callback=lambda t: flag.set())
        while True:
            await flag.wait()
            self._step()

    async def _run_asyncio(self) -> None:
        """The escape hatch (schedule_hz == 0): a plain asyncio loop -- reconfigure / debug, no timer."""
        while True:
            await asyncio.sleep_ms(self._period_ms)
            self._step()

    async def finish(self) -> None:
        if self._timer is not None:
            self._timer.deinit()
            self._timer = None
        self._neutral()  # leave the fins centred

    def progress(self) -> tuple:
        """(controlling, steps, stage, updated_us) -- the public control-loop heartbeat, so the watchdog
        (and anything else) need not read private attributes (finding 3.6.1). `controlling` is True only
        in a control stage (PID engaged); `steps` advances each control update; `stage` is the current
        control-stage Stage id (int, or None); `updated_us` is time.ticks_us() of the last control step,
        so a supervisor can judge staleness by TIME directly (not by step-count diffing against its own
        poll cadence)."""
        return self._active, self._steps, self._stage, self._last_step_us

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['schedule'] = 'timer' if self._schedule_hz > 0 else 'asyncio'
        status['schedule_hz'] = self._schedule_hz if self._schedule_hz > 0 else round(1000 / self._period_ms)
        status['active'] = self._active
        status['stage'] = _STAGE.STAGES.get(self._stage)  # id -> operator-facing name (None if not active)
        status['steps'] = self._steps  # load sweep: compare steps/sec + max_step_us vs board_health load
        status['max_step_us'] = self._max_step_us
        return status
