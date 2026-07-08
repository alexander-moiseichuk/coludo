# tasks/flight.py — Phase 3 stabilization loop. @task.activity('flight'). At `schedule_hz` it runs the
# control PIPELINE: dt -> airspeed Governor (fin-authority cap, adaptively throttled) -> control-stage
# gate -> attitude -> Guidance (per-stage setpoints + heading) -> PID per axis -> mixer actuate. The
# control LAW lives in guidance.py and the airspeed/authority POLICY in governor.py (doc/plan.md
# structural roadmap #1) — this task is the orchestration: databoard reads, arming/degraded gates,
# scheduling, and the PID->mixer->servo drive. Per-stage behaviour, the GPS-degrading heading tiers,
# boost hold and final approach are guidance.py's; the adaptive estimator throttle is governor.py's.
# Degraded: stale/absent attitude -> neutral. Disarmed / non-control stage -> neutral.
#
# Scheduling: schedule_hz > 0 -> a machine.Timer ticks the step, so the control law gets a regular slice
# independent of what other asyncio tasks are doing (deterministic, e.g. while the laser hammers I2C in
# landing). schedule_hz == 0 -> a plain asyncio loop at period_ms (reconfigure/debug; subject to the ~10 ms
# asyncio floor). Default 100 Hz timer = ~1 m per control step at 100 m/s. Gains default to 0 and the
# task is disabled by default -- it cannot move a surface until enabled + tuned on the airframe.

import asyncio
import time

import controller as controller_mod
import databoard
import fixed
import governor
import guidance
import inspector
import mixer
import pid
import task

_STAGE = controller_mod.Stage


@task.activity('flight')
class Flight(task.Task):
    """Attitude-hold stabilization: GLIDING-gated, timer- or asyncio-scheduled, fail-safe to neutral."""

    _AXES: tuple = ('roll', 'pitch', 'yaw')

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
        self._attitude = databoard.Databoard.parameter('attitude')  # (heading, roll, pitch)
        self._rate = databoard.Databoard.parameter('rate')  # (roll, pitch, yaw) angular rate -> PID D term
        # the governor (governor.py): airspeed estimate -> dynamic-pressure fin-authority cap on the
        # mixer, adaptively throttled once the glide settles. Reads accel (backbone) + GNSS speed
        # (corrector) through injected databoard handles; fin_limit_multiplier is the safety dial.
        accel = databoard.Databoard.parameter('accel')  # (x, y, z) in g -> airspeed integration
        gnss_speed = databoard.Databoard.parameter('speed')  # GNSS ground speed (m/s) corrector
        self._governor = governor.Governor(governor.GovernorConfig(self.config), self._mixer,
                                           accel, gnss_speed, board.get('fin_limit_multiplier', 1.0))
        # the guidance law (guidance.py): per-stage setpoints + the three-tier heading resolution.
        # The default tier-1 freshness gate is the GNSS channels' own databoard windows (the same
        # point the databoard drops `source` to None), so it tracks the GNSS rate, not a magic number.
        position = databoard.Databoard.parameter('position')  # (lat, lon) for landing-zone navigation
        agl = databoard.Databoard.parameter('agl')  # height above ground -> final-approach trigger
        elevation = databoard.Databoard.parameter('elevation')  # baro height -> the endgame band
        window_ms = max(position.window_us, gnss_speed.window_us) // 1000
        self._mission = inspector.Inspector.get('mission')  # the landing zone lives here (may be None)
        self._guidance = guidance.Guidance(guidance.GuidanceConfig(self.config, window_ms),
                                           self._mission, self._governor, position, agl, elevation)
        self._pitch_cd: int = 0  # last measured pitch (centidegrees) -> the governor's dive detector
        self._active: bool = False  # in a control stage (PID engaged)
        self._stage = None  # the current control-stage name (for inspect)
        self._steps: int = 0  # control steps run (self-timing for load characterization)
        self._max_step_us: int = 0
        self._last_step_us: int = 0  # ticks_us of the previous control step -> actual dt (finding 1.14.2)
        self._timer = None
        self._ok = True
        return True

    def _step(self) -> None:
        """One control update (sync, no await -> runs whole in a timer slice). The pipeline: dt ->
        governor (full rate pre-glide, throttled in the glide -> the deflection cap stays warm) -> stage
        gate -> attitude -> first-entry init -> guidance -> PID -> actuate. Guidance sets/reads instance
        slots rather than returning tuples -- decomposed WITHOUT adding a per-step heap allocation (GC is
        off in flight). Self-times for the load sweep."""
        start = time.ticks_us()
        dt_ms = self._compute_dt(start)  # also stores self._dt (float s) for the governor's integrator
        # stage authority stays with the task: pre-glide (boost + active decel) FORCES the governor to
        # full rate from outside; its own speed/dive triggers decide the rest (governor.step docstring)
        self._governor.step(self._dt, self.controller.stage < _STAGE.GLIDING, self._pitch_cd)
        setpoint = self._guidance.setpoint(self.controller.stage)  # int key -> None if not control
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
        self._pitch_cd = pitch  # fresh dive detector for the next step's governor full-rate override
        if not self._active:  # entering control (from a non-control stage): capture holds, reset PIDs
            self._active = True
            self._guidance.enter(heading, roll, pitch)
            for axis_pid in self._pid.values():
                axis_pid.reset()
        self._stage = self.controller.stage  # Stage id; may switch between control stages
        if not self._guidance.compute(self._stage, setpoint, heading, start):
            self._neutral()  # boost still on the rod (no q to bite) -> no actuation
            return
        self._run_pid(roll, pitch, dt_ms)
        self._steps += 1
        elapsed = time.ticks_diff(time.ticks_us(), start)
        if elapsed > self._max_step_us:
            self._max_step_us = elapsed

    def _compute_dt(self, start: int) -> int:
        """Integer-ms slice since the last step, and stash self._dt (float s) for the governor's airspeed
        integrator. ACTUAL elapsed -- a GC pause / delayed slice makes it longer, and the PID I/D + the
        airspeed integral must use the real interval (finding 1.14.2). First step or a long gap (>0.5 s)
        -> nominal slice."""
        dt_us = time.ticks_diff(start, self._last_step_us)
        self._last_step_us = start
        if dt_us <= 0 or dt_us > 500000:  # first step / long gap (GC pause, delayed slice) -> nominal slice
            dt_us = self._dt_us
        self._dt = dt_us / 1000000.0  # float seconds: the governor's integrator (isolated float, off the PID path)
        return dt_us // 1000  # integer ms: the fixed-point PID (no float box)

    def _run_pid(self, roll: int, pitch: int, dt_ms: int) -> None:
        """Fixed-point PID (the guidance setpoint slots vs the measured attitude) -> mixer -> fins.
        Setpoints AND the measured roll/pitch are both centidegree fixnum (bno055 reads them that way),
        so the error is a plain INT SUBTRACT -- no per-axis float conversion in the PID (the setpoint's
        from_float happened once in guidance). Output fixnum -> whole degrees for the mixer via
        // fixed.SCALE. heading_error is int degrees, so × SCALE stays a small int (no box).
        The gyro 'rate' (centideg/s fixnum, same unit as the error) feeds each PID's D term directly
        (derivative-on-measurement -- clean, no setpoint kick); None when no gyro -> the PID differentiates
        the error instead. Axis mapping assumes the IMU mounted gx->roll, gy->pitch, gz->yaw."""
        law = self._guidance  # the computed setpoint slots (no per-step tuple)
        rate = self._rate.value()  # (roll, pitch, yaw) rate or None -- no box: the gyro's stored tuple
        roll_rate, pitch_rate, yaw_rate = rate if rate is not None else (None, None, None)
        roll_cmd = self._pid['roll'].step(law.roll_setpoint - roll, dt_ms, roll_rate)
        pitch_cmd = self._pid['pitch'].step(law.pitch_setpoint - pitch, dt_ms, pitch_rate)
        yaw_cmd = self._pid['yaw'].step(law.heading_error * fixed.SCALE, dt_ms, yaw_rate)  # coordinate turn
        # positional (not roll=...) so no kwargs dict is built on the hot path
        self._actuate(roll_cmd // fixed.SCALE, pitch_cmd // fixed.SCALE, yaw_cmd // fixed.SCALE)

    def _actuate(self, roll: int, pitch: int, yaw: int) -> None:
        """Drive the fins through the mixer's fused mix-and-write loop. The fin objects are resolved
        ONCE and bound into the mixer on the first call (finding.A): controller.find() is a dict search,
        pure overhead per step. By the first actuation all servo tasks are up (bring-up finishes before
        any run loop), so the lookup is stable."""
        if not self._mixer.bound:
            names = list(self._mixer.surfaces)  # the config surface names (mixer.surfaces is the gain map)
            self._mixer.bind(dict(zip(names, self.controller.find(names))))
        self._mixer.actuate(roll, pitch, yaw)

    def _neutral(self) -> None:
        self._actuate(0, 0, 0)

    async def run(self) -> None:
        # finally covers BOTH exits -- a crash out of _step (uncaught exception in a control stage)
        # and an orderly cancel -- so the fins NEVER hold a live deflection while nobody is flying
        # them (finding 15.3: uncaught crash used to leave the last command standing for the ~1-2 s
        # watchdog window). finish() is idempotent (timer None check), so the Controller's own
        # finish() on shutdown is a harmless second call. Zero cost on the hot path.
        try:
            if self._schedule_hz > 0:
                await self._run_timer()
            else:
                await self._run_asyncio()
        finally:
            await self.finish()

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

    def vitals(self) -> dict:
        """The live flight-panel readout (CC dashboard): the governor's airspeed estimate + the
        dynamic-pressure fin-authority cap it set, and whether the control loop is engaged. Called at
        the ~0.5 Hz health rate, so the airspeed float box is off the hot path."""
        return {'airspeed': round(self._governor.airspeed(), 1), 'fin_cap': self._governor.cap(),
                'active': self._active}

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['schedule'] = 'timer' if self._schedule_hz > 0 else 'asyncio'
        status['schedule_hz'] = self._schedule_hz if self._schedule_hz > 0 else round(1000 / self._period_ms)
        status['active'] = self._active
        status['stage'] = _STAGE.STAGES.get(self._stage)  # id -> operator-facing name (None if not active)
        status['steps'] = self._steps  # load sweep: compare steps/sec + max_step_us vs board_health load
        status['max_step_us'] = self._max_step_us
        return status
