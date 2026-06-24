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

import databoard
import inspector
import mixer
import navigation
import pid
import task


@task.activity('flight')
class Flight(task.Task):
    """Attitude-hold stabilization: GLIDING-gated, timer- or asyncio-scheduled, fail-safe to neutral."""

    _AXES: tuple = ('roll', 'pitch', 'yaw')

    @staticmethod
    def _heading_error(target: float, current: float) -> int:
        """Shortest signed heading error (deg), wrapped to [-180, 180] so 350 -> 10 is +20, not -340.
        Integer degrees -- sub-degree precision is irrelevant to a servo and lets one modulo replace the
        wrap loop."""
        error = int(target - current)
        return error if -180 <= error <= 180 else (error + 180) % 360 - 180

    async def setup(self) -> bool:
        board = self.controller.config
        self._mixer = mixer.Mixer(board.get('mixer', {}))
        self._schedule_hz: int = self.config.get('schedule_hz', 100)  # > 0 -> timer; 0 -> asyncio at period_ms
        self._period_ms: int = self.config.get('period_ms', 20)
        self._dt: float = (1.0 / self._schedule_hz) if self._schedule_hz > 0 else (self._period_ms / 1000.0)
        gains = self.config.get('gains', {})
        limit = self._mixer.limit
        self._pid = {axis: pid.Pid(output_limit=limit,
                                   integral_limit=self.config.get('integral_limit', limit),
                                   **gains.get(axis, {})) for axis in self._AXES}
        # per-stage behaviour: which flight stages are CONTROL stages and their attitude setpoint.
        # Stages not listed hold the fins neutral (SETTING/BOOSTING/DONE -- no actuation under thrust /
        # on the ground). GLIDING = wings-level + heading hold; LANDING carries its own setpoint (flare).
        self._stages: dict = self.config.get('stages', {'gliding': {'roll': 0.0, 'pitch': 0.0}})
        self._attitude = databoard.Databoard.parameter('attitude')  # (heading, roll, pitch)
        self._position = databoard.Databoard.parameter('position')  # (lat, lon) for landing-zone navigation
        self._mission = inspector.Inspector.get('mission')  # the landing zone lives here (may be None)
        self._heading_hold = None  # captured on entering a control stage -> hold that heading
        self._active: bool = False  # in a control stage (PID engaged)
        self._stage = None  # the current control-stage name (for inspect)
        self._steps: int = 0  # control steps run (self-timing for load characterization)
        self._max_step_us: int = 0
        self._last_step_us: int = 0  # ticks_us of the previous control step -> actual dt (finding 1.14.2)
        self._timer = None
        self._ok = True
        return True

    def _step(self) -> None:
        """One control update (sync, no await -> runs whole in a timer slice): gate -> read attitude ->
        PID -> mix -> apply. Self-times for the load sweep."""
        start = time.ticks_us()
        setpoint = self._stages.get(self.controller.stage_name())  # None -> not a control stage
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
            self._last_step_us = start  # so the first dt below is ~0 -> nominal (no jump from a stale gap)
            for controller in self._pid.values():
                controller.reset()
        self._stage = self.controller.stage_name()  # may switch between control stages (glide -> landing)
        # ACTUAL elapsed since the last control step, not the nominal 1/schedule_hz (finding 1.14.2 / g5):
        # a GC pause or a delayed slice makes the real interval longer, and the PID I/D terms must use it
        # or they under/over-correct. The first step (dt 0) falls back to the nominal _dt.
        dt = time.ticks_diff(start, self._last_step_us) / 1000000.0
        self._last_step_us = start
        if dt <= 0:
            dt = self._dt
        roll_cmd = self._pid['roll'].step(setpoint.get('roll', 0.0) - roll, dt)
        pitch_cmd = self._pid['pitch'].step(setpoint.get('pitch', 0.0) - pitch, dt)
        yaw_cmd = self._pid['yaw'].step(self._heading_error(self._target_heading(), heading), dt)
        self._apply(self._mixer.mix(roll=round(roll_cmd), pitch=round(pitch_cmd), yaw=round(yaw_cmd)))
        self._steps += 1
        elapsed = time.ticks_diff(time.ticks_us(), start)
        if elapsed > self._max_step_us:
            self._max_step_us = elapsed

    def _target_heading(self) -> float:
        """The heading to steer in GLIDING (LANDING + non-control stages just hold). Three tiers,
        degrading gracefully as the GNSS does:
          1. a FRESH fix -> steer from the current position (closed-loop, corrects wind drift);
          2. no fix but a launch point (CC-set) -> hold the launch->gate bearing (open-loop, AIMED at
             the zone but wind-uncorrected -- the GPS-denied fallback);
          3. neither -> the captured glide heading (blind).
        Nav steers only in GLIDING; LANDING locks straight-and-level (coludo.md)."""
        if self.controller.stage_name() != 'gliding' or self._mission is None or not self._mission.zone:
            return self._heading_hold
        zone = self._mission.zone
        position, source, _age = self._position.read()
        if source is not None and position is not None:  # tier 1: live fix
            return navigation.steer(position, zone[0], zone[1])[0]
        launch = self._mission.launch_point()  # tier 2: open-loop from the launch point (CC-set)
        if launch is not None:
            return navigation.steer(launch, zone[0], zone[1])[0]
        return self._heading_hold  # tier 3: blind

    def _apply(self, angles: dict) -> None:
        for name, angle in angles.items():
            fin = self.controller.find([name])[0]
            if fin is not None:
                fin.update({'angle': angle})

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
        control-stage name (or None); `updated_us` is time.ticks_us() of the last control step, so a
        supervisor can judge staleness by TIME directly (not by step-count diffing against its own poll
        cadence)."""
        return self._active, self._steps, self._stage, self._last_step_us

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['schedule'] = 'timer' if self._schedule_hz > 0 else 'asyncio'
        status['schedule_hz'] = self._schedule_hz if self._schedule_hz > 0 else round(1000 / self._period_ms)
        status['active'] = self._active
        status['stage'] = self._stage
        status['steps'] = self._steps  # load sweep: compare steps/sec + max_step_us vs board_health load
        status['max_step_us'] = self._max_step_us
        return status
