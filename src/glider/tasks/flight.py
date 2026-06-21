# tasks/flight.py — Phase 3 stabilization loop. @task.activity('flight'). At `schedule_hz` it reads the
# IMU 'attitude' (heading, roll, pitch), runs a PID per axis to the setpoint (+ heading hold), mixes
# the result to the fins (mixer.py) and writes them via sg90.update(). ACTIVE ONLY IN GLIDING -- every
# other stage holds the fins neutral, so it never actuates under boost or on the ground. Degraded:
# stale/absent attitude -> neutral (never act on stale data).
#
# Scheduling: schedule_hz > 0 -> a machine.Timer ticks the step, so the control law gets a regular slice
# independent of what other asyncio tasks are doing (deterministic, e.g. while the laser hammers I2C in
# landing). schedule_hz == 0 -> a plain asyncio loop at period_ms (reconfigure/debug; subject to the ~10 ms
# asyncio floor). Default 100 Hz timer = ~1 m per control step at 100 m/s. Gains default to 0 and the
# task is disabled by default -- it cannot move a surface until enabled + tuned on the airframe.

import asyncio
import time

import databoard
import mixer
import pid
import task

_AXES = ('roll', 'pitch', 'yaw')


def _heading_error(target: float, current: float) -> float:
    """Shortest signed heading error (deg), wrapped to [-180, 180] so 350 -> 10 is +20, not -340."""
    error = target - current
    while error > 180:
        error -= 360
    while error < -180:
        error += 360
    return error


@task.activity('flight')
class Flight(task.Task):
    """Attitude-hold stabilization: GLIDING-gated, timer- or asyncio-scheduled, fail-safe to neutral."""

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
                                   **gains.get(axis, {})) for axis in _AXES}
        self._setpoint: dict = self.config.get('setpoint', {'roll': 0.0, 'pitch': 0.0})
        self._attitude = databoard.Databoard.parameter('attitude')  # (heading, roll, pitch)
        self._heading_hold = None  # captured on entering glide -> hold the glide heading
        self._gliding: bool = False
        self._steps: int = 0  # control steps run (self-timing for load characterization)
        self._max_step_us: int = 0
        self._timer = None
        self._ok = True
        return True

    def _step(self) -> None:
        """One control update (sync, no await -> runs whole in a timer slice): gate -> read attitude ->
        PID -> mix -> apply. Self-times for the load sweep."""
        start = time.ticks_us()
        if self.controller.stage_name() != 'gliding':
            if self._gliding:  # just left glide -> centre the fins
                self._neutral()
                self._gliding = False
            return
        value, source, _age = self._attitude.read()
        if source is None or value is None:  # stale / absent attitude -> degraded -> neutral
            self._neutral()
            return
        heading, roll, pitch = value
        if not self._gliding:  # entering glide: capture the heading to hold, reset integrators
            self._gliding = True
            self._heading_hold = heading
            for controller in self._pid.values():
                controller.reset()
        roll_cmd = self._pid['roll'].step(self._setpoint.get('roll', 0.0) - roll, self._dt)
        pitch_cmd = self._pid['pitch'].step(self._setpoint.get('pitch', 0.0) - pitch, self._dt)
        yaw_cmd = self._pid['yaw'].step(_heading_error(self._heading_hold, heading), self._dt)
        self._apply(self._mixer.mix(roll=round(roll_cmd), pitch=round(pitch_cmd), yaw=round(yaw_cmd)))
        self._steps += 1
        elapsed = time.ticks_diff(time.ticks_us(), start)
        if elapsed > self._max_step_us:
            self._max_step_us = elapsed

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

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['schedule'] = 'timer' if self._schedule_hz > 0 else 'asyncio'
        status['schedule_hz'] = self._schedule_hz if self._schedule_hz > 0 else round(1000 / self._period_ms)
        status['gliding'] = self._gliding
        status['steps'] = self._steps  # load sweep: compare steps/sec + max_step_us vs board_health load
        status['max_step_us'] = self._max_step_us
        return status
