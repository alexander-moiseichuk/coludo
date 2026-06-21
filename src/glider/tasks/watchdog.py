# tasks/watchdog.py — Phase 3 watchdog + heartbeat supervisor. @task.activity('watchdog'). Two layers:
#   1. a hardware machine.WDT fed every period -> a TOTAL event-loop wedge (any task stuck below the
#      await level, a hung I2C bus) stops the feed and the board hard-resets. The backstop.
#   2. a heartbeat check of the CONTROL LOOP: while the flight task is in a control phase it must keep
#      ticking (its step counter advances). A stalled control loop (live scheduler, dead control) ->
#      reset, since a soft restart cannot preempt a wedged native call and the HW (PWM, the I2C bus,
#      sensors mid-transaction) needs a clean reset to be trustworthy.
# Recovery is a full machine.reset() (fast on the P4; boot re-centres the fins) -- a soft event-loop
# restart is unreliable here. The flight loop already fail-safes to neutral on stale attitude (degraded
# mode), so that is NOT a watchdog trigger. Disabled by default -- a live WDT also resets the board when
# you drop the running firmware to the REPL for bench work; enable it for flight.

import asyncio

import recorder
import task


@task.activity('watchdog')
class Watchdog(task.Task):
    """Feed a hardware WDT (wedge backstop) + supervise the control loop (stall -> full reset)."""

    async def setup(self) -> bool:
        self._timeout_ms: int = self.config.get('wdt_timeout_ms', 2000)
        self._period_ms: int = self.config.get('period_ms', 200)
        self._wdt = None  # the hardware WDT (created in run(); injectable for tests)
        self._reset = lambda: __import__('machine').reset()  # overridable for tests
        self._last_steps: int = 0
        self._ok = True
        return True

    def _stalled(self, flight) -> bool:
        """True if the control loop is in a control phase but its step counter is not advancing. When
        it is not controlling there is nothing to supervise, so the step baseline just tracks along."""
        if flight is None or not getattr(flight, '_active', False):
            self._last_steps = getattr(flight, '_steps', 0) if flight is not None else 0
            return False
        stalled = flight._steps == self._last_steps
        self._last_steps = flight._steps
        return stalled

    async def run(self) -> None:
        if self._wdt is None:
            from machine import WDT

            self._wdt = WDT(timeout=self._timeout_ms)
        while True:
            await asyncio.sleep_ms(self._period_ms)
            flight = self.controller.find(['flight'])[0]  # None if the flight task is disabled
            if self._stalled(flight):
                recorder.Recorder.log(self.name, 'control loop stalled (phase=%s) -> reset' %
                                      getattr(flight, '_phase', None))
                self._reset()  # full HW reset; stopping the feed would also fire the WDT shortly
                return
            self._wdt.feed()
