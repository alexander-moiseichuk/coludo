# drivers/separation.py — stage-separation switch: two adhesive copper pads (one on the glider, one
# on the booster) that route 3V3 to a pin while nested (HIGH) and open on separation (LOW). A HAL
# input, @task.driver('separation'). An IRQ on either edge wakes run(), which debounces, and on a
# confirmed separation during the Boosting stage drives the documented Boosting -> Gliding transition
# (the booster ejects the glider at apogee). The event is logged and emitted to subscribers; the
# discrete event is NOT a blackboard quantity (per specs/coludo.md, events use notify/log).
#
# The pin uses an internal pull-down so an open (separated) circuit reads LOW reliably; while nested
# the pads override it HIGH. A separation while not Boosting (e.g. a ground test in Setting) is
# logged but does not transition -- the guard keeps go/no-go correct.

import asyncio

import controller
import recorder
import task


@task.driver('separation')
class Separation(task.Task):
    """Detect stage separation (HIGH=nested -> LOW=separated) and trigger Boosting -> Gliding."""

    async def setup(self) -> bool:
        from machine import Pin

        gpio = self.controller.config.get('pins', {}).get(self.config.get('pin', 'separation_switch'))
        if gpio is None:
            return False
        self._debounce_ms: int = self.config.get('debounce_ms', 20)
        self._flag = asyncio.ThreadSafeFlag()
        self._pin = Pin(gpio, Pin.IN, Pin.PULL_DOWN)
        self._separated: bool = self._pin.value() == 0  # LOW = pads open = separated
        self._pin.irq(self._on_edge, Pin.IRQ_RISING | Pin.IRQ_FALLING)
        self._ok = True
        return True

    def _on_edge(self, pin) -> None:
        """IRQ: the line changed -- wake run() to debounce and act. ThreadSafeFlag.set() is safe."""
        self._flag.set()

    def _apply(self, separated: bool) -> None:
        """Act on a confirmed pin level: on a change, emit + log, and (only) on separation during
        Boosting advance the flight stage to Gliding."""
        if separated == self._separated:
            return
        self._separated = separated
        event = 'separated' if separated else 'nested'
        self.emit(event)
        recorder.Recorder.log('separation', event)
        if separated and self.controller.stage == controller.Stage.BOOSTING:
            self.controller.set_stage(controller.Stage.GLIDING)

    async def run(self) -> None:
        while True:
            await self._flag.wait()
            await asyncio.sleep_ms(self._debounce_ms)  # let the contact bounce settle, then re-read
            self._apply(self._pin.value() == 0)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['separated'] = self._separated
        return status
