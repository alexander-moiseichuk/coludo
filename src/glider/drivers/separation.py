"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Stage-separation switch: two adhesive copper pads (one on the glider, one on the booster) that route
3V3 to a pin while nested (HIGH) and open on separation (LOW). A HAL input, @task.driver('separation').
An IRQ on either edge wakes run(), which debounces, and on a confirmed separation during the Boosting
stage drives the documented Boosting -> Gliding transition (the booster ejects the glider at apogee).
The event is logged and emitted to subscribers; the discrete event is NOT a databoard quantity (per
doc/specs/coludo.md, events use notify/log).

The pin uses an internal pull-down so an open (separated) circuit reads LOW reliably; while nested the
pads override it HIGH. A separation while not Boosting (e.g. a ground test in Setting) is logged but
does not transition -- the guard keeps go/no-go correct.

This transition calls controller.set_stage() directly, NOT the sequencer's _advance(), so it does not
write a row to sequencer.csv. That is deliberate -- separation is the PRIMARY Boosting -> Gliding
trigger and separation.csv (event + stage, durable) is its authoritative telemetry record; the
sequencer's burnout-timeout is only the fallback, and sequencer.csv records that fallback path. A
post-flight tool reading the BOOSTING->GLIDING reason must consult separation.csv first. (GC policy is
unaffected: gc.disable() already fired on the SETTING->BOOSTING transition.)
"""

import asyncio
import time

import controller
import recorder
import task

try:
    from machine import Pin
except ImportError:  # host (CPython): board-only; the latch pin is read only on the board
    Pin = None

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const

_SAMPLE_MS = const(5)      # re-read cadence while settling (the asyncio floor makes it ~10 ms here)
_MIN_SAMPLES = const(3)    # agreeing reads needed, however long the window -- one read is never enough
_SETTLE_WINDOWS = const(10)  # a line still chattering after this many debounce windows is not a verdict


@task.driver('separation')
class Separation(task.Task):
    """Detect stage separation (HIGH=nested -> LOW=separated) and trigger Boosting -> Gliding."""

    async def setup(self) -> bool:
        gpio = self._pin_gpio('pin', 'separation_switch')
        if gpio is None:
            return False
        self._debounce_ms: int = self.config.get('debounce_ms', 20)
        self._flag = asyncio.ThreadSafeFlag()
        self._pin = Pin(gpio, Pin.IN, Pin.PULL_DOWN)
        self._separated: bool = self._pin.value() == 0  # LOW = pads open = separated
        self._telemetry = recorder.Telemetry('separation.csv', ('event', 'stage'))  # durable, every event
        self._pin.irq(self._on_edge, Pin.IRQ_RISING | Pin.IRQ_FALLING)
        self._ok = True
        return True

    def _on_edge(self, _unused_pin) -> None:
        """IRQ: the line changed -- wake run() to debounce and act. ThreadSafeFlag.set() is safe."""
        self._flag.set()

    def _apply(self, separated: bool) -> None:
        """
        Act on a confirmed pin level.

        On a change, advance Boosting -> Gliding on separation, then record the event to telemetry
        (durable, committed as separation.csv) before the best-effort log, and notify subscribers.

        Args:
            separated - the debounced pin level (True = pads open = separated).

        Returns:
            None; on a level change updates state, may advance the stage, pushes telemetry, logs, and
            emits the event. A no-op when the level is unchanged.
        """
        if separated == self._separated:
            return
        self._separated = separated
        event = 'separated' if separated else 'nested'
        if separated and self.controller.stage == controller.Stage.BOOSTING:
            self.controller.set_stage(controller.Stage.GLIDING)
        try:  # the stage change above is made; a full ring must not kill the separation task
            self._telemetry.push((event, controller.Stage.STAGES[self.controller.stage]))
        except Exception as error:
            self.note('separation :: record %r', error)
        recorder.Recorder.log('separation', event)
        self.emit(event)

    async def run(self) -> None:
        while True:
            await self._flag.wait()
            level = await self._settle()
            if level is not None:
                self._apply(level)

    async def _settle(self):
        """
        The pin level once it has HELD for debounce_ms, confirmed by several reads -- or None.

        This used to read the pin ONCE, debounce_ms after the first edge, and commit on that one
        sample. The config promised "the LOW must hold this long", but nothing checked that it held:
        a ~7 ms open blip that happened to span the sample instant was a separation. During boost that
        forced BOOSTING -> GLIDING, and the transition cannot be undone. Reproduced on the board.

        Now the window restarts on every change of level, and a verdict needs at least _MIN_SAMPLES
        agreeing reads spanning debounce_ms. A real separation leaves the pads open for good, so this
        costs a few tens of milliseconds of latency; a vibration blip can no longer win. A line that is
        still chattering after _SETTLE_WINDOWS windows returns None -- and since its own edges keep
        setting the flag, run() simply settles it again once it quietens.

        Args:
            (none)

        Returns:
            True for a held separation (pads open), False for held nested, None if it never settled.
        """
        candidate = self._pin.value()
        since = time.ticks_ms()
        agreeing = 1
        deadline = time.ticks_add(since, self._debounce_ms * _SETTLE_WINDOWS)
        while True:
            await asyncio.sleep_ms(_SAMPLE_MS)
            now = time.ticks_ms()
            value = self._pin.value()
            if value != candidate:
                candidate, since, agreeing = value, now, 1  # the level moved: start the window again
            else:
                agreeing += 1
            if agreeing >= _MIN_SAMPLES and time.ticks_diff(now, since) >= self._debounce_ms:
                return candidate == 0
            if time.ticks_diff(now, deadline) >= 0:
                return None

    async def probe(self) -> str:
        """On-demand self-test: the separation pin reads a valid level (logged nested/separated)."""
        try:
            recorder.Recorder.log(self.name, 'probe: separation pin ...')
            value = self._pin.value()
            if value not in (0, 1):
                raise ValueError('pin read %r' % value)
            recorder.Recorder.log(self.name, 'probe: pin ok (%s)' % ('separated' if value == 0 else 'nested'))
        except Exception as error:
            message = 'separation pin: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    async def diagnose(self) -> str:
        """
        Deeper analysis: read the separation pin directly.

        During a pre-flight check it should be HIGH (the pads are nested, routing 3V3). LOW means the
        pads are open (already separated) or miswired. The Controller folds this into the failure reason.

        Args:
            (none)

        Returns:
            A human-readable verdict string: no-pin when the role is unmapped, pin-HIGH (switch ok) when
            nested, or pin-LOW (pads open / not contacting / miswired) otherwise.
        """
        gpio = self._pin_gpio('pin', 'separation_switch')
        if gpio is None:
            return 'no pin -- %r not defined in config pins' % self.config.get('pin', 'separation_switch')
        level = Pin(gpio, Pin.IN, Pin.PULL_DOWN).value()
        if level == 1:
            return 'pin GPIO%d HIGH (nested) -- switch ok; setup failed elsewhere' % gpio
        return 'pin GPIO%d LOW -- expected HIGH (nested) at check: pads open / not contacting / miswired' % gpio

    def separated(self) -> bool:
        """
        The last debounced latch level as a bool (True = pads open = separated).

        The AUTHORITATIVE reading for the warm-start gate -- this driver's own configured pin object, so
        the recovery path never constructs a second machine.Pin on the same GPIO (was
        main._restore_flight's bug).

        Args:
            (none)

        Returns:
            True when the pads are open (separated), False when nested.
        """
        return self._separated

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['separated'] = self._separated
        return status
