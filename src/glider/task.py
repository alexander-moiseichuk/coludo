"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Task base class and driver registry -- the unit the Controller creates and supervises.

Every component/system task follows the common lifecycle from doc/specs/coludo.md:
  setup() async; initialize or reset; return True on success
  probe() async; ON-DEMAND self-test (the CC `probe` command, never at boot) -> None if healthy,
      else an error string. Default None; a sensor reports 'X not found on i2c:0', an actuator
      exercises itself (the servo sweeps its range) -- so a mid-flight reboot never sweeps fins.
  run() async; the task's main activity loop
  notify() subscribe a callback for this task's updates
  validate() return True if the task is currently healthy
  finish() async; shut down and release resources
A Task is Inspectable: inspect()/update()/stats() expose it to the operator (the Controller registers
each task with the Inspector), so there is no separate report().

A task registers itself with @activity('name') (or its alias @driver('name') for the HAL ones in
drivers/) into ACTIVITIES, the CLASS registry: name -> Task subclass, "what can be built". It is a
module global on purpose -- the decorators fill it at IMPORT time, before any Controller exists, so it
cannot live on a Controller instance (that is why moving it into the Controller would be a mess, not a
tidy-up). The Controller READS it (injected as `registry`, defaulting to ACTIVITIES) to build a
component, and keeps its own INSTANCE directory -- find()/query(), "what is currently running" -- for
dependency lookup. Two deliberately separate lookups: class-by-name here, instance-by-name on the
Controller. The driver/activity names share one registry for now; splitting drivers out later.
"""

import asyncio

import inspector

ACTIVITIES: dict = {}  # CLASS registry: name -> Task subclass (instance lookup is Controller.find/query)


def activity(name: str):
    """
    Class decorator: register a Task subclass under a name.

    Registers a HAL driver or a higher-level activity so the Controller can build it from a config
    component.

    Args:
        name - the registry key a config component names to build this class.

    Returns:
        The class decorator, which registers the class and returns it unchanged.
    """

    def deco(cls):
        ACTIVITIES[name] = cls
        return cls

    return deco


driver = activity  # alias: drivers/ files read as @task.driver, tasks/ files as @task.activity


class Task(inspector.Inspectable):
    kind: str = 'task'

    def __init__(self, name: str, config: dict = None, controller=None):
        self.name: str = name
        self.config: dict = config or {}  # this task's sensor/component dict from board.config
        self.controller = controller  # back-reference for find()/query()/notify()
        self._ok: bool = False
        self._subs: list = []
        self._healthy: bool = True  # RUNTIME read health (distinct from _ok = setup ok); note() tracks it
        self._strikes: int = 0  # consecutive-failure run for strike() (see below)
        self._claimed: bool = False  # a multi-step device conversation is in progress (claim())

    def note(self, template: str = None, arg=None) -> None:
        """
        De-duplicated best-effort run-loop log + runtime-health flag.

        Call `note()` (template None) on a healthy pass; call `note('x :: %r', error)` on a failure. The
        failure is printed ONCE per healthy->error transition, and -- critically -- the `template % arg`
        is formatted only on that transition. `note('x :: %r' % error)` would instead format EAGERLY at
        the call site every tick, so a persistently-failing read in a GC-OFF flight loop leaks a string
        each iteration (and a 50 Hz sensor floods the USB-CDC, wedging the REPL). Pass the template
        literal + a single by-reference `arg` (fixed 2-arg signature, no *args tuple) so nothing
        allocates while the fault repeats. A healthy note() re-arms the next error to log afresh, and
        tracks _healthy so inspect()['healthy'] shows a flaky run loop that _ok (setup-time) cannot.

        Args:
            template - the log format string, or None to mark a healthy pass.
            arg - a single by-reference argument for `template % arg`; None when the template needs
                none.

        Returns:
            None; prints once on the healthy->error transition and updates _healthy.
        """
        if template is None:  # healthy pass -> clear the fault, re-arm the next error
            self._healthy = True
            return
        if self._healthy:  # first failure after a healthy pass -> format + print once
            print(template % arg if arg is not None else template)
        self._healthy = False

    def _pin_gpio(self, field: str, default: str = None) -> int:
        """
        The board GPIO NUMBER for this component's `field` pin (None if absent).

        Looks the component's pin name (config[field], or `default` when the component omits it) up in
        the board `pins` map. Returns a number, not a machine.Pin: chip-select / PWM sites consume the
        number directly, and the Pin-building sites (int / xshut / separation) each need their own mode
        + IRQ. One resolver for the cs_pin / int_pin / xshut_pin / pin lookup shared across drivers.

        DISABLED optional pin: a pins-map value of `null` (None) -- or `-1` -- means the feature is
        wired off on this board. Both resolve to None, exactly like an absent pin, so every optional-pin
        driver's `is None` guard skips the feature (poll instead of INT, no XSHUT toggle, no hardware
        ALERT). `null` is the preferred, self-documenting form.

        Args:
            field - the config key naming this component's pin.
            default - the pin name to use when the component omits `field`.

        Returns:
            The GPIO number; None when the pin is absent, null, or negative (the feature is wired off).
        """
        gpio = self.controller.config.get('pins', {}).get(self.config.get(field, default))
        return gpio if isinstance(gpio, int) and gpio >= 0 else None

    async def setup(self) -> bool:
        """Initialize or reset. Override. Return True on success, False otherwise."""
        raise NotImplementedError('Task.setup() must be overridden')

    async def claim(self) -> None:
        """
        Wait until no other caller owns this device's MULTI-STEP conversation, then take it.

        Some devices carry state across an await -- icp10111 is write-measure, sleep, read; vl53l4cx is
        status, distance, then the interrupt clear that arms the next sample. A second caller entering
        mid-sequence issues its own command into the middle of the first and both come back NAKing:
        measured at a 20.8 % failure rate when a diagnostic polled alongside the run loop. It is not
        bus SHARING (peers on the same bus cost nothing) -- it is one device having one conversation.

        A plain flag, not asyncio.Lock: `async with lock` measures ~288 B on this port (see the
        per-operation lock removed in 556f98c) and MicroPython's loop is cooperative, so the flag can
        only change across an await. Always release in a `finally` -- a raising read must not wedge
        every future caller.

        What is NOT here: whether a waiting caller can be served from cache instead. That predicate is
        device-specific (icp10111 by sample age, vl53l4cx by presence) and stays with the device.

        Args:
            (none)

        Returns:
            None; the caller owns the device until unclaim().
        """
        while self._claimed:
            await asyncio.sleep_ms(1)
        self._claimed = True

    def unclaim(self) -> None:
        """Release the claim taken by claim(); call it from a `finally`, never a happy path only."""
        self._claimed = False

    def strike(self, failed: bool, limit: int) -> bool:
        """
        Count a run of CONSECUTIVE failures; True exactly ONCE, when the run reaches `limit`.

        Four drivers had hand-rolled this (icp10111 read failures -> general-call reset, sdp810 ->
        continuous-mode restart, lsm6dso32 INT1 timeouts -> poll fallback, bno055 frozen-fusion ->
        withhold attitude) and they had already drifted apart: two fired at `== limit`, one at
        `>= limit`, and each kept its own counter and reset. The shape is identical every time and the
        subtle part is the same every time -- act ONCE while the fault persists, not on every tick, and
        rearm only on a genuine success.

        Returning True only on the transition is what gives "once": a caller can escalate directly
        without tracking whether it already has. A latch (bno055's withheld attitude, lsm6dso32's poll
        mode) sets its own flag on that True and clears it on its own terms, since "recovered" means
        something different per device and does not belong here.

        Args:
            failed - True when this pass failed; False on a good pass, which rearms the count.
            limit - consecutive failures that constitute a real fault rather than a blip.

        Returns:
            True on the pass that reaches `limit`, False every other time (including further failures).
        """
        if not failed:
            self._strikes = 0
            return False
        self._strikes += 1
        return self._strikes == limit

    def calibration(self) -> str:
        """
        What the OPERATOR must do to make this device flight-ready -- or '' when there is nothing.

        A sibling of probe(), and needed because the two answer different questions. probe() asks "does
        the hardware work" -- an uncalibrated BNO055 passes it, because the part responds perfectly.
        This asks "is it ready to FLY", which for several devices means a physical act nobody can infer
        from the config: the IMU wants motion, the pitot still air, the baro a settled ground reference.

        ONE STRING, deliberately. '' means nothing to do -- device needs no calibration, or already
        satisfied -- so a caller polls by simply re-reading until it empties, and the operator surfaces
        need no state machine. Fold the live reading INTO the text ("...until mag reads 3 (now mag 1)")
        so progress is visible while the instruction stays the thing being shown.

        Args:
            (none)

        Returns:
            '' when nothing to do, else the instruction for the operator.
        """
        return ''

    async def calibrate(self) -> str:
        """
        Start / enforce this device's calibration (the CC `calibrate <device>` command).

        Only meaningful where the board can DO something -- capture a tare, re-zero a reference. Where
        calibration is inherently physical (the BNO055 needs the airframe moved) the device says so
        through calibration() and this stays a no-op, so a caller can sweep every device without
        special-casing. The operator precondition still applies to both: the board cannot capture a
        still-air tare while somebody is waving the airframe about.

        Args:
            (none)

        Returns:
            None on success (or nothing to do), else a human-readable failure string.
        """
        return None

    async def probe(self) -> str:
        """
        On-demand self-test (the CC `probe` command, NOT run at boot).

        The operator runs it pre-flight; costly active checks (the servo range sweep) belong here, so a
        reboot never triggers them. Override per device; the default has nothing to probe.

        Convention -- write each step EXPLICITLY, wrapped in its own try/except with a Recorder.log
        before the action, after success (with the value got), and on failure; on failure return the
        step's message so probe()'s caller sees which step broke:
            try:
                recorder.Recorder.log(self.name, 'probe: chip id ...')
                chip = await self._read_id()
                recorder.Recorder.log(self.name, 'probe: chip id ok 0x%02x' % chip)
            except Exception as error:
                message = 'chip id: %s' % error
                recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
                return message
            ... next step ...
            return None

        Args:
            (none)

        Returns:
            None when healthy, or a human-readable error string (e.g. 'BMP280 not found on i2c:0').
        """
        return None

    async def run(self) -> None:
        """
        The task's main activity loop.

        Override. The default raises to catch missing overrides (the Controller catches the exception
        and logs the crash). Command-driven tasks with no loop (SG90, Bluetooth) override explicitly
        with `pass`.

        Returns:
            None; loops forever until the task is cancelled.

        Raises:
            NotImplementedError if a subclass does not override it.
        """
        raise NotImplementedError('Task.run() must be overridden')

    def notify(self, callback) -> None:
        """Register callback(task, event) to be invoked on this task's updates."""
        if callback not in self._subs:
            self._subs.append(callback)

    def emit(self, event=None) -> None:
        """Notify all subscribers of an update."""
        for callback in self._subs:
            callback(self, event)

    def find(self, names: list[str]) -> list:
        """Non-blocking sibling lookup via the Controller (None for any not up)."""
        return self.controller.find(names)

    async def query(self, names: list[str], waiting: bool = True) -> list:
        """
        Await sibling tasks by name via the Controller.

        With `waiting` (default) park until all are up (order is not fixed):
        `wifi, = await self.query(['wifi'])`.

        Args:
            names - the sibling task names to look up.
            waiting - True (default) parks until all are present; False returns immediately.

        Returns:
            A list aligned with `names` (see Controller.query for the None semantics).
        """
        return await self.controller.query(names, waiting)

    def validate(self) -> bool:
        """Return True if the task is currently healthy."""
        return self._ok

    async def finish(self) -> None:
        """Shut down and release resources."""
        self._ok = False

    """Inspectable: the operator-facing task-state snapshot."""
    def inspect(self) -> dict:
        """Status dict. Subclasses extend it."""
        return {'name': self.name, 'ok': self._ok, 'healthy': self._healthy}
