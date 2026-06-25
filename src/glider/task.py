# Task base class and driver registry — the unit the Controller creates and supervises.
#
# Every component/system task follows the common lifecycle from specs/coludo.md:
#   setup()    async; initialize or reset; return True on success
#   probe()    async; ON-DEMAND self-test (the CC `probe` command, never at boot) -> None if healthy,
#              else an error string. Default None; a sensor reports 'X not found on i2c:0', an actuator
#              exercises itself (the servo sweeps its range) -- so a mid-flight reboot never sweeps fins.
#   run()      async; the task's main activity loop
#   notify()   subscribe a callback for this task's updates
#   validate() return True if the task is currently healthy
#   finish()   async; shut down and release resources
# A Task is Inspectable: inspect()/update()/stats() expose it to the operator (the Controller
# registers each task with the Inspector), so there is no separate report().
#
# A task registers itself with @activity('name') (or its alias @driver('name') for the HAL ones in
# drivers/) into ACTIVITIES, the CLASS registry: name -> Task subclass, "what can be built". It is a
# module global on purpose -- the decorators fill it at IMPORT time, before any Controller exists, so
# it cannot live on a Controller instance (that is why moving it into the Controller would be a mess,
# not a tidy-up). The Controller READS it (injected as `registry`, defaulting to ACTIVITIES) to build
# a component, and keeps its own INSTANCE directory -- find()/query(), "what is currently running" --
# for dependency lookup. Two deliberately separate lookups: class-by-name here, instance-by-name on
# the Controller. The driver/activity names share one registry for now; splitting drivers out later.

import inspector

ACTIVITIES: dict = {}  # CLASS registry: name -> Task subclass (instance lookup is Controller.find/query)


def activity(name: str):
    """Class decorator: register a Task subclass (a HAL driver or a higher-level activity) under a
    name so the Controller can build it from a config component."""

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
        self._last_note = None  # last message passed to note() -> dedup a repeating run-loop error

    def note(self, message: str) -> None:
        """Print `message` only when it CHANGES from the previous note -- a de-duplicated best-effort
        log for run() loops. A persistently-failing read (a flaky/absent sensor, a missing CC hub) would
        otherwise log every iteration: a 50 Hz sensor floods the USB-CDC and wedges the REPL. So use this
        instead of a bare print() inside `while True` -- the first occurrence + any change/recovery show,
        the repeats are dropped. Call note(None) on a healthy pass so the next error logs afresh."""
        if message != self._last_note:
            self._last_note = message
            if message is not None:
                print(message)

    async def setup(self) -> bool:
        """Initialize or reset. Override. Return True on success, False otherwise."""
        self._ok = True
        return True

    async def probe(self) -> str:
        """On-demand self-test (the CC `probe` command, NOT run at boot): return None when healthy, or
        a human-readable error string (e.g. 'BMP280 not found on i2c:0'). The operator runs it
        pre-flight; costly active checks (the servo range sweep) belong here, so a reboot never
        triggers them. Override per device; default has nothing to probe.

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
            return None"""
        return None

    async def run(self) -> None:
        """Main activity loop. Override. Default returns immediately."""
        pass

    def notify(self, callback) -> None:
        """Register callback(task, event) to be invoked on this task's updates."""
        if callback not in self._subs:
            self._subs.append(callback)

    def emit(self, event=None) -> None:
        """Notify all subscribers of an update."""
        for cb in self._subs:
            cb(self, event)

    def find(self, names: list[str]) -> list:
        """Non-blocking sibling lookup via the Controller (None for any not up)."""
        return self.controller.find(names)

    async def query(self, names: list[str], waiting: bool = True) -> list:
        """Await sibling tasks by name via the Controller; with `waiting` (default) park until all
        are up (order is not fixed): `wifi, = await self.query(['wifi'])`."""
        return await self.controller.query(names, waiting)

    def validate(self) -> bool:
        """Return True if the task is currently healthy."""
        return self._ok

    async def finish(self) -> None:
        """Shut down and release resources."""
        self._ok = False

    # --- Inspectable ---
    def inspect(self) -> dict:
        """Status dict. Subclasses extend it."""
        return {'name': self.name, 'ok': self._ok}
