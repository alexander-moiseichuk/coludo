# Flight Controller — creates and supervises the tasks described by a validated config, and
# tracks the flight stage machine. See specs/coludo.md ('Flight Controller', 'Tasks').
#
# The Controller is the one task created explicitly; it creates the rest from config in a
# deterministic order. Task failures are reported, not fatal (the strict/operator-authority
# model): a component that fails setup is logged and skipped, and go/no-go stays with the
# operator via stats()/validate().

import asyncio

import inspector
import task

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)

    def const(value):
        return value


class Stage:
    """The flight stages, self-contained: int ids (cheap to compare/store on MicroPython) and the
    `STAGES` id->name mapping (operator-facing names; `in Stage.STAGES` is an O(1) key check). `NAMES`
    is the reverse (name->id) so config that names stages by string resolves to an id once."""

    SETTING = const(0)
    BOOSTING = const(1)
    GLIDING = const(2)
    LANDING = const(3)
    DONE = const(4)
    STAGES: dict[int, str] = {
        SETTING: 'setting',
        BOOSTING: 'boosting',
        GLIDING: 'gliding',
        LANDING: 'landing',
        DONE: 'done',
    }
    NAMES: dict[str, int] = {name: stage_id for stage_id, name in STAGES.items()}


class Controller(inspector.Inspectable):
    name: str = 'controller'
    kind: str = 'controller'

    def __init__(self, config: dict, registry: dict = None, log=None):
        self.config: dict = config
        # the CLASS registry (name -> Task class) used by create(); the INSTANCE directory is
        # self.tasks, looked up by find()/query(). Injected for tests; defaults to task.ACTIVITIES.
        self.registry: dict = registry if registry is not None else task.ACTIVITIES
        self.log = log if log is not None else (lambda msg: None)
        self.tasks: dict = {}  # name -> Task
        self.failures: dict = {}  # name -> reason, for enabled devices that did not come up (setup)
        self._runners: dict = {}  # name -> asyncio.Task
        self.stage: int = Stage.SETTING
        self.armed: bool = False  # actuation gate -- the control loop holds fins neutral until armed
        self.manual: bool = False  # operator holds the stage (ground test) -> sequencer pauses
        inspector.Inspector.register(self)

    # ------------------------------------------------------------------ scope
    def _devices(self) -> list:
        """Sensors (data providers) + components (consumers/actuators) — all are tasks."""
        return self.config.get('sensors', []) + self.config.get('components', [])

    def directory(self) -> list:
        """Names of enabled devices, in creation order (config order)."""
        return [d.get('name') for d in self._devices() if d.get('enabled', True) and d.get('name')]

    def _component(self, name: str) -> dict:
        for d in self._devices():
            if d.get('name') == name:
                return d
        return None

    def create(self, name: str) -> task.Task:
        """Create a task by component name via the registry. A component names its implementation
        with `driver` (from drivers/) or `activity` (from tasks/). Returns task or None."""
        comp = self._component(name)
        if comp is None:
            return None
        runs = comp.get('driver') or comp.get('activity')
        cls = self.registry.get(runs)
        if cls is None:
            self.log("controller :: no driver/activity '%s' for '%s'" % (runs, name))
            return None
        return cls(name, comp, self)

    def active(self, name: str = None):
        """Return the active task by name (None if absent), or a list of all active tasks if
        `name` is None."""
        if name is None:
            return list(self.tasks.values())
        return self.tasks.get(name)

    def find(self, names: list[str]) -> list:
        """Non-blocking: the active tasks for `names`, None for any not up. The fast lookup for
        sync code; `query` is the awaitable that can wait for dependencies."""
        return [self.tasks.get(name) for name in names]

    async def query(self, names: list[str], waiting: bool = True) -> list:
        """Look up sibling tasks by name from the registry: `gnss, baro = await self.query(['gnss',
        'baro_icp10111'])`.

        waiting=False: return immediately — a list aligned with `names`, with None for any task not
        yet enlisted. The caller must handle the Nones. Safe anywhere, including setup().

        waiting=True: await until every named task is present, then return them all.

        IMPORTANT — call waiting=True only from run(), never from setup():
          * setup() runs serially in the single bring-up coroutine: the Controller awaits each
            task's setup() before creating the next. Blocking there blocks the whole boot — if the
            dependency is set up later in the order, you deadlock bring-up.
          * run() loops are concurrent: awaiting here suspends only THIS task's coroutine while the
            event loop keeps scheduling every other task's run(), so the rest of boot progresses.
            When the dependency appears, the await resumes.

        The wait is await-based (poll + asyncio.sleep), so the current coroutine yields and never
        starves the single-core scheduler — never a busy `while not found`.

        Rule of thumb: discover-or-skip in setup() (waiting=False, handle None); block-for-ready in
        run() (waiting=True). A wait timeout (so a never-appearing dependency surfaces as a logged
        error rather than a task parked forever) fits the strict/operator-authority model — TODO."""
        while True:
            found = [self.tasks.get(name) for name in names]
            if not waiting or all(t is not None for t in found):
                return found
            await asyncio.sleep_ms(50)

    # -------------------------------------------------------------- lifecycle
    async def setup(self) -> bool:
        """Create + set up every enabled task in order. Skip (and report) failures. setup() brings a
        device to a SAFE state (sensors detect-or-skip, servos centre) with no costly side effects;
        the active self-test is probe(), run on demand (the CC `probe` command), never at boot -- a
        mid-flight reboot must not sweep the fins."""
        self.failures = {}  # recomputed each bring-up
        attempts = max(1, self.config.get('setup_retries', 1))  # retry flaky contacts (breadboard)
        for name in self.directory():
            if name in self.tasks:
                continue
            new_task = await self._bring_up(name, attempts)
            if new_task is not None:
                self.tasks[name] = new_task
                inspector.Inspector.register(new_task)  # operator can `inspect <task>`
                self.log("controller :: task '%s' up" % name)
        if self.failures:
            self.log('controller :: %d device(s) not up: %s' % (
                len(self.failures), ', '.join(sorted(self.failures))))
        return True

    async def _bring_up(self, name: str, attempts: int) -> task.Task:
        """Create + set up a device, retrying a flaky setup up to `attempts` times (breadboard contacts
        make and break). Returns the task on success, else None and records the failure reason."""
        reason = 'no driver/activity'  # if create() never yields a task (missing driver)
        for attempt in range(1, attempts + 1):
            new_task = self.create(name)
            if new_task is None:
                break  # missing driver/activity -> retrying cannot help
            try:
                if await new_task.setup():
                    return new_task
                reason = 'setup failed (absent / miswired?)'
            except Exception as error:
                reason = repr(error)
                self.log("controller :: task '%s' setup raised: %r" % (name, error))
            try:  # clean up the half-set-up device; a cleanup failure must NOT abort the rest of boot (1.2.1)
                await new_task.finish()
            except Exception as error:
                self.log("controller :: task '%s' cleanup raised: %r" % (name, error))
            if attempt < attempts:
                self.log("controller :: task '%s' setup attempt %d/%d failed, retrying" % (name, attempt, attempts))
                await asyncio.sleep_ms(200)  # let a flaky contact settle before the retry
        self.failures[name] = reason
        return None

    async def start(self) -> None:
        """Launch each task's run() loop as a supervised asyncio task."""
        for name, pending_task in self.tasks.items():
            if name not in self._runners:
                self._runners[name] = asyncio.create_task(self._supervise(name, pending_task))

    async def _supervise(self, name: str, supervised_task: task.Task) -> None:
        """Run a task to completion; on crash, log it (restart policy is a later concern)."""
        try:
            await supervised_task.run()
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self.log("controller :: task '%s' crashed: %r" % (name, e))

    async def close(self, name: str) -> None:
        """Deactivate a task and clean up its resources."""
        runner = self._runners.pop(name, None)
        if runner is not None:
            runner.cancel()
        closing_task = self.tasks.pop(name, None)
        if closing_task is not None:
            inspector.Inspector.unregister(name)
            await closing_task.finish()

    async def finish(self) -> None:
        """Shut down all tasks, in REVERSE bring-up order so a command PRODUCER (e.g. the flight loop,
        which centres the fins in its finish()) closes before the actuators/resources it writes to --
        otherwise teardown drives an already-released peripheral (PWM deinit'd -> RuntimeError). Best
        effort: a single task's finish() error is logged, never stranding the rest of the shutdown."""
        for name in reversed(list(self.tasks)):
            try:
                await self.close(name)
            except Exception as error:  # noqa: BLE001 -- teardown must continue past one bad task
                self.log("controller :: finish '%s' error: %r" % (name, error))
        self.stage = Stage.DONE

    # ------------------------------------------------------------------ stage
    def set_stage(self, stage: int) -> None:
        if stage not in Stage.STAGES:
            raise ValueError('unknown stage: %s' % stage)
        self.stage = stage
        self.log('controller :: stage -> %s' % Stage.STAGES[stage])

    def stage_name(self) -> str:
        """The current flight stage as its operator-facing name."""
        return Stage.STAGES[self.stage]

    # ------------------------------------------------------------------ arming
    def arm(self) -> None:
        """Enable actuation. The pre-flight precondition (probe all clean, mission set) is enforced by
        the caller (the CC `arm` command); here it is the bare state + log."""
        self.armed = True
        self.log('controller :: armed')

    def disarm(self) -> None:
        self.armed = False
        self.log('controller :: disarmed')

    def hold(self, stage_name: str) -> bool:
        """Operator stage override (ground test): force a stage and pause auto-sequencing. Returns
        False for an unknown stage name."""
        for stage_id, name in Stage.STAGES.items():
            if name == stage_name:
                self.set_stage(stage_id)
                self.manual = True
                return True
        return False

    def resume(self) -> None:
        """Clear the operator hold -> the sequencer drives the stages again."""
        self.manual = False
        self.log('controller :: stage auto')

    def validate(self) -> bool:
        """True if every active task is healthy."""
        for t in self.tasks.values():
            if not t.validate():
                return False
        return True

    # --- Inspectable ---
    def inspect(self) -> dict:
        return {'stage': self.stage_name(), 'armed': self.armed, 'manual': self.manual,
                'tasks': list(self.tasks.keys()), 'failures': self.failures}

    def stats(self) -> dict:
        return {'stage': self.stage_name(), 'tasks': dict((n, t.inspect()) for n, t in self.tasks.items())}
