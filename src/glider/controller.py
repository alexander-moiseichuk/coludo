"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Flight Controller -- creates and supervises the tasks described by a validated config, and tracks the
flight stage machine. See doc/specs/coludo.md ('Flight Controller', 'Tasks').

The Controller is the one task created explicitly; it creates the rest from config in a deterministic
order. Task failures are reported, not fatal (the strict/operator-authority model): a component that
fails setup is logged and skipped, and go/no-go stays with the operator via stats()/validate().
"""

import asyncio

import config as config_module
import inspector
import task

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const


class Stage:
    """
    The flight stages: int ids and their operator-facing names, self-contained.

    Int ids (cheap to compare/store on MicroPython) plus the `STAGES` id->name mapping (operator-facing
    names; `in Stage.STAGES` is an O(1) key check). `NAMES` is the reverse (name->id) so config that
    names stages by string resolves to an id once.

    NULL = 0 is a SENTINEL, not a flight stage: it is what an unwritten NVS checkpoint reads back, so
    a warm-start knows "no checkpoint saved". The live stages start at SETTING = 1, so any non-zero
    saved id IS a real stage to recover into. NULL is deliberately kept OUT of STAGES/NAMES -- it is
    never set_stage()-able and never has an operator name.

    Kept here, in the stage machine's own module. flight/sequencer/hitl/led import it from controller
    -- a LIGHT coupling (the module loads fast, no heavy deps pulled just for the enum). It could move
    to commons.py as the shared domain enum to drop even that import, but the gain is marginal versus
    the cross-file churn; revisit only if importing controller solely for Stage ever bites.
    """

    NULL = const(0)      # sentinel: no checkpoint saved (unwritten NVS reads 0); never a live stage
    SETTING = const(1)
    BOOSTING = const(2)
    GLIDING = const(3)
    LANDING = const(4)
    DONE = const(5)
    STAGES: dict[int, str] = {
        SETTING: 'setting',
        BOOSTING: 'boosting',
        GLIDING: 'gliding',
        LANDING: 'landing',
        DONE: 'done',
    }
    NAMES: dict[str, int] = {name: stage_id for stage_id, name in STAGES.items()}

    @staticmethod
    def active(stage: int) -> bool:
        """True during the powered boost (BOOSTING only) -- the one active-thrust stage."""
        return stage == Stage.BOOSTING

    @staticmethod
    def passive(stage: int) -> bool:
        """True while gliding unpowered, post-separation (GLIDING..LANDING) -- the boost stack is gone."""
        return Stage.GLIDING <= stage <= Stage.LANDING

    @staticmethod
    def airborne(stage: int) -> bool:
        """True while off the ground -- the union of the active boost and the passive glide."""
        return Stage.active(stage) or Stage.passive(stage)


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
        # name -> the up device on its I2C socket feeding the same data: an unfitted ALTERNATIVE, not a fault
        self.alternatives: dict = {}
        self._runners: dict = {}  # name -> asyncio.Task
        self.stage: int = Stage.SETTING
        self.armed: bool = False  # actuation gate -- the control loop holds fins neutral until armed
        self.manual: bool = False  # operator holds the stage (ground test) -> sequencer pauses
        self.warm_started: bool = False  # a mid-air reset was recovered by the warm-start gate (degraded)
        inspector.Inspector.register(self)

    """Scope: which config entries become tasks, plus creating and finding them by name."""
    def _devices(self) -> list:
        """Sensors (data providers) + components (consumers/actuators) -- all are tasks."""
        return self.config.get('sensors', []) + self.config.get('components', [])

    def directory(self) -> list:
        """Names of enabled devices, in creation order (config order)."""
        return [device.get('name') for device in self._devices()
                if device.get('enabled', True) and device.get('name')]

    def _component(self, name: str) -> dict:
        for device in self._devices():
            if device.get('name') == name:
                return device
        return None

    def driver(self, name: str) -> str:
        """
        The driver (or activity) a configured device runs, for the operator-facing verdicts.

        "FAIL laser_agl" did not say WHICH part had failed, and on a socket declared for two parts
        that is the whole question -- so every device verdict names the implementation its config
        entry declares.

        Args:
            name - the device name.

        Returns:
            The entry's `driver`, else its `activity`; None when no configured device has that name.
        """
        comp = self._component(name)
        if comp is None:
            return None
        return comp.get('driver') or comp.get('activity')

    def _socket(self, name: str) -> tuple:
        """
        Where an I2C device answers, as (bus kind, bus id, address); None for any other device.

        Only an I2C ADDRESS identifies a socket. Two UART or PWM devices on one bus are two devices,
        never two candidates for one place. An SPI entry may carry an `addr` too, but that is its
        fallback for an I2C wiring: on SPI the chip-select is the socket, so two SPI parts that share a
        fallback address are two parts, and the dead one must stay a failure. The id compares as text
        because a JSON config may carry it either way, as bustune() already allows.

        Args:
            name - the device name.

        Returns:
            ('i2c', id as text, addr); None for an unknown device, a non-I2C one or one without `addr`.
        """
        comp = self._component(name)
        if comp is None or comp.get('bus') != 'i2c' or comp.get('addr') is None:
            return None
        return ('i2c', str(comp.get('id')), comp.get('addr'))

    def _quantities(self, name: str) -> list:
        """
        The data a device feeds the databoard: its `provides` keys, sorted.

        Args:
            name - the device name.

        Returns:
            E.g. ['agl'] for either laser; [] for an unknown device or one that declares no `provides`.
        """
        comp = self._component(name)
        return sorted((comp or {}).get('provides') or {})

    def _reset_gpio(self, name: str) -> int:
        """
        The GPIO a device's setup pulses to reset its part BEFORE it has identified that part.

        Which pins count, from both laser drivers: `xshut_pin` alone. vl53l4cx/vl53l1x setup() pulse it
        low->high in _reset(), ahead of the model-id check, once per `setup_retries` attempt -- and XSHUT
        low reboots whatever part is soldered on the socket. Their `int_pin` is an input, wired only
        after the model id matched, so an entry whose part is absent never reaches it. No other I2C
        driver drives a pin in setup (the INA226 `alert_pin` is an input; `cs_pin` is SPI, never a
        socket -- see _socket). Resolved as task._pin_gpio does: a pin named in the board `pins` map.

        Args:
            name - the device name.

        Returns:
            The GPIO number; None when the entry routes no XSHUT (layout drops it on v1.0/v1.1, or the
            pins map nulls it).
        """
        gpio = self.config.get('pins', {}).get((self._component(name) or {}).get('xshut_pin'))
        return gpio if isinstance(gpio, int) and gpio >= 0 else None

    def _sort_alternatives(self) -> None:
        """
        Move each failed device that an up device can stand in for from failures to alternatives.

        config_default declares the VL53L4CX and the VL53L1X on one socket, 0x29 (i2c:1 once layout places
        it on v1.0/v1.1): an I2C scan cannot tell them apart, so the board carries both entries and the
        part that is soldered wins. The
        other entry's setup then fails BY DESIGN (its model id check reads the other part), and while
        that sat in failures every consumer called it a fault -- `probe all` printed FAIL and `arm`
        refused every board with the L1X fitted. Two parts cannot share one address on one bus, so the
        loser is simply not fitted.

        The winner must answer the SAME I2C socket (see _socket), provide the SAME data -- the same
        non-empty set of `provides` keys, both {'agl'} -- and run a DIFFERENT driver: two entries of one
        driver on one socket are a config mistake, and the duplicate's setup re-initialises the winner's
        own part, so an error mid-way would leave it stopped while the board armed. Without the data rule
        a config mistake read as a missing part: a baro declared at 0x29 beside a working VL53L1X was
        hidden as "not fitted" and the board armed with no baro, where it had refused before. An entry
        that provides nothing has nothing to stand in for, so it stays a failure too.

        And the loser's setup must not RESET the winner. On v0.1 (or an undecided layout) both laser
        entries route XSHUT to GPIO5 -- layout drops it only on v1.0/v1.1 -- and each setup pulses it
        before its model-id check, so the loser's setup reboots the fitted laser to its defaults: it
        still answers its model id (probe passes) and never ranges again. The loser stays a failure
        whenever its own setup pulses an XSHUT (_reset_gpio), in EITHER order: a loser set up first
        resets nothing this boot, but the declaration order is no contract -- the rule must not hang on
        it. Its reason then tells the operator the fix -- enable exactly one of the two.

        Decided after the WHOLE setup pass, because the loser may be set up before its winner. When no
        such device answers every entry stays a failure: a missing laser must still block arming.

        Args:
            (none)

        Returns:
            None; updates self.failures and self.alternatives (a loser that pulses XSHUT keeps its
            failure, with the shared reset appended to the reason).
        """
        for name in list(self.failures):
            socket = self._socket(name)
            quantities = self._quantities(name)
            if socket is None or not quantities:
                continue
            for winner in self.tasks:
                if (self._socket(winner) != socket or self._quantities(winner) != quantities or
                        self.driver(winner) == self.driver(name)):
                    continue
                gpio = self._reset_gpio(name)
                if gpio is None:
                    self.alternatives[name] = winner
                    del self.failures[name]
                else:
                    self.failures[name] += (
                        '; shares XSHUT GPIO%d with %s (%s): its setup resets the fitted laser -- enable '
                        'exactly one of %s on this board' % (gpio, winner, self.driver(winner),
                                                             ', '.join(sorted((name, winner)))))
                break

    def unfitted(self, name: str) -> str:
        """
        The operator's line for an unfitted alternative: which part is absent and which one took its socket.

        The winner identified itself at setup, so the loser stays not fitted even when the winner's run
        loop crashes later -- but the line must not then say the winner "answers": `probe` and `arm`
        call it failed, and the line follows them.

        Args:
            name - a device in self.alternatives.

        Returns:
            'vl53l4cx not fitted -- vl53l1x (laser_agl_l1x) answers i2c:1 0x29' while the winner runs;
            '... answered i2c:1 0x29 at setup, now down' once the winner's run loop has crashed.
        """
        winner = self.alternatives[name]
        where = '%s:%s 0x%02x' % self._socket(winner)
        if winner in self.failures:
            return '%s not fitted -- %s (%s) answered %s at setup, now down' % (
                self.driver(name), self.driver(winner), winner, where)
        return '%s not fitted -- %s (%s) answers %s' % (self.driver(name), self.driver(winner), winner, where)

    def create(self, name: str) -> task.Task:
        """
        Create a task by component name via the registry.

        A component names its implementation with `driver` (from drivers/) or `activity` (from tasks/).

        Args:
            name - the component name to build (looked up in this config).

        Returns:
            The task, or None if the component is absent or names no known driver/activity.
        """
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
        """
        The active task by name, or all active tasks.

        Args:
            name - the task name to look up; None returns every active task.

        Returns:
            The named task (None if absent), or a list of all active tasks when name is None.
        """
        if name is None:
            return list(self.tasks.values())
        return self.tasks.get(name)

    def find(self, names: list[str]) -> list:
        """
        The active tasks for `names`, without blocking.

        The fast lookup for sync code; query() is the awaitable that can wait for dependencies.

        Args:
            names - the task names to look up.

        Returns:
            A list aligned with `names`, with None for any task not up.
        """
        return [self.tasks.get(name) for name in names]

    async def query(self, names: list[str], waiting: bool = True) -> list:
        """
        Look up sibling tasks by name from the registry, optionally waiting until they are all up.

        Example: `gnss, baro = await self.query(['gnss', 'baro_icp10111'])`. waiting=False returns
        immediately, and the caller must handle the Nones (safe anywhere, including setup());
        waiting=True awaits until every named task is present, then returns them all.

        IMPORTANT -- call waiting=True only from run(), never from setup():
          * setup() runs serially in the single bring-up coroutine: the Controller awaits each
            task's setup() before creating the next. Blocking there blocks the whole boot -- if the
            dependency is set up later in the order, you deadlock bring-up.
          * run() loops are concurrent: awaiting here suspends only THIS task's coroutine while the
            event loop keeps scheduling every other task's run(), so the rest of boot progresses.
            When the dependency appears, the await resumes.

        The wait is await-based (poll + asyncio.sleep), so the current coroutine yields and never
        starves the single-core scheduler -- never a busy `while not found`. Rule of thumb:
        discover-or-skip in setup() (waiting=False, handle None); block-for-ready in run()
        (waiting=True). A wait timeout (so a never-appearing dependency surfaces as a logged error
        rather than a task parked forever) fits the strict/operator-authority model, and is not yet
        implemented.

        Args:
            names - the sibling task names to look up.
            waiting - True (default) parks until all are present; False returns immediately.

        Returns:
            A list aligned with `names`: with waiting=False, None for any task not yet enlisted;
            with waiting=True, every named task (never None).
        """
        while True:
            found = [self.tasks.get(name) for name in names]
            if not waiting or all(entry is not None for entry in found):
                return found
            await asyncio.sleep_ms(50)

    """Lifecycle: bring the configured devices up, supervise them, tear them down."""
    async def setup(self) -> bool:
        """
        Create + set up every enabled task in order, skipping (and reporting) failures.

        setup() brings a device to a SAFE state (sensors detect-or-skip, servos centre) with no costly
        side effects; the active self-test is probe(), run on demand (the CC `probe` command), never at
        boot -- a mid-flight reboot must not sweep the fins.

        Returns:
            True once bring-up has run; failures are recorded in self.failures, not raised, and a
            failed entry that an up device stands in for (same I2C socket, same data, no XSHUT pulse
            of its own -- see _sort_alternatives) lands in self.alternatives instead.
        """
        self.failures = {}  # recomputed each bring-up
        self.alternatives = {}
        attempts = max(1, self.config.get('board', {}).get('setup_retries', 1))  # retry flaky contacts (breadboard)
        for name in self.directory():
            if name in self.tasks:
                continue
            new_task = await self._bring_up(name, attempts)
            if new_task is not None:
                self.tasks[name] = new_task
                inspector.Inspector.register(new_task)  # operator can `inspect <task>`
                self.log("controller :: task '%s' up" % name)
        self._sort_alternatives()
        for name in self.alternatives:
            self.log('controller :: %s: %s' % (name, self.unfitted(name)))
        if self.failures:
            self.log('controller :: %d device(s) not up: %s' % (
                len(self.failures), ', '.join(sorted(self.failures))))
        return True

    async def _bring_up(self, name: str, attempts: int) -> task.Task:
        """
        Create + set up a device, retrying a flaky setup up to `attempts` times.

        Breadboard contacts make and break, so a setup gets several tries before it is called a failure.

        Args:
            name - the component name to bring up.
            attempts - how many setup tries before giving up.

        Returns:
            The task on success; else None, with the failure reason recorded in self.failures.
        """
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
            if attempt == attempts:  # final try -> deeper wire-level analysis while the transport is still alive
                reason = await self._diagnose(new_task, reason)
            try:  # clean up the half-set-up device; a cleanup failure must NOT abort the rest of boot
                await new_task.finish()
            except Exception as error:
                self.log("controller :: task '%s' cleanup raised: %r" % (name, error))
            if attempt < attempts:
                self.log("controller :: task '%s' setup attempt %d/%d failed, retrying" % (name, attempt, attempts))
                await asyncio.sleep_ms(200)  # let a flaky contact settle before the retry
        self.failures[name] = reason
        return None

    async def _diagnose(self, new_task, reason: str) -> str:
        """
        Fold a driver's optional diagnose() into the failure reason.

        A deeper, wire-level analysis (chip-select dead / MISO floating / wrong device /
        present-but-init-failed) that the operator sees in `verify`/`probe`. Best-effort: a driver
        without diagnose(), or one that raises, just keeps the generic reason.

        Args:
            new_task - the task whose optional diagnose() is consulted.
            reason - the generic failure reason to augment.

        Returns:
            The reason with the driver's diagnostic appended, or the unchanged reason when there is no
            diagnose() or it raised.
        """
        analyse = getattr(new_task, 'diagnose', None)
        if analyse is None:
            return reason
        try:
            detail = await analyse()
        except Exception as error:
            return '%s [diagnose raised %r]' % (reason, error)
        return '%s -- %s' % (reason, detail) if detail else reason

    async def bustune(self, kind: str, ident, freq: int) -> dict:
        """
        Retune a sensor bus in place and report which of its devices stay healthy.

        The bench frequency-calibration primitive the CC-side sweep drives: retune sensor bus
        <kind>:<ident> to <freq> Hz IN PLACE (no reboot), then check each up device's probe() (id
        reads back + a sample succeeds). Reporting the per-device verdicts + all_ok lets the host find
        the bus ceiling AND the limiting device (whoever drops out first as freq climbs). Nothing is
        persisted here: CC saves the chosen freq via set-config board + reboot. i2c/spi only (uart/pwm
        are not frequency-swept).

        Args:
            kind - the bus kind, 'i2c' or 'spi'.
            ident - the bus id (number) to retune.
            freq - the target frequency in Hz.

        Returns:
            {kind, id, freq, devices, all_ok}: devices {name: 'ok' | '<driver> -- <why>'}, leaving out
            an unfitted alternative (it is not on the bus); or {'error': ...} for a non-tunable kind or
            an undefined bus.
        """
        modules = {'i2c': 'i2cbus', 'spi': 'spibus'}
        if kind not in modules:
            return {'error': "bus kind '%s' is not tunable (i2c/spi only)" % kind}
        spec = config_module.bus(self.config, kind, ident)
        if spec is None:
            return {'error': 'bus %s:%s not defined' % (kind, ident)}
        bus = __import__(modules[kind]).get(int(ident), spec)
        await bus.retune(freq)
        devices = {}
        for item in self.config.get('sensors', []) + self.config.get('components', []):
            if item.get('bus') != kind or str(item.get('id')) != str(ident):
                continue
            name = item.get('name')
            if name in self.alternatives:
                continue  # not on this bus at all: counting it 'down' failed every rung of the sweep
            running = self.active(name)
            if running is None:
                verdict = 'down: ' + self.failures.get(name, 'not up')
            elif hasattr(running, 'probe'):
                verdict = await running.probe() or 'ok'  # probe() -> None when healthy
            else:
                verdict = 'no probe'
            # 'ok' is the contract the CC sweep reads; anything else names the part, as probe/verify do
            devices[name] = verdict if verdict == 'ok' else '%s -- %s' % (self.driver(name), verdict)
        return {'kind': kind, 'id': ident, 'freq': freq, 'devices': devices,
                'all_ok': bool(devices) and all(verdict == 'ok' for verdict in devices.values())}

    async def start(self) -> None:
        """Launch each task's run() loop as a supervised asyncio task."""
        for name, pending_task in self.tasks.items():
            if name not in self._runners:
                self._runners[name] = asyncio.create_task(self._supervise(name, pending_task))

    async def _supervise(self, name: str, supervised_task: task.Task) -> None:
        """
        Run a task to completion; on crash, mark it DOWN and record it durably.

        A crashed run loop used to be reported to the console and nowhere else: the task stayed in
        self.tasks, stayed out of self.failures, and kept `_ok` True -- so a sensor whose loop had died
        still answered `validate()` healthy and passed the operator readiness gate that `cc arm`
        checks. Its data simply stopped appearing, which reads like a quiet sensor rather than a dead
        one.

        All three surfaces are updated, because each answers a different question: `_ok` is what
        `verify`/`arm` consult, `failures` is what the bring-up report lists, and event() is DURABLE --
        Recorder.log is best-effort and a short flight ends without it, so a console-only record of the
        one moment a task died is exactly the fact task.event exists to preserve.

        Restart policy remains a later concern; being honest about the death is not.
        """
        try:
            await supervised_task.run()
        except asyncio.CancelledError:
            raise
        except Exception as e:
            supervised_task._ok = False  # verify/arm must not pass a task whose loop is gone
            self.failures[name] = 'run loop crashed: %r' % e
            try:
                supervised_task.event('run loop crashed: %r' % e)  # durable: reaches the capture
            except Exception:
                pass  # a dead task must not be able to take the supervisor down with it
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
        """
        Shut down all tasks, in REVERSE bring-up order.

        Reverse order so a command PRODUCER (e.g. the flight loop, which centres the fins in its
        finish()) closes before the actuators/resources it writes to -- otherwise teardown drives an
        already-released peripheral (PWM deinit'd -> RuntimeError). Best effort: a single task's
        finish() error is logged, never stranding the rest of the shutdown.

        Returns:
            None; every task closed and the stage left at DONE.
        """
        for name in reversed(list(self.tasks)):
            try:
                await self.close(name)
            except Exception as error:  # noqa: BLE001 -- teardown must continue past one bad task
                self.log("controller :: finish '%s' error: %r" % (name, error))
        self.stage = Stage.DONE

    """Stage: the flight stage machine and its operator-facing name."""
    def set_stage(self, stage: int) -> None:
        if stage not in Stage.STAGES:
            raise ValueError('unknown stage: %s' % stage)
        self.stage = stage
        self.log('controller :: stage -> %s' % Stage.STAGES[stage])

    def stage_name(self) -> str:
        """The current flight stage as its operator-facing name."""
        return Stage.STAGES.get(self.stage, '?')  # tolerate an out-of-range stage: inspect()/stats() must not crash

    """Arming: the actuation gate and the operator stage/health gates."""
    def arm(self) -> None:
        """
        Enable actuation.

        The pre-flight precondition (probe all clean, mission set) is enforced by the caller (the CC
        `arm` command / field auto-arm); arming also pins the live GNSS fix as the launch point
        (mission.freeze_launch) so the tier-2 heading survives a mid-flight fix loss.

        Returns:
            None; sets self.armed and freezes the launch point.
        """
        mission = inspector.Inspector.get('mission')
        if mission is not None:  # None only in unit rigs without a mission
            mission.freeze_launch()
        self.armed = True
        self.log('controller :: armed')

    def disarm(self) -> None:
        self.armed = False
        self.log('controller :: disarmed')

    def hold(self, stage_name: str) -> bool:
        """
        Operator stage override (ground test): force a stage and pause auto-sequencing.

        Args:
            stage_name - the stage to force, by its operator-facing name.

        Returns:
            True once held at the stage; False for an unknown stage name.
        """
        for stage_id, name in Stage.STAGES.items():
            if name == stage_name:
                self.set_stage(stage_id)
                """
                Going back to SETTING is a return to the ground, not a hold. Holding SETTING suppressed
                every launch detector, so a board that had a fin check and then `stage setting` -- the
                natural way back -- flew its whole flight recorded as SETTING unless someone also
                remembered `stage auto`, which field_test.md never mentioned. Nothing showed the hold.
                """
                self.manual = stage_id != Stage.SETTING
                self.log('controller :: stage %s (%s)' % (name, 'held' if self.manual else 'auto'))
                return True
        return False

    def resume(self) -> None:
        """Clear the operator hold -> the sequencer drives the stages again."""
        self.manual = False
        self.log('controller :: stage auto')

    def validate(self) -> bool:
        """True if every active task is healthy."""
        for entry in self.tasks.values():
            if not entry.validate():
                return False
        return True

    """Inspectable: the operator-facing state snapshot (inspect) and per-task stats."""
    def inspect(self) -> dict:
        return {'stage': self.stage_name(), 'armed': self.armed, 'manual': self.manual,
                'tasks': list(self.tasks.keys()), 'failures': self.failures, 'alternatives': self.alternatives}

    def stats(self) -> dict:
        return {'stage': self.stage_name(),
                'tasks': dict((name, entry.inspect()) for name, entry in self.tasks.items())}
