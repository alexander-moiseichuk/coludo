# Flight Controller — creates and supervises the tasks described by a validated config, and
# tracks the flight state machine. See specs/coludo.md ('Flight Controller', 'Tasks').
#
# The Controller is the one task created explicitly; it creates the rest from config in a
# deterministic order. Task failures are reported, not fatal (the strict/operator-authority
# model): a component that fails setup is logged and skipped, and go/no-go stays with the
# operator via report()/validate().

import asyncio

from inspector import Inspectable, Inspector
from task import DRIVERS

STATES = ('setting', 'boosting', 'gliding', 'landing', 'done')


class Controller(Inspectable):
    name = 'controller'
    kind = 'controller'

    def __init__(self, config, registry=None, log=None):
        self.config = config
        self.registry = registry if registry is not None else DRIVERS
        self.log = log if log is not None else (lambda msg: None)
        self.tasks = {}  # name -> Task
        self._runners = {}  # name -> asyncio.Task
        self.state = 'setting'
        Inspector.register(self)

    # ------------------------------------------------------------------ scope
    def _devices(self):
        """Sensors (data providers) + components (consumers/actuators) — all are tasks."""
        return self.config.get('sensors', []) + self.config.get('components', [])

    def directory(self):
        """Names of enabled devices, in creation order (config order)."""
        return [d.get('name') for d in self._devices() if d.get('enabled', True) and d.get('name')]

    def _component(self, name):
        for d in self._devices():
            if d.get('name') == name:
                return d
        return None

    def create(self, name):
        """Create a task by component name via the driver registry. Returns task or None."""
        comp = self._component(name)
        if comp is None:
            return None
        cls = self.registry.get(comp.get('driver'))
        if cls is None:
            self.log("controller :: no driver '%s' for task '%s'" % (comp.get('driver'), name))
            return None
        return cls(name, comp, self)

    def active(self, name=None):
        """Return the active task by name, or a list of all active tasks if name is None."""
        if name is None:
            return list(self.tasks.values())
        return self.tasks.get(name)

    # -------------------------------------------------------------- lifecycle
    async def setup(self):
        """Create + set up every enabled task in order. Skip (and report) failures."""
        for name in self.directory():
            if name in self.tasks:
                continue
            task = self.create(name)
            if task is None:
                continue
            try:
                ok = await task.setup()
            except Exception as e:
                self.log("controller :: task '%s' setup raised: %r" % (name, e))
                ok = False
            if ok:
                self.tasks[name] = task
                self.log("controller :: task '%s' up" % name)
            else:
                self.log("controller :: task '%s' failed setup" % name)
                await task.finish()
        return True

    async def start(self):
        """Launch each task's run() loop as a supervised asyncio task."""
        for name, task in self.tasks.items():
            if name not in self._runners:
                self._runners[name] = asyncio.create_task(self._supervise(name, task))

    async def _supervise(self, name, task):
        """Run a task to completion; on crash, log it (restart policy is a later concern)."""
        try:
            await task.run()
        except asyncio.CancelledError:
            raise
        except Exception as e:
            self.log("controller :: task '%s' crashed: %r" % (name, e))

    async def close(self, name):
        """Deactivate a task and clean up its resources."""
        runner = self._runners.pop(name, None)
        if runner is not None:
            runner.cancel()
        task = self.tasks.pop(name, None)
        if task is not None:
            await task.finish()

    async def finish(self):
        """Shut down all tasks."""
        for name in list(self.tasks):
            await self.close(name)
        self.state = 'done'

    # ------------------------------------------------------------------ state
    def set_state(self, state):
        if state not in STATES:
            raise ValueError('unknown state: %s' % state)
        self.state = state
        self.log('controller :: state -> %s' % state)

    # ----------------------------------------------------------- introspection
    def report(self):
        return {'state': self.state, 'tasks': dict((n, t.report()) for n, t in self.tasks.items())}

    def validate(self):
        """True if every active task is healthy."""
        for t in self.tasks.values():
            if not t.validate():
                return False
        return True

    # --- Inspectable ---
    def inspect(self):
        return {'state': self.state, 'tasks': list(self.tasks.keys())}

    def stats(self):
        return self.report()
