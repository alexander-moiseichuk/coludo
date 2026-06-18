# Task base class and driver registry — the unit the Controller creates and supervises.
#
# Every component/system task follows the common lifecycle from specs/coludo.md:
#   setup()    async; initialize or reset; return True on success
#   run()      async; the task's main activity loop
#   notify()   subscribe a callback for this task's updates
#   validate() return True if the task is currently healthy
#   finish()   async; shut down and release resources
# A Task is Inspectable: inspect()/update()/stats() expose it to the operator (the Controller
# registers each task with the Inspector), so there is no separate report().
#
# A task registers itself with @activity('name') (or its alias @driver('name') for the HAL ones in
# drivers/); the Controller maps a component's 'driver' field to the class via ACTIVITIES. The two
# names share one registry for now -- splitting drivers out is a later concern if it is needed.

import inspector

ACTIVITIES: dict = {}  # registered name -> Task subclass (drivers + activities, one registry)


def activity(name: str):
    """Class decorator: register a Task subclass (a HAL driver or a higher-level activity) under a
    name so the Controller can build it from a config component."""

    def deco(cls):
        ACTIVITIES[name] = cls
        return cls

    return deco


driver = activity  # alias: drivers/ files read as @task.driver, tasks/ files as @task.activity


class Task(inspector.Inspectable):
    kind = 'task'

    def __init__(self, name, config=None, controller=None):
        self.name = name
        self.config = config or {}  # this task's sensor/component dict from board.json
        self.controller = controller  # back-reference for active()/notify()
        self._ok = False
        self._subs = []

    async def setup(self):
        """Initialize or reset. Override. Return True on success, False otherwise."""
        self._ok = True
        return True

    async def run(self):
        """Main activity loop. Override. Default returns immediately."""
        pass

    def notify(self, callback):
        """Register callback(task, event) to be invoked on this task's updates."""
        if callback not in self._subs:
            self._subs.append(callback)

    def emit(self, event=None):
        """Notify all subscribers of an update."""
        for cb in self._subs:
            cb(self, event)

    def validate(self):
        """Return True if the task is currently healthy."""
        return self._ok

    async def finish(self):
        """Shut down and release resources."""
        self._ok = False

    # --- Inspectable ---
    def inspect(self):
        """Status dict. Subclasses extend it."""
        return {'name': self.name, 'ok': self._ok}
