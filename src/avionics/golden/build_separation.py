"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture separation.json: plan B's stage-separation switch (src/glider/drivers/separation.py) as the Rust
port (crates/drivers, module separation) must answer it, pin read by pin read, compared exactly.

Plan B's run() waits on a ThreadSafeFlag the pin IRQ sets on either edge, then _settle() reads the pin, sleeps 5 ms,
reads again, ... until >= 3 agreeing reads span debounce_ms (a verdict) or debounce_ms x 10 runs out (none); _apply()
then moves the latch, and a separation in BOOSTING moves the stage to GLIDING. The Rust port steps the same
debounce once per pin read on the control tick (avionics-port.md §4.3). So the fixture is the sequence of reads
plan B made, each with its time and level and whether an edge was flagged since the previous read; the port, fed
the same reads, must reach the same verdicts on the same reads and record the same events.

The builder runs plan B's own run() and _settle(), with:
  * a virtual clock (the shims'), moved 1 ms at a time: by plan B's own asyncio.sleep_ms while it settles, and by
    the builder while run() waits idle on its flag;
  * a scripted pin: a timeline of (ms, level) the pin follows (level 1 = HIGH = nested, 0 = open = separated);
  * the IRQ: fired on every change of the timeline's level as the clock passes it, as the both-edge GPIO interrupt
    does -- or, where a scenario says `irq: "first"`, once at the start and never again, which is how
    test_separation.py drives it (its scripted pin raises no IRQs);
  * a flag with ThreadSafeFlag's semantics (set by the IRQ, cleared by the wait it releases), since the shims leave
    ThreadSafeFlag unshimmed and plan B constructs it only in setup(), which needs a real Pin;
  * a stub Controller (stage and set_stage) and a telemetry stand-in for separation.csv.

Scenarios: test_separation.py's four timelines (the 7 ms blip, the chatter, the held separation, the bounce then
hold), each as the test drives it and again with real IRQs; the guards (not BOOSTING, a re-nest, a latch already
separated); the debounce corners (0 ms never settles, 1 ms, 50 ms); then seeded timelines.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import random
import time

import controller
import recorder
import shims
from drivers import separation

_SEED: int = 20261006  # the seeded timelines' random.Random seed: the fixture is a function of the code alone
_TAIL_MS: int = 400  # how long a scenario runs past its timeline's last change, for the settles to end
_SEEDED: int = 24  # seeded scenarios

_STAGE = controller.Stage


class _Flag:
    """asyncio.ThreadSafeFlag as separation.run() uses it: set() by the IRQ; wait() returns once set, clearing it."""

    def __init__(self) -> None:
        self._event = asyncio.Event()
        self.waiting: bool = False

    def set(self) -> None:
        self._event.set()

    def is_set(self) -> bool:
        return self._event.is_set()

    async def wait(self) -> None:
        self.waiting = True
        await self._event.wait()
        self.waiting = False
        self._event.clear()


class _Controller:
    """The stage holder separation reads and sets."""

    def __init__(self, stage: int) -> None:
        self.stage = stage

    def set_stage(self, stage: int) -> None:
        self.stage = stage


class _Rows:
    """separation.csv's stand-in: keeps the rows pushed."""

    def __init__(self) -> None:
        self.rows: list = []

    def push(self, row: tuple) -> None:
        self.rows.append(list(row))


class _Run:
    """One scenario on plan B's driver: the timeline, the virtual clock's origin, and the reads recorded."""

    def __init__(self, timeline: list, irq: str) -> None:
        self.timeline = timeline
        self.irq = irq
        self.origin = time.ticks_ms()
        self.edge = False  # an IRQ fired since the last read
        self.reads: list = []

    def now(self) -> int:
        """Milliseconds since the scenario began."""
        return time.ticks_diff(time.ticks_ms(), self.origin)

    def level(self, at: int) -> int:
        """The timeline's level at `at` ms."""
        level = self.timeline[0][1]
        for start, value in self.timeline:
            if at >= start:
                level = value
        return level

    def tick(self, driver: separation.Separation) -> None:
        """Move the clock 1 ms; a change of level on the way fires the IRQ (unless the scenario fires only one)."""
        before = self.level(self.now())
        shims.advance(1)
        if self.irq == 'every' and self.level(self.now()) != before:
            self.edge = True
            driver._on_edge(None)

    def value(self) -> int:
        """The scripted pin's read: recorded with its time, level and the edge flag since the previous read."""
        now, level = self.now(), self.level(self.now())
        self.reads.append({'in': {'now_ms': now, 'high': bool(level), 'edge': self.edge}, 'out': None})
        self.edge = False
        return level


def _scenario(name: str, timeline: list, stage: int = _STAGE.BOOSTING, debounce_ms: int = 20, separated: bool = False,
              irq: str = 'every') -> dict:
    """
    Run plan B's separation driver over one timeline.

    Args:
        name - the scenario name.
        timeline - [(ms, level), ...] ascending from 0; level 1 nested, 0 separated.
        stage - the stage the Controller holds at the start.
        debounce_ms - the config's debounce.
        separated - the latch's level at the start (setup's pin read).
        irq - 'every' for an IRQ on each change, 'first' for one IRQ at 0 ms only (test_separation.py's way).

    Returns:
        {name, init, steps}: a step per pin read; its out names the verdict on the read that concludes a settle,
        and the event and stage _apply then recorded (null for none).
    """
    run = _Run(timeline, irq)
    stub = _Controller(stage)
    driver = separation.Separation('separation', {'debounce_ms': debounce_ms}, stub)
    flag = _Flag()
    driver._flag, driver._debounce_ms, driver._separated = flag, debounce_ms, separated
    driver._telemetry = _Rows()
    driver._pin = run

    settle, apply = driver._settle, driver._apply
    concluding: list = []  # the step of the read that concluded the latest settle

    async def settle_recorded():
        """_settle(), its verdict attached to the read that concluded it (the latest read, at its return)."""
        verdict = await settle()
        step = run.reads[-1]
        step['out'] = {'verdict': {True: 'separated', False: 'nested', None: 'unsettled'}[verdict], 'event': None}
        concluding[:] = [step]
        return verdict

    def apply_recorded(level: bool) -> None:
        """_apply(), the separation.csv row it pushes attached to that same step."""
        rows = len(driver._telemetry.rows)
        apply(level)
        if len(driver._telemetry.rows) > rows:
            concluding[0]['out']['event'] = driver._telemetry.rows[-1]

    driver._settle, driver._apply = settle_recorded, apply_recorded
    original_sleep, original_log = asyncio.sleep_ms, recorder.Recorder.log

    async def sleep_ms(milliseconds: int) -> None:
        """plan B's settling wait: the clock moves 1 ms at a time, firing the IRQ on the way."""
        for _ in range(milliseconds):
            run.tick(driver)
        await asyncio.sleep(0)

    async def drive() -> None:
        """Run plan B's run() to the end of the timeline, moving the clock while it waits idle on its flag."""
        runner = asyncio.create_task(driver.run())
        await asyncio.sleep(0)
        if irq == 'first':
            run.edge = True
            driver._on_edge(None)
        end = timeline[-1][0] + _TAIL_MS
        while True:
            await asyncio.sleep(0)
            if flag.waiting and not flag.is_set():
                if run.now() >= end:
                    break
                run.tick(driver)
        runner.cancel()

    asyncio.sleep_ms = sleep_ms
    recorder.Recorder.log = staticmethod(lambda descriptor, message: None)
    try:
        asyncio.run(drive())
    finally:
        asyncio.sleep_ms, recorder.Recorder.log = original_sleep, original_log
    if not run.reads:
        raise ValueError('separation: scenario %s never read the pin' % name)
    return {'name': name,
            'init': {'debounce_ms': debounce_ms, 'separated': separated, 'stage': _STAGE.STAGES[stage],
                     'timeline': [list(change) for change in timeline], 'irq': irq},
            'final': {'separated': driver._separated, 'stage': _STAGE.STAGES[stub.stage]},
            'steps': run.reads}


def _test_timelines() -> list:
    """test_separation.py's four timelines: (name, timeline, the stage it should end in)."""
    chatter = [(0, 1)] + [(start, start // 8 % 2) for start in range(8, 200, 8)] + [(200, 1)]
    return [('a_7_ms_blip', [(0, 1), (19, 0), (26, 1)]),
            ('chatter_that_never_holds_open', chatter),
            ('a_held_separation', [(0, 1), (5, 0)]),
            ('bounce_then_hold', [(0, 1), (3, 0), (6, 1), (9, 0), (12, 1), (15, 0)])]


def _seeded(generator: random.Random, index: int) -> dict:
    """One seeded scenario: blips, chatter bursts and holds, in a random stage, with real IRQs."""
    timeline, now, level = [(0, 1)], 0, 1
    for _ in range(generator.randint(2, 10)):
        kind = generator.random()
        if kind < 0.4:  # a blip
            now += generator.randint(5, 80)
            timeline += [(now, 1 - level), (now + generator.randint(1, 15), level)]
            now = timeline[-1][0]
        elif kind < 0.6:  # a chatter burst
            for _ in range(generator.randint(3, 12)):
                now += generator.randint(1, 9)
                level = 1 - level
                timeline.append((now, level))
        else:  # a hold
            now += generator.randint(20, 300)
            level = 1 - level
            timeline.append((now, level))
    stage = generator.choice((_STAGE.BOOSTING, _STAGE.BOOSTING, _STAGE.SETTING, _STAGE.GLIDING))
    return _scenario('seeded_%02d' % index, timeline, stage=stage, debounce_ms=generator.choice((10, 20, 20, 30)))


def build() -> dict:
    """
    The separation fixture document.

    Returns:
        {meta, compare, note, scenarios}.
    """
    scenarios = []
    for name, timeline in _test_timelines():
        scenarios.append(_scenario('test_%s' % name, timeline, irq='first'))
        scenarios.append(_scenario('irq_%s' % name, timeline))
    scenarios += [
        _scenario('separated_in_setting_moves_no_stage', [(0, 1), (5, 0)], stage=_STAGE.SETTING),
        _scenario('separated_in_gliding_moves_no_stage', [(0, 1), (5, 0)], stage=_STAGE.GLIDING),
        _scenario('a_re_nest_never_reverses_gliding', [(0, 1), (5, 0), (400, 1)]),
        _scenario('nested_again_from_a_separated_latch', [(0, 0), (10, 1)], separated=True),
        _scenario('the_same_level_records_nothing', [(0, 1), (10, 0), (14, 1)]),
        _scenario('long_chatter_then_open', [(0, 1)] + [(start, start // 4 % 2) for start in range(4, 320, 4)]
                  + [(320, 0)]),
        _scenario('debounce_0_never_settles', [(0, 1), (5, 0)], debounce_ms=0),
        _scenario('debounce_1', [(0, 1), (5, 0)], debounce_ms=1),
        _scenario('debounce_50_rejects_a_40_ms_open', [(0, 1), (5, 0), (45, 1)], debounce_ms=50),
        _scenario('debounce_50_accepts_a_held_open', [(0, 1), (5, 0)], debounce_ms=50),
    ]
    generator = random.Random(_SEED)
    scenarios += [_seeded(generator, index) for index in range(_SEEDED)]
    return {
        'meta': {'source': 'src/glider/drivers/separation.py, src/glider/test/test_separation.py'},
        'compare': {'kind': 'exact'},
        'note': ('per scenario: init (debounce_ms, the latch, the stage, and the timeline and IRQ mode it was made '
                 'from), then a step per pin read plan B made: in {now_ms since the start, high, edge since the '
                 'previous read}; out null, or on the read that concludes a settle {verdict: separated | nested | '
                 'unsettled, event: [event, stage after] as separation.csv records it, or null}; final the latch '
                 'and the stage at the end'),
        'scenarios': scenarios,
    }
