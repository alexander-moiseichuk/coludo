# tasks/fusion.py — sensor fusion: turn the blackboard's raw per-provider readings into one selected
# value per quantity (specs/coludo.md). @task.activity('fusion'). Each sensor's config declares what
# it `provides` with a priority (lower = preferred) and a timeout_ms (how long a reading stays
# fresh); fusion groups the providers by quantity and, every cycle, publishes the highest-priority
# provider whose latest reading is still fresh, falling back to the next when the preferred one goes
# stale (e.g. ICP-10111 altitude preferred, BMP280 as backup). A pure read-from-blackboard task, so
# it needs no sensor dependency wait -- absent sensors simply never produce fresh data and are
# skipped. Inspectable: `selected` shows which source currently wins each quantity.

import asyncio
import time

import blackboard
import recorder
import task


@task.activity('fusion')
class Fusion(task.Task):
    """Pick the live value per quantity from its providers by priority + freshness."""

    async def setup(self) -> bool:
        self._period_ms: int = self.config.get('period_ms', 20)
        cfg = self.controller.config
        # quantity -> [(priority, timeout_us, source), ...] sorted by priority (preferred first)
        self._map: dict = {}
        for dev in cfg.get('sensors', []) + cfg.get('components', []):
            if not dev.get('enabled', True):
                continue
            for quantity, spec in (dev.get('provides') or {}).items():
                timeout_us = spec.get('timeout_ms', 1000) * 1000
                self._map.setdefault(quantity, []).append((spec.get('priority', 0), timeout_us, dev.get('name')))
        for quantity in self._map:
            self._map[quantity].sort()
            blackboard.Blackboard.declare(quantity)
        self._selected: dict = {}  # quantity -> the source currently chosen (None if all stale)
        # one wide fused.csv row per cycle: a column per quantity (vectors are pipe-joined), so the
        # stream's decimate_us decimates the whole row together.
        self._fields: tuple = tuple(sorted(self._map.keys()))
        self._telemetry = recorder.Telemetry('fused.csv', self._fields, decimate_us=self.config.get('telemetry_us', 0))
        self._ok = True
        return True

    @staticmethod
    def _cell(value) -> str:
        """Format a fused value for one CSV cell: '' if absent, pipe-joined if a vector, else %g."""
        if value is None:
            return ''
        if isinstance(value, (tuple, list)):
            return '|'.join('%g' % v for v in value)
        return '%g' % value

    def fuse_once(self) -> None:
        """One arbitration pass: for each quantity publish the preferred provider that is fresh."""
        now = time.ticks_us()
        for quantity, candidates in self._map.items():
            chosen = None
            for _priority, timeout_us, source in candidates:
                slot = blackboard.Blackboard.raw(quantity, source)
                if slot is not None and slot.value is not None and time.ticks_diff(now, slot.timestamp) <= timeout_us:
                    blackboard.Blackboard.publish(quantity, slot.value, source)
                    chosen = source
                    break  # first fresh provider in priority order wins
            self._selected[quantity] = chosen

    async def run(self) -> None:
        while True:
            self.fuse_once()
            self._telemetry.push([self._cell(blackboard.Blackboard.value(q)) for q in self._fields])
            await asyncio.sleep_ms(self._period_ms)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['selected'] = dict(self._selected)
        return status

    def stats(self) -> dict:
        return {
            'selected': dict(self._selected),
            'providers': {quantity: [c[2] for c in cands] for quantity, cands in self._map.items()},
        }
