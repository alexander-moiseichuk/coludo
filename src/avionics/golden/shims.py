"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Run the frozen MicroPython plan B (src/glider) on CPython 3.12, so the golden-vector builders can call the
real plan B code instead of a re-implementation of it.

Most of src/glider already imports on the host: board-only modules are guarded (`try: from machine import
Pin` ... `except ImportError: Pin = None`). The shims supply only what is still missing, and give it
MicroPython's semantics where those differ from CPython's:
  * `micropython` -- `const`, and identity `viper`/`native` decorators. gnss.py imports it unguarded.
  * the viper builtin types `ptr8`/`ptr32`/`uint` as `int`. With a real-looking `micropython` module the
    guarded modules (recorder.py, fixed.py) take their board branch, which does not define the dummies
    their CPython branch does, and their viper annotations are evaluated at def time.
  * `machine` with `Pin`, `UART` and `RTC` only. Pin and UART are the names bmi323.py and gnss.py import
    unguarded; constructing either raises, so a builder that reaches hardware fails loudly instead of
    recording a fake. RTC sets the virtual wall clock below, so a mission `update {epoch}` sets the time as
    it does on the board. Every importer of those names now sees the stub instead of its host fallback:
    warmstart.py (`import machine`: the module, which has no reset_cause), recorder.py (UART), mission.py
    (RTC), and the Pin-only drivers led, lsm6dso32, ina226, vl53l4cx, vl53l1x, separation and adxl375 (a
    pin constant or a construction raises where they held None). An importer of any other name (I2C, SPI,
    PWM, Timer, WDT: layout, i2cbus, spibus, sg90, tasks/flight, tasks/watchdog) still gets ImportError
    and keeps its fallback.
  * `esp32` with `mcu_temperature()` only, a fixed figure, so health (cc_client.py, tasks/board_health.py)
    carries a float temperature as on every board. Its other name, NVS, still raises ImportError, so
    warmstart and the NVS-backed drivers (bno055, bmm350, sdp810) keep their host fallback; main.py, the
    one unguarded importer, is never run.
  * a VIRTUAL clock instead of the host's. Every fixture is a function of its builder alone, and no vector
    can straddle the 2**30 tick wrap by accident (the uptime starts 10 s after boot and moves only by
    advance(), time.sleep_ms, asyncio.sleep_ms and an asyncio.wait_for_ms that times out):
      - `time.ticks_ms/ticks_us/ticks_diff` wrap at 2**30 as on the board, and `time.ticks_add` refuses a
        delta outside +-2**29 with OverflowError('ticks interval overflow'), as MicroPython has since 1.20
        (a big integer raises OverflowError there too, with another text);
      - `time.time/gmtime/localtime` on MicroPython's esp32 epoch (2000-01-01, an int, no time zone), and
        the board starts COLD: time() is the uptime in seconds since 2000 until RTC.datetime() sets it,
        like a board nobody has synced. A mission `epoch` is then an int inside 2000, as the hub expects
        before it syncs the clock.
  * `gc.mem_free/mem_alloc` (a fixed, roomy heap) and `asyncio.sleep_ms/wait_for_ms`.
  * `asyncio.ThreadSafeFlag` is NOT shimmed: only Recorder.setup, drivers/separation.py and tasks/flight.py
    construct one, and no builder runs them yet. The builder that first does adds it here.

Importing this module only defines things; install() applies them, process-wide (see its docstring).
"""

import asyncio
import builtins
import calendar
import gc
import os
import sys
import time
import types
from collections.abc import Awaitable, Callable

_GLIDER: str = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'glider'))

_TICKS_PERIOD: int = 1 << 30  # MicroPython's ticks_* wrap here (the small-int range)
_EPOCH_2000: int = 946684800  # Unix seconds at 2000-01-01, the esp32 port's time epoch
_MICROSECONDS_PER_SECOND: int = 1000000

"""
The virtual clock's uptime when a builder starts: a board some seconds after boot, as one is when its first
CC request arrives, and far from the 2**30 us (17.9 min) wrap of ticks_us.
"""
_BOOT_UPTIME_US: int = 10 * _MICROSECONDS_PER_SECOND

"""
A host has no MicroPython heap. A fixed figure keeps every generated fixture deterministic, and a roomy one
(the P4's PSRAM heap is ~8 MB) keeps any low-memory path from tripping by accident.
"""
_HEAP_FREE_BYTES: int = 8 * 1024 * 1024
_HEAP_ALLOCATED_BYTES: int = 0

"""A plausible P4 die temperature, fixed: health reports a float as on a board, and no fixture moves with it."""
_MCU_TEMPERATURE_C: float = 41.5

"""The host modules install() must never share a process with: the hub reads the real clock (server.py)."""
_HOST_MODULES: tuple = ('server', 'web')

_host_gmtime: Callable[..., time.struct_time] = time.gmtime  # the calendar the 2000-epoch gmtime() is built on

_uptime_us: int = _BOOT_UPTIME_US  # the virtual clock: microseconds since boot
_wall_offset_s: int = 0  # the RTC minus the uptime, in seconds since 2000: 0 on a cold board


def advance(milliseconds: int) -> None:
    """
    Let `milliseconds` of board time pass: the one way a builder moves the virtual clock.

    Args:
        milliseconds - how long; never negative (time does not run backwards).

    Returns:
        None; moves the uptime, the ticks and time() alike.

    Raises:
        ValueError - a negative duration.
    """
    global _uptime_us
    if milliseconds < 0:
        raise ValueError('the clock cannot go back %d ms' % -milliseconds)
    _uptime_us += milliseconds * 1000


def _ticks_ms() -> int:
    """MicroPython time.ticks_ms(): the virtual uptime in milliseconds, wrapping at 2**30."""
    return (_uptime_us // 1000) % _TICKS_PERIOD


def _ticks_us() -> int:
    """MicroPython time.ticks_us(): the virtual uptime in microseconds, wrapping at 2**30."""
    return _uptime_us % _TICKS_PERIOD


def _ticks_add(ticks: int, delta: int) -> int:
    """
    MicroPython time.ticks_add(): `ticks` moved by `delta`, wrapped into the tick period.

    Args:
        ticks - a ticks_ms/ticks_us value.
        delta - the signed distance to move it.

    Returns:
        The moved ticks value.

    Raises:
        OverflowError - |delta| reaches half the period, which ticks_diff could not undo (MicroPython 1.20+).
    """
    half = _TICKS_PERIOD // 2
    if not -half < delta < half:
        raise OverflowError('ticks interval overflow')
    return (ticks + delta) % _TICKS_PERIOD


def _ticks_diff(new: int, old: int) -> int:
    """MicroPython time.ticks_diff(): the signed distance new - old, correct across one wrap."""
    half = _TICKS_PERIOD // 2
    return (new - old + half) % _TICKS_PERIOD - half


def _sleep_ms(milliseconds: int) -> None:
    """MicroPython time.sleep_ms(): the board blocks, so the virtual clock moves and nothing else runs."""
    advance(milliseconds)


def _time() -> int:
    """MicroPython time.time() on the esp32 port: whole seconds since 2000-01-01, from the virtual RTC."""
    return _wall_offset_s + _uptime_us // _MICROSECONDS_PER_SECOND


def _gmtime(seconds: int | None = None) -> tuple:
    """
    MicroPython time.gmtime(): the 8-tuple (year, month, mday, hour, minute, second, weekday, yearday).

    Args:
        seconds - seconds since 2000-01-01 (MicroPython's epoch); None for now.

    Returns:
        The broken-down UTC time, laid out as MicroPython returns it.
    """
    if seconds is None:
        seconds = _time()
    return tuple(_host_gmtime(seconds + _EPOCH_2000))[:8]


class _Rtc:
    """machine.RTC: setting it moves the virtual wall clock, while the uptime counts on."""

    def datetime(self, fields: tuple) -> None:
        """
        Set the wall clock, as RTC.datetime(fields) does on the board.

        Args:
            fields - (year, month, day, weekday, hour, minute, second, subseconds), as mission.set_time writes.

        Returns:
            None; time(), gmtime() and localtime() read the set time from now on.
        """
        global _wall_offset_s
        year, month, day, _weekday, hour, minute, second = fields[:7]
        seconds = calendar.timegm((year, month, day, hour, minute, second)) - _EPOCH_2000
        _wall_offset_s = seconds - _uptime_us // _MICROSECONDS_PER_SECOND


def _mem_free() -> int:
    """MicroPython gc.mem_free(): the fixed host figure."""
    return _HEAP_FREE_BYTES


def _mem_alloc() -> int:
    """MicroPython gc.mem_alloc(): the fixed host figure."""
    return _HEAP_ALLOCATED_BYTES


def _mcu_temperature() -> float:
    """esp32.mcu_temperature(): the fixed host figure."""
    return _MCU_TEMPERATURE_C


async def _asyncio_sleep_ms(milliseconds: int) -> None:
    """
    MicroPython asyncio.sleep_ms(): yield for `milliseconds`, then move the virtual clock by as much.

    It also sleeps for real: the event loop keeps its pacing, so a reset plan B defers 200 ms past its reply
    still fires after that reply, not before. Two coroutines sleeping at once each move the clock, so a
    builder that needs the time exact runs one at a time.
    """
    await asyncio.sleep(milliseconds / 1000)
    advance(milliseconds)


async def _asyncio_wait_for_ms(awaitable: Awaitable, timeout_ms: int) -> object:
    """
    MicroPython asyncio.wait_for_ms(): the awaitable's result, or asyncio.TimeoutError past the timeout.

    Args:
        awaitable - what to wait for.
        timeout_ms - how long, in milliseconds.

    Returns:
        The awaitable's result.

    Raises:
        asyncio.TimeoutError - it did not finish in time; the virtual clock has moved by the timeout.
    """
    try:
        return await asyncio.wait_for(awaitable, timeout_ms / 1000)
    except asyncio.TimeoutError:
        advance(timeout_ms)
        raise


def _identity(function: Callable) -> Callable:
    """The viper/native emitter decorators on the host: the function runs as plain Python."""
    return function


class _BoardOnly:
    """A machine peripheral class: importable for its name, but constructing one means a builder reached hardware."""

    def __init__(self, *arguments: object, **keywords: object) -> None:
        raise OSError('board-only: machine.%s has no host stand-in' % type(self).__name__)


def _micropython() -> types.ModuleType:
    """
    The `micropython` module as the plan B sources use it.

    Returns:
        A module with `const` and the `viper`/`native` decorators, all identities.
    """
    module = types.ModuleType('micropython')
    module.const = _identity
    module.viper = _identity
    module.native = _identity
    return module


def _machine() -> types.ModuleType:
    """
    The `machine` module, holding only the names the stubs cover.

    Returns:
        A module with `Pin` and `UART` classes whose construction raises, and `RTC` over the virtual clock.
    """
    module = types.ModuleType('machine')
    module.Pin = type('Pin', (_BoardOnly,), {})
    module.UART = type('UART', (_BoardOnly,), {})
    module.RTC = _Rtc
    return module


def _esp32() -> types.ModuleType:
    """
    The `esp32` module, holding only the die temperature.

    Returns:
        A module with `mcu_temperature()`.
    """
    module = types.ModuleType('esp32')
    module.mcu_temperature = _mcu_temperature
    return module


def install() -> None:
    """
    Make src/glider importable and runnable on this interpreter, with MicroPython semantics.

    Call it once, before importing anything from src/glider. Repeated calls change nothing further.

    It is PROCESS-WIDE and cannot be undone: time.time() then counts from 2000, gmtime()/localtime() return
    MicroPython's 8-tuples (time.strftime and tm_gmtoff fail on them), and the board-only module names
    resolve to stubs. So it must never run in a process that also runs host code: the hub (server.py)
    would date everything 30 years early and send a board that epoch. generate.py's one child per builder
    is the model, and a contract runner's fake board must run plan B in a subprocess the same way. It
    refuses outright when the hub's modules are already loaded.

    Two more settings keep the output a function of the tracked tree alone:
      * `version` is blocked, so config_default stamps firmware_version 'dev' as on a clean checkout. The
        version.py deploy.sh writes is untracked and changes on every deploy, and a fixture that changed
        with it would fail the freshness check after every deploy.
      * no bytecode is written: src/glider is frozen, and the generator only reads it.

    Args:
        (none)

    Returns:
        None; patches sys.path, sys.modules, builtins and the time, gc and asyncio modules in place.

    Raises:
        RuntimeError - a hub module (server, web) is loaded in this process.
    """
    loaded = [name for name in _HOST_MODULES if name in sys.modules]
    if loaded:
        raise RuntimeError('shims.install() would break the host code in this process (%s): run plan B in a '
                           'subprocess' % ', '.join(loaded))
    if _GLIDER not in sys.path:
        sys.path.insert(0, _GLIDER)
    sys.dont_write_bytecode = True
    sys.modules['version'] = None
    sys.modules.setdefault('micropython', _micropython())
    sys.modules.setdefault('machine', _machine())
    sys.modules.setdefault('esp32', _esp32())
    builtins.ptr8 = builtins.ptr32 = builtins.uint = int
    time.ticks_ms = _ticks_ms
    time.ticks_us = _ticks_us
    time.ticks_add = _ticks_add
    time.ticks_diff = _ticks_diff
    time.sleep_ms = _sleep_ms
    time.time = _time
    time.gmtime = _gmtime
    time.localtime = _gmtime  # MicroPython keeps no time zone: local time is UTC
    gc.mem_free = _mem_free
    gc.mem_alloc = _mem_alloc
    asyncio.sleep_ms = _asyncio_sleep_ms
    asyncio.wait_for_ms = _asyncio_wait_for_ms
