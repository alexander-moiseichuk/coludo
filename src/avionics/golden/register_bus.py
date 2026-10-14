"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

What the IMU and barometer builders (build_drivers_lsm6dso32.py, build_drivers_bmi323.py, build_drivers_icp10111.py,
build_drivers_bmp581.py, build_drivers_bmm350.py) add to driver_bus.py, without changing it:
  * Bus -- driver_bus.Bus plus the two register reads those drivers use and the pitot/power/laser drivers do not:
    read_into() (readfrom_mem_into, a caller's buffer filled) and read_chip_id() (a one-byte register read). Both
    record the same `write_read` step as driver_bus's read(), so the transcript format is unchanged;
  * run_for() -- run a plan B run loop, which never returns, until the virtual clock reaches an end, recording its
    waits into the transcript with the time each step happened at;
  * Stop -- what ends it: a BaseException, so the run loops' `except Exception` cannot swallow it.

Imported by the builders, which generate.py runs with the shims installed.
"""

import asyncio

import driver_bus
import shims


class Stop(BaseException):
    """The end of a run_for() window: raised out of a wait, past every `except Exception` in a plan B run loop."""


class Bus(driver_bus.Bus):
    """driver_bus.Bus with read_into() and read_chip_id(); each step also carries the virtual time it happened at."""

    def __init__(self, part) -> None:
        """
        A bus with one scripted part on it.

        Args:
            part - the part's answer function (see driver_bus).
        """
        super().__init__(part)
        self.clock_ms: int = 0  # the virtual time since the bus was made, moved by run_for()'s waits

    def _call(self, step: dict, op: str, address: int, register, count: int, data: bytes):
        """driver_bus's transaction, with the virtual time stamped on the step (run_for() strips it when unwanted)."""
        step['at_ms'] = self.clock_ms
        return super()._call(step, op, address, register, count, data)

    async def read_into(self, addr: int, reg: int, buf, addrsize: int = 8) -> None:
        """readfrom_mem_into: the register address, a repeated start, len(buf) bytes into the caller's buffer."""
        buf[:] = await self.read(addr, reg, len(buf), addrsize)

    async def read_chip_id(self, addr: int, reg: int, addrsize: int = 8) -> int:
        """i2cbus.Bus.read_chip_id: a one-byte register read."""
        return (await self.read(addr, reg, 1, addrsize))[0]


def timeless(transcript: list) -> list:
    """The transcript without the `at_ms` stamps: what the Rust replay scripts (driver_bus's format)."""
    return [{key: value for key, value in step.items() if key != 'at_ms'} for step in transcript]


def run(bus: Bus, coroutine):
    """
    Run one plan B coroutine to its end over `bus`, its waits recorded, and return its answer with the transcript
    in driver_bus's format (no time stamps).

    Args:
        bus - the bus whose transcript records the waits.
        coroutine - the coroutine (setup(), _read(), rearm(), ...).

    Returns:
        What the coroutine returned.
    """
    answer = driver_bus.run(bus, coroutine)
    bus.transcript[:] = timeless(bus.transcript)
    return answer


def run_for(bus: Bus, coroutine, end_ms: int) -> None:
    """
    Run a plan B coroutine that never returns (a run loop) until the virtual clock reaches `end_ms`.

    Each asyncio.sleep_ms records a {"op": "delay_ms"} step stamped with the time it began, moves the bus clock and
    the shims' clock, and ends the run with Stop once the clock has reached `end_ms`. Transactions are stamped by
    the bus (Bus._call), so every step carries the virtual time it happened at: the run loop's TIMELINE.

    Args:
        bus - the bus the loop talks over.
        coroutine - the run loop.
        end_ms - when to stop, ms of virtual time from the bus's creation.

    Returns:
        None; the transcript is in `bus.transcript`.
    """
    original = asyncio.sleep_ms

    async def sleep_ms(milliseconds: int) -> None:
        bus.transcript.append({'op': 'delay_ms', 'ms': milliseconds, 'at_ms': bus.clock_ms})
        bus.clock_ms += milliseconds
        shims.advance(milliseconds)
        if bus.clock_ms >= end_ms:
            raise Stop()
        await asyncio.sleep(0)

    asyncio.sleep_ms = sleep_ms
    try:
        asyncio.run(coroutine)
    except Stop:
        pass
    finally:
        asyncio.sleep_ms = original
