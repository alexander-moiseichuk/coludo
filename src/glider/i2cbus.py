"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Shared, lock-serialized I2C buses. Several sensor drivers sit on one physical bus (i2c:0 carries the
ADXL375, BNO055 and BMP280), so they must not interleave transactions on the single peripheral: each
bus id has ONE machine.I2C plus an asyncio.Lock, and get() hands back the shared wrapper. The
read/write methods are async (they acquire the lock) but the underlying I2C op is fast and
synchronous, so the lock is held only for the transaction. A glider-only module.
"""

import asyncio

import commons

try:
    from machine import I2C, Pin
except ImportError:  # host (CPython): board-only; the bus is constructed only on the board
    I2C = Pin = None

_buses: dict = {}  # bus id -> Bus


class _Device:
    """
    A register window on a shared I2C bus for one address.

    Mirrors spibus.Bus.device so a driver can use either bus the same way:
    read(reg, n) / read_into(reg, buf) / write(reg, data).
    """

    def __init__(self, bus, addr: int):
        self._bus = bus
        self._addr = addr

    async def read(self, reg: int, count: int, addrsize: int = 8) -> bytes:
        return await self._bus.read(self._addr, reg, count, addrsize=addrsize)

    async def read_into(self, reg: int, buf) -> None:
        await self._bus.read_into(self._addr, reg, buf)

    async def write(self, reg: int, data: bytes) -> None:
        await self._bus.write(self._addr, reg, data)

    async def diagnose(self, reg: int, expected: int, addrsize: int = 8) -> str:
        """
        Read this chip's id/WHO_AM_I register and classify the wire-level result for a failed setup().

        commons.id_classify sorts the read into no I2C ack / wrong device / present-but-init. A driver's
        diagnose() just awaits this with its id register + expected value -- the read and verdict live
        with the bus, mirroring spibus._Device.diagnose.

        Args:
            reg - the id/WHO_AM_I register to read.
            expected - the id value a healthy device returns.
            addrsize - register address width in bits, passed through for 16-bit register devices
                (e.g. VL53L4CX); default is 8.

        Returns:
            The commons.id_classify verdict string (no ack / wrong device / present-but-init).
        """
        try:
            read = (await self.read(reg, 1, addrsize=addrsize))[0]
        except Exception:
            read = None
        return commons.id_classify(read, expected)


class Bus:
    """
    One physical I2C bus, shared by every device on it.

    NO PER-OPERATION LOCK, deliberately. Every operation below is ONE synchronous machine.I2C call
    with no `await` inside it, and MicroPython's asyncio is cooperative and single-threaded: a section
    that never yields cannot be interrupted, so the event loop already serialises them. The lock that
    used to wrap each call therefore protected nothing -- and it was not free: MEASURED on the board,
    `async with self._lock` cost **288 B of the 320 B** an `await bus.read_into()` allocated, against
    0 B for the underlying `readfrom_mem_into`. With eight drivers sampling at 10-50 Hz that was the
    single largest GC-off allocator on the board, far above telemetry.

    It also never gave what a lock is usually for: it was released BETWEEN calls, so a multi-step
    sequence (icp10111's measure-command, sleep, read) already interleaves with other drivers today
    and works, because an I2C device holds its own state per address. Anything that genuinely needs a
    sequence to be atomic across awaits should say so explicitly with `async with bus.transaction():`
    -- which is honest about the cost, rather than paying it on every single-shot read.
    """

    def __init__(self, bus_id: int, spec: dict):
        self._bus_id: int = bus_id
        self._spec: dict = spec
        self._i2c = I2C(bus_id, scl=Pin(spec['scl']), sda=Pin(spec['sda']), freq=spec.get('freq', 400000))
        self._lock = asyncio.Lock()  # NOT per-operation: only transaction() and retune() take it

    def transaction(self):
        """
        Hold the bus across a MULTI-STEP sequence that must not interleave (the explicit escape hatch).

        Single operations do not need this (see the class docstring). Use it only when a driver must
        await between steps and another driver touching the bus in that gap would corrupt the
        exchange -- and expect to pay ~288 B for the privilege.

        Returns:
            The bus lock, as an async context manager.
        """
        return self._lock

    async def retune(self, freq: int) -> None:
        """
        Re-init this I2C peripheral at `freq` Hz in place (bench frequency calibration; no reboot).

        Held under the lock so it never swaps mid-transaction -- the shared device windows keep working.
        Not persisted: the CC-side sweep finds the ceiling, then saves the chosen freq to board.config +
        reboots.

        Args:
            freq - the new I2C clock frequency in Hz.

        Returns:
            None; replaces self._i2c with a peripheral running at the new frequency.
        """
        async with self._lock:
            self._i2c = I2C(self._bus_id, scl=Pin(self._spec['scl']), sda=Pin(self._spec['sda']), freq=freq)

    async def read(self, addr: int, reg: int, count: int, addrsize: int = 8) -> bytes:
        return self._i2c.readfrom_mem(addr, reg, count, addrsize=addrsize)

    async def read_chip_id(self, addr: int, reg: int, addrsize: int = 8) -> int:
        """
        Read a device's one-byte identity register (WHO_AM_I / CHIP_ID).

        Several drivers confirm the right chip answers at an address by reading a single id byte in
        both probe and diagnose; this is that read, named so the intent is obvious at the call site.

        Args:
            addr - the device's I2C address.
            reg - the identity register.
            addrsize - the register-address width in bits (8 or 16).

        Returns:
            The identity byte.
        """
        return (await self.read(addr, reg, 1, addrsize))[0]

    async def read_into(self, addr: int, reg: int, buf, addrsize: int = 8) -> None:
        self._i2c.readfrom_mem_into(addr, reg, buf, addrsize=addrsize)

    async def write(self, addr: int, reg: int, data: bytes, addrsize: int = 8) -> None:
        self._i2c.writeto_mem(addr, reg, data, addrsize=addrsize)

    async def writeto(self, addr: int, data: bytes) -> None:
        """Raw write (no register) -- for command-based devices like the ICP-10111."""
        self._i2c.writeto(addr, data)

    async def readfrom(self, addr: int, count: int) -> bytes:
        """Raw read (no register) -- pairs with writeto() for command-based devices."""
        return self._i2c.readfrom(addr, count)

    def device(self, addr: int) -> _Device:
        """A register window for one address on this bus (matches spibus.Bus.device)."""
        return _Device(self, addr)

    def scan(self) -> list:
        return self._i2c.scan()


def get(bus_id: int, spec: dict) -> Bus:
    """The shared Bus for `bus_id`, created once from `spec` (scl/sda/freq) and cached thereafter."""
    if bus_id not in _buses:
        _buses[bus_id] = Bus(bus_id, spec)
    return _buses[bus_id]
