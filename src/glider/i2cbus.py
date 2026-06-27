# i2cbus.py — shared, lock-serialized I2C buses. Several sensor drivers sit on one physical bus
# (i2c:0 carries the ADXL375, BNO055 and BMP280), so they must not interleave transactions on the
# single peripheral: each bus id has ONE machine.I2C plus an asyncio.Lock, and get() hands back the
# shared wrapper. The read/write methods are async (they acquire the lock) but the underlying I2C op
# is fast and synchronous, so the lock is held only for the transaction. A glider-only module.

import asyncio

import commons

_buses: dict = {}  # bus id -> Bus


class _Device:
    """A register window on a shared I2C bus for one address, mirroring spibus.Bus.device so a driver
    can use either bus the same way: read(reg, n) / read_into(reg, buf) / write(reg, data)."""

    def __init__(self, bus, addr: int):
        self._bus = bus
        self._addr = addr

    async def read(self, reg: int, count: int) -> bytes:
        return await self._bus.read(self._addr, reg, count)

    async def read_into(self, reg: int, buf) -> None:
        await self._bus.read_into(self._addr, reg, buf)

    async def write(self, reg: int, data: bytes) -> None:
        await self._bus.write(self._addr, reg, data)

    async def diagnose(self, reg: int, expected: int) -> str:
        """Read this chip's id/WHO_AM_I register and classify the wire-level result for a failed setup()
        (commons.id_classify: no I2C ack / wrong device / present-but-init). A driver's diagnose() just
        awaits this with its id register + expected value -- the read and verdict live with the bus,
        mirroring spibus._Device.diagnose."""
        try:
            read = (await self.read(reg, 1))[0]
        except Exception:
            read = None
        return commons.id_classify(read, expected)


class Bus:
    """One physical I2C bus, shared by every device on it; transactions are serialized by a lock."""

    def __init__(self, bus_id: int, spec: dict):
        from machine import I2C, Pin

        self._i2c = I2C(bus_id, scl=Pin(spec['scl']), sda=Pin(spec['sda']), freq=spec.get('freq', 400000))
        self._lock = asyncio.Lock()

    async def read(self, addr: int, reg: int, count: int, addrsize: int = 8) -> bytes:
        async with self._lock:
            return self._i2c.readfrom_mem(addr, reg, count, addrsize=addrsize)

    async def read_into(self, addr: int, reg: int, buf, addrsize: int = 8) -> None:
        async with self._lock:
            self._i2c.readfrom_mem_into(addr, reg, buf, addrsize=addrsize)

    async def write(self, addr: int, reg: int, data: bytes, addrsize: int = 8) -> None:
        async with self._lock:
            self._i2c.writeto_mem(addr, reg, data, addrsize=addrsize)

    async def writeto(self, addr: int, data: bytes) -> None:
        """Raw write (no register) — for command-based devices like the ICP-10111."""
        async with self._lock:
            self._i2c.writeto(addr, data)

    async def readfrom(self, addr: int, count: int) -> bytes:
        """Raw read (no register) — pairs with writeto() for command-based devices."""
        async with self._lock:
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
