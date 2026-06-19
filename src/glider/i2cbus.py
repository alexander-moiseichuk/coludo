# i2cbus.py — shared, lock-serialized I2C buses. Several sensor drivers sit on one physical bus
# (i2c:0 carries the ADXL375, BNO055 and BMP280), so they must not interleave transactions on the
# single peripheral: each bus id has ONE machine.I2C plus an asyncio.Lock, and get() hands back the
# shared wrapper. The read/write methods are async (they acquire the lock) but the underlying I2C op
# is fast and synchronous, so the lock is held only for the transaction. A glider-only module.

import asyncio

_buses: dict = {}  # bus id -> Bus


class Bus:
    """One physical I2C bus, shared by every device on it; transactions are serialized by a lock."""

    def __init__(self, bus_id: int, spec: dict):
        from machine import I2C, Pin

        self._i2c = I2C(bus_id, scl=Pin(spec['scl']), sda=Pin(spec['sda']), freq=spec.get('freq', 400000))
        self._lock = asyncio.Lock()

    async def read(self, addr: int, reg: int, count: int) -> bytes:
        async with self._lock:
            return self._i2c.readfrom_mem(addr, reg, count)

    async def read_into(self, addr: int, reg: int, buf) -> None:
        async with self._lock:
            self._i2c.readfrom_mem_into(addr, reg, buf)

    async def write(self, addr: int, reg: int, data: bytes) -> None:
        async with self._lock:
            self._i2c.writeto_mem(addr, reg, data)

    def scan(self) -> list:
        return self._i2c.scan()


def get(bus_id: int, spec: dict) -> Bus:
    """The shared Bus for `bus_id`, created once from `spec` (scl/sda/freq) and cached thereafter."""
    if bus_id not in _buses:
        _buses[bus_id] = Bus(bus_id, spec)
    return _buses[bus_id]
