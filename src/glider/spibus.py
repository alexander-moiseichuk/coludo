"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Shared, lock-serialized SPI buses, mirroring i2cbus. A sensor may move off the shared I2C bus onto SPI
(e.g. the ADXL375, for clean high-rate reads): each bus id gets ONE machine.SPI plus an asyncio.Lock,
and get() hands back the shared wrapper. device(cs) returns a register window with the SAME
read/read_into/write(reg, ...) interface as i2cbus, so a driver is bus-agnostic. The chip-select is a
plain GPIO held low only around each locked transaction (the SPI peripheral does not own it, so several
devices can share one bus). A glider-only module (MicroPython).
"""

import asyncio

import commons

try:
    from machine import SPI, Pin
except ImportError:  # host (CPython): board-only; buses/devices are constructed only on the board
    SPI = Pin = None

_buses: dict = {}  # bus id -> Bus


class _Device:
    """
    A register window for one chip-select on a shared SPI bus.

    Same interface as i2cbus.Bus.device so a driver works over either bus. The command byte is
    (0x80 if read) | (the multi-byte bit if the transfer spans >1 register) | reg -- the convention of
    the ADXL/LSM family. `mb_bit` is the multi-byte/auto-increment bit position (6 for the ADXL family);
    pass None for chips that auto-increment from a config bit instead of an address bit (e.g. LSM6DSO32
    via CTRL3_C.IF_INC), so the command byte is just (0x80 if read) | reg with no spurious address bit
    set.
    """

    def __init__(self, bus, cs: int, mb_bit: int = 6):
        self._bus = bus
        self._cs = Pin(cs, Pin.OUT, value=1)  # idle high; pulled low only during a transaction
        self._multi = (1 << mb_bit) if mb_bit is not None else 0

    async def read(self, reg: int, count: int) -> bytes:
        buf = bytearray(count)
        await self.read_into(reg, buf)
        return bytes(buf)

    async def read_into(self, reg: int, buf) -> None:
        cmd = 0x80 | reg | (self._multi if len(buf) > 1 else 0)
        async with self._bus._lock:
            self._cs(0)
            self._bus._spi.write(bytes((cmd,)))
            self._bus._spi.readinto(buf)
            self._cs(1)

    async def write(self, reg: int, data: bytes) -> None:
        cmd = reg | (self._multi if len(data) > 1 else 0)
        async with self._bus._lock:
            self._cs(0)
            self._bus._spi.write(bytes((cmd,)) + bytes(data))
            self._cs(1)

    async def diagnose(self, reg: int, expected: int) -> str:
        """
        Read this chip's id/WHO_AM_I register and classify the wire-level result for a failed setup().

        commons.id_classify sorts the read into chip-select not asserting / MISO floating / wrong device
        / present-but-init. A driver's diagnose() just awaits this with its id register + expected value
        -- the read and the verdict live with the bus, not duplicated in every driver.

        Args:
            reg - the id/WHO_AM_I register to read.
            expected - the id value a healthy device returns.

        Returns:
            The commons.id_classify verdict string.
        """
        try:
            read = (await self.read(reg, 1))[0]
        except Exception:
            read = None
        return commons.id_classify(read, expected)


class Bus:
    """One physical SPI bus, shared by every device on it; transactions are serialized by a lock."""

    def __init__(self, bus_id: int, spec: dict):
        self._bus_id: int = bus_id
        self._spec: dict = spec
        mode = spec.get('mode', 3)  # SPI mode; ADXL375 = mode 3 (CPOL=1, CPHA=1)
        self._spi = SPI(bus_id, baudrate=spec.get('baud', 5_000_000), polarity=mode >> 1, phase=mode & 1,
                        sck=Pin(spec['sck']), mosi=Pin(spec['mosi']), miso=Pin(spec['miso']))
        self._lock = asyncio.Lock()

    def device(self, cs: int, mb_bit: int = 6) -> _Device:
        """A register window for one chip-select on this bus (matches i2cbus.Bus.device)."""
        return _Device(self, cs, mb_bit)

    async def retune(self, freq: int) -> None:
        """
        Re-init this SPI peripheral at `freq` Hz in place (bench frequency calibration; no reboot).

        Held under the lock so it never swaps mid-transaction -- the shared device windows keep working,
        they transact through self._spi which now runs at the new baud. Not persisted: the CC-side sweep
        finds the ceiling, then saves the chosen freq to board.config + reboots.

        Args:
            freq - the new SPI baud rate in Hz.

        Returns:
            None; replaces self._spi with a peripheral running at the new baud.
        """
        async with self._lock:
            mode = self._spec.get('mode', 3)
            self._spi = SPI(self._bus_id, baudrate=freq, polarity=mode >> 1, phase=mode & 1,
                            sck=Pin(self._spec['sck']), mosi=Pin(self._spec['mosi']),
                            miso=Pin(self._spec['miso']))


def get(bus_id: int, spec: dict) -> Bus:
    """The shared Bus for `bus_id`, created once from `spec` (sck/mosi/miso/baud/mode) and cached."""
    if bus_id not in _buses:
        _buses[bus_id] = Bus(bus_id, spec)
    return _buses[bus_id]
