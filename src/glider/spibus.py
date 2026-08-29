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
import config

try:
    from machine import SPI, Pin
except ImportError:  # host (CPython): board-only; buses/devices are constructed only on the board
    SPI = Pin = None

_buses: dict = {}  # bus id -> Bus


_RESYNC_READS: int = 16  # discarded framed reads after a peripheral appears; measured need is 1..11


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
        self._synced: bool = False  # see _resync: the FIRST transaction on a new peripheral is discarded

    def _resync(self) -> None:
        """
        Discard one framed read the first time this window is used on a given peripheral.

        Measured on the board: after an SPI peripheral is created (or replaced by retune) the first
        framed reads return 0x00 and every read after them is correct. The count VARIES -- 1 read in the
        steady case, 6 to 11 observed on a freshly created peripheral -- which is why this discards
        _RESYNC_READS of them rather than the single read a first fix assumed. It is reproducible at
        every baud from 1 to 8 MHz, so it is not signal integrity, and unaffected by delays from 0 to
        50 ms, so it is not settling time. The device is simply out of step with a peripheral that
        appeared underneath it.

        Left unhandled this is a quiet correctness bug, not a noisy one: adxl375.setup() checks its DEVID
        with a SINGLE read and no retry, so a discarded-read-shaped fault at bring-up reads as a chip
        that is absent or wrong -- exactly the verdict that sent this project chasing a dead LSM6DSO32
        through two netlist errors.

        One bool test per transaction pays for it, which is cheaper than the alternative of every caller
        knowing to throw its first read away.
        """
        self._synced = True
        scratch = bytearray(1)
        try:
            for _ in range(_RESYNC_READS):
                self._cs(0)
                self._bus._spi.write(b'\x80')   # read register 0: side-effect-free on both parts here
                self._bus._spi.readinto(scratch)
                self._cs(1)
        except Exception:
            pass  # an absent device stays absent; this is a resync, never a probe

    async def read(self, reg: int, count: int) -> bytes:
        buf = bytearray(count)
        await self.read_into(reg, buf)
        return bytes(buf)

    async def read_into(self, reg: int, buf) -> None:
        """
        One CS-framed register read into a caller buffer.

        NOT locked, for the reason i2cbus.Bus documents: the CS-low .. CS-high section contains no
        `await`, and MicroPython's asyncio is cooperative, so nothing can run between asserting and
        releasing CS -- a second device on this bus cannot steal it mid-transaction. That is the
        property that matters here (LSM6DSO32 and ADXL375 share SPI1), and it comes from the
        scheduler, not from the lock. The lock cost ~288 B per call, measured.
        """
        if not self._synced:
            self._resync()
        cmd = 0x80 | reg | (self._multi if len(buf) > 1 else 0)
        self._cs(0)
        self._bus._spi.write(bytes((cmd,)))
        self._bus._spi.readinto(buf)
        self._cs(1)

    async def write(self, reg: int, data: bytes) -> None:
        """One CS-framed register write; unlocked for the same reason as read_into above."""
        if not self._synced:
            self._resync()
        cmd = reg | (self._multi if len(data) > 1 else 0)
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
        self._devices: list = []  # every window handed out, so retune() can resync them

    def device(self, cs: int, mb_bit: int = 6) -> _Device:
        """A register window for one chip-select on this bus (matches i2cbus.Bus.device)."""
        window = _Device(self, cs, mb_bit)
        self._devices.append(window)  # remembered so retune() can resync each one; see there
        return window

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
            """
            Resync every device: replacing the peripheral leaves some parts one transaction out of step.

            Measured on the LSM6DSO32, deterministically -- after a retune its FIRST framed read returns
            0x00 and the second is correct, on every trial. The ADXL375 alongside it on the same bus is
            unaffected, which is why this cannot be left to the caller: the symptom is one device
            reporting dead at whatever frequency was just set.

            That matters because the only caller is the bustune frequency sweep, which retunes and then
            immediately health-checks. Without this, the sweep would blame the LSM6DSO32 at EVERY step of
            the ladder and pick a needlessly low SPI speed -- a wrong answer from the tool whose entire
            job is choosing that number.

            One throwaway read of register 0 per device; both parts treat it as side-effect-free. The
            cost lands here, in a bench-only operation, rather than as a per-transaction check on the
            100 Hz IMU path.
            """
            for window in self._devices:
                window._synced = False  # re-arm the one-read discard; _resync runs on next use


def get(bus_id: int, spec: dict) -> Bus:
    """The shared Bus for `bus_id`, created once from `spec` (sck/mosi/miso/baud/mode) and cached."""
    if bus_id not in _buses:
        _buses[bus_id] = Bus(bus_id, spec)
    return _buses[bus_id]


def bind(board: dict, device: dict):
    """
    Resolve a device's config block to the SPI bus it talks over -- i2cbus.bind's twin.

    The two dual-bus drivers (adxl375, lsm6dso32) pick their family at runtime, so each carried its own
    copy of the same `config.bus()` preamble to resolve EITHER family. With both modules exposing
    bind(), that preamble is gone from both and each driver's transport helper is just the dispatch.

    Returns the BUS ALONE, not i2cbus.bind's (bus, addr) pair, and the difference is real rather than an
    oversight: an I2C address is a plain integer sitting in the device block, but a chip-select is a
    board PIN, resolved through the pin map by Task._pin_gpio. Pin mapping is the task's job, so the
    caller passes the resolved GPIO to bus.device() itself.

    Args:
        board - the whole board config (holds the `buses` section).
        device - the component's own config block ('bus', 'id').

    Returns:
        The shared Bus, or None when the config declares no such bus. Default id is 1: spi:1 is the
        only SPI bus the board declares (i2cbus.bind defaults to 0 for the same reason).
    """
    bus_id = device.get('id', 1)
    spec = config.bus(board, device.get('bus', 'spi'), bus_id)
    return None if spec is None else get(bus_id, spec)
