"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

BMI323 6-axis IMU (on the SEN0697) over the shared I2C bus: accel + gyro to the databoard.

Provides the SAME channels as the LSM6DSO32 -- `accel` (float g) and `rate` (centideg/s fixnum) -- so the
two are interchangeable sources for one channel and `tasks/attitude.py` runs off whichever the databoard
hands it. That is the point of fitting this part: with the BNO055's on-chip fusion gone, BOTH attitude
paths become the same complementary filter over two independent 6-axis sensors, instead of a black box
plus a backup.

Two BMI323 details the register map does not make obvious:

  * every I2C register read returns TWO DUMMY BYTES before the data, so a 6-word burst is a 14-byte read
    and the payload starts at offset 2. Reading it like an LSM6DSO32 returns plausible nonsense.
  * registers are 16-bit and written LSB first.

Scaling is exact integer where it feeds the control path: +/-16 g is 1/2048 g per LSB, and +/-2000 dps in
centideg/s is raw * 3125 // 512 (= raw * 2000 * 100 / 32768, exactly). Register values are Bosch's own
(bmi3_defs.h). @task.driver('bmi323').

MOUNTING: the axis-to-airframe mapping (gx->roll, gy->pitch, gz->yaw) is the convention attitude.py and
the PID D term assume. It is a property of how the board is glued in, not of the part -- field
calibration flips a sign here exactly as it does for the mixer gains.
"""

import asyncio
import struct

import commons
import databoard
import i2cbus
import recorder
import task
from fixed import SCALE  # gyro rate -> centideg/s fixnum, the one control scale
from machine import Pin

try:
    from micropython import const
except ImportError:
    from commons import const

_ADDR = const(0x69)  # default I2C address (0x68 when the address pin is pulled the other way)
_REG_CHIP_ID = const(0x00)
_REG_ACC_DATA = const(0x03)  # accel x/y/z then gyro x/y/z -- 6 contiguous 16-bit words
_REG_ACC_CONF = const(0x20)
_REG_GYR_CONF = const(0x21)
_REG_CMD = const(0x7E)
_REG_IO_INT_CTRL = const(0x38)
_REG_INT_MAP2 = const(0x3B)  # accel/gyro data-ready mapping lives here
_INT1_PUSH_PULL_HIGH = const(0x0005)  # INT1 output enabled (bit 2) | active high (bit 0), push-pull
_MAP_ACC_DRDY_INT1 = const(0x0400)  # acc_drdy -> INT1: selection 1 << ACC_DRDY_POS(10)
_INT_SILENT_LIMIT = const(20)  # consecutive timed-out waits before calling the line dead
_CHIP_ID = const(0x43)
_CMD_SOFT_RESET = const(0xDEAF)
_CFG_ACC = const(0x7038)  # mode high-performance | +/-16 g | 100 Hz
_CFG_GYR = const(0x7048)  # mode high-performance | +/-2000 dps | 100 Hz
_DUMMY = const(2)  # bytes the part prepends to every I2C register read
_FRAME = const(14)  # _DUMMY + 6 x int16
_INVALID = const(-32768)  # 0x8000: "no sample yet" while the gyro starts up

_SCALE_ACCEL = 1.0 / 2048.0  # g/LSB at +/-16 g -- accel stays float g (feeds sqrt magnitude math)

# Gyro raw -> centideg/s fixnum, exactly: full scale is +/-2000 dps over 16 bits, so the conversion is
# raw * 2000 * SCALE / 32768 and every term is integral. Derived from SCALE rather than folded into a
# literal (it reduces to raw * 3125 // 512 today) so the control scale cannot change underneath it.
_RATE_NUMERATOR: int = 2000 * SCALE
_RATE_DIVISOR: int = 32768


@task.driver('bmi323')
class Bmi323(task.Task):
    """Accel + gyro to the databoard: `accel` in float g, `rate` in centideg/s fixnum."""

    _bus = None  # class default: no transport until setup() builds it (diagnose reads directly)

    async def setup(self) -> bool:
        self._bus, self._addr = i2cbus.bind(self.controller.config, self.config, _ADDR)
        if self._bus is None:
            return False  # no such bus in config -> the Controller skips this device
        self._period_ms: int = self.config.get('period_ms', 10)  # 100 Hz, matching the configured ODR
        self._buf = bytearray(_FRAME)
        self._ready = commons.Waiter()  # IRQ-kicked wake + sliced fallback (see commons.Waiter)
        self._int = None
        self._irq_runs: int = 0
        self._int_silent: bool = False  # the INT line is dead -> poll at period_ms instead
        try:
            if await self._chip_id() != _CHIP_ID:
                return False  # not a BMI323 at this address
            await self._configure()
            if not await self._await_gyro():
                print('bmi323 :: gyro never left its start-up state')
                return False
            await self._setup_interrupt()
        except Exception as error:
            print('bmi323 :: %r' % error)
            return False
        self._accel, self._rate = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'accel', 'rate')
        self._telemetry = recorder.Telemetry(
            '%s.csv' % self.name, ('ax', 'ay', 'az', 'gx', 'gy', 'gz', 'irq_runs'),
            decimate_us=self.config.get('telemetry_ms', 0) * 1000)  # 0 -> global
        self._ok = True
        return True

    async def _write16(self, register: int, value: int) -> None:
        """BMI323 registers are 16-bit, written LSB first."""
        await self._bus.write(self._addr, register, bytes((value & 0xFF, value >> 8)))

    async def _chip_id(self) -> int:
        """
        The chip id, read PAST the two dummy bytes this part prepends to every I2C register read.

        The shared `read_chip_id()` helper reads one byte at the register, which for this part is a dummy
        -- so it never matches and setup() rejects a perfectly good sensor. That is exactly what happened
        on the first hardware bring-up, while the BMP581 beside it (no dummy bytes) came up fine.
        """
        buf = bytearray(_DUMMY + 1)
        await self._bus.read_into(self._addr, _REG_CHIP_ID, buf)
        return buf[_DUMMY]

    async def _configure(self) -> None:
        """Soft reset, then accel and gyro to high-performance mode at their flight ranges."""
        await self._write16(_REG_CMD, _CMD_SOFT_RESET)
        await asyncio.sleep_ms(5)
        await self._write16(_REG_ACC_CONF, _CFG_ACC)
        await self._write16(_REG_GYR_CONF, _CFG_GYR)

    async def _setup_interrupt(self) -> None:
        """
        Route accel data-ready to INT1 when the component declares an int_pin; else stay poll-only.

        Interrupt-driven sampling costs far less CPU than polling at the ODR: the loop sleeps until the
        part says a conversion landed instead of waking 100 times a second to ask. The wire is optional
        BY DESIGN -- a board without it, or with a dead line, falls back to the period poll on its own
        (see run()), so this is an optimisation that cannot become a dependency.

        Arm the IRQ before the first clearing read, so a conversion that landed during config produces a
        clean rising edge rather than a missed one -- the same ordering the LSM6DSO32 driver uses.

        Args:
            (none)

        Returns:
            None; wires the INT1 pin + handler, or leaves the driver poll-only when no int_pin.
        """
        name = self.config.get('int_pin')
        gpio = self.controller.config.get('pins', {}).get(name) if name else None
        if gpio is None:
            return
        await self._write16(_REG_IO_INT_CTRL, _INT1_PUSH_PULL_HIGH)
        await self._write16(_REG_INT_MAP2, _MAP_ACC_DRDY_INT1)
        self._int = Pin(gpio, Pin.IN)
        self._int.irq(self._ready.kick, Pin.IRQ_RISING)
        await self._bus.read_into(self._addr, _REG_ACC_DATA, self._buf)  # clear -> next edge is clean

    async def _await_gyro(self) -> bool:
        """
        Wait out the gyro's start-up, which reports 0x8000 until it has a sample.

        Measured at ~20 ms on the bench. Waiting here rather than publishing the marker keeps a value that
        scales to -2000 dps out of the `rate` channel the PID's D term reads -- a start-up artefact
        arriving as a full-scale rotation is the kind of thing a control loop acts on before anyone sees it.

        Args:
            (none)

        Returns:
            True once real samples arrive; False if they never do.
        """
        for _ in range(50):
            await asyncio.sleep_ms(10)
            await self._bus.read_into(self._addr, _REG_ACC_DATA, self._buf)
            _ax, _ay, _az, gx, gy, gz = struct.unpack_from('<6h', self._buf, _DUMMY)
            if _INVALID not in (gx, gy, gz):
                return True
        return False

    async def _read(self) -> tuple:
        """
        Read one accel + gyro sample as a flat 6-tuple.

        Accel (ax, ay, az) in float g, then gyro (gx, gy, gz) as fixnum CENTIDEG/S -- the same contract
        the LSM6DSO32 publishes, so either part can feed the same channel. Accel comes FIRST out of this
        part (the LSM6DSO32 puts gyro first), and the payload starts past the two dummy bytes.

        Args:
            (none)

        Returns:
            (ax, ay, az, gx, gy, gz): accel in float g, gyro in centideg/s fixnum.
        """
        await self._bus.read_into(self._addr, _REG_ACC_DATA, self._buf)
        ax, ay, az, gx, gy, gz = struct.unpack_from('<6h', self._buf, _DUMMY)
        return (ax * _SCALE_ACCEL, ay * _SCALE_ACCEL, az * _SCALE_ACCEL,
                gx * _RATE_NUMERATOR // _RATE_DIVISOR,
                gy * _RATE_NUMERATOR // _RATE_DIVISOR,
                gz * _RATE_NUMERATOR // _RATE_DIVISOR)

    async def rearm(self) -> None:
        """
        Re-apply the configuration after something reset the part underneath it.

        The ICP-10111's recovery sends an I2C GENERAL-CALL reset that every device honouring it obeys; a
        BMI323 that took one powers up with both sensors DISABLED and would publish nothing. The baro
        drivers carry the same method for the same reason -- the fault belongs to the bus, not the part.

        Args:
            (none)

        Returns:
            None; best-effort -- a part that is still gone fails its next read as before.
        """
        try:
            await self._configure()
            await self._await_gyro()
        except Exception as error:
            recorder.Recorder.log(self.name, 'rearm failed: %r' % error)

    async def run(self) -> None:
        while True:
            """
            One wait covers both modes. A live edge returns on the first slice; a dead line runs the
            period out and we sample anyway -- so the driver never has to decide which mode it is in.
            `irq_runs` is the operator's view of which actually happened: 0 is a timed-out fallback,
            1 is healthy, >1 means the loop was late and edges piled up (a scheduling symptom, not a
            sensor one, and invisible unless recorded).
            """
            self._irq_runs = await self._ready.wait(self._period_ms)
            # A board with NO int_pin is polling BY DESIGN, so there is no line to have gone silent:
            # warning about one every boot trains the operator to read past a message that does mean
            # something on a board where the wire exists. `interrupt_silent` reports the same way.
            if self._int is not None and self.strike(self._irq_runs == 0, _INT_SILENT_LIMIT):
                self._int_silent = True
                self.note('bmi323 :: INT1 silent -- sampling on the %d ms fallback', self._period_ms)
            elif self._irq_runs:
                self._int_silent = False  # an edge arrived -> interrupt-driven again
            try:
                sample = await self._read()
                self._accel.push(sample[:3])
                self._rate.push(sample[3:])  # (roll, pitch, yaw) rate in centideg/s fixnum -> PID D term
                self._telemetry.push(sample + (self._irq_runs,))
                self.note(None)  # healthy pass -> let the next error log afresh
            except Exception as error:
                self.note('bmi323 :: read %r', error)  # deduped: a persistent error logs once, not every tick

    async def probe(self) -> str:
        """
        On-demand self-test: the chip id reads back, then one sample reads (each step logged).

        Args:
            (none)

        Returns:
            None on success; a short failure message (also logged) at the first failing step.
        """
        try:
            recorder.Recorder.log(self.name, 'probe: chip id ...')
            chip = await self._chip_id()
            if chip != _CHIP_ID:
                raise ValueError('BMI323 id 0x%02x != 0x%02x at i2c:%s 0x%02x' % (
                    chip, _CHIP_ID, self.config.get('id'), self._addr))
            recorder.Recorder.log(self.name, 'probe: chip id ok 0x%02x' % chip)
        except Exception as error:
            message = 'chip id: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        try:
            recorder.Recorder.log(self.name, 'probe: read ...')
            sample = await self._read()
            magnitude = (sample[0] ** 2 + sample[1] ** 2 + sample[2] ** 2) ** 0.5
            # gravity is the one reference available on a bench: a part reading far from 1 g at rest is
            # mis-scaled or mis-configured, which a bare "it answered" probe would pass
            recorder.Recorder.log(self.name, 'probe: read ok |a| %.3f g' % magnitude)
        except Exception as error:
            message = 'read: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    async def diagnose(self) -> str:
        """
        Deeper analysis when setup() failed: classify the wire-level fault.

        Args:
            (none)

        Returns:
            A wire-level fault description; a config-fault message when setup never built the bus.
        """
        bus = self._bus  # None until setup builds the transport
        if bus is None:  # setup never built the bus -> a config fault
            return 'no transport -- i2c bus %s undefined in config' % self.config.get('id', 0)
        # NOT bus.device().diagnose(): that helper reads one byte at the register, which on this part
        # is a dummy byte, so it would report a healthy sensor as the wrong device.
        try:
            chip = await self._chip_id()
        except Exception as error:
            return 'no ack at i2c:%s 0x%02x -- %r' % (self.config.get('id'), self._addr, error)
        if chip != _CHIP_ID:
            return 'wrong device at i2c:%s 0x%02x: id 0x%02x, expected 0x%02x' % (
                self.config.get('id'), self._addr, chip, _CHIP_ID)
        return 'present and answering, but setup did not complete'

    def inspect(self) -> dict:
        status = task.Task.inspect(self)  # our channels' latest (no hot-path I2C here)
        status.update({'addr': hex(self._addr),
                       'accel_g': self._accel.value(),
                       'rate_cds': self._rate.value(),
                       # None = no int_pin declared (polled BY DESIGN); False = INT1 driving;
                       # True = a pin IS declared and went silent, which is the case worth seeing.
                       # Reporting True for an unwired board made "not connected" and "connected but
                       # dead" read identically -- and the second is a fault while the first is a
                       # configuration.
                       'interrupt_silent': self._int_silent if self._int is not None else None,
                       'irq_runs': self._irq_runs})
        return status
