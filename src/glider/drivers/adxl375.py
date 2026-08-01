"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

ADXL375 ±200 g high-G accelerometer: the boost-phase accel channel. Works over I2C (shared bus) OR SPI
(its own bus, for clean high-rate reads) -- the component's `bus` field selects, and a shared
register-window device (i2cbus/spibus .device()) keeps the driver code bus-agnostic.
@task.driver('adxl375'). setup() probes the device id and configures it; run() writes the latest
(x, y, z) acceleration in g to the databoard 'accel' slot. If the device is absent (no ack / wrong
device id) setup() returns False and the Controller skips it -- the board boots fine with the sensor
unplugged.

Sampling is interrupt-driven when an `int_pin` (INT1) is wired: the chip raises DATA_READY when a new
sample is ready, an IRQ sets a plain bool, and run() waits on it in slices -- so the coroutine sleeps until
there is genuinely fresh data instead of blind-polling. A `fallback_ms` timeout still forces a sample if
interrupts go silent (dead sensor / wiring). With no int_pin it falls back to a plain `period_ms` poll.
Uses the shared locked I2C bus (i2cbus), as it shares i2c:0 with other sensors.
"""

import struct

import commons
import databoard
import i2cbus
import recorder
import spibus
import task

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const

try:
    from machine import Pin
except ImportError:  # host (CPython): board-only; the INT1 pin is wired only on the board
    Pin = None


_ADDR = const(0x53)  # default I2C address (SDO low; 0x1D when high)
_REG_DEVID = const(0x00)  # reads 0xE5 on the whole ADXL34x/375 family
_REG_BW_RATE = const(0x2C)  # output data rate
_REG_POWER_CTL = const(0x2D)  # measure bit = 0x08
_REG_INT_ENABLE = const(0x2E)  # DATA_READY = bit 7
_REG_INT_MAP = const(0x2F)  # 0 -> INT1, 1 -> INT2 (per interrupt)
_REG_DATA_FORMAT = const(0x31)
_REG_DATAX0 = const(0x32)  # X0,X1,Y0,Y1,Z0,Z1 -- 6 bytes, signed LE
_DEVID = const(0xE5)
_DATA_READY = const(0x80)  # INT_ENABLE / INT_MAP bit for DATA_READY
_SCALE_G = 0.049  # ADXL375 ≈ 49 mg/LSB (full-resolution, fixed ±200 g)


@task.driver('adxl375')
class Adxl375(task.Task):
    """High-G accel: samples (x, y, z) in g to the databoard 'accel' slot, interrupt-driven."""

    _dev = None  # class default: no transport until setup() builds it (diagnose reads directly)

    async def setup(self) -> bool:
        self._dev = self._transport()  # register window over whichever bus the config names
        if self._dev is None:
            return False  # no such bus in config, or SPI selected with no cs_pin wired
        """
        ONE knob. `period_us` is both the sample period and the interrupt fallback, because
        commons.Waiter made them the same thing: a live edge returns on the first slice, a dead one
        runs the slices out and the sample is taken anyway. Two constants that had to be kept
        consistent became one that cannot disagree with itself.
        """
        self._period_us: int = self.config.get('period_us', 500000)
        self._period_ms: int = max(1, self._period_us // 1000)  # the wait's unit, resolved once
        self._buf = bytearray(6)
        self._ready = commons.Waiter()  # IRQ-kicked wake + sliced fallback (see commons.Waiter)
        self._int = None
        try:
            if (await self._dev.read(_REG_DEVID, 1))[0] != _DEVID:
                return False  # not an ADXL375 at this address / chip-select
            await self._dev.write(_REG_DATA_FORMAT, b'\x0b')  # full-res, 4-wire SPI, INT active-high
            await self._dev.write(_REG_BW_RATE, b'\x0a')  # 100 Hz ODR
            await self._dev.write(_REG_POWER_CTL, b'\x08')  # measure mode
            await self._setup_interrupt()
        except Exception as error:
            print('adxl375 :: %r' % error)
            return False
        self._accel = databoard.Databoard.provide(self.name, self.config.get('provides', {}), 'accel')
        self._irq_runs: int = 0
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('ax', 'ay', 'az', 'irq_runs'),
                                       decimate_us=self.config.get('telemetry_us', 0))  # 0 -> Recorder global rate
        # peak-hold across the decimation window (see run()): a LIST, not a tuple, so the hot loop
        # updates in place instead of rebuilding a tuple per sample with GC off
        self._peak: list = [0.0, 0.0, 0.0]
        self._ok = True
        return True

    def _transport(self):
        """
        A register window over the bus family the config names, so the rest of the driver is
        bus-agnostic. I2C addresses by device address; SPI by chip-select.

        The bus lookup itself lives in i2cbus.bind()/spibus.bind(); this is only the dispatch.

        Args:
            (none) -- reads the component config.

        Returns:
            The register-window device; None when the bus is undefined, or SPI is selected
            with no cs_pin wired.
        """
        if self.config.get('bus', 'i2c') == 'spi':
            bus, cs = spibus.bind(self.controller.config, self.config), self._pin_gpio('cs_pin')
            return bus.device(cs) if (bus is not None and cs is not None) else None
        bus, addr = i2cbus.bind(self.controller.config, self.config, _ADDR)
        return None if bus is None else bus.device(addr)

    async def _setup_interrupt(self) -> None:
        """Wire INT1 -> DATA_READY if the component declares an int_pin; else stay poll-only."""
        gpio = self._pin_gpio('int_pin')
        if gpio is None:
            return
        await self._dev.write(_REG_INT_MAP, b'\x00')  # DATA_READY -> INT1
        await self._dev.write(_REG_INT_ENABLE, bytes([_DATA_READY]))
        """
        Arm the IRQ BEFORE clearing the pending DATA_READY: if a conversion landed
        during the writes above, INT1 is already high and stays high (DATA_READY is level, not a
        pulse) -- clearing FIRST then arming would miss that edge and, since the line is static-high,
        the RISING IRQ would never fire until the fallback sample. Armed first, the clear-read drops
        INT1 low, so the next conversion is a clean rising edge the IRQ catches.
        """
        self._int = Pin(gpio, Pin.IN)
        self._int.irq(self._ready.kick, Pin.IRQ_RISING)
        await self._dev.read_into(_REG_DATAX0, self._buf)  # clear -> INT1 low -> next conversion = clean edge

    async def sample(self) -> tuple:
        """
        Read one acceleration sample from the device.

        The register read also clears DATA_READY, so it re-arms the interrupt for the next conversion.

        Args:
            (none)

        Returns:
            (x, y, z) acceleration in g.
        """
        await self._dev.read_into(_REG_DATAX0, self._buf)
        x, y, z = struct.unpack('<hhh', self._buf)
        return (x * _SCALE_G, y * _SCALE_G, z * _SCALE_G)

    async def run(self) -> None:
        """
        Sample on DATA_READY (or every fallback_ms if interrupts go silent); plain poll with no INT wired.

        Either way, write the latest acceleration to the databoard.

        Args:
            (none)

        Returns:
            None; loops forever, pushing each sample to the databoard 'accel' slot and telemetry.
        """
        while True:
            # no branch on whether an interrupt exists: wait() covers both. With one wired it
            # returns on the first slice; with none it simply runs the period out and we sample.
            self._irq_runs = await self._ready.wait(self._period_ms)
            try:
                accel = await self.sample()
                self._accel.push(accel)  # one step: push our channel directly
                """
                PEAK-HOLD between telemetry rows, rather than letting the decimator pick one sample.

                This is a +/-200 g SHOCK sensor: it exists for separation, ignition and impact
                transients, which are exactly the events that live between rows. Telemetry decimation
                DROPS samples (recorder.Telemetry.push returns early inside the window), so at the
                global 25 Hz a 100 Hz stream records one sample in four and silently discards the other
                three -- including the peak, most of the time. A capture would show a plausible
                acceleration trace with the shock simply missing, and nothing would say so.

                So hold the largest-magnitude sample per axis across the window and emit THAT. Sign is
                preserved (the extreme excursion, not its absolute value) because impact direction is
                what distinguishes a nose-in from a tail-slide. Peaks are compared per axis rather than
                on the vector magnitude: an axis-aligned shock is the common case, and a magnitude
                comparison would let a large steady axis mask a spike on a quiet one.

                due() is asked first, against the SAME clock push() uses, so the row tuple is not built
                on the 75 % of iterations that would be decimated away.
                """
                self._fold_peak(accel)
                if self._telemetry.due(recorder.Recorder.timestamp()):
                    self._telemetry.push((self._peak[0], self._peak[1], self._peak[2], self._irq_runs))
                    self._peak[0] = self._peak[1] = self._peak[2] = 0.0
                self.note(None)  # healthy pass -> let the next error log afresh
            except Exception as error:
                self.note('adxl375 :: read %r', error)  # deduped: a persistent error logs once, not every tick

    def _fold_peak(self, accel) -> None:
        """
        Fold one sample into the per-axis peak held across the decimation window.

        Keeps the largest-MAGNITUDE value per axis while preserving its sign, so a capture reports the
        extreme excursion rather than its absolute value -- impact direction is what separates a
        nose-in from a tail-slide. Per axis rather than by vector magnitude so a large steady axis
        cannot mask a spike on a quiet one.

        Args:
            accel - the (ax, ay, az) sample in g.

        Returns:
            None; updates self._peak in place.
        """
        for axis in range(3):
            if abs(accel[axis]) > abs(self._peak[axis]):
                self._peak[axis] = accel[axis]

    async def probe(self) -> str:
        """
        On-demand self-test: the device id reads back, then one sample succeeds (each step logged).

        Args:
            (none)

        Returns:
            None on success; a short failure message (also logged) at the first failing step.
        """
        try:
            recorder.Recorder.log(self.name, 'probe: device id ...')
            devid = (await self._dev.read(_REG_DEVID, 1))[0]
            if devid != _DEVID:
                raise ValueError('ADXL375 id 0x%02x != 0x%02x on %s:%s' % (
                    devid, _DEVID, self.config.get('bus'), self.config.get('id')))
            recorder.Recorder.log(self.name, 'probe: device id ok 0x%02x' % devid)
        except Exception as error:
            message = 'device id: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        try:
            recorder.Recorder.log(self.name, 'probe: sample ...')
            ax, ay, az = await self.sample()
            recorder.Recorder.log(self.name, 'probe: sample ok (%.2f,%.2f,%.2f)g' % (ax, ay, az))
        except Exception as error:
            message = 'sample: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    async def diagnose(self) -> str:
        """
        Deeper analysis when setup() failed: classify the wire-level fault.

        The bus reads our DEVID and classifies the fault (CS dead / MISO floating / wrong device /
        present-but-init), so the Controller can fold it into the failure reason and `verify`/`probe`
        show the 'why', not just 'absent / miswired?'.

        Args:
            (none)

        Returns:
            A wire-level fault description; a config-fault message when setup never built the transport
            (bus undefined / cs_pin unwired).
        """
        if self._dev is None:  # setup never built the transport
            return 'no transport -- bus %s:%s undefined or cs_pin %s unwired' % (
                self.config.get('bus'), self.config.get('id'), self.config.get('cs_pin'))
        return await self._dev.diagnose(_REG_DEVID, _DEVID)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status.update({'interrupt': self._int is not None,
                       'accel_g': self._accel.value()})  # our channel's latest (no hot-path I2C here)
        return status
