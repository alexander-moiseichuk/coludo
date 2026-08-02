"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

LSM6DSO32 6-DoF IMU: the primary raw accel + the sole gyro 'rate'. A +/-32 g accel range (covers the
8-12 g boost without clipping, fine 1 g resolution for the airspeed integrator) plus a +/-2000 dps
gyro. @task.driver('lsm6dso32'). setup() checks WHO_AM_I, configures accel/gyro, and provides both the
'accel' (x,y,z in g) and 'rate' (x,y,z in deg/s) databoard slots; run() writes the latest reading. If
the device is absent (wrong WHO_AM_I) setup() returns False and the Controller skips it.

Wired on SPI1 (its own chip-select, shared with the ADXL375) for clean high-rate reads -- see
doc/waveshare_esp32p4_pins.md. SPI is 4-wire mode 3; multi-byte reads auto-increment via CTRL3_C.IF_INC
(so the bus device takes mb_bit=None -- no address multi-byte bit). I2C (addr 0x6A) also works if the
component sets bus 'i2c'. Sampling is interrupt-driven on INT1 (accel data-ready) when an 'int_pin' is
wired, else a plain period_ms poll, mirroring the ADXL375 driver. Gyro + accel sit in contiguous output
registers (0x22..0x2D), so one 12-byte read fetches both.
"""

import struct

import commons
import databoard
import i2cbus
import recorder
import spibus
import task
from fixed import SCALE, to_float  # gyro rate -> centideg/s fixnum; to_float for the operator view

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const

try:
    from machine import Pin
except ImportError:  # host (CPython): board-only; the INT1 pin is wired only on the board
    Pin = None


_ADDR = const(0x6A)       # default I2C address (SDO low; 0x6B when high)
_WHO_AM_I = const(0x0F)   # reads 0x6C on the LSM6DSO32
_CTRL1_XL = const(0x10)   # accel: ODR + full-scale
_CTRL2_G = const(0x11)    # gyro: ODR + full-scale
_CTRL3_C = const(0x12)    # BDU + IF_INC (auto-increment) + SIM (4-wire SPI)
_INT1_CTRL = const(0x0D)  # INT1 routing (accel data-ready = bit 0)
_OUTX_L_G = const(0x22)   # gyro X..Z then accel X..Z (12 bytes, signed LE, contiguous)
_WHOAMI = const(0x6C)
_DRDY_XL = const(0x01)    # INT1_CTRL: accel data-ready -> INT1
_INT_SILENT_LIMIT = const(3)  # consecutive INT1 timeouts before declaring the line dead
_CFG_XL = const(0x44)     # 104 Hz ODR, FS_XL = 01 = +/-32 g
_CFG_G = const(0x4C)      # 104 Hz ODR, FS_G = 11 = +/-2000 dps
_CFG_C = const(0x44)      # BDU=1, IF_INC=1, SIM=0 (4-wire)
_SCALE_A = 0.000976       # g/LSB at +/-32 g (0.976 mg) -- accel stays float g (feeds sqrt magnitude math)
_MDPS = const(70)         # gyro milli-deg/s per LSB at +/-2000 dps -> rate = raw * _MDPS * SCALE // 1000


@task.driver('lsm6dso32')
class Lsm6dso32(task.Task):
    """6-DoF IMU: samples accel (x,y,z g) -> 'accel' and gyro (x,y,z deg/s) -> 'rate', interrupt-driven."""

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
        # INT-silent fallback cadence. Reads `period_ms`, the key the CONFIG actually supplies --
        # this used to read `period_us`, which appears in no config, so every one of these drivers
        # silently fell back to 500 ms against a 20 ms freshness window. That is the exact
        # "dead wire masquerading as a healthy sensor" the irq_runs work exists to expose.
        self._period_ms: int = max(1, self.config.get('period_ms', 10))
        self._buf = bytearray(12)  # gyro(6) + accel(6)
        self._ready = commons.Waiter()  # IRQ-kicked wake + sliced fallback (see commons.Waiter)
        self._int = None
        self._edge_seen: bool = False  # non-blocking INT1 mark for the polling fallback
        self._int_silent: bool = False  # the INT line is dead -> poll at period_ms instead
        try:
            whoami = 0
            for _ in range(5):  # the first SPI read after bus bring-up can glitch; retry the id check
                whoami = (await self._dev.read(_WHO_AM_I, 1))[0]
                if whoami == _WHOAMI:
                    break
            if whoami != _WHOAMI:
                return False  # not an LSM6DSO32 at this chip-select / address
            await self._dev.write(_CTRL3_C, bytes([_CFG_C]))   # BDU + auto-increment first
            await self._dev.write(_CTRL1_XL, bytes([_CFG_XL]))  # accel +/-32 g @ 104 Hz
            await self._dev.write(_CTRL2_G, bytes([_CFG_G]))    # gyro +/-2000 dps @ 104 Hz
            await self._setup_interrupt()
        except Exception as error:
            print('lsm6dso32 :: %r' % error)
            return False
        self._accel, self._rate = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'accel', 'rate')
        self._irq_runs: int = 0
        self._telemetry = recorder.Telemetry(
            '%s.csv' % self.name, ('ax', 'ay', 'az', 'gx', 'gy', 'gz', 'irq_runs'),
            decimate_us=self.config.get('telemetry_ms', 0) * 1000)  # 0 -> global
        self._ok = True
        return True

    def _transport(self):
        """
        A register window over the bus family the config names, so the rest of the driver is
        bus-agnostic. LSM6DSO32 auto-increments via IF_INC, so the SPI window needs no address multi-byte bit.

        The bus lookup itself lives in i2cbus.bind()/spibus.bind(); this is only the dispatch.

        Args:
            (none) -- reads the component config.

        Returns:
            The register-window device; None when the bus is undefined, or SPI is selected
            with no cs_pin wired.
        """
        if self.config.get('bus', 'spi') == 'spi':
            bus, cs = spibus.bind(self.controller.config, self.config), self._pin_gpio('cs_pin')
            return bus.device(cs, mb_bit=None) if (bus is not None and cs is not None) else None
        bus, addr = i2cbus.bind(self.controller.config, self.config, _ADDR)
        return None if bus is None else bus.device(addr)

    async def _setup_interrupt(self) -> None:
        """
        Route accel data-ready to INT1 if the component declares an int_pin; else stay poll-only.

        Arm the IRQ before the first clearing read so a conversion that landed during config is a clean
        rising edge (the same ordering as the ADXL375 driver).

        Args:
            (none)

        Returns:
            None; wires the INT1 pin + IRQ handler, or leaves the driver poll-only when no int_pin.
        """
        gpio = self._pin_gpio('int_pin')
        if gpio is None:
            return
        await self._dev.write(_INT1_CTRL, bytes([_DRDY_XL]))  # accel data-ready -> INT1
        self._int = Pin(gpio, Pin.IN)
        self._int.irq(self._ready.kick, Pin.IRQ_RISING)
        await self._dev.read_into(_OUTX_L_G, self._buf)  # clear data-ready -> next conversion = clean edge

    async def sample(self) -> tuple:
        """
        Read one accel + gyro sample as a flat 6-tuple.

        A FLAT 6-tuple (run() slices it, no concat): accel (ax, ay, az) in float g, then gyro (gx, gy,
        gz) as fixnum CENTIDEG/S (raw*_MDPS*SCALE//1000, exact -- the PID's rate-damping D term reads
        these). One 12-byte read clears data-ready.

        Args:
            (none)

        Returns:
            (ax, ay, az, gx, gy, gz): accel in float g, gyro in centideg/s fixnum.
        """
        await self._dev.read_into(_OUTX_L_G, self._buf)
        gx, gy, gz, ax, ay, az = struct.unpack('<hhhhhh', self._buf)
        return (ax * _SCALE_A, ay * _SCALE_A, az * _SCALE_A,
                gx * _MDPS * SCALE // 1000, gy * _MDPS * SCALE // 1000, gz * _MDPS * SCALE // 1000)

    async def run(self) -> None:
        """
        The sampling loop: publish the latest accel + gyro to the databoard, forever.

        Sample on INT1 data-ready, then push accel + rate to the databoard and telemetry.

        INT-SILENT FALLBACK. `fallback_ms` was a SAFETY net for an occasional missed edge, but it also
        silently became the sampling rate when the interrupt never arrives at all -- measured on the
        bench at exactly 2.0 Hz (1/500 ms) with the driver still reporting healthy, because it IS
        sampling, just 50x too slowly. `rate` has a 20 ms freshness window and NO backup provider, so
        at 2 Hz it is stale 96 % of the time: `read()` hands the flight loop source=None and the PID's
        D term quietly degrades to derivative-on-error. A dead wire must not masquerade as a healthy IMU.

        So after `_INT_SILENT_LIMIT` consecutive timeouts the loop DROPS TO POLLING at period_ms (which
        tracks the sensor's ODR, not a safety timeout) and says so once through note(). A single late
        edge resumes interrupt mode, so a noisy line costs nothing permanent.

        Args:
            (none)

        Returns:
            None; runs forever (a wedged board reboots rather than exits).
        """
        while True:
            """
            No branch, and no degradation dance. wait() covers both cases -- a live edge returns on
            the first slice, a dead one runs the period out and we sample anyway -- so the driver no
            longer has to decide which mode it is in, nor switch between them. `interrupt_silent`
            survives as the OPERATOR view, derived from what the wait actually returned rather than
            from a separate counter that could disagree with it: irq_runs 0 is a timed-out fallback,
            and _INT_SILENT_LIMIT of them in a row means the line is dead rather than jittery.
            """
            self._irq_runs = await self._ready.wait(self._period_ms)
            if self.strike(self._irq_runs == 0, _INT_SILENT_LIMIT):
                self._int_silent = True
                self.note('lsm6dso32 :: INT1 silent -- sampling on the %d ms fallback',
                          self._period_ms)
            elif self._irq_runs:
                self._int_silent = False  # an edge arrived -> interrupt-driven again
            try:
                sample = await self.sample()  # flat 6-tuple
                self._accel.push(sample[:3])
                self._rate.push(sample[3:])  # (roll, pitch, yaw) rate in centideg/s fixnum -> PID D term
                # accel float g; gyro is centideg/s fixnum -> to_str for a human-readable, float-free column
                # irq_runs, per row: 0 = no interrupt (the fallback timed out -- a dead or quiet
                # line), 1 = healthy, >1 = the loop was late and edges piled up while it was elsewhere.
                # That third case is a SCHEDULING symptom, not a sensor one, and it is invisible
                # without recording it.
                # gyro columns are the RAW centideg/s fixnum, not a formatted decimal. to_str() built
                # three strings per sample and measured 224 B -- the single largest piece of this
                # driver's sample, and it was FORMATTING, not measurement. The host tools divide by
                # fixed.SCALE when they render; the board has no business spending heap on decimals.
                self._telemetry.push((sample[0], sample[1], sample[2],
                                      sample[3], sample[4], sample[5], self._irq_runs))
                self.note(None)  # healthy pass -> let the next error log afresh
            except Exception as error:
                self.note('lsm6dso32 :: read %r', error)  # deduped: a persistent error logs once

    async def probe(self) -> str:
        """
        On-demand self-test: WHO_AM_I reads back, then one sample succeeds (each step logged).

        Args:
            (none)

        Returns:
            None when both steps pass; a short failure message (the failing step) otherwise.
        """
        try:
            recorder.Recorder.log(self.name, 'probe: who_am_i ...')
            whoami = (await self._dev.read(_WHO_AM_I, 1))[0]
            if whoami != _WHOAMI:
                raise ValueError('LSM6DSO32 id 0x%02x != 0x%02x on %s:%s' % (
                    whoami, _WHOAMI, self.config.get('bus'), self.config.get('id')))
            recorder.Recorder.log(self.name, 'probe: who_am_i ok 0x%02x' % whoami)
        except Exception as error:
            message = 'who_am_i: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        try:
            recorder.Recorder.log(self.name, 'probe: sample ...')
            ax, ay, az, gx, gy, gz = await self.sample()
            recorder.Recorder.log(self.name, 'probe: sample ok %.2fg (%.1f,%.1f,%.1f) dps' % (
                (ax * ax + ay * ay + az * az) ** 0.5, to_float(gx), to_float(gy), to_float(gz)))
        except Exception as error:
            message = 'sample: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    async def diagnose(self) -> str:
        """
        Deeper analysis when setup() failed: classify the wire-level fault behind an absent IMU.

        The bus reads WHO_AM_I and classifies the fault (CS dead / MISO floating / wrong device /
        present-but-init). The Controller folds it into the failure reason so 'verify'/'probe' show the
        'why', not just 'absent / miswired?'. A None transport means setup never built it -- a config
        fault (bus undefined / cs_pin unwired).

        Args:
            (none)

        Returns:
            A one-line fault classification for the failure reason.
        """
        if self._dev is None:  # setup never built the transport
            return 'no transport -- bus %s:%s undefined or cs_pin %s unwired' % (
                self.config.get('bus'), self.config.get('id'), self.config.get('cs_pin'))
        return await self._dev.diagnose(_WHO_AM_I, _WHOAMI)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['interrupt'] = self._int is not None
        # a wired-but-DEAD INT1 is the dangerous state: sampling continues, `rate` goes stale,
        # and the PID D term degrades -- so the operator sees the degradation, not just 'ok'
        status['interrupt_silent'] = self._int_silent
        status['accel_g'] = self._accel.value()
        status['rate_dps'] = self._rate.value()
        return status
