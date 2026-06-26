# drivers/lsm6dso32.py — LSM6DSO32 6-DoF IMU: the primary raw accel + the sole gyro `rate`. ±32 g accel
# (covers the 8-12 g boost without clipping, fine 1 g resolution for the airspeed integrator) + ±2000 dps
# gyro. @task.driver('lsm6dso32'). setup() checks WHO_AM_I, configures accel/gyro, and provides both the
# 'accel' (x,y,z in g) and 'rate' (x,y,z in deg/s) databoard slots; run() writes the latest reading. If
# the device is absent (wrong WHO_AM_I) setup() returns False and the Controller skips it.
#
# Wired on SPI1 (its own chip-select, shared with the ADXL375) for clean high-rate reads — see
# doc/waveshare_esp32p4_pins.md. SPI is 4-wire mode 3; multi-byte reads auto-increment via CTRL3_C.IF_INC
# (so the bus device takes mb_bit=None — no address multi-byte bit). I2C (addr 0x6A) also works if the
# component sets bus 'i2c'. Sampling is interrupt-driven on INT1 (accel data-ready) when an `int_pin` is
# wired, else a plain period_ms poll, mirroring the ADXL375 driver. Gyro + accel sit in contiguous output
# registers (0x22..0x2D), so one 12-byte read fetches both.

import asyncio
import struct

import config
import databoard
import i2cbus
import recorder
import spibus
import task

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)

    def const(value):
        return value


_WHO_AM_I = const(0x0F)   # reads 0x6C on the LSM6DSO32
_CTRL1_XL = const(0x10)   # accel: ODR + full-scale
_CTRL2_G = const(0x11)    # gyro: ODR + full-scale
_CTRL3_C = const(0x12)    # BDU + IF_INC (auto-increment) + SIM (4-wire SPI)
_INT1_CTRL = const(0x0D)  # INT1 routing (accel data-ready = bit 0)
_OUTX_L_G = const(0x22)   # gyro X..Z then accel X..Z (12 bytes, signed LE, contiguous)
_WHOAMI = const(0x6C)
_DRDY_XL = const(0x01)    # INT1_CTRL: accel data-ready -> INT1
_CFG_XL = const(0x44)     # 104 Hz ODR, FS_XL = 01 = +/-32 g
_CFG_G = const(0x4C)      # 104 Hz ODR, FS_G = 11 = +/-2000 dps
_CFG_C = const(0x44)      # BDU=1, IF_INC=1, SIM=0 (4-wire)
_SCALE_A = 0.000976       # g/LSB at +/-32 g (0.976 mg)
_SCALE_G = 0.070          # deg/s per LSB at +/-2000 dps (70 mdps)


@task.driver('lsm6dso32')
class Lsm6dso32(task.Task):
    """6-DoF IMU: samples accel (x,y,z g) -> 'accel' and gyro (x,y,z deg/s) -> 'rate', interrupt-driven."""

    async def setup(self) -> bool:
        kind = self.config.get('bus', 'spi')
        bus_id = self.config.get('id', 1)
        spec = config.bus(self.controller.config, kind, bus_id)
        if spec is None:
            return False
        self._dev = self._transport(kind, bus_id, spec)  # register window: SPI chip-select or I2C addr
        if self._dev is None:
            return False  # SPI selected but no cs_pin wired
        self._period_ms: int = self.config.get('period_ms', 100)  # poll interval with no INT wired
        self._fallback_ms: int = self.config.get('fallback_ms', 500)  # safety sample if INT silent
        self._buf = bytearray(12)  # gyro(6) + accel(6)
        self._ready = asyncio.ThreadSafeFlag()
        self._int = None
        try:
            if (await self._dev.read(_WHO_AM_I, 1))[0] != _WHOAMI:
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
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('ax', 'ay', 'az', 'gx', 'gy', 'gz'),
                                             decimate_us=self.config.get('telemetry_us', 100000))  # 10 Hz
        self._ok = True
        return True

    def _transport(self, kind: str, bus_id: int, spec: dict):
        """A register window over SPI (by chip-select) or I2C (by address). LSM6DSO32 auto-increments via
        IF_INC, so the SPI device uses mb_bit=None (no address multi-byte bit)."""
        if kind == 'spi':
            cs = self.controller.config.get('pins', {}).get(self.config.get('cs_pin'))
            return spibus.get(bus_id, spec).device(cs, mb_bit=None) if cs is not None else None
        return i2cbus.get(bus_id, spec).device(self.config.get('addr', 0x6A))

    async def _setup_interrupt(self) -> None:
        """Route accel data-ready to INT1 if the component declares an int_pin; else stay poll-only.
        Arm the IRQ before the first clearing read so a conversion that landed during config is a clean
        rising edge (the same ordering as the ADXL375 driver)."""
        gpio = self.controller.config.get('pins', {}).get(self.config.get('int_pin'))
        if gpio is None:
            return
        from machine import Pin

        await self._dev.write(_INT1_CTRL, bytes([_DRDY_XL]))  # accel data-ready -> INT1
        self._int = Pin(gpio, Pin.IN)
        self._int.irq(self._on_data_ready, Pin.IRQ_RISING)
        await self._dev.read_into(_OUTX_L_G, self._buf)  # clear data-ready -> next conversion = clean edge

    def _on_data_ready(self, pin) -> None:
        """IRQ: a fresh sample is ready -- wake run(). ThreadSafeFlag.set() is interrupt-safe."""
        self._ready.set()

    async def sample(self) -> tuple:
        """Read and return ((ax, ay, az) g, (gx, gy, gz) deg/s); one 12-byte read clears data-ready."""
        await self._dev.read_into(_OUTX_L_G, self._buf)
        gx, gy, gz, ax, ay, az = struct.unpack('<hhhhhh', self._buf)
        return ((ax * _SCALE_A, ay * _SCALE_A, az * _SCALE_A),
                (gx * _SCALE_G, gy * _SCALE_G, gz * _SCALE_G))

    async def run(self) -> None:
        """Sample on INT1 data-ready (or every fallback_ms if interrupts go silent; plain poll with no
        INT wired) and write the latest accel + gyro to the databoard."""
        while True:
            if self._int is not None:
                try:
                    await asyncio.wait_for_ms(self._ready.wait(), self._fallback_ms)
                except asyncio.TimeoutError:
                    pass  # no interrupt within the window -> sample anyway (safety)
            else:
                await asyncio.sleep_ms(self._period_ms)
            try:
                accel, rate = await self.sample()
                self._accel.push(accel)
                self._rate.push(rate)
                self._telemetry.push(accel + rate)
                self.note(None)  # healthy pass -> let the next error log afresh
            except Exception as error:
                self.note('lsm6dso32 :: read %r' % error)  # deduped: a persistent error logs once

    async def probe(self) -> str:
        """On-demand self-test: WHO_AM_I reads back, then one sample succeeds (each step logged)."""
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
            (ax, ay, az), (gx, gy, gz) = await self.sample()
            recorder.Recorder.log(self.name, 'probe: sample ok %.2fg (%.0f,%.0f,%.0f) dps' % (
                (ax * ax + ay * ay + az * az) ** 0.5, gx, gy, gz))
        except Exception as error:
            message = 'sample: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['interrupt'] = self._int is not None
        status['accel_g'] = self._accel.value()
        status['rate_dps'] = self._rate.value()
        return status
