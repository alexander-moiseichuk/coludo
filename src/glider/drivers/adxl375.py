# drivers/adxl375.py — ADXL375 ±200 g high-G accelerometer: the boost-phase accel channel. Works over
# I2C (shared bus) OR SPI (its own bus, for clean high-rate reads) -- the component's `bus` field
# selects, and a shared register-window device (i2cbus/spibus .device()) keeps the driver code
# bus-agnostic. @task.driver('adxl375'). setup() probes the device id and configures it; run() writes
# the latest (x, y, z) acceleration in g to the databoard 'accel' slot. If the device is absent (no
# ack / wrong device id) setup() returns False and the Controller skips it -- the board boots fine
# with the sensor unplugged.
#
# Sampling is interrupt-driven when an `int_pin` (INT1) is wired: the chip raises DATA_READY when a
# new sample is ready, an IRQ sets a ThreadSafeFlag, and run() awaits it -- so the coroutine sleeps
# until there is genuinely fresh data instead of blind-polling. A `fallback_ms` timeout still forces
# a sample if interrupts go silent (dead sensor / wiring). With no int_pin it falls back to a plain
# `period_ms` poll. Uses the shared locked I2C bus (i2cbus), as it shares i2c:0 with other sensors.

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

    async def setup(self) -> bool:
        kind = self.config.get('bus', 'i2c')
        bus_id = self.config.get('id', 0)
        spec = config.bus(self.controller.config, kind, bus_id)
        if spec is None:
            return False
        self._dev = self._transport(kind, bus_id, spec)  # register window: I2C addr or SPI chip-select
        if self._dev is None:
            return False  # SPI selected but no cs_pin wired
        self._period_ms: int = self.config.get('period_ms', 100)  # poll interval with no INT wired
        self._fallback_ms: int = self.config.get('fallback_ms', 500)  # safety sample if INT silent
        self._buf = bytearray(6)
        self._ready = asyncio.ThreadSafeFlag()
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
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('ax', 'ay', 'az'),
                                       decimate_us=self.config.get('telemetry_us', 100000))  # default 10 Hz
        self._ok = True
        return True

    def _transport(self, kind: str, bus_id: int, spec: dict):
        """A register window over I2C (by address) or SPI (by chip-select), so the rest of the driver
        is bus-agnostic. SPI needs a `cs_pin` in the component; returns None if it is missing."""
        if kind == 'spi':
            cs = self.controller.config.get('pins', {}).get(self.config.get('cs_pin'))
            return spibus.get(bus_id, spec).device(cs) if cs is not None else None
        return i2cbus.get(bus_id, spec).device(self.config.get('addr', 0x53))

    async def _setup_interrupt(self) -> None:
        """Wire INT1 -> DATA_READY if the component declares an int_pin; else stay poll-only."""
        gpio = self.controller.config.get('pins', {}).get(self.config.get('int_pin'))
        if gpio is None:
            return
        from machine import Pin

        await self._dev.write(_REG_INT_MAP, b'\x00')  # DATA_READY -> INT1
        await self._dev.write(_REG_INT_ENABLE, bytes([_DATA_READY]))
        await self._dev.read_into(_REG_DATAX0, self._buf)  # clear the pending DATA_READY
        self._int = Pin(gpio, Pin.IN)  # so the next conversion gives a clean rising edge promptly
        self._int.irq(self._on_data_ready, Pin.IRQ_RISING)

    def _on_data_ready(self, pin) -> None:
        """IRQ: a fresh sample is ready -- wake run(). ThreadSafeFlag.set() is interrupt-safe."""
        self._ready.set()

    async def sample(self) -> tuple:
        """Read and return (x, y, z) acceleration in g (also clears DATA_READY)."""
        await self._dev.read_into(_REG_DATAX0, self._buf)
        x, y, z = struct.unpack('<hhh', self._buf)
        return (x * _SCALE_G, y * _SCALE_G, z * _SCALE_G)

    async def run(self) -> None:
        """Sample on DATA_READY (or every fallback_ms if interrupts go silent); plain poll with no
        INT wired. Either way, write the latest acceleration to the databoard."""
        while True:
            if self._int is not None:
                try:
                    await asyncio.wait_for_ms(self._ready.wait(), self._fallback_ms)
                except asyncio.TimeoutError:
                    pass  # no interrupt within the window -> sample anyway (safety)
            else:
                await asyncio.sleep_ms(self._period_ms)
            try:
                accel = await self.sample()
                self._accel.push(accel)  # one step: push our channel directly
                self._telemetry.push(accel)
            except Exception as error:
                print('adxl375 :: read %r' % error)

    async def probe(self) -> str:
        """On-demand self-test: the device id reads back, then one sample succeeds (each step logged)."""
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

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['interrupt'] = self._int is not None
        status['accel_g'] = self._accel.value()  # our channel's latest (no hot-path I2C here)
        return status
