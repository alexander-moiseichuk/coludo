# drivers/vl53l4cx.py — VL53L4CX time-of-flight laser ranger (Adafruit 5425) over the shared I2C bus:
# the above-ground-level (AGL) channel for the last metres of the glide, where the barometer is
# useless. @task.driver('vl53l4cx'). The VL53 family uses 16-BIT register addresses (i2cbus addrsize=
# 16) and continuous-ranging mode: setup() optionally pulses XSHUT to reset, waits for the firmware to
# boot, writes the default configuration block and starts ranging; run() waits for data-ready (the
# GPIO1 interrupt if wired, else a poll), reads the distance and writes AGL (m) to the databoard.
#
# The init + ranging follow the VL53L1X Ultra-Lite-Driver register protocol, which the L4CX is
# register-compatible with for single-target distance (its multi-target / long-range extras need the
# full ST ULD upload and are not used here). Graceful: no I2C ack -> setup False -> Controller skips
# it. Shares i2c:0 with the other sensors via the locked i2cbus. Tuned/validated on the bench.

import asyncio
import struct

import config
import databoard
import i2cbus
import recorder
import task

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)

    def const(value):
        return value


_REG_FIRMWARE_STATUS = const(0x00E5)  # bit0 = 1 once the firmware has booted
_REG_MODEL_ID = const(0x010F)  # 2 bytes: identification (0xEACC on VL53L1X; non-zero when present)
_REG_CONFIG_START = const(0x002D)  # the default-configuration block is written from here
_REG_GPIO_HV_STATUS = const(0x0031)  # data-ready poll (bit0 vs the interrupt polarity)
_REG_SYSTEM_INTERRUPT_CLEAR = const(0x0086)
_REG_SYSTEM_MODE_START = const(0x0087)  # 0x40 = start continuous ranging, 0x00 = stop
_REG_RANGE_STATUS = const(0x0089)  # device range status (low 5 bits)
_REG_DISTANCE = const(0x0096)  # 2 bytes, final crosstalk-corrected range in mm
_BOOT_TIMEOUT_MS = const(100)
_VALID_STATUS = (9, 11)  # raw range-status codes the ULD maps to a valid measurement

# VL53L1X default configuration, registers 0x2D..0x86 (90 bytes) — the canonical ULD block; ranging
# is started separately by writing MODE_START. Interrupt is configured for new-sample-ready (0x46).
_DEFAULT_CONFIG = bytes((
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08,  # 0x2d..0x36
    0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F,  # 0x37..0x40
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02,  # 0x41..0x4a
    0x0A, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,  # 0x4b..0x54
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01,  # 0x55..0x5e
    0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08,  # 0x5f..0x68
    0xB8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00,  # 0x69..0x72
    0x00, 0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00, 0x00,  # 0x73..0x7c
    0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01,  # 0x7d..0x86
))


@task.driver('vl53l4cx')
class Vl53l4cx(task.Task):
    """Laser ToF: writes above-ground-level distance (m) to the databoard 'agl' slot, for the final
    low-altitude metres where the barometer cannot resolve height. Interrupt-driven when GPIO1 wired."""

    async def setup(self) -> bool:
        bus_id = self.config.get('id', 0)
        spec = config.bus(self.controller.config, self.config.get('bus', 'i2c'), bus_id)
        if spec is None:
            return False
        self._bus = i2cbus.get(bus_id, spec)
        self._addr: int = self.config.get('addr', 0x29)
        self._period_ms: int = self.config.get('period_ms', 50)  # poll interval with no INT wired
        self._fallback_ms: int = self.config.get('fallback_ms', 500)  # safety sample if INT silent
        self._ready = asyncio.ThreadSafeFlag()
        self._int = None
        try:
            await self._reset()  # pulse XSHUT (if wired) and wait for the firmware to boot
            if not int.from_bytes(await self._read(_REG_MODEL_ID, 2), 'big'):
                return False  # nothing answering at this address (0 model id)
            await self._bus.write(self._addr, _REG_CONFIG_START, _DEFAULT_CONFIG, addrsize=16)
            await self._write(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
            await self._write(_REG_SYSTEM_MODE_START, 0x40)  # start continuous ranging
            self._setup_interrupt()
        except Exception as error:
            print('vl53l4cx :: %r' % error)
            return False
        self._agl = databoard.Databoard.provide(self.name, self.config.get('provides', {}), 'agl')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('agl',),
                                       decimate_us=self.config.get('telemetry_us', 0))
        self._ok = True
        return True

    async def _read(self, reg: int, count: int) -> bytes:
        return await self._bus.read(self._addr, reg, count, addrsize=16)

    async def _write(self, reg: int, value: int) -> None:
        await self._bus.write(self._addr, reg, bytes((value,)), addrsize=16)

    async def _reset(self) -> None:
        """Drive XSHUT low->high to reset the sensor (recovers a wedged ToF without a board reboot),
        then wait for the firmware to boot. With no xshut_pin the sensor is assumed always-on."""
        gpio = self.controller.config.get('pins', {}).get(self.config.get('xshut_pin'))
        if gpio is not None:
            from machine import Pin

            xshut = Pin(gpio, Pin.OUT, value=0)  # active-low shutdown
            await asyncio.sleep_ms(2)
            xshut.value(1)  # enable
        await asyncio.sleep_ms(2)
        for _ in range(_BOOT_TIMEOUT_MS):  # poll FIRMWARE__SYSTEM_STATUS until booted (bit0)
            if (await self._read(_REG_FIRMWARE_STATUS, 1))[0] & 0x01:
                return
            await asyncio.sleep_ms(1)

    def _setup_interrupt(self) -> None:
        """Wire GPIO1 -> data-ready (active-low in continuous mode) if an int_pin is declared."""
        gpio = self.controller.config.get('pins', {}).get(self.config.get('int_pin'))
        if gpio is None:
            return
        from machine import Pin

        self._int = Pin(gpio, Pin.IN, Pin.PULL_UP)
        self._int.irq(lambda pin: self._ready.set(), Pin.IRQ_FALLING)

    async def _range(self) -> float:
        """Read the latest measurement and clear the interrupt; return AGL in metres, or None if the
        range status is invalid (out of range / low signal)."""
        status = (await self._read(_REG_RANGE_STATUS, 1))[0] & 0x1F
        mm = struct.unpack('>H', await self._read(_REG_DISTANCE, 2))[0]
        await self._write(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)  # release the interrupt for the next sample
        return mm / 1000.0 if status in _VALID_STATUS else None

    async def run(self) -> None:
        """Sample on data-ready (GPIO1) or every period_ms; write AGL (m) to the databoard. Runs
        forever."""
        while True:
            if self._int is not None:
                try:
                    await asyncio.wait_for_ms(self._ready.wait(), self._fallback_ms)
                except asyncio.TimeoutError:
                    pass  # no interrupt within the window -> sample anyway (safety)
            else:
                await asyncio.sleep_ms(self._period_ms)
            try:
                agl = await self._range()
                if agl is not None:
                    self._agl.push(agl)  # one step: push our channel directly
                    self._telemetry.push((agl,))
            except Exception as error:
                print('vl53l4cx :: read %r' % error)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['interrupt'] = self._int is not None
        status['agl_m'] = self._agl.value()  # our channel's latest (no hot-path I2C here)
        return status
