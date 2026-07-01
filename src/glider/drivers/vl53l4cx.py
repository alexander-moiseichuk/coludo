# drivers/vl53l4cx.py — VL53L4CX time-of-flight laser ranger (Adafruit 5425) over the shared I2C bus:
# the above-ground-level (AGL) channel for the last metres of the glide, where the barometer is
# useless. @task.driver('vl53l4cx'). The VL53 family uses 16-BIT register addresses (i2cbus addrsize=
# 16). This part is the newer 0xEBAA silicon (shared by the VL53L4CD/L4CX), so it uses the VL53L4CD
# Ultra-Lite-Driver init -- the older VL53L1X (0xEACC) config does NOT produce ranges on it.
#
# setup(): optional XSHUT reset -> wait for boot -> write the default configuration -> run one VHV
# calibration ranging cycle (start/wait/clear/stop, then the VHV config writes) -> start continuous
# ranging. run(): wait for data-ready (the GPIO1 interrupt if wired, else a poll), read the distance
# and write AGL (m) to the databoard. Single-target distance; the L4CX multi-target extras are unused.
# Graceful: no I2C ack -> setup False -> Controller skips it. Shares i2c:0 via the locked i2cbus.

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
    from commons import const


_REG_FIRMWARE_STATUS = const(0x00E5)  # reads 0x03 once the firmware has booted
_REG_MODEL_ID = const(0x010F)  # 2 bytes: 0xEBAA for the VL53L4CD/L4CX silicon
_REG_CONFIG_START = const(0x002D)  # the default-configuration block is written from here
_REG_GPIO_HV_MUX = const(0x0030)  # bit4 -> interrupt polarity
_REG_GPIO_HV_STATUS = const(0x0031)  # data-ready poll (bit0 vs the interrupt polarity)
_REG_SYSTEM_INTERRUPT_CLEAR = const(0x0086)
_REG_SYSTEM_MODE_START = const(0x0087)  # 0x21 = start continuous ranging, 0x00 = stop
_REG_RANGE_STATUS = const(0x0089)  # device range status (low 5 bits, mapped via _STATUS_RTN)
_REG_DISTANCE = const(0x0096)  # 2 bytes, distance in mm
_START_CONTINUOUS = const(0x21)
_BOOT_TIMEOUT_MS = const(100)
_RANGE_TIMEOUT_MS = const(200)  # VHV calibration ranging can take tens of ms

# Raw range-status (reg 0x0089 & 0x1F) -> ULD status; 0 == valid measurement (255 == reserved).
_STATUS_RTN = (255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6, 255, 255, 11, 12)

# VL53L4CD default configuration, registers 0x2D..0x87 (91 bytes) — the ST/Adafruit ULD block for the
# 0xEBAA silicon; ranging is started separately by writing _START_CONTINUOUS to MODE_START.
_DEFAULT_CONFIG = (
    b'\x12\x00\x00\x11\x02\x00\x02\x08\x00\x08\x10\x01\x01\x00\x00\x00'
    b'\x00\xff\x00\x0f\x00\x00\x00\x00\x00\x20\x0b\x00\x00\x02\x14\x21'
    b'\x00\x00\x05\x00\x00\x00\x00\xc8\x00\x00\x38\xff\x01\x00\x08\x00'
    b'\x00\x01\xcc\x07\x01\xf1\x05\x00\xa0\x00\x80\x08\x38\x00\x00\x00'
    b'\x00\x0f\x89\x00\x00\x00\x00\x00\x00\x00\x01\x07\x05\x06\x06\x00'
    b'\x00\x02\xc7\xff\x9b\x00\x00\x00\x01\x00\x00'
)


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
            if (await self._read(_REG_MODEL_ID, 1))[0] != 0xEB:
                return False  # not a VL53L4CD/L4CX at this address
            await self._bus.write(self._addr, _REG_CONFIG_START, _DEFAULT_CONFIG, addrsize=16)
            # data-ready polarity from GPIO_HV_MUX, then a VHV calibration ranging cycle
            self._polarity = 0 if (await self._read(_REG_GPIO_HV_MUX, 1))[0] & 0x10 else 1
            await self._write(_REG_SYSTEM_MODE_START, _START_CONTINUOUS)
            await self._await_ready(_RANGE_TIMEOUT_MS)
            await self._write(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
            await self._write(_REG_SYSTEM_MODE_START, 0x00)  # stop after the calibration sample
            await self._write(0x0008, 0x09)  # VHV config: timeout macrop loop bound
            await self._write(0x000B, 0x00)
            await self._bus.write(self._addr, 0x0024, b'\x05\x00', addrsize=16)
            await self._set_timing_budget(self.config.get('timing_budget_ms', 50))  # longer = lower sigma
            await self._write(_REG_SYSTEM_MODE_START, _START_CONTINUOUS)  # start ranging for real
            self._setup_interrupt()
        except Exception as error:
            print('vl53l4cx :: %r' % error)
            return False
        self._agl = databoard.Databoard.provide(self.name, self.config.get('provides', {}), 'agl')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('agl',),
                                       decimate_us=self.config.get('telemetry_us', 0))  # 0 -> Recorder global rate
        self._ok = True
        return True

    async def _read(self, reg: int, count: int) -> bytes:
        return await self._bus.read(self._addr, reg, count, addrsize=16)

    async def _write(self, reg: int, value: int) -> None:
        await self._bus.write(self._addr, reg, bytes((value,)), addrsize=16)

    async def _reset(self) -> None:
        """Drive XSHUT low->high to reset the sensor (recovers a wedged ToF without a board reboot),
        then wait for the firmware to boot. With no xshut_pin the sensor is assumed always-on."""
        gpio = self._pin_gpio('xshut_pin')
        if gpio is not None:
            from machine import Pin

            xshut = Pin(gpio, Pin.OUT, value=0)  # active-low shutdown
            await asyncio.sleep_ms(2)
            xshut.value(1)  # enable
        await asyncio.sleep_ms(2)
        for _ in range(_BOOT_TIMEOUT_MS):  # poll FIRMWARE__SYSTEM_STATUS until booted
            if (await self._read(_REG_FIRMWARE_STATUS, 1))[0] & 0x01:
                return
            await asyncio.sleep_ms(1)

    async def _set_timing_budget(self, budget_ms: int) -> None:
        """Set the ranging integration time (RANGE_CONFIG_A/B) for continuous mode (inter-measurement
        0). Longer budget -> more integration -> lower sigma / longer range. ULD integer math (the
        2**30 term overflows single-precision float on MicroPython, so it stays integer)."""
        osc = struct.unpack('>H', await self._read(0x0006, 2))[0]
        if not osc:
            return
        macro = (2304 * 1073741824 // osc) >> 6  # macro period (us), 2**30 = 1073741824
        budget_us = (budget_ms * 1000 - 2500) << 12  # continuous: subtract the fixed overhead
        for reg, mult in ((0x005E, 16), (0x0061, 12)):  # RANGE_CONFIG_A, RANGE_CONFIG_B
            denom = (macro * mult) >> 6
            ls_byte = (budget_us + (denom >> 1)) // denom - 1
            ms_byte = 0
            while ls_byte > 0xFF:  # normalise into mantissa (<=8 bits) + exponent byte
                ls_byte >>= 1
                ms_byte += 1
            await self._bus.write(self._addr, reg, struct.pack('>H', (ms_byte << 8) + (ls_byte & 0xFF)),
                                  addrsize=16)

    async def _await_ready(self, timeout_ms: int) -> None:
        """Poll GPIO__TIO_HV_STATUS until a measurement is ready (bit0 == the interrupt polarity)."""
        for _ in range(timeout_ms):
            if ((await self._read(_REG_GPIO_HV_STATUS, 1))[0] & 0x01) == self._polarity:
                return
            await asyncio.sleep_ms(1)

    def _setup_interrupt(self) -> None:
        """Wire GPIO1 -> data-ready (active-low in continuous mode) if an int_pin is declared."""
        gpio = self._pin_gpio('int_pin')
        if gpio is None:
            return
        from machine import Pin

        self._int = Pin(gpio, Pin.IN, Pin.PULL_UP)
        self._int.irq(lambda pin: self._ready.set(), Pin.IRQ_FALLING)

    async def _range(self) -> float:
        """Read the latest measurement and clear the interrupt; return AGL in metres, or None if the
        range status is not valid (out of range / low signal)."""
        raw = (await self._read(_REG_RANGE_STATUS, 1))[0] & 0x1F
        mm = struct.unpack('>H', await self._read(_REG_DISTANCE, 2))[0]
        await self._write(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)  # release the interrupt for the next sample
        status = _STATUS_RTN[raw] if raw < len(_STATUS_RTN) else 255
        return mm / 1000.0 if status == 0 else None

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

    async def probe(self) -> str:
        """On-demand self-test: the model id reads back -- a single locked op, safe alongside the run
        loop's multi-op range sequence. The agl reading is legitimately None with no target in range,
        so it is not checked here."""
        try:
            recorder.Recorder.log(self.name, 'probe: model id ...')
            model = (await self._read(_REG_MODEL_ID, 1))[0]
            if model != 0xEB:
                raise ValueError('VL53L4CX id 0x%02x != 0xEB at i2c:%s 0x%02x' % (
                    model, self.config.get('id'), self._addr))
            recorder.Recorder.log(self.name, 'probe: model id ok 0x%02x' % model)
        except Exception as error:
            message = 'model id: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    async def diagnose(self) -> str:
        """Deeper analysis when setup() failed: re-read the 16-bit MODEL_ID high byte (0xEB) and
        classify it via the i2cbus _Device helper. The Controller folds this into the failure reason so
        verify/probe show the 'why', not just 'absent / miswired?'."""
        if getattr(self, '_bus', None) is None:
            return 'no transport -- i2c bus %s undefined in config' % self.config.get('id', 0)
        return await self._bus.device(self._addr).diagnose(_REG_MODEL_ID, 0xEB, addrsize=16)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['interrupt'] = self._int is not None
        status['agl_m'] = self._agl.value()  # our channel's latest (no hot-path I2C here)
        return status
