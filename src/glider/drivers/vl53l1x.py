"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

VL53L1X time-of-flight laser ranger over the shared I2C bus: the above-ground-level (AGL) channel for
the last metres of the glide, where the barometer is useless. Registered as a driver named vl53l1x.
The VL53 family uses 16-BIT register addresses (i2cbus addrsize=16).

THE SAME 0x29 ADDRESS AS THE VL53L4CX, different silicon: this part is 0xEACC and takes the VL53L1X
Ultra-Lite-Driver init; the VL53L4CD/L4CX is 0xEBAA and takes a different one. Neither config produces
ranges on the other part, so BOTH drivers check their model id strictly and return False on a mismatch
-- which means a board can declare both and the fitted one simply wins, exactly as an absent device is
skipped today. An I2C scan cannot tell them apart, so `layout` cannot either; it only makes the second
entry FOLLOW the first onto the revision's bus, without casting a second vote for the one address.

RANGE IS THE REASON TO CARE WHICH IS FITTED: the L1X is declared 2-4 m against the L4CX's 4-6 m, and
the landing trigger (`land_agl_m`, sequencer) defaults to 5.0 m -- unreachable by an L1X, so the
transition would silently fall back to barometric elevation, which is the thing the laser exists to
replace. A board fitted with this part wants `land_agl_m` around 3.0; see doc/hardware.md.

setup(): optional XSHUT reset -> wait for boot -> check the model id -> write the default
configuration -> one calibration ranging cycle (start/wait/clear/stop, then the VHV writes) -> start
continuous ranging. run(): wait for data-ready (the GPIO1 interrupt if wired, else a poll), read the
distance and write AGL (m) to the databoard. Graceful: no I2C ack -> setup False -> Controller skips
it. Shares its bus via the locked i2cbus.

NO TIMING BUDGET IS SET. The L4CX driver computes RANGE_CONFIG_A/B from the macro period; that math is
the VL53L4CD ULD's and does not apply to this silicon, whose ULD uses a lookup table keyed on distance
mode instead. The config block's own timing stands until that table can be checked against the part.
"""

import asyncio
import struct
import time

import commons
import databoard
import i2cbus
import recorder
import task

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const

try:
    from machine import Pin
except ImportError:  # host (CPython): board-only; the XSHUT/INT pins are wired only on the board
    Pin = None


_ADDR = const(0x29)  # default I2C address
_REG_FIRMWARE_STATUS = const(0x00E5)  # reads 0x03 once the firmware has booted
_REG_MODEL_ID = const(0x010F)  # 2 bytes: 0xEACC for the VL53L1X silicon
_REG_CONFIG_START = const(0x002D)  # the default-configuration block is written from here
_REG_GPIO_HV_MUX = const(0x0030)  # bit4 -> interrupt polarity
_REG_GPIO_HV_STATUS = const(0x0031)  # data-ready poll (bit0 vs the interrupt polarity)
_REG_SYSTEM_INTERRUPT_CLEAR = const(0x0086)
_REG_SYSTEM_MODE_START = const(0x0087)  # 0x40 = start continuous ranging, 0x00 = stop
_REG_RANGE_STATUS = const(0x0089)  # device range status (low 5 bits, mapped via _STATUS_RTN)
_REG_DISTANCE = const(0x0096)  # 2 bytes, distance in mm
_START_CONTINUOUS = const(0x40)  # the L1X value; the L4CD/L4CX uses 0x21
_BOOT_TIMEOUT_MS = const(100)
_RANGE_TIMEOUT_MS = const(200)  # VHV calibration ranging can take tens of ms

# Raw range-status (reg 0x0089 & 0x1F) -> ULD status; 0 == valid measurement (255 == reserved).
_STATUS_RTN = (255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6, 255, 255, 11, 12)

"""
VL53L1X default configuration, registers 0x2D..0x87 (91 bytes) -- ST's canonical ULD array.

DO NOT hand-edit or re-transcribe this. The version this driver carried in its own history was WRONG
in three ways and the part simply never ranged: 0x2E/0x2F were 0x00 instead of 0x01 (I2C and GPIO
pulled up at AVDD, not 1.8 V), 0x5F was 0xCC instead of 0xDB, and -- the one that mattered -- a single
0x00 was missing around 0x76, so every register from there on was written ONE ADDRESS LOW. That block
was authored when the project had no VL53L1X to test against and was replaced by the VL53L4CX init
before anyone met the silicon, so nothing ever caught it.

Measured on hardware 2026-09-20: with this array the part reports data-ready in 7 ms; with the
historical one, no measurement completes in 3000 ms. Confirmed ranging properly against a palm placed
on the sensor -- 8-34 mm at status 0 with the signal rate jumping from ~32 to ~8700 -- and 68 of 70
samples valid against a pale target held at 1.0-1.5 m.

Register 0x87 (MODE_START) is included and is 0x00 -- ranging is stopped here and started explicitly.
"""
_DEFAULT_CONFIG = bytes((
    0x00, 0x01, 0x01, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08,  # 0x2d..0x36
    0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F,  # 0x37..0x40
    0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02,  # 0x41..0x4a
    0x0A, 0x21, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,  # 0x4b..0x54
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01,  # 0x55..0x5e
    0xDB, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08,  # 0x5f..0x68
    0xB8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00,  # 0x69..0x72
    0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,  # 0x73..0x7c
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00,  # 0x7d..0x86
    0x00,                                                        # 0x87 MODE_START: stopped
))


@task.driver('vl53l1x')
class Vl53l1x(task.Task):
    """
    Laser ToF: writes above-ground-level distance (m) to the databoard 'agl' slot.

    For the final low-altitude metres where the barometer cannot resolve height. Interrupt-driven when
    GPIO1 is wired.
    """

    _bus = None  # class default: no transport until setup() builds it (diagnose reads directly)

    async def setup(self) -> bool:
        self._bus, self._addr = i2cbus.bind(self.controller.config, self.config, _ADDR)
        if self._bus is None:
            return False  # no such bus in config -> the Controller skips this device
        """
        ONE knob. `period_us` is both the sample period and the interrupt fallback, because
        commons.Waiter made them the same thing: a live edge returns on the first slice, a dead one
        runs the slices out and the sample is taken anyway. Two constants that had to be kept
        consistent became one that cannot disagree with itself.
        """
        # INT-silent fallback cadence. Reads `period_ms`, the key the CONFIG actually supplies --
        # this used to read `period_us`, which appears in no config, so every one of these drivers
        # silently fell back to 500 ms against a 100 ms freshness window. That is the exact
        # "dead wire masquerading as a healthy sensor" the irq_runs work exists to expose.
        self._period_ms: int = max(1, self.config.get('period_ms', 50))
        self._ready = commons.Waiter()  # IRQ-kicked wake + sliced fallback (see commons.Waiter)
        self._int = None
        try:
            if not await self._reset():  # pulse XSHUT (if wired) and wait for the firmware to boot
                return False  # firmware wedged -> reject (the model id below is silicon, would false-pass)
            if struct.unpack('>H', await self._read(_REG_MODEL_ID, 2))[0] != 0xEACC:
                return False  # not a VL53L1X -- an L4CX answers 0xEBAA here and needs the other driver
            await self._bus.write(self._addr, _REG_CONFIG_START, _DEFAULT_CONFIG, addrsize=16)
            # data-ready polarity from GPIO_HV_MUX, then a VHV calibration ranging cycle
            self._polarity = 0 if (await self._read(_REG_GPIO_HV_MUX, 1))[0] & 0x10 else 1
            await self._write(_REG_SYSTEM_MODE_START, _START_CONTINUOUS)
            await self._await_ready(_RANGE_TIMEOUT_MS)
            await self._write(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
            await self._write(_REG_SYSTEM_MODE_START, 0x00)  # stop after the calibration sample
            await self._write(0x0008, 0x09)  # VHV config: timeout macrop loop bound
            await self._write(0x000B, 0x00)
            await self._write(_REG_SYSTEM_MODE_START, _START_CONTINUOUS)  # start ranging for real
            self._setup_interrupt()
        except Exception as error:
            print('vl53l1x :: %r' % error)
            return False
        self._agl = databoard.Databoard.provide(self.name, self.config.get('provides', {}), 'agl')
        self._irq_runs: int = 0
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('agl', 'irq_runs'),
                                       decimate_us=self.config.get('telemetry_ms', 0) * 1000)  # 0 -> global
        self._ok = True
        return True

    async def _read(self, reg: int, count: int) -> bytes:
        return await self._bus.read(self._addr, reg, count, addrsize=16)

    async def _write(self, reg: int, value: int) -> None:
        await self._bus.write(self._addr, reg, bytes((value,)), addrsize=16)

    async def _reset(self) -> None:
        """
        Drive XSHUT low->high to reset the sensor, then wait for the firmware to boot.

        Recovers a wedged ToF without a board reboot. With no xshut_pin the sensor is assumed
        always-on.

        Args:
            (none)

        Returns:
            True once the firmware boots; False on timeout -- a dead-firmware sensor still ACKs its
            hard-silicon model id, so setup() must gate on this to avoid a false-present detection.
        """
        gpio = self._pin_gpio('xshut_pin')
        if gpio is not None:
            xshut = Pin(gpio, Pin.OUT, value=0)  # active-low shutdown
            await asyncio.sleep_ms(2)
            xshut.value(1)  # enable
        await asyncio.sleep_ms(2)
        for _ in range(_BOOT_TIMEOUT_MS):  # poll FIRMWARE__SYSTEM_STATUS until booted
            if (await self._read(_REG_FIRMWARE_STATUS, 1))[0] & 0x01:
                return True
            await asyncio.sleep_ms(1)
        return False  # firmware never booted -> model-id below is hard silicon (false-present); setup gates on this

    async def _await_ready(self, timeout_ms: int) -> None:
        """
        Poll GPIO__TIO_HV_STATUS until a measurement is ready (bit0 == the interrupt polarity).

        Args:
            timeout_ms - how long to poll, in milliseconds (1 ms per step).

        Returns:
            None; returns as soon as a measurement is ready, or after timeout_ms if none arrives.
        """
        for _ in range(timeout_ms):
            if ((await self._read(_REG_GPIO_HV_STATUS, 1))[0] & 0x01) == self._polarity:
                return
            await asyncio.sleep_ms(1)

    def _setup_interrupt(self) -> None:
        """Wire GPIO1 -> data-ready (active-low in continuous mode) if an int_pin is declared."""
        gpio = self._pin_gpio('int_pin')
        if gpio is None:
            return
        self._int = Pin(gpio, Pin.IN, Pin.PULL_UP)
        self._int.irq(self._ready.kick, Pin.IRQ_FALLING)

    _sample = None        # last good range (m), or None when out of range
    _sample_ms: int = 0   # when it was taken (0 = never)

    async def _range(self, cached_ok: bool = False) -> float:
        """
        Read the latest measurement and clear the interrupt -- SERIALISED, one owner at a time.

        Same shape that bit icp10111: status read, distance read and the interrupt CLEAR are three bus
        operations with awaits between them, and the clear is what arms the next sample. A second
        caller entering mid-sequence would read a half-updated pair and clear an interrupt that was not
        theirs. Today run() is the only caller, so this is latent rather than live -- but on icp10111
        the second caller turned out to be a DIAGNOSTIC, which invented a 20.8 % failure rate and cost
        real time to disbelieve. The guard is two integer checks; the bug it prevents is not.

        Args:
            cached_ok - True returns the last good range instead of touching the bus (an inspect-style
                caller wants the latest value, not a fresh conversation).

        Returns:
            AGL in metres; None when the range status is not valid (out of range / low signal).
        """
        if cached_ok and self._sample_ms:
            return self._sample
        # someone may own the status->distance->clear sequence; while waiting, take their answer
        while self._claimed:
            await asyncio.sleep_ms(1)
            if self._sample_ms:
                return self._sample
        await self.claim()
        try:
            return await self._range_locked()
        finally:
            self.unclaim()  # a raising read must not wedge every future caller

    async def _range_locked(self) -> float:
        """The bus conversation itself; only ever entered by the single owner _range() admits."""
        raw = (await self._read(_REG_RANGE_STATUS, 1))[0] & 0x1F
        distance_mm = struct.unpack('>H', await self._read(_REG_DISTANCE, 2))[0]
        await self._write(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)  # release the interrupt for the next sample
        status = _STATUS_RTN[raw] if raw < len(_STATUS_RTN) else 255
        self._sample = distance_mm / 1000.0 if status == 0 else None
        self._sample_ms = time.ticks_ms()
        return self._sample

    async def run(self) -> None:
        """
        The sampling loop: write AGL (m) to the databoard, forever.

        Sample on data-ready (GPIO1) or every period_ms, then push a valid AGL to the databoard and
        telemetry (an invalid range is skipped).

        Args:
            (none)

        Returns:
            None; runs forever (a wedged board reboots rather than exits).
        """
        while True:
            # no branch on whether an interrupt exists: wait() covers both (see adxl375)
            self._irq_runs = await self._ready.wait(self._period_ms)
            try:
                agl = await self._range()
                if agl is not None:
                    self._agl.push(agl)  # one step: push our channel directly
        # irq_runs: how many interrupt edges this wake consumed. 0 = the fallback timed out (a
        # dead or quiet line), 1 = healthy, >1 = an OVERRUN, edges arriving faster than the loop
        # consumes them. Recorded per row so a capture shows sampling health, not just samples.
                    self._telemetry.push((agl, self._irq_runs))
            except Exception as error:
                self.note('vl53l4cx :: read %r', error)  # deduped: a persistent I2C error logs once, not at poll rate

    async def probe(self) -> str:
        """
        On-demand self-test: the model id reads back.

        A single locked op, safe alongside the run loop's multi-op range sequence. The agl reading is
        legitimately None with no target in range, so it is not checked here.

        Args:
            (none)

        Returns:
            None when the model id reads back; a short failure message otherwise.
        """
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
        """
        Deeper analysis when setup() failed: classify the wire-level fault behind an absent ranger.

        Re-read the 16-bit MODEL_ID high byte (0xEB) and classify it via the i2cbus _Device helper. The
        Controller folds this into the failure reason so verify/probe show the 'why', not just 'absent /
        miswired?'.

        Args:
            (none)

        Returns:
            A one-line fault classification for the failure reason.
        """
        if self._bus is None:  # setup never built the transport
            return 'no transport -- i2c bus %s undefined in config' % self.config.get('id', 0)
        return await self._bus.device(self._addr).diagnose(_REG_MODEL_ID, 0xEB, addrsize=16)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['interrupt'] = self._int is not None
        status['agl_m'] = self._agl.value()  # our channel's latest (no hot-path I2C here)
        return status
