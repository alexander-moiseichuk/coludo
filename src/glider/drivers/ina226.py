# drivers/ina226.py — INA226 high-side current / voltage / power monitor over the shared I2C bus:
# the battery (or 5 V) supply-line sensor for consumption tracking. @task.driver('ina226'). setup()
# verifies the die id, programs the conversion config, and computes + writes the calibration register
# from the shunt resistance + the expected max current (the only board-specific numbers); run() polls
# the bus voltage (V), current (A) and power (W) to the databoard + telemetry. Graceful: wrong/absent
# die id -> setup False -> the Controller skips it.
#
# The INA226 measures the SHUNT VOLTAGE directly (2.5 uV/LSB), so the absolute accuracy comes from the
# CAL register, not a precise resistor: Current_LSB = max_current / 2**15, CAL = 0.00512 / (Current_LSB
# * shunt_ohms). To trust the watt-hours, calibrate `shunt_ohms` against a KNOWN current once and back
# out the effective value -- a 2-wire ohmmeter cannot resolve a 0.01 ohm shunt.

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


_REG_CONFIG = const(0x00)
_REG_BUS_V = const(0x02)    # u16, 1.25 mV/LSB
_REG_POWER = const(0x03)    # u16, Power_LSB = 25 * Current_LSB
_REG_CURRENT = const(0x04)  # s16, Current_LSB per bit
_REG_CALIB = const(0x05)
_REG_DIE_ID = const(0xFF)   # = 0x2260 on the INA226
_DIE_ID = const(0x2260)
_DIE_HI = const(0x22)       # die-id high byte -- the 8-bit value the bus diagnose() reads back
_CONFIG_DEFAULT = const(0x4327)  # continuous shunt+bus, 4-sample average, 1.1 ms conv (~9 ms/update)
# INTEGER milli-unit scaling (no float): the driver reads/reports mV, mA, mW. Derived from the datasheet:
#   bus:  1.25 mV/LSB           -> mV  = bus_raw * 5 // 4
#   cur:  Current_LSB = max/2^15 -> mA  = current_raw * max_current_ma // 32768
#   pow:  Power_LSB = 25*Cur_LSB -> mW  = power_raw * max_current_ma // 32768 * 25   (reordered: no 2^30 mpz)
#   cal = 0.00512 / (Current_LSB[A] * shunt[Ω]) = 0.00512 * 2^15 * 1e6 / (max_current_ma * shunt_mohms)
_CAL_NUM = const(167772160)  # 0.00512 * 2**15 * 1e6 -> integer cal numerator for milli-unit inputs
_POWER_LSB_RATIO = const(25)  # Power_LSB = 25 * Current_LSB
_REG_MASK = const(0x06)      # Mask/Enable: which condition drives ALERT, plus the read-back flags
_REG_ALERT_LIM = const(0x07)
_MASK_SOL = const(0x8000)    # ALERT on Shunt-Over-Voltage (== over-current: rail-voltage independent)
# shunt-voltage LSB is 2.5 uV -> alert limit LSBs = alert_ma[mA] * shunt_mohms[mΩ] * 2 // 5 (÷2.5 µV)


@task.driver('ina226')
class Ina226(task.Task):
    """High-side power monitor: bus voltage (mV), current (mA) and power (mW) -- INTEGER milli-units, no
    float -- to the databoard + per-sample telemetry. Current/power scale from `shunt_mohms` +
    `max_current_ma` (the CAL register). The same driver serves the 5 V USB phase and the LiPo phase --
    it reports the INA's own bus voltage, so power is correct as the base rail changes. Graceful: a
    wrong/absent die id -> setup False."""

    async def setup(self) -> bool:
        bus_id = self.config.get('id', 0)
        spec = config.bus(self.controller.config, self.config.get('bus', 'i2c'), bus_id)
        if spec is None:
            return False
        self._bus = i2cbus.get(bus_id, spec)
        self._addr: int = self.config.get('addr', 0x40)
        self._period_ms: int = self.config.get('period_ms', 100)
        shunt_mohms: int = self.config.get('shunt_mohms', 10)  # shunt resistance in milli-ohms (was shunt_ohms)
        self._max_current_ma: int = self.config.get('max_current_ma', 5000)  # full-scale current, mA
        try:
            if struct.unpack('>H', await self._bus.read(self._addr, _REG_DIE_ID, 2))[0] != _DIE_ID:
                return False  # not an INA226 at this address
            await self._bus.write(self._addr, _REG_CONFIG, struct.pack('>H', _CONFIG_DEFAULT))
            cal = _CAL_NUM // (self._max_current_ma * shunt_mohms)  # integer, milli-unit inputs
            await self._bus.write(self._addr, _REG_CALIB, struct.pack('>H', cal))
        except Exception as error:
            print('ina226 :: %r' % error)
            return False
        self._voltage, self._current, self._power = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'voltage', 'current', 'power')  # values now mV / mA / mW
        self._telemetry = recorder.Telemetry(
            '%s.csv' % self.name, ('voltage_mv', 'current_ma', 'power_mw', 'alerts'),
            decimate_us=self.config.get('telemetry_us', 0))  # 0 -> Recorder global rate
        # optional hardware over-current ALERT (config `alert_pin` -> a GPIO): the INA asserts it
        # (open-drain, active-low) each time the shunt voltage crosses the trip -- a stall / short flag
        # independent of the poll rate. Transient (no latch), and the IRQ COUNTS every trip: the running
        # total over a flight (in telemetry + inspect) is the statistic, not the fact of a single alert.
        self._alerts: int = 0
        self._logged_alerts: int = 0
        self._alert_ma: int = 0
        gpio = self._pin_gpio('alert_pin')
        if gpio is not None:
            try:
                self._alert_ma = self.config.get('alert_ma', 3000)
                limit = self._alert_ma * shunt_mohms * 2 // 5  # mA·mΩ -> shunt-voltage LSBs (÷2.5 µV)
                await self._bus.write(self._addr, _REG_ALERT_LIM, struct.pack('>H', limit))
                await self._bus.write(self._addr, _REG_MASK, struct.pack('>H', _MASK_SOL))  # transient, no latch
                from machine import Pin
                self._alert_pin = Pin(gpio, Pin.IN, Pin.PULL_UP)  # ALERT is open-drain, active-low
                self._alert_pin.irq(self._on_alert, Pin.IRQ_FALLING)
            except Exception as error:  # a bad alert wire must not sink the whole monitor -> poll-only
                print('ina226 :: alert setup failed, poll-only: %r' % error)
                self._alert_ma = 0
        self._ok = True
        return True

    def _on_alert(self, pin) -> None:
        """IRQ: the ALERT line fell -- current crossed the over-current trip. COUNT it (the per-flight
        total is the value; a soft IRQ so the small-int increment is safe -- only run() reads it)."""
        self._alerts += 1

    async def _read(self) -> tuple:
        """Read (bus voltage mV, current mA, power mW) from the live registers -- INTEGER milli-units, no
        float. Current is signed (a reversed shunt / charging current reads negative); power is positive.
        The power term divides by 32768 BEFORE the ×25 so the intermediate stays a small int (no mpz)."""
        bus_raw = struct.unpack('>H', await self._bus.read(self._addr, _REG_BUS_V, 2))[0]
        current_raw = struct.unpack('>h', await self._bus.read(self._addr, _REG_CURRENT, 2))[0]
        power_raw = struct.unpack('>H', await self._bus.read(self._addr, _REG_POWER, 2))[0]
        return (bus_raw * 5 // 4,                                             # mV (1.25 mV/LSB)
                current_raw * self._max_current_ma // 32768,                 # mA
                power_raw * self._max_current_ma // 32768 * _POWER_LSB_RATIO)  # mW

    async def run(self) -> None:
        while True:
            try:
                voltage, current, power = await self._read()  # mV, mA, mW (integers)
                self._voltage.push(voltage)  # push our channels directly
                self._current.push(current)
                self._power.push(power)
                self._telemetry.push((voltage, current, power, self._alerts))  # count is a per-flight series
                if self._alerts != self._logged_alerts:  # new over-current since the last pass -> log once
                    recorder.Recorder.log(self.name, 'OVER-CURRENT alerts: %d (> %d mA) -- servo stall / short?'
                                          % (self._alerts, self._alert_ma))
                    self._logged_alerts = self._alerts
                self.note(None)  # healthy pass -> the next error logs afresh
            except Exception as error:
                self.note('ina226 :: read %r', error)  # deduped: a persistent error logs once
            await asyncio.sleep_ms(self._period_ms)

    async def probe(self) -> str:
        """On-demand self-test: the die id reads back, then one live read (each step logged)."""
        try:
            recorder.Recorder.log(self.name, 'probe: die id ...')
            die = struct.unpack('>H', await self._bus.read(self._addr, _REG_DIE_ID, 2))[0]
            if die != _DIE_ID:
                raise ValueError('INA226 die 0x%04x != 0x%04x at i2c:%s 0x%02x' % (
                    die, _DIE_ID, self.config.get('id'), self._addr))
            recorder.Recorder.log(self.name, 'probe: die id ok 0x%04x' % die)
        except Exception as error:
            message = 'die id: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        try:
            recorder.Recorder.log(self.name, 'probe: read ...')
            voltage, current, power = await self._read()  # mV, mA, mW
            recorder.Recorder.log(self.name, 'probe: read ok %d mV %d mA %d mW' % (voltage, current, power))
        except Exception as error:
            message = 'read: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    async def diagnose(self) -> str:
        """Deeper analysis when setup() failed: the bus reads the die id and classifies the wire-level
        fault (no ack / wrong device / present-but-init). The Controller folds it into the reason."""
        bus = getattr(self, '_bus', None)
        if bus is None:  # setup never built the bus -> a config fault
            return 'no transport -- i2c bus %s undefined in config' % self.config.get('id', 0)
        return await bus.device(self._addr).diagnose(_REG_DIE_ID, _DIE_HI)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)  # our channels' latest (no hot-path I2C here)
        status['voltage_mv'] = self._voltage.value()
        status['current_ma'] = self._current.value()
        status['power_mw'] = self._power.value()
        status['alerts'] = self._alerts  # over-current ALERT trips since boot (0 if no alert pin / none)
        return status
