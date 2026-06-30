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
_CAL_CONST = 0.00512        # INA226 fixed calibration constant (datasheet)
_BUS_V_LSB = 0.00125        # 1.25 mV per bus-voltage bit
_POWER_LSB_RATIO = const(25)  # Power_LSB = 25 * Current_LSB


@task.driver('ina226')
class Ina226(task.Task):
    """High-side power monitor: bus voltage (V), current (A) and power (W) to the databoard + per-sample
    telemetry. Current/power scale from `shunt_ohms` + `max_current_a` (the CAL register). The same
    driver serves the 5 V USB phase and the LiPo phase -- it reports the INA's own bus voltage, so the
    power is correct as the base rail changes. Graceful: a wrong/absent die id -> setup False."""

    async def setup(self) -> bool:
        bus_id = self.config.get('id', 0)
        spec = config.bus(self.controller.config, self.config.get('bus', 'i2c'), bus_id)
        if spec is None:
            return False
        self._bus = i2cbus.get(bus_id, spec)
        self._addr: int = self.config.get('addr', 0x40)
        self._period_ms: int = self.config.get('period_ms', 100)
        shunt_ohms: float = self.config.get('shunt_ohms', 0.01)
        max_current: float = self.config.get('max_current_a', 5.0)
        self._current_lsb: float = max_current / 32768.0  # amperes per current-register bit
        try:
            if struct.unpack('>H', await self._bus.read(self._addr, _REG_DIE_ID, 2))[0] != _DIE_ID:
                return False  # not an INA226 at this address
            await self._bus.write(self._addr, _REG_CONFIG, struct.pack('>H', _CONFIG_DEFAULT))
            cal = round(_CAL_CONST / (self._current_lsb * shunt_ohms))
            await self._bus.write(self._addr, _REG_CALIB, struct.pack('>H', cal))
        except Exception as error:
            print('ina226 :: %r' % error)
            return False
        self._voltage, self._current, self._power = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'voltage', 'current', 'power')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name, ('voltage', 'current', 'power'),
                                             decimate_us=self.config.get('telemetry_us', 0))
        self._ok = True
        return True

    async def _read(self) -> tuple:
        """Read (bus voltage V, current A, power W) from the live registers. Current is signed (a
        reversed shunt or a charging current reads negative); power is always positive."""
        bus_raw = struct.unpack('>H', await self._bus.read(self._addr, _REG_BUS_V, 2))[0]
        current_raw = struct.unpack('>h', await self._bus.read(self._addr, _REG_CURRENT, 2))[0]
        power_raw = struct.unpack('>H', await self._bus.read(self._addr, _REG_POWER, 2))[0]
        return (bus_raw * _BUS_V_LSB, current_raw * self._current_lsb,
                power_raw * _POWER_LSB_RATIO * self._current_lsb)

    async def run(self) -> None:
        while True:
            try:
                voltage, current, power = await self._read()
                self._voltage.push(voltage)  # push our channels directly
                self._current.push(current)
                self._power.push(power)
                self._telemetry.push((voltage, current, power))
                self.note(None)  # healthy pass -> the next error logs afresh
            except Exception as error:
                self.note('ina226 :: read %r' % error)  # deduped: a persistent error logs once
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
            voltage, current, power = await self._read()
            recorder.Recorder.log(self.name, 'probe: read ok %.2fV %.3fA %.2fW' % (voltage, current, power))
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
        status['voltage_v'] = self._voltage.value()
        status['current_a'] = self._current.value()
        status['power_w'] = self._power.value()
        return status
