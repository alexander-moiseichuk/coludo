"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

SDP810-500Pa differential-pressure sensor (Sensirion SDP8xx, thermal flow-through) over the shared I2C
bus: the pitot/static AIRSPEED channel. @task.driver('sdp810'). Command-based, not register-mapped:
setup() clears any prior continuous mode, starts continuous measurement (differential-pressure temp-comp,
average-till-read) and validates one CRC-checked frame; run() reads the 9-byte frame each period, applies
the sensor scale factor + the pad-tared calibration, and writes dynamic pressure (Pa) and indicated
airspeed (m/s) to the databoard. Graceful: nothing acks / a corrupt frame -> setup False -> skipped.

Bench-verified: 0x25 on i2c:0 (SDA 7 / SCL 8), scale factor 60 (Pa = raw/60), zero ~0.02 Pa. Tube
polarity (blow-verified): P+ = pitot (total), P- = interior static. The interior-static position error is
absorbed by `zero_offset_pa` (a pad tare, `update {"zero": true}`) and `pressure_scale` (a GNSS-vs-q trim).

Polled at period_ms in continuous mode (no per-sample write). Uses the shared locked bus (i2cbus);
shares i2c:0 with the other forward sensors.
"""

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


_ADDR = const(0x25)  # the fixed I2C address (SDP800/810-500Pa / -125Pa; 0x26 for the -x1 variants)
_CMD_START = b'\x36\x15'  # start continuous: differential-pressure temp-comp, average-till-read
_CMD_STOP = b'\x3f\xf9'  # stop continuous -> idle (required before re-starting after an unclean reboot)
_FRAME = const(9)  # DP[0,1],CRC, T[3,4],CRC, Scale[6,7],CRC
_FIRST_MS = const(20)  # first continuous result lands ~8 ms after start (with margin)
_STOP_MS = const(3)  # stop settles in ~0.5 ms before the part accepts the next command (with margin)
_START_TRIES = const(2)  # start attempts: a stop+restart clears a part left mid-continuous
_CRC_POLY = const(0x31)  # Sensirion CRC-8 polynomial
_CRC_INIT = const(0xFF)  # Sensirion CRC-8 seed
_DEFAULT_SCALE = 60.0  # scale factor for the -500Pa part (Pa = raw / scale); -125Pa reports 240
_AIR_DENSITY = 1.225  # kg/m^3 (ISA sea level, 15 °C); config 'air_density' trims for field elevation
_TEMP_LSB = 200.0  # sensor temperature = raw / 200 -> °C


def _crc8(data: bytes, index: int) -> int:
    """
    Sensirion CRC-8 (polynomial 0x31, seed 0xFF) over the two bytes at `data[index:index + 2]`.

    Args:
        data - the raw frame.
        index - the offset of the 2-byte word whose checksum follows at index + 2.

    Returns:
        The computed 8-bit checksum.
    """
    crc = _CRC_INIT
    for offset in (index, index + 1):
        crc ^= data[offset]
        for _ in range(8):
            crc = ((crc << 1) ^ _CRC_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _frame_ok(data: bytes) -> bool:
    """Validate the differential-pressure word's CRC (the flight-relevant field of the 9-byte frame)."""
    return len(data) == _FRAME and _crc8(data, 0) == data[2]


@task.driver('sdp810')
class Sdp810(task.Task):
    """
    Pitot/static airspeed to the databoard: dynamic pressure (Pa, signed) and indicated airspeed (m/s).

    Airspeed is v = sqrt(2 q / rho) from the tared dynamic pressure q (negative q -> 0 m/s). The
    interior-static reference has a position error, so a pad tare (`update {"zero": true}`) captures the
    at-rest bias and `pressure_scale` trims the span against a known GNSS ground speed on a calm pass.
    """

    _bus = None  # class default: no transport until setup() builds it (diagnose reads directly)

    async def setup(self) -> bool:
        bus_id = self.config.get('id', 0)
        spec = config.bus(self.controller.config, self.config.get('bus', 'i2c'), bus_id)
        if spec is None:
            return False
        self._bus = i2cbus.get(bus_id, spec)
        self._addr: int = self.config.get('addr', _ADDR)
        self._period_ms: int = self.config.get('period_ms', 20)  # ~50 Hz (tau63 < 3 ms allows fast poll)
        self._density: float = self.config.get('air_density', _AIR_DENSITY)
        self._zero_pa: float = self.config.get('zero_offset_pa', 0.0)  # pad-static bias, subtracted
        self._gain: float = self.config.get('pressure_scale', 1.0)  # position-error span trim
        self._scale: float = _DEFAULT_SCALE
        self._raw_pa: float = 0.0  # last un-tared reading (Pa) -> the tare source
        self._pressure: float = 0.0
        self._airspeed: float = 0.0
        self._temperature: float = 0.0
        """
        A prior unclean reboot can leave the part mid-continuous, where a re-issued start is rejected;
        the datasheet requires a stop first. Touch it plainly -- stop (harmless when already idle),
        then start and read one CRC-checked frame as the presence + health test. Nothing acks -> absent;
        acks but the frame is corrupt/short -> not a healthy SDP8xx here. Both fail setup gracefully
        (the Controller skips it) rather than crashing the boot.
        """
        for attempt in range(_START_TRIES):
            try:
                await self._bus.writeto(self._addr, _CMD_STOP)
                await asyncio.sleep_ms(_STOP_MS)
                await self._bus.writeto(self._addr, _CMD_START)
                await asyncio.sleep_ms(_FIRST_MS)
                frame = await self._bus.readfrom(self._addr, _FRAME)
            except OSError:
                if attempt == _START_TRIES - 1:
                    return False  # nothing acked across the retries -> absent
                continue
            if _frame_ok(frame):
                self._scale = float((frame[6] << 8) | frame[7]) or _DEFAULT_SCALE
                break
            if attempt == _START_TRIES - 1:
                await self._quiet_stop()  # acked but garbage -> leave it idle, fail gracefully
                return False
        self._dynamic_pressure, self._indicated = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'dynamic_pressure', 'airspeed')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name,
                                       ('dynamic_pressure', 'airspeed', 'temperature'),
                                       decimate_us=self.config.get('telemetry_us', 0))  # 0 -> Recorder global rate
        self._ok = True
        return True

    async def _quiet_stop(self) -> None:
        """Best-effort stop of continuous mode (setup-failure / teardown path); swallow a dead bus."""
        try:
            await self._bus.writeto(self._addr, _CMD_STOP)
        except OSError:
            pass

    def _convert(self, frame: bytes) -> tuple:
        """
        Turn a CRC-valid frame into (dynamic pressure Pa, indicated airspeed m/s, temperature °C).

        The raw signed differential pressure is scaled to Pa, tared for the interior-static bias and
        span-trimmed; airspeed follows the incompressible pitot relation q = ½ρv² (a negative q -- reverse
        flow or sub-zero noise -- yields 0 m/s, never a complex root).

        Args:
            frame - the 9-byte sensor frame (differential-pressure CRC already checked).

        Returns:
            (dynamic_pressure Pa, airspeed m/s, temperature °C).
        """
        raw = struct.unpack('>h', frame[0:2])[0]  # signed differential pressure
        self._raw_pa = raw / self._scale
        pressure = self._raw_pa * self._gain - self._zero_pa
        airspeed = (2.0 * pressure / self._density) ** 0.5 if pressure > 0.0 else 0.0
        temperature = struct.unpack('>h', frame[3:5])[0] / _TEMP_LSB
        return pressure, airspeed, temperature

    async def run(self) -> None:
        while True:
            try:
                frame = await self._bus.readfrom(self._addr, _FRAME)
                if not _frame_ok(frame):
                    self.note('sdp810 :: crc reject')  # deduped: a persistent corruption logs once
                else:
                    self._pressure, self._airspeed, self._temperature = self._convert(frame)
                    self._dynamic_pressure.push(self._pressure)  # one step: push our channels directly
                    self._indicated.push(self._airspeed)
                    self._telemetry.push((self._pressure, self._airspeed, self._temperature))
                    self.note(None)  # healthy pass -> let the next error log afresh
            except Exception as error:
                self.note('sdp810 :: read %r', error)  # deduped: a persistent error logs once, not every tick
            await asyncio.sleep_ms(self._period_ms)

    def update(self, props: dict) -> list:
        """
        Apply an operator property change: pad-tare the zero, or set a calibration constant directly.

        `{"zero": true}` captures the current at-rest reading as the zero offset (the pad tare that cancels
        the interior-static position error -- do it with the glider still). `{"zero_offset_pa": <Pa>}` sets
        that offset directly; `{"pressure_scale": <k>}` sets the span trim (from a GNSS-vs-q calm pass).

        Args:
            props - the property dict; honours 'zero' (a truthy flag), 'zero_offset_pa' and 'pressure_scale'.

        Returns:
            The list of changed property names.
        """
        changed = []
        if props.get('zero'):
            self._zero_pa = self._raw_pa * self._gain
            changed.append('zero_offset_pa')
        if 'zero_offset_pa' in props:
            self._zero_pa = float(props['zero_offset_pa'])
            changed.append('zero_offset_pa')
        if 'pressure_scale' in props:
            self._gain = float(props['pressure_scale'])
            changed.append('pressure_scale')
        return changed

    async def probe(self) -> str:
        """
        On-demand self-test: confirm the run loop is producing dynamic pressure.

        We issue NO I2C here -- a read would race the run loop's own continuous-mode read on the shared
        bus; a present, healthy sensor instead keeps its channel fresh, so probe just waits for and checks it.

        Args:
            (none)

        Returns:
            None on success; a short failure message (also logged) when no reading appears.
        """
        try:
            recorder.Recorder.log(self.name, 'probe: data ...')
            await asyncio.sleep_ms(300)  # let the run loop produce a fresh reading
            pressure = self._dynamic_pressure.value()
            if pressure is None:
                raise ValueError('no dp from run loop (i2c:%s 0x%02x)' % (self.config.get('id'), self._addr))
            recorder.Recorder.log(self.name, 'probe: data ok %.1f Pa' % pressure)
        except Exception as error:
            message = 'data: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    async def diagnose(self) -> str:
        """
        Deeper analysis when setup() failed: read one frame and classify the fault at the wire level.

        SDP8xx has no cheap product-id register (identity needs a two-command sequence), so presence is
        judged from the continuous frame: no ack -> absent/miswired; ack but a bad CRC -> a device that is
        not a healthy SDP8xx at this address (or bus corruption); a clean frame -> present but setup
        rejected it (e.g. a scale mismatch), worth an operator look.

        Args:
            (none)

        Returns:
            A wire-level fault category; a config-fault message when setup never built the transport.
        """
        if self._bus is None:  # setup never built the transport
            return 'no transport -- i2c bus %s undefined in config' % self.config.get('id', 0)
        try:
            frame = await self._bus.readfrom(self._addr, _FRAME)
        except Exception:
            return 'absent -- no ack at 0x%02x (check wiring / bus id)' % self._addr
        if not _frame_ok(frame):
            return 'garbage -- acks at 0x%02x but CRC fails (wrong device or bus noise)' % self._addr
        return 'present at 0x%02x but setup rejected the frame (scale %d?)' % (
            self._addr, (frame[6] << 8) | frame[7])

    def inspect(self) -> dict:
        status = task.Task.inspect(self)  # our channels' latest (no hot-path I2C here)
        status['dynamic_pressure_pa'] = self._dynamic_pressure.value()
        status['airspeed_ms'] = self._indicated.value()
        status['temperature_c'] = round(self._temperature, 1)
        status['scale'] = self._scale
        status['zero_offset_pa'] = round(self._zero_pa, 2)
        status['pressure_scale'] = self._gain
        return status
