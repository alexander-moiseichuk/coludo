"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

SDP810-500Pa differential-pressure sensor (Sensirion SDP8xx, thermal flow-through) over the shared I2C
bus: the pitot/static AIRSPEED channel. @task.driver('sdp810'). Command-based, not register-mapped:
setup() clears any prior continuous mode, starts continuous measurement (differential-pressure temp-comp,
average-till-read) and validates one CRC-checked frame; run() reads the 9-byte frame each period, scales
the tared dynamic pressure (a `fixed` fixnum, Pa × SCALE) and derives airspeed once, publishing both to
the databoard. Graceful: nothing acks / a corrupt frame -> setup False -> skipped.

Bench-verified: 0x25 on i2c:0 (SDA 7 / SCL 8), scale factor 60 (Pa = raw/60), zero ~0.02 Pa. Tube
polarity (blow-verified): P+ = pitot (total), P- = interior static. The interior-static PRESSURE bias is
tared out by `zero_offset_pa` (a pad tare, `update {"zero": true}`); the position-span error folds into
`air_density`, the single q->v knob (a GNSS-vs-q calm pass trims it).

INTEGER internals, ONE float: the raw scaling and the pad-tared dynamic pressure stay a `fixed` fixnum
(a small int, so the store never boxes). Airspeed = sqrt(2q/rho) is the ONE float, computed ONCE per read
and reused for the airspeed channel (-> the governor's estimator, airspeed.py) AND the telemetry row -- no
second conversion anywhere. The governor consumes the ready airspeed; it does no sqrt of its own.

Polled at period_ms in continuous mode (no per-sample write). Uses the shared locked bus (i2cbus);
shares i2c:0 with the other forward sensors.
"""

import asyncio

try:
    from esp32 import NVS
    _nvs = NVS('coludo')
except Exception:  # host / no NVS partition -- the tare simply stays in RAM
    _nvs = None

import databoard
import fixed
import i2cbus
import recorder
import task

# @micropython.viper / .native are compiler directives keyed on the literal decorator name; commons owns
# the one shim (real module on-board, identity off-board) + the fold-friendly const -- borrow, don't dupe.
from commons import const, micropython

_ADDR = const(0x25)  # the fixed I2C address (SDP800/810-500Pa / -125Pa; 0x26 for the -x1 variants)
_CMD_START: bytes = b'\x36\x15'  # start continuous: differential-pressure temp-comp, average-till-read
_CMD_STOP: bytes = b'\x3f\xf9'  # stop continuous -> idle (required before re-starting after an unclean reboot)
_FRAME = const(9)  # DP[0,1],CRC, T[3,4],CRC, Scale[6,7],CRC
_FIRST_MS = const(20)  # first continuous result lands ~8 ms after start (with margin)
_NVS_ZERO = 'sdp810_zero'  # NVS key: the tare as a fixnum i32, so it survives any reboot
_RESTART_AFTER = const(5)  # consecutive read failures before re-issuing _CMD_START (see _restart)
_STOP_MS = const(3)  # stop settles in ~0.5 ms before the part accepts the next command (with margin)
_START_TRIES = const(2)  # start attempts: a stop+restart clears a part left mid-continuous
_DEFAULT_SCALE = const(60)  # scale factor for the -500Pa part (Pa = raw / scale); the -125Pa reports 240
_TEMP_LSB = const(200)  # sensor temperature = raw / 200 -> °C; logged as a °C fixnum (raw // 2 = raw/200 × SCALE)
_AIR_DENSITY: float = 1.18  # kg/m^3 (~25 °C sea level -- Florida, not ISA 15 °C); the calm-pass trim overrides it


@micropython.viper
def _crc8(byte0: int, byte1: int) -> int:
    """
    Sensirion CRC-8 (polynomial 0x31, seed 0xFF) over the two data bytes of a frame word.

    Integer-only -> @viper. Takes the two bytes as ints (not a buffer) so it stays a pure typed
    function; the caller compares the result against the word's trailing checksum byte.

    Args:
        byte0 - the word's first (high) byte.
        byte1 - the word's second (low) byte.

    Returns:
        The computed 8-bit checksum.
    """
    crc: int = 0xFF
    current: int = byte0
    word: int = 0
    bit: int = 0
    while word < 2:
        crc = crc ^ current
        bit = 0
        while bit < 8:
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            bit += 1
        current = byte1
        word += 1
    return crc & 0xFF


@micropython.native
def _frame_ok(data: bytes) -> bool:
    """Validate the differential-pressure word's CRC (the flight-relevant field of the 9-byte frame)."""
    return len(data) == _FRAME and _crc8(data[0], data[1]) == data[2]


@micropython.native
def _signed16(hi: int, lo: int) -> int:
    """A big-endian signed 16-bit word from its two bytes (the sensor's two's-complement fields)."""
    value = (hi << 8) | lo
    return value - 0x10000 if value & 0x8000 else value


@task.driver('sdp810')
class Sdp810(task.Task):
    """
    Pitot/static airspeed + dynamic pressure to the databoard.

    `dynamic_pressure` is a `fixed` fixnum (Pa × SCALE, signed); `airspeed` is m/s, v = sqrt(2q/rho),
    derived once per read and shared with telemetry. The pad-tared zero cancels the interior-static
    PRESSURE bias; `air_density` is the single pressure->speed knob (it absorbs the position-span error,
    trimmed on a GNSS-vs-q calm pass). The saturation guard lives with the consumer (the governor drops
    back to the accel backbone when the pitot rails), so this driver just reports what it reads.
    """

    _bus = None  # class default: no transport until setup() builds it (diagnose reads directly)

    async def setup(self) -> bool:
        self._bus, self._addr = i2cbus.bind(self.controller.config, self.config, _ADDR)
        if self._bus is None:
            return False  # no such bus in config -> the Controller skips this device
        self._period_ms: int = self.config.get('period_ms', 20)  # ~50 Hz (tau63 < 3 ms allows fast poll)
        self._density: float = self.config.get('air_density', _AIR_DENSITY)  # the single q->v knob
        self._zero: fixed.fixnum = fixed.from_float(self.config.get('zero_offset_pa', 0.0))  # pad bias
        """
        A tare stored by a previous session WINS over the config default: it was measured on this
        airframe with this tube run, where the config value is a guess. Restoring it means a reboot --
        including one mid-glide, where nobody can re-tare -- comes back with a real zero instead of 0,
        which would otherwise put the whole interior-static bias into the airspeed the governor caps off.
        """
        if _nvs is not None:
            try:
                self._zero = _nvs.get_i32(_NVS_ZERO)
                # print(), not Recorder.log(): setup runs BEFORE the recorder task is up, so a logged
                # line here goes nowhere (logs are best-effort by policy). The same reason icp10111 and
                # bno055 print their setup failures. Runtime messages below still use the recorder.
                print('sdp810 :: zero restored from NVS: %.2f Pa' % fixed.to_float(self._zero))
            except Exception:
                pass  # never stored yet -- keep the config default
        self._scale: int = _DEFAULT_SCALE
        # None until the first frame, NOT 0: calibrate()/update({'zero': True}) both guard on
        # `_raw is None` to refuse a tare before there is a reading, and initialising to 0 made that
        # guard dead code -- an operator taring straight after boot silently captured a zero tare.
        self._raw = None  # last un-tared reading (Pa fixnum) -> the tare source
        self._temp_raw: int = 0  # last raw temperature word (raw / 200 -> °C)
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
                self._scale = ((frame[6] << 8) | frame[7]) or _DEFAULT_SCALE
                break
            if attempt == _START_TRIES - 1:
                await self._quiet_stop()  # acked but garbage -> leave it idle, fail gracefully
                return False
        self._pressure_ch, self._airspeed_ch = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'dynamic_pressure', 'airspeed')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name,
                                       ('dynamic_pressure', 'airspeed_cms', 'temperature'),  # all fixnums (x SCALE)
                                       decimate_us=self.config.get('telemetry_us', 0))  # 0 -> Recorder global rate
        self._ok = True
        return True

    async def _quiet_stop(self) -> None:
        """Best-effort stop of continuous mode (setup-failure / teardown path); swallow a dead bus."""
        try:
            await self._bus.writeto(self._addr, _CMD_STOP)
        except OSError:
            pass

    def _pressure(self, raw: int) -> fixed.fixnum:
        """
        Scale a raw signed differential-pressure reading to a tared dynamic-pressure fixnum (Pa × SCALE).

        raw × SCALE // scale is Pa as a fixnum (a small int at ±500 Pa, so the store never boxes); the
        pad tare then subtracts the at-rest interior-static bias. Integer only, no per-tick allocation.

        Args:
            raw - the signed 16-bit differential-pressure word.

        Returns:
            The tared dynamic pressure as a `fixed` fixnum (Pa × SCALE).
        """
        self._raw = raw * fixed.SCALE // self._scale  # Pa fixnum, un-tared (the tare source)
        return self._raw - self._zero

    def _airspeed(self, pressure: fixed.fixnum) -> float:
        """
        Indicated airspeed (m/s) from a dynamic-pressure fixnum -- the ONE float, computed once per read.

        q = ½ρv² inverted; a negative q (reverse flow, sub-zero noise) yields 0 m/s, never a complex root.

        Args:
            pressure - the tared dynamic-pressure fixnum (Pa × SCALE).

        Returns:
            The indicated airspeed (m/s).
        """
        q = fixed.to_float(pressure)
        return (2.0 * q / self._density) ** 0.5 if q > 0.0 else 0.0

    async def _restart(self) -> None:
        """
        Re-issue the continuous-measurement command after the part has been reset out from under us.

        This sensor holds its MODE in volatile state: it only streams because setup() sent _CMD_START.
        Anything that resets it mid-flight silently drops it to idle, and every later read then fails
        forever -- the run loop had no way back. The concrete trigger is a PEER: icp10111's latch-up
        recovery fires an I2C GENERAL-CALL reset (0x00 0x06), which by design resets every device on
        the bus that honours it, this one included. Observed here as `sdp810 :: read OSError(19)` with
        the channel flatlined at 0.0 Hz while the part itself was perfectly healthy.

        stop-then-start, because the part refuses a fresh start while still streaming (the same
        sequence setup() uses after an unclean reboot).

        Args:
            (none)

        Returns:
            None; best-effort -- a still-dead part just fails the next read and retries later.
        """
        try:
            await self._bus.writeto(self._addr, _CMD_STOP)
            await asyncio.sleep_ms(_STOP_MS)
            await self._bus.writeto(self._addr, _CMD_START)
            await asyncio.sleep_ms(_FIRST_MS)
            recorder.Recorder.log(self.name, 'restarted continuous mode after %d failures' % _RESTART_AFTER)
        except OSError:
            pass  # still gone -- the next read fails and we try again

    def calibration(self) -> str:
        """The still-air tare instruction; '' once a zero offset has been captured."""
        if self._zero != 0:
            return ''
        return ('keep the pitot in STILL AIR -- do NOT blow into it, this captures the zero tare '
                '(now %.2f Pa)' % fixed.to_float(self._pressure_ch.value() or 0))

    async def calibrate(self) -> str:
        """Capture the current still-air reading as the zero offset -- the board can do this itself."""
        if self._raw is None:
            return 'no reading yet'
        self.update({'zero': True})
        self._persist_zero()
        recorder.Recorder.log(self.name, 'calibrated: zero_offset %.2f Pa' % fixed.to_float(self._zero))
        return None

    def _persist_zero(self) -> None:
        """
        Commit the tare to NVS so it survives ANY reboot, warm or cold.

        The warm-start crumb already carries it, but only while the board is ARMED AND AIRBORNE, and it
        is discarded whenever the recovery gate rejects -- so a reboot the gate refuses (a cold power
        cycle at the pad, a cause it does not trust) still comes back untared. This is the same value
        with none of those conditions: written when the operator tares, read at setup, so dynamic
        pressure never silently carries the full interior-static bias.

        Best-effort by design -- a board with no NVS partition simply keeps the in-RAM tare, exactly as
        before. A failed commit must never take down a sensor that is otherwise working.

        Args:
            (none)

        Returns:
            None.
        """
        if _nvs is None:
            return
        try:
            _nvs.set_i32(_NVS_ZERO, int(self._zero))  # the fixnum itself: an int, no float in NVS
            _nvs.commit()
        except Exception as error:
            self.note('sdp810 :: zero persist %r', error)

    async def run(self) -> None:
        while True:
            try:
                frame = await self._bus.readfrom(self._addr, _FRAME)
                if not _frame_ok(frame):
                    self.note('sdp810 :: crc reject')  # deduped: a persistent corruption logs once
                else:
                    pressure = self._pressure(_signed16(frame[0], frame[1]))  # fixnum (integer path)
                    airspeed = self._airspeed(pressure)  # the ONE float, reused below (channel + telemetry)
                    self._temp_raw = _signed16(frame[3], frame[4])
                    self._pressure_ch.push(pressure)  # Pa fixnum -> small int, no boxing
                    self._airspeed_ch.push(airspeed)  # m/s -> the governor's estimator (airspeed.py)
                    # every collected value, all as fixnums -- a float in a row boxes on MicroPython
                    self._telemetry.push((pressure, int(airspeed * fixed.SCALE), self._temp_raw // 2))
                    self.note(None)  # healthy pass -> let the next error log afresh
                    self.strike(False, _RESTART_AFTER)  # good read rearms the run
            except Exception as error:
                self.note('sdp810 :: read %r', error)  # deduped: a persistent error logs once, not every tick
                if self.strike(True, _RESTART_AFTER):  # once, not every tick while it stays dead
                    await self._restart()
            await asyncio.sleep_ms(self._period_ms)

    def update(self, props: dict) -> list:
        """
        Apply an operator property change: pad-tare the zero, set the zero offset, or set the density.

        `{"zero": true}` captures the current at-rest reading as the zero offset (the pad tare that cancels
        the interior-static pressure bias -- do it with the glider still). `{"zero_offset_pa": <Pa>}` sets
        that offset directly; `{"air_density": <kg/m^3>}` sets the q->v span (the GNSS-vs-q calm-pass trim).

        Args:
            props - the property dict; honours 'zero' (a truthy flag), 'zero_offset_pa' and 'air_density'.

        Returns:
            The list of changed property names.
        """
        changed = []
        if props.get('zero'):
            self._zero = self._raw
            changed.append('zero_offset_pa')
        if 'zero_offset_pa' in props:
            self._zero = fixed.from_float(float(props['zero_offset_pa']))
            changed.append('zero_offset_pa')
        if 'air_density' in props:
            self._density = float(props['air_density'])
            changed.append('air_density')
        return changed

    async def probe(self) -> str:
        """
        On-demand self-test: confirm the run loop is producing readings.

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
            pressure = self._pressure_ch.value()
            if pressure is None:
                raise ValueError('no dp from run loop (i2c:%s 0x%02x)' % (self.config.get('id'), self._addr))
            recorder.Recorder.log(self.name, 'probe: data ok %s Pa (%.1f m/s)' % (
                fixed.to_str(pressure), self._airspeed_ch.value() or 0.0))
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
        pressure = self._pressure_ch.value()
        status = task.Task.inspect(self)  # our channels' latest (no hot-path I2C here)
        status.update({
            'dynamic_pressure_pa': None if pressure is None else fixed.to_float(pressure),
            'airspeed_ms': self._airspeed_ch.value(),
            'temperature_c': round(self._temp_raw / _TEMP_LSB, 1),
            'scale': self._scale,
            'zero_offset_pa': fixed.to_float(self._zero),
            'air_density': self._density,
        })
        return status
