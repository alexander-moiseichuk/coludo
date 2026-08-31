"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

BNO055 9-DOF IMU (on the SEN0253) over the shared I2C bus: the attitude channel.
@task.driver('bno055'). In NDOF fusion mode the chip computes absolute orientation on-chip; run() reads
the Euler angles (heading, roll, pitch in degrees) to the databoard 'attitude' slot. Graceful: a
wrong/absent chip id -> setup False -> the Controller skips it.

BNO055's INT pin signals motion/threshold events, not a fusion data-ready, so this driver polls at
period_ms (the fusion engine runs at 100 Hz internally); the wired int_pin is reserved for future event
detection (e.g. high-g). Uses the shared locked bus (i2cbus) since it shares i2c:0 with the ADXL375 and
BMP280.
"""

import asyncio
import struct

try:
    from esp32 import NVS
    _nvs = NVS('coludo')
except Exception:  # host / no NVS partition -- the profile simply is not persisted
    _nvs = None

import databoard
import i2cbus
import recorder
import task
from fixed import SCALE  # attitude roll/pitch -> centidegree fixnum

try:
    from micropython import const
except ImportError:  # CPython (tooling / off-board checks)
    from commons import const


_ADDR = const(0x28)  # default I2C address (COM3 low; 0x29 when high)
_REG_CHIP_ID = const(0x00)  # = 0xA0
_REG_OPR_MODE = const(0x3D)  # operating mode
_REG_PWR_MODE = const(0x3E)  # power mode
_REG_DATA = const(0x08)  # ACC..EUL block: acc(6) mag(6) gyro(6) eul(6) = 24 bytes, all int16 LE
_OFF_EUL = const(18)  # EUL heading/roll/pitch within the block (16 LSB/degree)
_CHIP_ID = const(0xA0)
_MODE_CONFIG = const(0x00)
_MODE_NDOF = const(0x0C)  # full 9-DOF absolute-orientation fusion
_PWR_NORMAL = const(0x00)
# consecutive bit-identical Euler reads (WHILE accel moves) that prove the fusion has latched;
# 50 at the 50 Hz default is ~1 s -- long enough that a still airframe never trips it
_REG_CALIB_STAT = const(0x35)  # sys[7:6] gyr[5:4] acc[3:2] mag[1:0], 3 = fully calibrated
_MAG_CALIBRATED = const(3)     # magnetometer level that means the figure-8 is done
_REG_CALIB_DATA = const(0x55)  # ACC_OFFSET_X_LSB .. MAG_RADIUS_MSB: the fusion's learned profile
_CALIB_BYTES = const(22)       # that block's length; readable/writable in CONFIG mode ONLY
_NVS_CALIB = 'bno055_calib'    # NVS key holding it across power cycles
_OFF_GYR = const(12)  # gyro within the ACC..EUL block (bytes 12..17)
# |gx|+|gy|+|gz| above this means the part is genuinely ROTATING, so a frozen Euler is a fault
# and not merely a still airframe. 16 LSB/deg/s, so ~5 deg/s summed across the axes.
_TURNING_LSB = const(80)
_STALL_SAMPLES = const(50)
_DEG = 1.0 / 16.0
_ACC_G = 1.0 / 980.665  # ACC_DATA is m/s² at 100 LSB/(m/s²); /100/9.80665 -> g (incl gravity)


@task.driver('bno055')
class Bno055(task.Task):
    """
    9-DOF IMU to the databoard: fused attitude and a calibrated low-g accelerometer.

    NDOF fusion attitude (heading, roll, pitch in degrees) -> 'attitude', plus the calibrated
    accelerometer (g, including gravity) -> 'accel' as a low-g backup to the ADXL375 (priority 1).
    """

    _bus = None  # class default: no transport until setup() builds it (diagnose reads directly)

    async def setup(self) -> bool:
        self._bus, self._addr = i2cbus.bind(self.controller.config, self.config, _ADDR)
        if self._bus is None:
            return False  # no such bus in config -> the Controller skips this device
        self._period_ms: int = self.config.get('period_ms', 20)  # 50 Hz (fusion runs at 100 Hz)
        self._buf = bytearray(24)  # ACC..EUL block
        self._last_euler = None    # fusion-stall detector state (see _fusion_alive)
        self._stalled: bool = False
        self.calibration_state = None  # (sys, gyr, acc, mag) 0..3 each; None until first poll
        self._converged: bool = False  # LATCH: mag has reached 3 at some point (see _poll_calibration)
        self._restored: bool = False   # the latch came from NVS rather than a figure-8 this session
        self._calib_due: int = 1  # countdown to the next CALIB_STAT read (see _poll_calibration)
        try:
            if await self._bus.read_chip_id(self._addr, _REG_CHIP_ID) != _CHIP_ID:
                return False  # not a BNO055 at this address
            await self._bus.write(self._addr, _REG_OPR_MODE, bytes([_MODE_CONFIG]))
            await asyncio.sleep_ms(25)  # mode switch settle
            await self._bus.write(self._addr, _REG_PWR_MODE, bytes([_PWR_NORMAL]))
            await self._restore_profile()  # HERE: the offsets are writable in CONFIG mode only
            await self._bus.write(self._addr, _REG_OPR_MODE, bytes([_MODE_NDOF]))
            await asyncio.sleep_ms(25)  # config -> fusion settle
        except Exception as error:
            print('bno055 :: %r' % error)
            return False
        self._attitude, self._accel = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'attitude', 'accel')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name,
                                             ('heading', 'roll', 'pitch', 'ax', 'ay', 'az'),
                                             decimate_us=self.config.get('telemetry_ms', 0) * 1000)  # 0 -> global rate
        self._ok = True
        return True

    async def sample(self) -> tuple:
        """
        Read the ACC..EUL block and return a FLAT 6-tuple (run() slices it).

        Heading stays float degrees (it feeds the navigation trig island); roll + pitch are fixnum
        CENTIDEGREES (raw·SCALE//16, exact -- 16 LSB/deg, so they feed the fixed-point PID with no float
        conversion); accel (x, y, z) is float g.

        Args:
            (none)

        Returns:
            (heading°, roll_cd, pitch_cd, ax, ay, az): heading in float degrees, roll/pitch in
            centidegree fixnums, and accel in g.
        """
        await self._bus.read_into(self._addr, _REG_DATA, self._buf)
        ax, ay, az = struct.unpack_from('<hhh', self._buf, 0)
        heading, roll, pitch = struct.unpack_from('<hhh', self._buf, _OFF_EUL)
        return (heading * _DEG, roll * SCALE // 16, pitch * SCALE // 16,
                ax * _ACC_G, ay * _ACC_G, az * _ACC_G)

    def _fusion_alive(self, sample: tuple) -> bool:
        """
        Is the fusion engine still COMPUTING, or has it latched a constant?

        A stalled BNO055 fusion core is the worst failure this driver can have, because the channel
        stays FRESH: every staleness guard downstream passes, and the priority-1 attitude backup
        (tasks/attitude.py) -- built for exactly this -- only takes over when the primary goes stale, so
        it would never engage. The PID would be handed a constant attitude and nothing would notice.

        Measured on this bench: the part returns a bit-identical Euler triple indefinitely while its RAW
        accel and gyro keep streaming normally in the same 24-byte block read -- across a power cycle,
        in both NDOF and IMU fusion modes, and on either clock source.

        The judgement is made ONLY WHILE THE PART IS ROTATING, off its own gyro in the same block read.
        A stationary BNO055 legitimately repeats its fused output bit for bit -- measured on a healthy
        replacement sitting on the bench -- so an earlier version of this that keyed on accel dither
        fired on a GOOD sensor, and would have withheld attitude on the pad and pushed the glider onto
        the gyro backup before launch. Rotation is the only condition under which a frozen Euler is
        provably wrong: turn the part and a working fusion MUST move.

        The cost is that a stalled fusion is not detectable while the airframe is still, which is
        correct rather than a gap -- the two are genuinely indistinguishable then, and a stationary
        glider is not being controlled by attitude anyway. In flight there is always rotation.

        Args:
            sample - the flat 6-tuple from sample().

        Returns:
            True while the fusion output is live (or not yet proven dead).
        """
        euler = sample[:3]
        if euler != self._last_euler:
            self._last_euler = euler
            self.strike(False, _STALL_SAMPLES)  # the fusion moved
            self._stalled = False  # ...so clear the latch HERE: returning `not self._stalled` below
            return True            # would otherwise keep reporting stalled forever after a recovery
        # the part's OWN gyro, bytes 12..17 of the block sample() just read (16 LSB/deg/s)
        gx, gy, gz = struct.unpack_from('<hhh', self._buf, _OFF_GYR)
        rotating = abs(gx) + abs(gy) + abs(gz) > _TURNING_LSB  # only then is a frozen euler PROOF
        if self.strike(rotating, _STALL_SAMPLES):
            self._stalled = True  # latch: stays until the fusion moves again (cleared in run())
        return not self._stalled

    async def _poll_calibration(self) -> None:
        """
        Refresh CALIB_STAT at ~1 Hz so the OPERATOR can be told the IMU still needs calibrating.

        NDOF fusion does not converge without motion, and a glider sits still on the pad -- so a
        perfectly healthy part can reach LAUNCH with a frozen attitude, which is what made a working
        module look broken on this bench. That is a pre-flight procedure item, and it only becomes one
        if the board reports it: hence `calibrated` on the operator surfaces (cc_client degraded +
        the `verify` readiness gate).

        MAG is the gate. The magnetometer is the axis that needs the deliberate figure-8; the gyro
        reaches 3 by itself within seconds of sitting still, and the accelerometer wants a few held
        orientations but does not block heading. Decimated to ~1 Hz -- one extra byte off the bus, well
        off the 50 Hz sampling path.

        Args:
            (none)

        Returns:
            None; refreshes self.calibration (sys, gyr, acc, mag) as a side effect.
        """
        self._calib_due -= 1
        if self._calib_due > 0:
            return
        self._calib_due = max(1, 1000 // self._period_ms)
        raw = (await self._bus.read(self._addr, _REG_CALIB_STAT, 1))[0]
        self.calibration_state = (raw >> 6, (raw >> 4) & 3, (raw >> 2) & 3, raw & 3)
        if not self._converged and self.calibration_state[3] >= _MAG_CALIBRATED:
            self._converged = True  # LATCHED here and never cleared -- see calibrated()
            recorder.Recorder.log(self.name, 'magnetometer converged (mag 3)'
                                             ' -- run `calibrate imu_bno055` to keep it across reboots')

    def calibrated(self) -> bool:
        """
        Has the magnetometer EVER converged this session (or been restored from a saved profile)?

        Deliberately a latch, not the live register. CALIB_STAT is the chip's confidence in its RECENT
        magnetometer data, not a record of what it has learned: measured on this bench, mag reaches 3
        during the operator's figure-8 and falls back to 2 within a minute of the airframe sitting
        still, while the learned offsets -- and the heading they produce -- are unchanged. Reading it
        live made the ready gate a coin toss, and an operator who had done the figure-8 correctly was
        told to do it again. The offsets, once learned, do not un-learn.
        """
        return self._converged

    def calibration(self) -> str:
        """The figure-8 instruction while NDOF is unconverged, with the live reading folded in; '' once done."""
        if self.calibration_state is None:
            return 'move the airframe in a slow figure-8 (BNO055 calibration not read yet)'
        sys_, gyr, acc, mag = self.calibration_state
        if self._converged:
            return ''  # the latch, not `mag` -- see calibrated() for why the live value regresses
        return ('move the airframe in a slow FIGURE-8 until mag reads 3 '
                '(now sys %d gyr %d acc %d mag %d)' % (sys_, gyr, acc, mag))

    async def _restore_profile(self) -> None:
        """
        Write a previously saved calibration profile back into the chip (CONFIG mode, at setup).

        Without this every power cycle starts the fusion from nothing, and the magnetometer needs the
        operator's figure-8 again -- on the pad, with the airframe on the rail, which is not a thing
        anyone can do. The BNO055 keeps its learned offsets in a 22-byte block that is meant to be read
        out once and written back on each boot; that is the manufacturer's intended flow and the same
        shape as the pitot tare this board already persists.

        Restoring also SETS the converged latch. The chip rebuilds its own CALIB_STAT confidence over
        the following seconds, so a boot that has valid offsets would otherwise still report
        `needs-calibration` and hold the arm gate shut with nothing for the operator to do about it.

        Best-effort by design, exactly like the pitot tare: no NVS, no saved profile, or a bus error
        simply leaves the chip in its power-on state and the operator is told to do the figure-8.

        Args:
            (none)

        Returns:
            None.
        """
        if _nvs is None:
            return
        buffer = bytearray(_CALIB_BYTES)
        try:
            if _nvs.get_blob(_NVS_CALIB, buffer) != _CALIB_BYTES:
                return  # a short/garbage blob is not a profile -- leave the chip alone
        except Exception:
            return  # never saved on this board yet: the figure-8 is still owed
        try:
            await self._bus.write(self._addr, _REG_CALIB_DATA, bytes(buffer))
            self._converged = True
            self._restored = True
            # print(), not Recorder.log(): setup runs before the recorder task is up (same reason
            # as the sdp810 tare restore and this driver's own setup failures).
            print('bno055 :: calibration profile restored from NVS')
        except Exception as error:
            print('bno055 :: calibration restore failed %r' % error)

    async def calibrate(self) -> str:
        """
        Persist the chip's learned calibration profile, once the operator's figure-8 has landed.

        Overrides the base no-op, which returned None -- the protocol's SUCCESS -- for a device that
        could not calibrate itself. `calibrate imu_bno055` therefore answered "ok" however many times
        it was run, while the magnetometer sat at 0 and nothing on the board had changed. An operator
        following that reply had no way to learn the command was inert.

        So it now does the half the board CAN do. The figure-8 itself is still physical and still the
        operator's: until it converges this reports what is outstanding, as a failure string. After it
        converges this captures the result so no future boot needs one.

        Args:
            (none)

        Returns:
            None once the profile is saved; the outstanding figure-8 instruction otherwise.
        """
        if not self._converged:
            return self.calibration()  # honest: nothing was saved, and here is what is still owed
        if _nvs is None:
            return 'no NVS partition -- calibration holds for this session only'
        try:
            profile = await self._save_profile()
        except Exception as error:
            return 'profile read failed: %s' % error
        try:
            _nvs.set_blob(_NVS_CALIB, profile)
            _nvs.commit()
        except Exception as error:
            return 'profile persist failed: %s' % error
        recorder.Recorder.log(self.name, 'calibration profile saved (%d bytes) -- survives power cycles'
                                         % len(profile))
        return None

    async def _save_profile(self) -> bytes:
        """
        Read the 22-byte calibration block, which the chip exposes in CONFIG mode ONLY.

        Fusion stops for the round trip, so attitude pauses for ~50 ms. That is why this is on the
        operator command and not on the poll path: it is a ground action, and the run loop must never
        drop the attitude channel to bookkeep. NDOF is restored in `finally` so a failed read cannot
        leave the part parked in CONFIG with the fusion off and the channel silently frozen.

        Args:
            (none)

        Returns:
            The profile bytes; raises on a bus error (the caller reports it).
        """
        try:
            await self._bus.write(self._addr, _REG_OPR_MODE, bytes([_MODE_CONFIG]))
            await asyncio.sleep_ms(25)  # fusion -> config settle
            return bytes(await self._bus.read(self._addr, _REG_CALIB_DATA, _CALIB_BYTES))
        finally:
            await self._bus.write(self._addr, _REG_OPR_MODE, bytes([_MODE_NDOF]))
            await asyncio.sleep_ms(25)  # config -> fusion settle

    async def run(self) -> None:
        while True:
            try:
                sample = await self.sample()  # flat 6-tuple (heading°, roll_cd, pitch_cd, ax, ay, az g)
                if self._fusion_alive(sample):
                    self._attitude.push(sample[:3])  # push our channels directly (roll/pitch fixnum)
                    self._stalled = False
                elif not self._stalled:
                    self._stalled = True
                    # STOP publishing attitude so the channel goes stale and the databoard hands over to
                    # the priority-1 backup. Accel keeps flowing -- that half of the part still works.
                    recorder.Recorder.log(self.name, 'fusion STALLED (frozen euler while accel moves)'
                                                     ' -- attitude withheld, backup takes over')
                self._accel.push(sample[3:])  # low-g backup to the ADXL375
                # roll/pitch columns are the RAW centidegree fixnum, not formatted decimals. to_str()
                # built two strings per sample and MEASURED 170 B -- the single largest piece of this
                # driver's sample, and it was FORMATTING, not measurement. At 50 Hz that is 8.5 KB/s of
                # heap that never comes back with GC off in flight. The host tools divide by fixed.SCALE
                # when they render (flight_report already does it for the gyro columns); the board has no
                # business spending heap on decimals. Same fix lsm6dso32 took for its gyro columns --
                # this driver simply never got it.
                self._telemetry.push((sample[0], sample[1], sample[2],
                                      sample[3], sample[4], sample[5]))
                self.note(None)  # healthy pass -> let the next error log afresh
                await self._poll_calibration()
            except Exception as error:
                self.note('bno055 :: read %r', error)  # deduped: a persistent error logs once, not at 50 Hz
            await asyncio.sleep_ms(self._period_ms)

    async def probe(self) -> str:
        """
        On-demand self-test: the chip id reads back, then one fused sample succeeds (each step logged).

        Args:
            (none)

        Returns:
            None on success; a short failure message (also logged) at the first failing step.
        """
        try:
            recorder.Recorder.log(self.name, 'probe: chip id ...')
            chip = await self._bus.read_chip_id(self._addr, _REG_CHIP_ID)
            if chip != _CHIP_ID:
                raise ValueError('BNO055 id 0x%02x != 0x%02x at i2c:%s 0x%02x' % (
                    chip, _CHIP_ID, self.config.get('id'), self._addr))
            recorder.Recorder.log(self.name, 'probe: chip id ok 0x%02x' % chip)
        except Exception as error:
            message = 'chip id: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        try:
            recorder.Recorder.log(self.name, 'probe: sample ...')
            sample = await self.sample()
            recorder.Recorder.log(self.name, 'probe: sample ok heading=%.1f deg' % sample[0])
        except Exception as error:
            message = 'sample: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    async def diagnose(self) -> str:
        """
        Deeper analysis when setup() failed: classify the wire-level fault.

        The bus reads the chip id and classifies the fault (no ack / wrong device / present-but-init), so
        the Controller can fold it into the failure reason and `verify`/`probe` show the 'why', not just
        'absent / miswired?'.

        Args:
            (none)

        Returns:
            A wire-level fault description; a config-fault message when setup never built the bus.
        """
        bus = self._bus  # None until setup builds the transport
        if bus is None:  # setup never built the bus -> a config fault
            return 'no transport -- i2c bus %s undefined in config' % self.config.get('id', 0)
        return await bus.device(self._addr).diagnose(_REG_CHIP_ID, _CHIP_ID)

    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status['attitude_deg'] = self._attitude.value()  # our channels' latest (no hot-path I2C)
        status['accel_g'] = self._accel.value()
        # the operator must see a WITHHELD attitude: the channel simply going quiet looks like a
        # missing sensor, and this says the part is alive with a dead fusion core
        status['fusion_stalled'] = self._stalled
        status['calibration'] = self.calibration_state  # (sys, gyr, acc, mag), the LIVE register
        status['calibrated'] = self.calibrated()        # the latch -- regularly disagrees with mag
        status['calibration_restored'] = self._restored  # latched from NVS, not from a figure-8 here
        return status
