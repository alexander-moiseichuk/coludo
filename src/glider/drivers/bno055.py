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

import config
import databoard
import i2cbus
import recorder
import task
from fixed import SCALE, to_str  # attitude roll/pitch -> centidegree fixnum; to_str for float-free telemetry

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
        bus_id = self.config.get('id', 0)
        spec = config.bus(self.controller.config, self.config.get('bus', 'i2c'), bus_id)
        if spec is None:
            return False
        self._bus = i2cbus.get(bus_id, spec)
        self._addr: int = self.config.get('addr', _ADDR)
        self._period_ms: int = self.config.get('period_ms', 20)  # 50 Hz (fusion runs at 100 Hz)
        self._buf = bytearray(24)  # ACC..EUL block
        self._last_euler = None    # fusion-stall detector state (see _fusion_alive)
        self._frozen: int = 0
        self._stalled: bool = False
        try:
            if await self._bus.read_chip_id(self._addr, _REG_CHIP_ID) != _CHIP_ID:
                return False  # not a BNO055 at this address
            await self._bus.write(self._addr, _REG_OPR_MODE, bytes([_MODE_CONFIG]))
            await asyncio.sleep_ms(25)  # mode switch settle
            await self._bus.write(self._addr, _REG_PWR_MODE, bytes([_PWR_NORMAL]))
            await self._bus.write(self._addr, _REG_OPR_MODE, bytes([_MODE_NDOF]))
            await asyncio.sleep_ms(25)  # config -> fusion settle
        except Exception as error:
            print('bno055 :: %r' % error)
            return False
        self._attitude, self._accel = databoard.Databoard.provide(
            self.name, self.config.get('provides', {}), 'attitude', 'accel')
        self._telemetry = recorder.Telemetry('%s.csv' % self.name,
                                             ('heading', 'roll', 'pitch', 'ax', 'ay', 'az'),
                                             decimate_us=self.config.get('telemetry_us', 0))  # 0 -> global rate
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
            self._frozen = 0
            return True
        # the part's OWN gyro, bytes 12..17 of the block sample() just read (16 LSB/deg/s)
        gx, gy, gz = struct.unpack_from('<hhh', self._buf, _OFF_GYR)
        if abs(gx) + abs(gy) + abs(gz) > _TURNING_LSB:  # rotating, yet the fusion has not moved
            self._frozen += 1
        return self._frozen < _STALL_SAMPLES

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
                # roll/pitch are centidegree fixnum -> to_str for a human-readable, float-free CSV column
                self._telemetry.push((sample[0], to_str(sample[1]), to_str(sample[2]),
                                      sample[3], sample[4], sample[5]))
                self.note(None)  # healthy pass -> let the next error log afresh
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
        return status
