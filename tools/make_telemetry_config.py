"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Generate the per-airframe board.config profiles (TMS-7C telemetry-only, TMS-7D full control).

Named for what it produces rather than how the airframe is launched: the sequencer thresholds
below are still sized for the catapult, but the profiles now carry each board's whole identity --
which devices it physically has, its telemetry rates, its slew concurrency, its id.

The firmware defaults are shaped for a rocket motor and would FAIL on a rubber catapult -- not degrade,
fail. Sized for the SMALLEST intended hop, a near-vertical ~3 m toss, so the thresholds stay valid for
anything larger; a bigger launch only ever gives them more margin:

    apogee height       h (the design case)  =  3 m        (10 m at the top of the range)
    release velocity    v_v = sqrt(2*g*h)    =  7.7 m/s    (14.1 m/s)
    time to apogee      v_v / g              =  0.78 s     (1.44 s)
    boost acceleration  v_v^2 / 2s over 1 m  =  3.0 g      (~11 g), lasting ~260 ms (~130 ms)

Sizing for 3 m rather than 10 m moves which threshold is marginal, which is the whole reason to do it.
At 3 m the boost is only **3.0 g against a launch_g of 2.5 -- 20 % of margin**, where a 10 m launch
pulls ~11 g and clears it four times over. launch_g is therefore the one that needs loosening, not the
one that has room to spare.

Against that, the motor defaults break in four separate places:

  * `apogee_arm_ms` 4000 -- the apogee detector (peak tracking included) is blind for 4 s, but apogee
    arrives at ~1.2 s. Apogee would NEVER be detected.
  * `boost_timeout_ms` 12000 -- so the fallback fires 12 s in, long after the airframe has landed. The
    glider would spend its entire flight in BOOSTING with the fins held at the boost attitude.
  * `launch_alt_m` 10.0 -- a baro backup set AT or ABOVE the whole arc, so it can never trip.
  * `apogee_drop_m` 5.0 -- larger than the entire 3 m arc, so even an armed detector could not fire.

`launch_g` drops 2.5 -> 1.5 because the 3 m case only pulls 3.0 g. Erring LOW is deliberate: a false
launch on the ground merely advances the stage and is recoverable, while a missed launch yields no
flight data at all, which is the entire point of 7C/7D. The 40 ms dwell plus the operator arming step
are what keep a carry bump from tripping it, and `launch_alt_m` 1.0 m is an independent second path --
if the accel threshold is somehow missed, the baro still catches the climb well below the 3 m apogee.

`launch_ms` stays 40 ms: the gentler launch actually LENGTHENS the pulse to ~260 ms (lower
acceleration over the same 1 m), so the dwell sits comfortably inside it at either end of the range.

The two shock/rate streams are additionally recorded at full 100 Hz rather than the global 25 Hz --
see _FULL_RATE below.

Usage:
    python3 tools/make_telemetry_config.py          # writes configs/tms7c.config, configs/tms7d.config
Then upload the chosen profile to the board as board.config (via CC) and power-cycle.
"""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'glider'))

import config_default  # noqa: E402 -- needs the path above

_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'configs')

# Sequencer thresholds for a CATAPULT hop, and ONLY a catapult hop. Opt-in per profile via
# launch_mode='catapult'; every one is derived from a ~3 m arc and is actively unsafe on a rocket:
#
#   apogee_arm_ms 200      -- arms the apogee detector DURING the motor burn, whose pressure wave is
#                             exactly what the 4000 ms rocket default exists to sit out. False apogee.
#   boost_timeout_ms 1200  -- the last-resort stage fallback fires 1.2 s in, i.e. mid-burn.
#   flight_timeout_ms 30 s -- a rocket flight is minutes; this aborts it.
#   launch_g 1.5 / launch_ms 40 / launch_alt_m 1.0 / apogee_drop_m 0.5 -- all sized for a 3 m hop, all
#                             loose enough that handling or baro noise can trip them on the pad.
#
# The catapult was a MEASUREMENT instrument for the glide polar (it yielded AIR_QUALITY 5.5, which is a
# property of the airframe and stays), not a launch mode. It was never flown, because it puts 2x or more
# shock through the airframe at release. So these values must never be the default.
_CATAPULT_SEQUENCER: dict = {
    'launch_g': 1.5,          # the 3 m case pulls only 3.0 g; err LOW -- a missed launch costs the flight
    'launch_ms': 40,          # the pulse is ~260 ms at 3 m / ~130 ms at 10 m -- the dwell fits both
    'launch_alt_m': 1.0,      # independent baro path, well below even the 3 m apogee
    'apogee_drop_m': 0.5,     # a third of the 3 m arc; still ~2.5x the baro's ~20 cm real noise
    'apogee_arm_ms': 200,     # apogee lands at 0.78 s -- leaves ~580 ms of tracking before it
    'boost_timeout_ms': 1200,  # last-resort fallback just past the 0.78 s apogee, not 12 s
    'flight_timeout_ms': 30000,  # RSO backstop: a 3 m hop is over in seconds, not the 300 s of a rocket
}

# The launch these profiles are generated FOR. Rocket, for the 2026-09-05 flights of TMS-7/7A/7B/7C/7D.
# Switch to 'catapult' only to regenerate profiles for a bench hop, and switch it back afterwards.
_LAUNCH_MODE: str = 'rocket'

_SERVOS: tuple = ('servo_yaw', 'servo_eleron_left', 'servo_eleron_right')

# Streams recorded at FULL rate, overriding the recorder global.
# 7C/7D exist precisely to capture the launch, separation and impact transients, which are what
# decimation drops. Recording every sample keeps the WAVEFORM (duration, ringing), not just its
# extreme, which is what a shock trace has to show to be worth anything.
#
# This used to be justified by the hop being SHORT ("the leak argument that justifies decimating a 60 s
# rocket flight does not apply"). That argument inverts on the rocket flights these profiles now build
# for, so it was replaced with a measurement: a full HITL capture runs 642 KB over 49 s of flight, about
# 13 KB/s, against the ~92 KB/s a 921600-baud UART carries. That is 14 % utilisation, so a 300 s rocket
# flight streams roughly 4 MB and never approaches the link -- full rate stays, on its own merits.
# 0 = inherit the recorder global, which is itself 0 = NO decimation. This was 10 ms, which made sense
# when the global was 25 Hz -- it RAISED these two streams to 100 Hz. With the global uncapped it does
# the opposite and CAPS them at 100 Hz, so it is now the only thing prorating the shock traces these
# flights exist to capture. Kept as an explicit 0 rather than deleted, because the two streams named
# below are the ones whose rate must never be quietly reduced again.
_FULL_RATE_MS: int = 0
_FULL_RATE: tuple = ('accel_adxl375', 'imu_lsm6dso32')
"""
What TMS-7C does NOT carry, so its config must not expect them.

- power_ina226: 7C flies a small battery->5 V board (~1 A) sized for telemetry alone. There is no
  current-sense shunt on it, so the part is not merely unused, it is absent.
Its LSM6DSO32 is NOT absent: it reads 0x6c after the same two-jumper rework as 7D (SCK to the primary
SCL, MISO to the primary DO -- see _TMS7D_ABSENT below for the netlist errors behind it), so the gyro,
the PID D term and the complementary-filter attitude backup are all live on this airframe too.
"""
_TMS7C_ABSENT: tuple = ('power_ina226',)
"""
What TMS-7D does not carry. It is 7C's list MINUS the INA226, because 7D HAS one and it probes
healthy -- disabling a fitted, working current sensor would cost the energy budget and, more
immediately, the servo probe's closed-loop draw check, which is the only thing that distinguishes a
live servo from a lost PWM pin or an unpowered rail.

Its LSM6DSO32 is now WORKING and therefore absent from this list. It read WHO_AM_I 0x00 until two
netlist errors were patched, both the same mistake -- a signal taken from the module's AUXILIARY row
instead of its primary one. The part has SCX/SDX/CS/DO on a second row beside SCL/SDA/DO/CS on the
first, and the layout picked the clock and the data-out from the wrong side:

    SCK  (GPIO48) -> SCX (aux clock)     should be SCL, bottom row
    MISO (GPIO46) -> aux DO              should be DO,  bottom row

MOSI, CS, INT1, VIN and GND were correct throughout. Two jumpers to the primary pads bring it up at
0x6c on the shipped mode-3/5 MHz bus, alongside the ADXL375 at 0xe5. FIX THE NETLIST FOR v0.2 -- both
built boards need the same rework, and nothing was ever wrong with the part or the soldering.
"""
_TMS7D_ABSENT: tuple = ()


def _profile(name: str, board_id: str, servos: bool, flight: bool, absent: tuple = (),
             launch_mode: str = 'rocket',
             concurrency: int = None, raw_telemetry: bool = False) -> dict:
    """
    Build one catapult profile from the firmware defaults.

    Args:
        name - profile name, recorded in the config so a capture identifies its own provenance.
        board_id - the airframe identity; it reaches the boot log, CC and every capture, so a config
            built for one board cannot quietly fly on another.
        servos - False disables all three surfaces (7C flies as ballast, nothing may deflect).
        flight - False disables the control activity (no PID, no mixer, no fin commands at all).
        absent - devices this airframe does not physically carry, disabled by name. Distinct from
            `servos`/`flight`, which are a POLICY choice about a fitted part; these are simply not
            there, and leaving them enabled costs a failed setup and a failed probe on every boot --
            and `cc arm` refuses on any failed probe.
        concurrency - max fins slewing at once; None keeps the firmware default. See below.
        raw_telemetry - True logs every sample (no decimation, global or per-device).

    Returns:
        The complete config dict, ready to serialise as board.config.
    """
    cfg = config_default.default()
    cfg['name'] = name
    cfg['board']['id'] = board_id
    """
    Slew concurrency follows the POWER BOARD, so it is per-profile and never global.

    7C carries a small battery->5 V board rated ~1 A, and one MG90S alone draws ~1.3 A at the battery
    on a full-throw slew -- three cannot be served. Its surfaces are disabled anyway, so 1 simply
    stands as the safe value if any are ever fitted for a bench check.

    7D keeps the default 3: it flies the ND3A05SD (5 V / 3 A) against a ~2.4 A three-servo peak, and
    capping it there would SERIALISE the fin commands the control loop issues together -- a real loss
    of authority, not a saving. An earlier revision of this generator set 1 for both and would have
    done exactly that.
    """
    if concurrency is not None:
        cfg['fins']['concurrency'] = concurrency

    # sensors and components are SEPARATE top-level lists; the shock/rate streams live under
    # 'sensors', the servos and the flight activity under 'components'
    for sensor in cfg['sensors']:
        if sensor.get('name') in absent:
            sensor['enabled'] = False
            """
            Drop `alert_pin` with the part. The INA226's hardware over-current ALERT is gated on that
            pin resolving, so removing it is what switches the alert off -- NOT `alert_ma: 0`, which
            computes a trip limit of zero and would fire on any current at all. Moot while the device
            is disabled, stated so the intent survives someone re-enabling it.
            """
            sensor.pop('alert_pin', None)
        if sensor.get('name') in _FULL_RATE:
            sensor['telemetry_ms'] = _FULL_RATE_MS
    for component in cfg['components']:
        component_name = component.get('name')
        if component_name == 'sequencer':
            """
            Sequencer thresholds follow launch_mode, and the DEFAULT is rocket.

            This used to apply _CATAPULT_SEQUENCER unconditionally, so every generated profile carried
            3 m-hop thresholds whatever it was going to fly on. Two of them fire during a motor burn
            (apogee_arm_ms 200, boost_timeout_ms 1200), so a rocket flight on a catapult profile does
            not merely degrade -- it advances stages mid-boost.

            Rocket mode deliberately writes NOTHING: the profile inherits config_default's thresholds,
            so there is one definition of the rocket numbers and a profile cannot drift from it.
            """
            if launch_mode == 'catapult':
                component.update(_CATAPULT_SEQUENCER)
            elif launch_mode != 'rocket':
                raise ValueError('unknown launch_mode %r (expected rocket or catapult)' % launch_mode)
        elif component_name in _SERVOS:
            component['enabled'] = servos
        elif component_name in absent:
            component['enabled'] = False
            component.pop('alert_pin', None)  # no part -> no hardware ALERT; see the sensors loop
        elif component_name == 'flight':
            # set EXPLICITLY both ways, never only cleared: the firmware default ships `flight`
            # disabled, so a profile that merely refrains from disabling it produces a 7D that would
            # have flown with no control loop at all. Caught by validating the output instead of
            # trusting it.
            component['enabled'] = flight

    if raw_telemetry:
        """
        NO DECIMATION: one global knob, and every stream inherits it.

        0 at the GLOBAL means no decimation at all: a stream's own 0 inherits the global, the global's
        0 makes the window 0, and a 0 window admits every push. (0 at a STREAM still means "inherit",
        which is why per-device values are removed rather than zeroed.)

        Only the profile is set to 0, not the firmware default `_DEFAULT_TELEMETRY_MS`. That 50 Hz
        default protects every OTHER board -- a control flight has a 100 Hz loop, a finite UART and a
        telemetry path that RAISES on overflow, so uncapping it board-wide is a different decision
        from uncapping a ballast airframe whose entire purpose is data.

        This works only because Telemetry resolves the global AT USE. It used to latch it in
        __init__, and since drivers set up before the recorder task they captured the class default
        and ignored the config entirely -- measured on the board, and it silently held the 100 Hz
        accelerometer at 50. Per-device overrides were the workaround; the fix removed the need.

        Affordable because almost nothing samples above the 50 Hz it replaces: the ADXL375 is the only
        100 Hz source. It buys full-rate baro and pitot through the glide, which is what L/D is made
        of. Telemetry RAISES on overflow by policy, so this spends a little of that margin -- fine on
        7C, which carries no control loop for an exception to endanger.
        """
        cfg['recorder']['telemetry_ms'] = 0
        for device in cfg['sensors'] + cfg['components']:
            device.pop('telemetry_ms', None)  # inherit the global; an override here is a CAP
    return cfg


def main() -> None:
    """Write every board profile to configs/ and report the launch mode they were built for."""
    os.makedirs(_OUT, exist_ok=True)
    for name, board_id, servos, flight, absent, concurrency, raw_telemetry, note in (
        ('tms7c', 'TMS-7C', False, False, _TMS7C_ABSENT, 1, True,
         'telemetry only -- no decimation, servos and control DISABLED, the airframe is ballast'),
        # 7D as it flies the telemetry ladder: same data posture as 7C, but the servos ARE fitted, so
        # they stay enabled and exercisable. Control stays OFF -- fins move only for probe()/CC, never
        # under a PID. concurrency 3 (all three may slew together) needs the flight power board: one
        # MG90S alone draws ~1.3 A at the battery, so three is ~4 A and a 1 A bench supply browns out.
        ('tms7d', 'TMS-7D', True, False, _TMS7D_ABSENT, 3, True,
         'telemetry + servos fitted, no active control -- no decimation'),
        # kept for when the ladder reaches active control; nothing flies it yet
        ('tms7d_control', 'TMS-7D', True, True, (), None, False, 'full active control'),
    ):
        cfg = _profile(name, board_id, servos, flight, absent, _LAUNCH_MODE, concurrency, raw_telemetry)
        path = os.path.join(_OUT, '%s.config' % name)
        with open(path, 'w') as handle:
            json.dump(cfg, handle, indent=1, sort_keys=True)
            handle.write('\n')
        print('%-6s %s' % (name, note))
        print('       -> %s' % os.path.normpath(path))
    print()
    print('launch mode: %s' % _LAUNCH_MODE.upper())
    if _LAUNCH_MODE == 'rocket':
        print('  sequencer thresholds INHERITED from config_default -- no overrides written, so the')
        print('  rocket numbers have exactly one definition and a profile cannot drift from it.')
    else:
        print('  CATAPULT thresholds applied to every profile -- BENCH HOP ONLY, do not fly a motor:')
        for key, value in sorted(_CATAPULT_SEQUENCER.items()):
            print('    %-20s %s' % (key, value))


if __name__ == '__main__':
    main()
