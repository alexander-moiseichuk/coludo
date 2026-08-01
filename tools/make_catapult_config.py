"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Generate the CATAPULT board.config profiles (TMS-7C telemetry-only, TMS-7D full control).

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

Usage:
    python3 tools/make_catapult_config.py          # writes configs/tms7c.config, configs/tms7d.config
Then upload the chosen profile to the board as board.config (via CC) and power-cycle.
"""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'glider'))

import config_default  # noqa: E402 -- needs the path above

_OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'configs')

# Sequencer thresholds every catapult profile shares; see the module docstring for the derivation.
_CATAPULT_SEQUENCER: dict = {
    'launch_g': 1.5,          # the 3 m case pulls only 3.0 g; err LOW -- a missed launch costs the flight
    'launch_ms': 40,          # the pulse is ~260 ms at 3 m / ~130 ms at 10 m -- the dwell fits both
    'launch_alt_m': 1.0,      # independent baro path, well below even the 3 m apogee
    'apogee_drop_m': 0.5,     # a third of the 3 m arc; still ~2.5x the baro's ~20 cm real noise
    'apogee_arm_ms': 200,     # apogee lands at 0.78 s -- leaves ~580 ms of tracking before it
    'boost_timeout_ms': 1200,  # last-resort fallback just past the 0.78 s apogee, not 12 s
    'flight_timeout_ms': 30000,  # RSO backstop: a 3 m hop is over in seconds, not the 300 s of a rocket
}

_SERVOS: tuple = ('servo_yaw', 'servo_eleron_left', 'servo_eleron_right')


def _profile(name: str, servos: bool, flight: bool) -> dict:
    """
    Build one catapult profile from the firmware defaults.

    Args:
        name - profile name, recorded in the config so a capture identifies its own provenance.
        servos - False disables all three surfaces (7C flies as ballast, nothing may deflect).
        flight - False disables the control activity (no PID, no mixer, no fin commands at all).

    Returns:
        The complete config dict, ready to serialise as board.config.
    """
    cfg = config_default.default()
    cfg['name'] = name
    for component in cfg['components']:
        component_name = component.get('name')
        if component_name == 'sequencer':
            component.update(_CATAPULT_SEQUENCER)
        elif component_name in _SERVOS:
            component['enabled'] = servos
        elif component_name == 'flight':
            # set EXPLICITLY both ways, never only cleared: the firmware default ships `flight`
            # disabled, so a profile that merely refrains from disabling it produces a 7D that would
            # have flown with no control loop at all. Caught by validating the output instead of
            # trusting it.
            component['enabled'] = flight
    return cfg


def main() -> None:
    """Write both catapult profiles to configs/ and report what differs from the defaults."""
    os.makedirs(_OUT, exist_ok=True)
    for name, servos, flight, note in (
        ('tms7c', False, False, 'telemetry only -- servos and control DISABLED, the airframe is ballast'),
        ('tms7d', True, True, 'full active control'),
    ):
        cfg = _profile(name, servos, flight)
        path = os.path.join(_OUT, '%s.config' % name)
        with open(path, 'w') as handle:
            json.dump(cfg, handle, indent=1, sort_keys=True)
            handle.write('\n')
        print('%-6s %s' % (name, note))
        print('       -> %s' % os.path.normpath(path))
    print()
    print('sequencer thresholds applied to both:')
    for key, value in sorted(_CATAPULT_SEQUENCER.items()):
        print('  %-20s %s' % (key, value))


if __name__ == '__main__':
    main()
