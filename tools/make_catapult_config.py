"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Generate the CATAPULT board.config profiles (TMS-7C telemetry-only, TMS-7D full control).

The firmware defaults are shaped for a rocket motor and would FAIL on a rubber catapult -- not degrade,
fail. Worked from the catapult's own geometry (~1 m of boost, launched at 45 deg, 15-20 m/s at release):

    vertical component  v_v = v * sin45      = 10.6 .. 14.1 m/s
    apogee height       v_v^2 / 2g           =  5.7 .. 10.2 m
    time to apogee      v_v / g              =  1.08 .. 1.44 s
    boost acceleration  v^2 / 2s over 1 m    = ~11 g, lasting ~130 ms

Against that, the motor defaults break in four separate places:

  * `apogee_arm_ms` 4000 -- the apogee detector (peak tracking included) is blind for 4 s, but apogee
    arrives at ~1.2 s. Apogee would NEVER be detected.
  * `boost_timeout_ms` 12000 -- so the fallback fires 12 s in, long after the airframe has landed. The
    glider would spend its entire flight in BOOSTING with the fins held at the boost attitude.
  * `launch_alt_m` 10.0 -- a baro backup set AT or ABOVE the whole arc, so it can never trip.
  * `apogee_drop_m` 5.0 -- most of the total arc height, so even an armed detector would be marginal.

`launch_g` 2.5 stays: the catapult pulls ~11 g, four times the threshold, so it is not the marginal
one. `launch_ms` drops to 40 ms because the pulse only lasts ~130 ms, and the dwell must complete
comfortably inside it rather than racing it.

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
    'launch_g': 2.5,          # the catapult pulls ~11 g -- unchanged, this one has margin to spare
    'launch_ms': 40,          # the ~130 ms pulse must contain the dwell, not race it
    'launch_alt_m': 3.0,      # baro backup BELOW the ~6-10 m arc so it can actually trip
    'apogee_drop_m': 1.0,     # the whole arc is 5-10 m; 5 m was most of it
    'apogee_arm_ms': 300,     # apogee lands at ~1.2 s -- 4000 meant it was never detected
    'boost_timeout_ms': 1500,  # last-resort fallback just past the expected apogee, not 12 s
    'flight_timeout_ms': 60000,  # RSO backstop: a catapult hop is seconds, not the 300 s of a rocket
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
