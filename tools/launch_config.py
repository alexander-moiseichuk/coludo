"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Generate a FLIGHT-READY board.config from config_default: the field deployment config that passes the
`verify` readiness gate. It flips exactly what the gate checks -> watchdog ON, flight ON with proposed
gains, radios QUIESCED (policy auto: silent BOOSTING..LANDING, live pre-launch + post-land), fin cap
nominal. The gains are SIM-DERIVED PROPOSALS (the HITL-validated config_hitl values) -- they get you a
config that arms, but MUST be verified on the first real glide; the banner says so.

Usage:
    python3 tools/launch_config.py [-o flight.config]      # write (default: stdout)
    tools/cc.py <board> set-config board @flight.config    # then push it to the board
The board config is only half of a field deployment -- pair it with a launch.config carrying the
site/zone (via `set-config launch` / the CC-less `sites` list); the zone half of the gate is that.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'glider'))

import cc_client
import config as config_mod
import config_default
import config_hitl

_BANNER = """\
================================ FLIGHT-READY CONFIG ================================
 watchdog ON | flight ON | radios quiesced (auto) | fin cap 1.0 -> passes `verify`.
 GAINS ARE SIM-DERIVED PROPOSALS (config_hitl), NOT airframe-tuned. They let the board
 ARM, but VERIFY them on the first real glide before trusting -- save & use, or edit.
 This is the BOARD config; pair it with a launch.config (site/zone) for the full gate.
====================================================================================="""


def _sim_gains() -> dict:
    """
    The HITL-validated flight gains, sourced from config_hitl so the proposal stays in sync.

    Returns:
        The flight component's gains dict from config_hitl.

    Raises:
        RuntimeError - config_hitl has no flight component.
    """
    for component in config_hitl.default()['components']:
        if component['name'] == 'flight':
            return component['gains']
    raise RuntimeError('config_hitl has no flight component')


def flight_ready() -> dict:
    """config_default with the field-flight knobs flipped to the ready state the `verify` gate wants."""
    cfg = config_default.default()
    by_name = {component['name']: component for component in cfg['components']}
    by_name['watchdog']['enabled'] = True         # a wedged flight loop must reboot
    by_name['flight']['enabled'] = True           # the control loop must run
    by_name['flight']['gains'] = _sim_gains()      # proposed (sim) gains -> the loop can act
    cfg['fin_limit_multiplier'] = 1.0             # no bench derating
    cfg['wifi']['policy'] = 'auto'                # quiesce: silent airborne, live on the ground
    return cfg


def main() -> int:
    parser = argparse.ArgumentParser(description='generate a flight-ready board.config')
    parser.add_argument('-o', '--out', help='output file (default: stdout)')
    args = parser.parse_args()

    cfg = flight_ready()
    errors = config_mod.validate(cfg)
    readiness = cc_client._readiness(cfg)  # the board half of the gate (zone comes from launch.config)
    if errors or readiness:
        print('launch_config: generated config is NOT ready: validate=%s readiness=%s'
              % (errors, readiness), file=sys.stderr)
        return 1
    text = json.dumps(cfg, indent=2)
    if args.out:
        open(args.out, 'w').write(text)
        print('wrote %s' % args.out, file=sys.stderr)
    else:
        print(text)
    print(_BANNER, file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
