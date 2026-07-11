"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for tools/launch_config.py: the generated flight-ready board.config validates
clean AND passes the `verify` readiness gate's board half (watchdog/flight/gains/fin-cap/radios),
while the plain default (bench) config does not. Run by the control suite.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'tools'))
import launch_config  # noqa: E402  -- also puts src/glider on the path for config / cc_client


def test():
    import cc_client  # noqa: E402
    import config  # noqa: E402
    import config_default  # noqa: E402

    cfg = launch_config.flight_ready()
    # positive: the generated config is valid and flight-ready (the board half of the gate is clean)
    assert config.validate(cfg) == [], config.validate(cfg)
    assert cc_client._readiness(cfg) == {}, cc_client._readiness(cfg)
    by_name = {component['name']: component for component in cfg['components']}
    assert by_name['watchdog']['enabled'] is True
    assert by_name['flight']['enabled'] is True
    assert by_name['flight']['gains']['roll']  # proposed (sim) gains present -> the loop can act
    assert cfg['wifi']['policy'] == 'auto'  # quiesced, not disabled (live pre-launch / post-land)
    assert cfg['fin_limit_multiplier'] == 1.0

    # negative: the plain default (bench) config is NOT flight-ready -- watchdog + flight are off
    assert set(cc_client._readiness(config_default.default())) == {'watchdog', 'flight'}

    print('ok: launch_config -- flight-ready passes validate + readiness; default (bench) is not ready')


test()
