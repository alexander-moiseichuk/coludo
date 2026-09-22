"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test for tools/launch_config.py: the generated flight-ready board.config validates
clean AND passes the `verify` readiness gate's board half (watchdog/flight/gains/fin-cap/radios),
while the plain default (bench) config does not. Run by the control suite.
"""

import contextlib
import copy
import io
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'tools'))
import launch_config  # noqa: E402  -- also puts src/glider on the path for config / cc_client


def test():
    import cc_client  # noqa: E402
    import config  # noqa: E402
    import config_default  # noqa: E402

    cfg = launch_config.flight_ready(config_default.default())
    # positive: the generated config is valid and flight-ready (the board half of the gate is clean)
    assert config.validate(cfg) == [], config.validate(cfg)
    assert cc_client._readiness(cfg) == {}, cc_client._readiness(cfg)
    by_name = {component['name']: component for component in cfg['components']}
    assert by_name['watchdog']['enabled'] is True
    assert by_name['flight']['enabled'] is True
    assert by_name['flight']['gains']['roll']  # proposed (sim) gains present -> the loop can act
    assert cfg['wifi']['policy'] == 'auto'  # quiesced, not disabled (live pre-launch / post-land)
    # the REAL knob is nested under `fins` -- the one tasks/flight.py and cc_client.py read. A
    # top-level `fin_limit_multiplier` is read by nobody on the board, and this assertion used to pin
    # that phantom key, so the generator and the host sim agreed with each other and with nothing else.
    assert cfg['fins']['limit_multiplier'] == 1.0
    assert 'fin_limit_multiplier' not in cfg, 'the phantom top-level key must not come back'

    # negative: the plain default (bench) config is NOT flight-ready -- watchdog + flight are off
    assert set(cc_client._readiness(config_default.default())) == {'watchdog', 'flight'}

    print('ok: launch_config -- flight-ready passes validate + readiness; default (bench) is not ready')


def _run(*argv: str) -> tuple:
    """
    Run launch_config.main() under a given command line, capturing what it prints.

    Returns:
        (exit_code, stdout): the code main() returned, or the one argparse exited with.
    """
    out = io.StringIO()
    saved = sys.argv
    sys.argv = ['launch_config.py'] + list(argv)
    try:
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(io.StringIO()):
            code = launch_config.main()
    except SystemExit as stop:
        code = stop.code
    finally:
        sys.argv = saved
    return code, out.getvalue()


def test_base_keeps_the_airframe():
    """
    The output replaces a board's whole config, so it must keep the airframe's identity and wiring.

    Built from config_default it named the bench board (`taster`, layout `auto`) and, pushed as
    documented, renamed the airframe on the hub. Now a base is required and survives, and gains the base
    already carries are never swapped for the sim proposal.
    """
    import config_default  # noqa: E402

    base = config_default.default()
    base['board']['id'] = 'TMS-7X'
    base['board']['layout'] = 'v0.1'
    tuned = {'roll': {'kp': 7}, 'pitch': {'kp': 8}, 'yaw': {'kp': 9}}
    by_name = {component['name']: component for component in base['components']}
    by_name['flight']['gains'] = copy.deepcopy(tuned)
    cfg = launch_config.flight_ready(base)
    assert cfg['board']['id'] == 'TMS-7X' and cfg['board']['layout'] == 'v0.1'
    assert {c['name']: c for c in cfg['components']}['flight']['gains'] == tuned, 'tuned gains replaced'

    # negative: no base at all is refused, rather than silently emitting the bench board's identity
    code, out = _run()
    assert code == 2 and out == '', (code, out)
    # a new airframe comes out of config_default under ITS id, never `taster`
    code, out = _run('--board-id', 'TMS-7G')
    assert code == 0 and json.loads(out)['board']['id'] == 'TMS-7G', code
    # an airframe's own file: the id and layout it carries are the ones that come out
    with tempfile.NamedTemporaryFile('w', suffix='.config', delete=False) as handle:
        json.dump(base, handle)
    try:
        code, out = _run('--base', handle.name)
    finally:
        os.remove(handle.name)
    assert code == 0 and json.loads(out)['board']['id'] == 'TMS-7X', code
    print('ok: launch_config -- a base airframe keeps its id, layout and gains; no base is refused')


test()
test_base_keeps_the_airframe()
