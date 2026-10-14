"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The `fake` board's own process: plan B's dispatcher (src/glider/cc_client.py) booted on CPython through the
golden shims (src/avionics/golden/shims.py), answering request lines over stdin/stdout. transports.Fake
starts it; nothing else should.

A process of its own because shims.install() is process-wide and cannot be undone: time.time() counts from
2000 on a virtual clock, gmtime() returns MicroPython's tuples, and the board-only module names resolve to
stubs. The runner, the hub transport and every test around them keep the real clock. Plan B's registries
(the Inspector, the Databoard, the Recorder's tees) are class state too, so each fake board is one process
and one plan B.

The boot is the golden cc_dispatch fixture's, which is main.py's minus the hardware: config.load() with no
board.config (the built-in default, source 'default'), the boot counted, the layout resolved (a host scan
finds nothing: 'undecided'), the Mission registered with no launch.config (the HPRC default), and plan B's own
Controller over that config. The Controller is never set up, so it runs no tasks and records no failures:
its stage, hold, arm gate and disarm are plan B's, and nothing it answers depends on hardware. The process
works inside a scratch directory, so every file plan B writes (board.config, launch.config, an OTA staging
file) lands there and is gone when it exits. Its reset is intercepted and counted, and it starts a new boot as
far as whoami can tell: the boot id advances, as main.py's NVS boot counter would on a board. Nothing else
restarts, but the runner's proof of a reboot (a whoami from a new boot) then holds on plan B as on a board.

Protocol, one JSON object per line each way; EOF on stdin ends the process:
  (on start)              -> {"ready": true}
  {"line": "<request>"}   -> {"reply": "<reply line>" | null, "reboots": n}
  {"await_reboot": true}  -> {"reply": null, "reboots": n}, once a deferred reset has had time to fire
Plan B's own prints go to stderr: stdout carries the protocol alone.
"""

import asyncio
import contextlib
import json
import os
import sys
import tempfile

_GOLDEN: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'avionics', 'golden')
_BOOT_ID: int = 1  # stands in for main.py's NVS boot counter, as the golden cc_dispatch builder does
_RESET_WAIT_S: float = 0.3  # plan B resets 200 ms after its reboot `ok`; this lets that task run


def _boot(scratch: str, on_reboot: object) -> object:
    """
    Boot plan B as main.py does, minus the hardware (the Controller is built, never set up).

    Args:
        scratch - the directory plan B's files live in.
        on_reboot - what plan B's reboot handler calls instead of machine.reset().

    Returns:
        The wired cc_client.Dispatcher.
    """
    sys.path.insert(0, _GOLDEN)
    import shims

    shims.install()
    import cc_client
    import config as config_module
    import controller as controller_module
    import layout
    import mission
    import recorder

    board_path = os.path.join(scratch, 'board.config')
    configuration, source, _errors = config_module.load(board_path)
    config_module.BOOT_SOURCE = source
    recorder.Recorder.boot_id = _BOOT_ID
    layout.resolve(configuration, log=lambda line: None)
    field = config_module.device(configuration, name='field') or {}
    mission.Mission(path=os.path.join(scratch, 'launch.config'), max_range_m=field.get('max_range_m', 200))
    controller = controller_module.Controller(configuration)
    return cc_client.create_dispatcher(configuration, controller=controller, on_reboot=on_reboot,
                                       config_path=board_path)


def main() -> None:
    """
    Boot, announce ready, then answer requests until stdin closes.

    Returns:
        None; exits when the parent closes the pipe.
    """
    protocol = sys.stdout
    reboots = 0

    def reset() -> None:
        """machine.reset(), intercepted: counted, the boot id advanced, and plan B keeps running."""
        import recorder  # plan B's, loaded by _boot() before any reset can fire

        nonlocal reboots
        reboots += 1
        recorder.Recorder.boot_id += 1

    def answer(document: dict) -> None:
        """Write one protocol line to the parent."""
        protocol.write(json.dumps(document) + '\n')
        protocol.flush()

    home = os.getcwd()
    with tempfile.TemporaryDirectory(prefix='cc_contract_') as scratch, contextlib.redirect_stdout(sys.stderr):
        os.chdir(scratch)
        try:
            dispatcher = _boot(scratch, reset)
            loop = asyncio.new_event_loop()  # one loop for the run: a reboot's deferred reset outlives its reply
            answer({'ready': True})
            for text in sys.stdin:
                request = json.loads(text)
                reply = None
                if 'line' in request:
                    reply = loop.run_until_complete(dispatcher.handle(request['line']))
                else:
                    loop.run_until_complete(asyncio.sleep(_RESET_WAIT_S))
                answer({'reply': reply, 'reboots': reboots})
        finally:
            os.chdir(home)  # leave the scratch directory before it is removed


if __name__ == '__main__':
    main()
