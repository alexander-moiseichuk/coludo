"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Host (CPython) test: the generated pin doc (doc/waveshare_esp32p4_pins.md) is IN SYNC with
config_default -- `gen_pinmap.py --check` must pass. So a bus/pin change that forgot to regenerate
the doc fails CI here, instead of leaving a stale doc that once nearly sent a solder to phantom pins.
"""

import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..', '..')


def test():
    result = subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'gen_pinmap.py'), '--check'],
                            capture_output=True, text=True)
    assert result.returncode == 0, (
        'doc/waveshare_esp32p4_pins.md is STALE -- run: python3 tools/gen_pinmap.py\n'
        + result.stdout + result.stderr)
    print('ok: gen_pinmap --check -- the pin doc is in sync with config_default')


test()
