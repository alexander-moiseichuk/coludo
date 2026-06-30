# tasks/ — higher-level subsystem tasks (the Recorder adapter, board health, Wi-Fi, the CC link,
# and later fusion / control / navigation): @task.activity Tasks that orchestrate or process rather
# than drive a single device. load() imports every module so its registration runs; the Controller
# then builds the *enabled* ones from the board config. Adding a task is dropping a file here.

import os


def load() -> None:
    """Import every task module in this package so its @task.activity registration runs."""
    # List this package's OWN directory, taken from __file__ ('tasks/__init__.py' -> 'tasks'):
    # __name__ is a DOTTED module name that breaks os.listdir once the package is nested, and os.path
    # is absent on MicroPython (so os.path.dirname is not an option).
    count = 0
    for entry in os.listdir(__file__.rsplit('/', 1)[0]):
        if entry.startswith('__') or not (entry.endswith('.py') or entry.endswith('.mpy')):
            continue  # skip __init__ / __pycache__ / any non-module entry (a dir would import as junk)
        name = entry.rsplit('.', 1)[0]  # strip .py / .mpy
        print('%s: load %s' % (__name__, name))  # trace per import -> the LAST line printed names
        __import__('%s.%s' % (__name__, name))    # whatever module hangs or crashes during boot
        count += 1
    if not count:  # nothing discovered (wrong CWD, or a frozen build os.listdir cannot see) -> fail loud
        raise RuntimeError('%s.load(): no modules found in %s' % (__name__, __file__))
