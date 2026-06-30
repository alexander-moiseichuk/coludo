# drivers/ — HAL drivers: each a @task.driver Task that talks directly to hardware (the status LED,
# the sensors, the servos, ...). load() imports every module so its registration runs; the
# Controller then builds the *enabled* ones from the board config. Adding a driver is dropping a
# file here -- main.py and the Controller never change.

import os


def load() -> None:
    """Import every driver module in this package so its @task.driver registration runs."""
    # List this package's OWN directory, taken from __file__ ('drivers/__init__.py' -> 'drivers'):
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
