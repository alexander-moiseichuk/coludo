# drivers/ — HAL drivers: each a @task.driver Task that talks directly to hardware (the status LED,
# the sensors, the servos, ...). load() imports every module so its registration runs; the
# Controller then builds the *enabled* ones from the board config. Adding a driver is dropping a
# file here -- main.py and the Controller never change.

import os


def load() -> None:
    """Import every driver module in this package so its @task.driver registration runs."""
    for entry in os.listdir(__name__):
        name = entry.rsplit('.', 1)[0]  # strip .py / .mpy
        if name and name != '__init__':
            __import__('%s.%s' % (__name__, name))
