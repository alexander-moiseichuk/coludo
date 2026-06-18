# tasks/ — higher-level subsystem tasks (the Recorder adapter, board health, and later fusion /
# control / navigation): @task.driver Tasks that orchestrate or process rather than drive a single
# device. load() imports every module so its registration runs; the Controller then builds the
# *enabled* ones from the board config. Adding a task is dropping a file here.

import os


def load() -> None:
    """Import every task module in this package so its @task.driver registration runs."""
    for entry in os.listdir(__name__):
        name = entry.rsplit('.', 1)[0]  # strip .py / .mpy
        if name and name != '__init__':
            __import__('%s.%s' % (__name__, name))
