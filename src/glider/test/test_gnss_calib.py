"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for tasks/gnss_calib.py: averaging the reported ground velocity while stationary on the
pad recovers the GNSS drift velocity, freezing at launch locks it, and too few samples leaves it 0
(untrusted). Pure logic driven off stub databoard channels. Run by `make test`.
"""

import asyncio
import math

import config_default
import databoard
from controller import Stage
from tasks import gnss_calib


class _StubController:
    def __init__(self):
        self.config = config_default.default()
        self.stage = Stage.SETTING


async def amain():
    ctrl = _StubController()
    speed = databoard.Databoard.provide('sim', {'speed': {'priority': 0, 'timeout_ms': 5000}}, 'speed')
    course = databoard.Databoard.provide('sim', {'course': {'priority': 0, 'timeout_ms': 5000}}, 'course')

    # a stationary receiver drifting 0.4 m/s toward the EAST (course 90) -> the calibration recovers it
    calib = gnss_calib.GnssCalib('gnss_calib', {'min_samples': 3}, ctrl)
    assert await calib.setup() is True
    assert calib.drift() == (0.0, 0.0)  # nothing frozen yet
    speed.push(0.4)
    course.push(90.0)
    for _ in range(10):
        calib._sample()
    calib._freeze()
    east, north = calib.drift()
    assert abs(east - 0.4) < 0.02 and abs(north) < 0.02, (east, north)
    assert calib._frozen and calib._count == 10

    # too few samples (a fast arm / no pad fix) -> the mean is untrusted, drift stays 0
    sparse = gnss_calib.GnssCalib('gnss_calib', {'min_samples': 100}, ctrl)
    assert await sparse.setup() is True
    speed.push(0.4)
    course.push(90.0)
    sparse._sample()
    sparse._sample()
    sparse._freeze()
    assert sparse.drift() == (0.0, 0.0)  # below min_samples -> not trusted

    # a diagonal drift (0.3 E, 0.3 N -> course 45, speed ~0.42) decomposes back to its components
    diag = gnss_calib.GnssCalib('gnss_calib', {'min_samples': 1}, ctrl)
    assert await diag.setup() is True
    speed.push(math.sqrt(0.18))  # |(0.3, 0.3)|
    course.push(45.0)
    for _ in range(5):
        diag._sample()
    diag._freeze()
    east, north = diag.drift()
    assert abs(east - 0.3) < 0.02 and abs(north - 0.3) < 0.02, (east, north)

    print('ok: gnss_calib -- pad drift recovered from stationary velocity, frozen at launch, '
          'sparse samples untrusted')


asyncio.run(amain())
