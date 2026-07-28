"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

HITL simulator allocation bench (run ON the board: `make bench-hitl`, or `mpremote run bench_hitl.py`).

Answers one question: how much of an on-board HITL run's GC-off leak is injected BY THE SIMULATOR and
therefore does NOT exist in a real flight. The sim is float physics -- every intermediate is a boxed
float on MicroPython -- so it is the standing suspect whenever a HITL capture reports a leak.

It prices the real `tasks/hitl.py` pieces with GC off (`gc.mem_free()` deltas over N reps, the same
method as bench_flight.py): the Body sub-step (boost + glide), `_publish()`, and the `sensors()` dict
the inspector builds.

These are per-call figures measured in ISOLATION, so they are an UPPER bound on the flight cost -- a
tight bench loop takes branches a real flight skips. The share reported at the end therefore comes from
a flown A/B instead (see the table below), not from multiplying these by a rate; the two are printed
side by side so a drift between them is visible.

Needs the firmware deployed (`tools/deploy.sh` or `make test` first). Results live in
doc/sims/TMS-7-physics_refresh/README.md.
"""

import asyncio
import gc

import config_hitl
import recorder
from controller import Stage
from tasks import hitl

_REPS = 2000        # reps per measurement -- big enough that the per-rep byte figure is stable
_FREE_AT_BOOT = 32_670_000  # heap free at the start of a capture (bytes) -> time-to-OOM
"""
FLOWN A/B (F15, noise 0.05, calm, 285 g, ~56 s airborne; least-squares over health.csv mem_free, taken
to the minimum because the GC comes back at LANDING and free RISES again). Each arm changes exactly one
rate, so the difference IS that term -- this is the evidence for the split, not the per-call bench above.

  arm                             leak      what it isolates
  inject_hz 50 (default)      224, 255 KB/s baseline (two runs -- the spread is real, ~±16)
  inject_hz 10                193, 194 KB/s -40 Hz of _publish  -> 46 KB/s, ~1150 B per publish
  inject_hz 10 + sim_hz 25         168 KB/s -25 Hz of glide_step -> 26 KB/s, ~1040 B per sub-step

Extrapolated to the default rates: publish 50 Hz ~58 KB/s + physics 50 Hz ~52 KB/s = ~110 KB/s of the
~240 KB/s baseline. NOTE the first sim_hz attempt measured nothing because `hitl_run` imports
`config_hitl` FROM THE BOARD -- editing the host copy without deploying changes nothing.
"""
_BASELINE_BPS = 240000      # mean of the inject_hz-50 arms (bytes/s)
_SIM_SHARE_BPS = 110000     # publish + physics, from the A/B above -- absent in a real flight


class FakeFin:
    """A servo stand-in: the sim reads `.angle` back, which is all `_fin_angles()` needs."""

    angle = 95


class FakeUart:
    """A drain-able sink so the Recorder accepts telemetry without a real UART."""

    async def drain(self) -> None:
        pass

    def write(self, data) -> int:
        return len(data)


class Ctrl:
    """The minimal controller surface the Hitl task touches: config, stage, find()."""

    config = config_hitl.default()
    stage = Stage.GLIDING
    armed = True

    def find(self, names: list) -> list:
        return [FakeFin() for _ in names]


def _cost(label: str, call, reps: int = _REPS) -> float:
    """
    Bytes allocated per call, measured with the GC off exactly as the airborne firmware runs.

    Args:
        label: what is being priced (printed).
        call: the zero-argument callable to measure.
        reps: how many times to call it.

    Returns:
        Bytes per call; prints the line as a side effect.
    """
    gc.collect()
    gc.disable()
    before = gc.mem_free()
    for _ in range(reps):
        call()
    used = before - gc.mem_free()
    gc.enable()
    gc.collect()
    per_call = used / reps
    print('  %-28s: %7.1f B/call' % (label, per_call))
    return per_call


def _rows(sim) -> None:
    """The seven telemetry pushes _publish makes, with representative rows and the same decimation."""
    sim._tlm_accel.push((0.1, 0.2, 1.02))
    sim._tlm_imu.push((123.4, 5.6, -3.2))
    sim._tlm_gyro.push((0.1, 0.2, 1.02, 2.1, -1.4, 0.6))
    sim._tlm_baro.push((212.34, 21.0, 100000, 210.12))
    sim._tlm_laser.push((0.512,))
    sim._tlm_fins.push((95, 87, 91))
    sim._tlm_gnss.push(('25.514379', '-80.391795', 28.0, 123.4))


def _boost(body) -> None:
    """One boost sub-step, held at altitude so the measurement stays in one regime."""
    body.alt = 200.0
    body.boost_step(0.02, 14.4, 0.0, 0.0)


def _glide(body) -> None:
    """One glide sub-step, held airborne and off the stall so every branch of the step runs."""
    body.alt = 200.0
    body.speed = 14.0
    body.glide_step(0.02, 5.0, 2.0, 1.0)


def main() -> None:
    """Price every allocating piece of the HITL sim and report its share of a real capture's leak."""
    # a ring big enough that the un-drained pushes fit -- nothing consumes it in the bench, and an
    # overflow RAISES (the telemetry error policy), which would abort the measurement mid-way.
    settings = dict(Ctrl.config)
    settings['recorder'] = dict(settings.get('recorder', {}), tlm_capacity=8192)
    recorder.Recorder.setup(settings, FakeUart())
    sim = hitl.Hitl('hitl', {c['name']: c for c in Ctrl.config['components']}['hitl'], Ctrl())
    asyncio.run(sim.setup())
    body = sim._body

    print('\nHITL simulator allocation (GC off, %d reps):' % _REPS)
    _cost('baseline (empty call)', lambda: None)     # the harness itself must price at 0
    boost = _cost('Body.boost_step', lambda: _boost(body))
    body.begin_glide()
    glide = _cost('Body.glide_step', lambda: _glide(body))
    publish = _cost('_publish (whole)', sim._publish)
    """
    _publish also pushes seven telemetry rows, but each stream DECIMATES to record_hz -- in this tight
    loop wall time barely advances, so almost none of them emit and the figure above is essentially pure
    sim work (noise math, boxed-float intermediates, databoard pushes). Priced separately to show that,
    and because the telemetry rows are the one part of _publish a real flight ALSO pays: the same streams
    come off the real drivers. Their real cost is per-emitted-row (see bench_emitters), not per call.
    """
    telemetry = _cost('  of which: telemetry (decimated)', lambda: _rows(sim))
    sensors = _cost('Body.sensors (inspector)', body.sensors, 500)

    inject_hz = sim._inject_hz
    sim_hz = sim._sim_hz
    isolated = publish * inject_hz + glide * sim_hz          # what the bench alone would predict
    print('\n  config: sim_hz %d (integration), inject_hz %d (publish)' % (sim_hz, inject_hz))
    print('  telemetry inside _publish   : %5.1f B/call -- decimated to record_hz, so almost none of'
          % telemetry)
    print('     the 1630 B is telemetry; a real flight pays those rows from the real drivers instead.')
    print('\n  sim cost at these rates:')
    print('    bench, isolated (upper bound) : %3.0f KB/s' % (isolated / 1000.0))
    print('    FLOWN A/B (the real figure)   : %3.0f KB/s   <- use this one' % (_SIM_SHARE_BPS / 1000.0))
    real = _BASELINE_BPS - _SIM_SHARE_BPS
    print('\n  against the %.0f KB/s flown baseline:' % (_BASELINE_BPS / 1000.0))
    print('    sim-injected share        : %.0f %%  -- no real flight runs any of it'
          % (100.0 * _SIM_SHARE_BPS / _BASELINE_BPS))
    print('    REAL-FLIGHT leak estimate : %3.0f KB/s -> OOM ~%.0f s from %.1f MB free'
          % (real / 1000.0, _FREE_AT_BOOT / real, _FREE_AT_BOOT / 1e6))
    print('\n  boost phase (burn only)     : %7.0f B/s (boost_step %.0f B)'
          % (publish * inject_hz + boost * sim_hz, boost))
    print('  (sensors() %.0f B/call is inspector-only, ~0.5 Hz -> negligible)\n' % sensors)


main()
