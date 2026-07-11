"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

In-flight reboot recovery (specs/coludo.md "In-flight reboot & warm start"). A mid-air reset
(watchdog, brownout-survivor, crash) must not turn the glider ballistic: the sequencer drops a tiny
BREADCRUMB into NVS at BOOSTING entry (never a VFS file -- a filesystem write locks the scheduler and
wears the data flash; esp32.NVS commits to its own partition in milliseconds) and clears it at DONE.
At boot, main.py restores GLIDING when the breadcrumb AND two physical signals agree -- see
should_restore() for the gate.

Storage layout: `flight` is a bare i32 flag (cheap to flip on the clear path), the payload is ONE JSON
blob (`crumb`) -- full float precision, no per-field key bookkeeping, and a new field is a dict entry
rather than an NVS schema change. The module degrades to no-ops off-board (CPython).
"""

import asyncio
import gc
import json
import time

import controller
import databoard
import inspector

try:
    from esp32 import NVS
    _nvs = NVS('coludo')
except ImportError:  # CPython (host tools / sim): warm start is board-only, everything no-ops
    _nvs = None

try:
    import machine
except ImportError:  # host (CPython): board-only; restore()'s reset-cause read never runs off-board
    machine = None

_BLOB_MAX: int = 512  # read buffer for the crumb blob (the JSON is ~150 B; headroom for new fields)


def save(launch: tuple, zone: tuple, pad_altitude: float, stamp: int) -> bool:
    """
    Drop the breadcrumb (called ONCE at BOOSTING entry, on the rod, before GC goes off).

    Args:
        launch - (lat, lon) of the live fix.
        zone - ((lat, lon) TL, (lat, lon) BR), the landing-zone corners.
        pad_altitude - the baro ABSOLUTE altitude at the pad (m -- NOT the boot-relative elevation, a
            rebooted baro re-zeroes mid-air).
        stamp - RTC epoch seconds.

    Returns:
        True when the breadcrumb was committed; False (never raises) when NVS is absent or full -- a
        failed breadcrumb must not block a launch.
    """
    if _nvs is None:
        return False
    try:
        crumb = {'launch': [launch[0], launch[1]],
                 'zone': [[zone[0][0], zone[0][1]], [zone[1][0], zone[1][1]]],
                 'pad_altitude': pad_altitude, 'stamp': stamp}
        _nvs.set_blob('crumb', json.dumps(crumb))
        _nvs.set_i32('flight', 1)  # the flag LAST: a torn write leaves flight=0 -> cold boot
        _nvs.commit()
        return True
    except OSError:
        return False


def clear() -> None:
    """
    Down the flag (at DONE / after a rejected warm start).

    The blob stays -- the flag alone decides, so the clear is a single fast i32 write.

    Returns:
        None. Clears the NVS `flight` flag; never raises (a failed clear is swallowed).
    """
    if _nvs is None:
        return
    try:
        _nvs.set_i32('flight', 0)
        _nvs.commit()
    except OSError:
        pass


def load():
    """
    Read back the breadcrumb dropped at BOOSTING entry.

    Returns:
        The breadcrumb dict ({launch: [lat, lon], zone: [[TL], [BR]], pad_altitude, stamp}), or None
        when no flight was in progress (flag absent/0) or the blob is missing/torn (-> cold boot).
    """
    if _nvs is None:
        return None
    try:
        if _nvs.get_i32('flight') != 1:
            return None
        buffer = bytearray(_BLOB_MAX)
        length = _nvs.get_blob('crumb', buffer)
        return json.loads(bytes(buffer[:length]))
    except (OSError, ValueError):  # never written / torn / unparsable -> cold boot
        return None


def should_restore(crumb, separated: bool, altitude, cause_is_reset: bool, now_s,
                   min_height_m: float = 15.0, max_age_s: int = 600) -> tuple:
    """
    The warm-start gate -- ALL conditions must agree (defense in depth; any doubt -> cold boot).

      1. a breadcrumb exists AND carries its `pad_altitude` + `stamp` (a torn/partial JSON blob with a
         missing key REFUSES here rather than crashing the boot);
      2. the separation switch reads SEPARATED -- the physical latch no software state can fake
         (post-separation it stays LOW for the whole glide; a stack on the pad reads nested);
      3. the baro ABSOLUTE altitude reads at least `min_height_m` above the breadcrumb's pad -- still
         clearly in the air (None = baro not up in time -> refuse);
      4. `cause_is_reset` -- machine.reset_cause() was WDT/SOFT/HARD. A battery insertion or power
         switch reads PWRON -- exactly what a RECOVERY CREW's hands do to a glider that crash-landed
         on a rise above the pad (where gate 3 alone would pass). A mid-air brownout also reads PWRON
         and stays cold: a browning-out battery cannot be trusted to finish the glide;
      5. `age_s` = `now_s` - crumb stamp is positive and under `max_age_s`. The RTC survives soft/WDT
         resets, so the arithmetic holds exactly when a warm start is legitimate (even an unsynced
         RTC -- continuity matters, not absolute truth); a power cycle restarts the RTC and breaks it
         -> cold. Age is computed HERE from the crumb's stamp, so a missing stamp refuses cleanly.

    Pure function of its inputs (host-testable).

    Args:
        crumb - the breadcrumb dict from load(), or None.
        separated - the separation driver's latch reading (True = separated).
        altitude - the baro ABSOLUTE altitude (m), or None if the baro is not up yet.
        cause_is_reset - True when machine.reset_cause() was WDT/SOFT/HARD (not a power-on).
        now_s - the current time (RTC epoch seconds).
        min_height_m - the minimum height above the pad to accept as airborne (m).
        max_age_s - the maximum breadcrumb age to accept (seconds).

    Returns:
        (restore, reason): restore True with the passing reason when ALL gates agree, else False with
        the first failing reason.
    """
    if crumb is None:
        return False, 'no breadcrumb'
    pad_altitude = crumb.get('pad_altitude')
    stamp = crumb.get('stamp')
    if pad_altitude is None or stamp is None:
        return False, 'breadcrumb missing pad_altitude/stamp -> cold boot'
    if not separated:
        return False, 'separation switch reads nested'
    if altitude is None:
        return False, 'no altitude reading'
    height = altitude - pad_altitude
    if height < min_height_m:
        return False, 'altitude %.0fm above pad < %.0fm' % (height, min_height_m)
    if not cause_is_reset:
        return False, 'power-on boot (human hands), not a reset'
    age_s = now_s - stamp
    if not 0 <= age_s <= max_age_s:
        return False, 'breadcrumb age %ds outside 0..%ds' % (age_s, max_age_s)
    return True, 'airborne %.0fm above pad, reset %ds after boost' % (height, age_s)


async def _await_altitude(parameter, timeout_ms: int = 5000):
    """
    Wait up to `timeout_ms` for the baro task's first ABSOLUTE altitude reading.

    Its setup races the reset, so on a fast reboot the parameter is briefly None.

    Args:
        parameter - the databoard altitude parameter to poll.
        timeout_ms - how long to wait for the first reading (milliseconds).

    Returns:
        The absolute altitude reading (m), or None on timeout.
    """
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    altitude = parameter.value()
    while altitude is None and time.ticks_diff(deadline, time.ticks_ms()) > 0:
        await asyncio.sleep_ms(100)
        altitude = parameter.value()
    return altitude


def _apply_restore(flight, crumb, cfg: dict) -> None:
    """
    Apply a PASSED gate: restore the flight state from the crumb.

    Restore the mission identity + zone from the crumb, rebase the rebooted baros to the crumb's pad
    altitude (their setup re-zeroed mid-air, so `elevation` would read ~0 up there and the landing
    detect would flare immediately), then set GLIDING + armed + the warm-started flag. Controller/
    mission mutations ONLY -- the hardware reads + gc live in restore() -- so this is host-testable
    with stubs. The gate guarantees `pad_altitude`; `launch`/`zone` are guarded (a torn crumb could
    carry pad_altitude+stamp yet drop them).

    Args:
        flight - the controller (its stage / arm / warm_started + find() for the baros).
        crumb - the passed breadcrumb dict.
        cfg - the board config (its `sensors` list names the baros to rebase).

    Returns:
        None. Mutates the mission + baros + controller in place.
    """
    mission_obj = inspector.Inspector.get('mission')
    launch = crumb.get('launch')
    zone = crumb.get('zone')
    if mission_obj is not None and launch and zone:
        mission_obj.update({'latitude': launch[0], 'longitude': launch[1],
                            'zone': [list(zone[0]), list(zone[1])]})
    baro_names = [sensor['name'] for sensor in cfg.get('sensors', [])
                  if sensor.get('driver') in ('icp10111', 'bmp280')]
    for baro in flight.find(baro_names):
        if baro is not None:
            baro.update({'ground': crumb['pad_altitude']})
    flight.set_stage(controller.Stage.GLIDING)
    flight.arm()  # we were armed when the crumb dropped (the sequencer only saves armed flights)
    flight.warm_started = True  # degraded-mode annunciation: this flight was recovered from a reset


async def restore(flight, cfg: dict, log=print) -> bool:
    """
    Warm start (specs/coludo.md "In-flight reboot & warm start") -- was main._restore_flight, moved
    here so main.py stays a thin bring-up.

    A mid-air reset must not turn the glider ballistic: restore GLIDING when the NVS breadcrumb AND two
    physical signals agree -- the separation latch (read via the separation DRIVER, not a raw Pin) and
    the baro absolute altitude clearly above the crumb's pad. Any doubt -> the crumb is cleared and
    this is a normal cold boot.

    Args:
        flight - the controller to move into GLIDING on a passed gate.
        cfg - the board config (warm_start toggle + the sensor list for the baros).
        log - the log sink (defaults to print).

    Returns:
        True when a warm start was applied (-> gliding, armed, GC off); False on a normal cold boot or
        a rejected gate.
    """
    crumb = load()
    if crumb is None:
        return False  # no flight was in progress: the normal boot, zero extra work
    if not cfg.get('warm_start', True):
        clear()
        log('warmstart :: disabled by config')
        return False
    separation = flight.find(['separation'])[0]  # the driver's own latch reading (no second Pin)
    separated = separation.separated() if separation is not None else False
    altitude = await _await_altitude(databoard.Databoard.parameter('altitude'))
    cause = machine.reset_cause()
    cause_is_reset = cause in (machine.WDT_RESET, machine.SOFT_RESET, machine.HARD_RESET)
    allowed, reason = should_restore(crumb, separated, altitude, cause_is_reset, time.time())
    log('warmstart :: gate: %s (reset_cause %d)' % (reason, cause))
    if not allowed:
        clear()  # rejected -> make the NEXT boot unambiguously cold
        return False
    _apply_restore(flight, crumb, cfg)
    gc.collect()  # the sequencer's BOOSTING GC hook was skipped: compact + GC OFF for the descent
    gc.disable()
    log('warmstart :: WARM START -> gliding, armed (%.0fm above pad)'
        % (altitude - crumb['pad_altitude']))
    return True
