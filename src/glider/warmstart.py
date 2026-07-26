"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

In-flight reboot recovery (specs/coludo.md "In-flight reboot & warm start"). A mid-air reset
(watchdog, brownout-survivor, crash) must not turn the glider ballistic: the Checkpoint task keeps a
tiny CRUMB in NVS (never a VFS file -- a filesystem write locks the scheduler and wears the data
flash; esp32.NVS commits to its own partition in milliseconds) carrying the live flight state. At
boot, main.py restores the SAVED stage when the crumb and the per-stage physical signals agree -- see
should_restore() for the gate -- and the normal detectors (separation / apogee / landing) re-evaluate
from there.

The Checkpoint task writes the crumb every `checkpoint_s` while airborne (BOOSTING/GLIDING/LANDING;
floored at 1 s) and once on entering each stage, but ONLY for an ARMED flight (a disarmed passive
flight must never warm-start into an armed stage); it always writes the telemetry/log timeline.
Recovery is stage-aware: SETTING recovers as a plain cold boot, DONE stays DONE (a landed glider never
re-enters the flight sequence), and the passive glide stages must show the separation latch.

Storage layout: the `stage` i32 IS the flag (Stage.NULL = 0 -> cold, non-zero -> recover it -- no
separate key), the payload is ONE JSON blob (`crumb`) -- full float precision, no per-field key
bookkeeping, and a new field is a dict entry rather than an NVS schema change. The module degrades to
no-ops off-board (CPython).
"""

import asyncio
import gc
import json
import time

import config
import controller
import databoard
import inspector
import recorder
import task

try:
    from esp32 import NVS
    _nvs = NVS('coludo')
except ImportError:  # CPython (host tools / sim): warm start is board-only, everything no-ops
    _nvs = None

try:
    import machine
except ImportError:  # host (CPython): board-only; restore()'s reset-cause read never runs off-board
    machine = None

try:
    from micropython import const
except ImportError:  # CPython (host tools / sim)
    def const(value):
        return value

_BLOB_MAX: int = const(512)  # read buffer for the crumb blob (the JSON is ~150 B; headroom for new fields)
_MAX_AGE_S: int = const(600)  # max crumb age (s): the DONE operator-wait dwell (~10 min). Airborne
#                               crumbs re-stamp every checkpoint so they arrive reboot-fresh; this bounds
#                               the post-landing DONE recovery and rejects a stale prior-session crumb


def save(crumb: dict) -> bool:
    """
    Commit the checkpoint crumb to NVS: the blob, then the stage i32 LAST.

    The Checkpoint task builds the crumb (the live flight state + the BOOSTING-captured identity);
    this is the raw write. There is no separate flag -- the `stage` i32 IS the flag via the Stage.NULL
    = 0 sentinel (0 -> cold, non-zero -> recover it), and it is written last so a torn write can never
    leave a live stage pointing at a half-written blob.

    Args:
        crumb - the checkpoint dict: stage / armed / altitude / speed / ticks_ms / stamp, plus the
            launch / zone / pad_altitude frozen at BOOSTING.

    Returns:
        True when committed; False (never raises) when NVS is absent or full -- a failed checkpoint
        must never block the flight.
    """
    if _nvs is None:
        return False
    try:
        _nvs.set_blob('crumb', json.dumps(crumb))
        # the stage i32 IS the flag (Stage.NULL=0 -> cold, non-zero -> recover it); written LAST so a
        # torn write leaves the previous stage, never a new stage pointing at a half-written blob.
        _nvs.set_i32('stage', crumb['stage'])
        _nvs.commit()
        return True
    except OSError:
        return False


def clear() -> None:
    """
    Reset the stage i32 to Stage.NULL = 0 (after a rejected warm start).

    The blob stays -- the stage i32 alone decides, so the clear is a single fast write. Cold-boots the
    next start unambiguously (a fresh power-on already re-writes SETTING; this makes a rejected crumb
    stop pointing at a stale flight).

    Returns:
        None. Zeroes the NVS `stage`; never raises (a failed clear is swallowed).
    """
    if _nvs is None:
        return
    try:
        _nvs.set_i32('stage', controller.Stage.NULL)
        _nvs.commit()
    except OSError:
        pass


def load():
    """
    Read back the last checkpoint crumb (the stage i32 gates it, then the blob fills in).

    Returns:
        The crumb dict (the blob fields + `stage` from the authoritative i32), or None when no flight
        was checkpointed (stage absent / Stage.NULL) or the blob is missing/torn (-> cold boot).
    """
    if _nvs is None:
        return None
    try:
        stage = _nvs.get_i32('stage')
        if stage == controller.Stage.NULL:
            return None
        buffer = bytearray(_BLOB_MAX)
        length = _nvs.get_blob('crumb', buffer)
        crumb = json.loads(bytes(buffer[:length]))
        crumb['stage'] = stage  # the i32 is authoritative (a torn write can lag the blob's own copy)
        return crumb
    except (OSError, ValueError):  # never written / torn / unparsable -> cold boot
        return None


def should_restore(crumb, separated: bool, cause_is_reset: bool, now_s) -> tuple:
    """
    The warm-start gate: a legitimate mid-flight reset to recover the crumb's stage into?

    The periodic checkpoint keeps the crumb's `stage` fresh (re-stamped every second aloft, so it is
    at most ~1 s stale at a reset), so the STAGE itself is trustworthy -- the gate only has to confirm
    this is a genuine RECENT reset of a real flight, not that the altitude independently agrees (a
    height cross-check was an atavism of the old single-breadcrumb design and is gone):

      * universal (every stage): a valid crumb (carries stage + stamp); `cause_is_reset` -- WDT/SOFT/
        HARD, never a power-on (a battery insertion / power switch is a human -- a fresh flight or a
        recovery crew -> cold); the crumb age in 0.._MAX_AGE_S (the RTC survives soft/WDT resets so the
        continuity holds; a power cycle restarts it and breaks the arithmetic -> cold).
      * passive stages (GLIDING/LANDING, the unpowered post-separation glide): the separation switch
        reads SEPARATED -- the physical latch no software can fake, so a landed-then-nested glider can
        never recover into a glide. BOOSTING is the active boost (pre-separation, latch nested), so
        reset + age carry it; SETTING/DONE are on the ground and need neither (SETTING recovers as a
        plain cold boot, DONE just stays DONE).

    Pure function of its inputs (host-testable).

    Args:
        crumb - the crumb dict from load(), or None.
        separated - the separation driver's latch reading (True = separated).
        cause_is_reset - True when machine.reset_cause() was WDT/SOFT/HARD (not a power-on).
        now_s - the current time (RTC epoch seconds).

    Returns:
        (restore, reason): restore True with the passing reason when the gate agrees, else False with
        the first failing reason.
    """
    if crumb is None:
        return False, 'no crumb'
    stage = crumb.get('stage')
    stamp = crumb.get('stamp')
    if stage is None or stamp is None:
        return False, 'crumb missing stage/stamp -> cold boot'
    if not cause_is_reset:
        return False, 'power-on boot (human hands), not a reset'
    age_s = now_s - stamp
    if not 0 <= age_s <= _MAX_AGE_S:
        return False, 'crumb age %ds outside 0..%ds' % (age_s, _MAX_AGE_S)
    if controller.Stage.passive(stage) and not separated:  # a gliding stage must really have separated
        return False, 'separation switch reads nested'
    return True, 'recover %s, %ds after checkpoint' % (controller.Stage.STAGES.get(stage, '?'), age_s)


def _apply_restore(flight, crumb, cfg: dict) -> None:
    """
    Apply a PASSED gate: restore the flight state from the crumb.

    Restore the mission identity + zone when the crumb carries them (frozen at BOOSTING), rebase the
    rebooted baros to the crumb's pad altitude (their setup re-zeroed mid-air, so `elevation` would
    read ~0 up there and the landing detect would flare immediately), set the SAVED stage + the
    warm-started flag, and re-arm only if the crumb was armed. Controller/mission mutations ONLY (the
    hardware reads + gc live in restore()), so this is host-testable with stubs. The normal detectors
    (separation / apogee / landing) re-evaluate from the restored stage. A pre-BOOSTING (SETTING)
    crumb carries no launch/zone/pad, so those are all guarded.

    Args:
        flight - the controller (its stage / arm / warm_started + find() for the baros).
        crumb - the passed crumb dict.
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
    pad_altitude = crumb.get('pad_altitude')
    if pad_altitude is not None:
        baro_names = [sensor['name'] for sensor in cfg.get('sensors', [])
                      if sensor.get('driver') in ('icp10111', 'bmp280')]
        for baro in flight.find(baro_names):
            if baro is not None:
                baro.update({'ground': pad_altitude})
    """
    AIRSPEED (findings §23.4): hand the saved airspeed back to the flight task BEFORE the loop runs, so
    the fin cap comes off a real speed rather than the blunt `airspeed_unconfident_ms` floor. Recovery
    order is pitot -> saved -> GNSS: this is the immediate one, the accel backbone integrates on from
    it, and the first in-band pitot read overrides it. Absent on an older crumb -> unchanged behaviour.
    """
    airspeed = crumb.get('airspeed')
    if airspeed is not None:
        flight_task = flight.find(['flight'])[0]
        if flight_task is not None:
            flight_task.seed_airspeed(airspeed)
    flight.set_stage(crumb['stage'])  # the SAVED stage; the detectors re-evaluate from here
    if crumb.get('armed'):
        flight.arm()  # only an armed flight ever checkpoints a recovery crumb -- re-arm to match
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
        cfg - the board config (the checkpoint component's warm_start toggle + the sensor list for the baros).
        log - the log sink (defaults to print).

    Returns:
        True when a warm start was applied (-> gliding, armed, GC off); False on a normal cold boot or
        a rejected gate.
    """
    crumb = load()
    if crumb is None:
        return False  # no flight was in progress: the normal boot, zero extra work
    if not (config.device(cfg, name='checkpoint') or {}).get('warm_start', True):
        clear()
        log('warmstart :: disabled by config')
        return False
    separation = flight.find(['separation'])[0]  # the driver's own latch reading (no second Pin)
    separated = separation.separated() if separation is not None else False
    cause = machine.reset_cause()
    cause_is_reset = cause in (machine.WDT_RESET, machine.SOFT_RESET, machine.HARD_RESET)
    allowed, reason = should_restore(crumb, separated, cause_is_reset, time.time())
    log('warmstart :: gate: %s (reset_cause %d)' % (reason, cause))
    if not allowed:
        clear()  # rejected -> make the NEXT boot unambiguously cold
        return False
    _apply_restore(flight, crumb, cfg)
    stage = crumb['stage']
    if controller.Stage.airborne(stage):  # airborne recovery
        gc.collect()  # compact + GC OFF for the rest of the flight (the BOOSTING hook was skipped)
        gc.disable()  # SETTING/DONE recover on the ground -> GC stays on
    log('warmstart :: WARM START -> %s%s' % (controller.Stage.STAGES.get(stage, '?'),
                                             ', armed' if crumb.get('armed') else ''))
    return True


@task.activity('checkpoint')
class Checkpoint(task.Task):
    """
    Periodic + on-stage-change flight-state checkpoint to NVS -- the warm-start source.

    Every `checkpoint_s` while AIRBORNE (BOOSTING/GLIDING/LANDING; floored at 1 s) and once on
    entering each stage, write the live state to the NVS crumb -- but ONLY for an ARMED flight (a
    disarmed passive flight must never warm-start into an armed stage). The telemetry/log timeline is
    written on every checkpoint regardless. SETTING and DONE checkpoint once on entry and never
    periodically -- nothing moves during the long pad dwell / post-landing wait, so re-writing every
    period would only wear the flash.
    """

    async def setup(self) -> bool:
        self._period_ms: int = max(1, self.config.get('checkpoint_s', 1)) * 1000  # active-stage cadence, >= 1 s
        self._poll_ms: int = min(500, self._period_ms)  # tick fast enough to catch a stage change promptly
        self._altitude = databoard.Databoard.parameter('altitude')
        self._speed = databoard.Databoard.parameter('speed')
        # the FUSED airspeed (not the GNSS ground speed above) is what a warm start needs back: it is the
        # quantity the fin-authority cap is computed from, and a reacquiring GNSS cannot supply it in time
        self._flight = None  # resolved lazily -- task setup order is not guaranteed
        self._telemetry = recorder.Telemetry('checkpoint.csv',
                                             ('stage', 'altitude', 'speed', 'airspeed', 'ticks_ms'))
        self._static: dict = {}  # launch / zone / pad_altitude, frozen at BOOSTING entry (empty until then)
        self._pad = None  # tracked while on the pad (SETTING) so BOOSTING freezes a true GROUND altitude
        self._ok = True
        return True

    def _freeze_static(self) -> None:
        """
        Freeze the recovery IDENTITY at BOOSTING entry: the launch fix, the zone, the pad altitude.

        These do not change through the flight, so they are captured once (the launch fix pinned at
        the last ground moment) and ride every later crumb. No fix / CC point -> the zone centre keeps
        the crumb usable (the same tier-2 fallback the sequencer used).

        Args:
            (none)

        Returns:
            None; fills self._static with launch / zone / pad_altitude as a side effect.
        """
        mission = inspector.Inspector.get('mission')
        static = {}
        if mission is not None and mission.zone:
            mission.freeze_launch()  # a fix that arrived AFTER arm pins here -- the last ground moment
            zone = mission.zone
            launch = mission.launch_point()
            if launch is None:  # no fix / CC point -> the zone centre keeps the crumb usable
                launch = ((zone[0][0] + zone[1][0]) / 2, (zone[0][1] + zone[1][1]) / 2)
            static['launch'] = [launch[0], launch[1]]
            static['zone'] = [[zone[0][0], zone[0][1]], [zone[1][0], zone[1][1]]]
        pad = self._pad if self._pad is not None else self._altitude.value()  # last on-pad reading (fallback: now)
        if pad is not None:
            static['pad_altitude'] = pad
        self._static = static

    def _checkpoint(self, stage: int) -> None:
        """
        Write one checkpoint: the timeline always, the recoverable NVS crumb only when armed.

        Args:
            stage - the controller's current stage id.

        Returns:
            None; pushes a telemetry row + a log line, and (armed only) commits the NVS crumb.
        """
        altitude = self._altitude.value()
        speed = self._speed.value()
        if self._flight is None:
            self._flight = self.controller.find(['flight'])[0]
        airspeed = None if self._flight is None else round(self._flight.airspeed(), 1)
        ticks_ms = time.ticks_ms()
        self._telemetry.push((stage, altitude, speed, airspeed, ticks_ms))
        if self.controller.armed:  # only an armed flight is worth -- and safe -- to recover
            crumb = dict(self._static)  # launch/zone/pad from BOOSTING
            crumb.update({'stage': stage, 'armed': True, 'altitude': altitude, 'speed': speed,
                          'airspeed': airspeed, 'ticks_ms': ticks_ms, 'stamp': int(time.time())})
            save(crumb)
        recorder.Recorder.log(self.name, 'checkpoint %s alt=%s' % (controller.Stage.STAGES.get(stage), altitude))

    async def run(self) -> None:
        """
        Checkpoint on every stage change + every period_ms while airborne; forever.

        Args:
            (none)

        Returns:
            None; loops forever writing the crumb + the telemetry timeline.
        """
        last_stage = None
        last_us = time.ticks_us()
        while True:
            await asyncio.sleep_ms(self._poll_ms)
            stage = self.controller.stage
            if stage == controller.Stage.SETTING:  # on the pad -> keep the latest ground altitude
                pad = self._altitude.value()
                if pad is not None:
                    self._pad = pad
            changed = stage != last_stage
            if changed and stage == controller.Stage.BOOSTING:
                self._freeze_static()  # capture the recovery identity as we leave the ground
            due = (controller.Stage.airborne(stage)  # periodic writes only aloft (never the long ground dwells)
                   and time.ticks_diff(time.ticks_us(), last_us) >= self._period_ms * 1000)
            if changed or due:
                self._checkpoint(stage)
                last_stage = stage
                last_us = time.ticks_us()
