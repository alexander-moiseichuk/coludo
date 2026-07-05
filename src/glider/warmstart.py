# warmstart.py — in-flight reboot recovery (specs/coludo.md "In-flight reboot & warm start").
# A mid-air reset (watchdog, brownout-survivor, crash) must not turn the glider ballistic: the
# sequencer drops a tiny BREADCRUMB into NVS at BOOSTING entry (never a VFS file — a filesystem
# write locks the scheduler and wears the data flash; esp32.NVS commits to its own partition in
# milliseconds) and clears it at DONE. At boot, main.py restores GLIDING when the breadcrumb AND
# two physical signals agree — see should_restore() for the gate.
#
# Storage layout: `flight` is a bare i32 flag (cheap to flip on the clear path), the payload is ONE
# JSON blob (`crumb`) — full float precision, no per-field key bookkeeping, and a new field is a
# dict entry rather than an NVS schema change. The module degrades to no-ops off-board (CPython).

try:
    from esp32 import NVS
    _nvs = NVS('coludo')
except ImportError:  # CPython (host tools / sim): warm start is board-only, everything no-ops
    _nvs = None

_BLOB_MAX: int = 512  # read buffer for the crumb blob (the JSON is ~150 B; headroom for new fields)


def save(launch: tuple, zone: tuple, pad_altitude: float, stamp: int) -> bool:
    """Drop the breadcrumb (called ONCE at BOOSTING entry, on the rod, before GC goes off).
    `launch` = (lat, lon) of the live fix; `zone` = ((lat, lon) TL, (lat, lon) BR); `pad_altitude`
    = the baro ABSOLUTE altitude at the pad (m — NOT the boot-relative elevation, a rebooted baro
    re-zeroes mid-air); `stamp` = RTC epoch seconds. Returns False (and never raises) when NVS is
    absent or full — a failed breadcrumb must not block a launch."""
    if _nvs is None:
        return False
    import json
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
    """Down the flag (at DONE / after a rejected warm start). The blob stays — the flag alone
    decides, so the clear is a single fast i32 write. Never raises."""
    if _nvs is None:
        return
    try:
        _nvs.set_i32('flight', 0)
        _nvs.commit()
    except OSError:
        pass


def load():
    """The breadcrumb dict ({launch: [lat, lon], zone: [[TL], [BR]], pad_altitude, stamp}), or None
    when no flight was in progress (flag absent/0) or the blob is missing/torn (-> cold boot)."""
    if _nvs is None:
        return None
    import json
    try:
        if _nvs.get_i32('flight') != 1:
            return None
        buffer = bytearray(_BLOB_MAX)
        length = _nvs.get_blob('crumb', buffer)
        return json.loads(bytes(buffer[:length]))
    except (OSError, ValueError):  # never written / torn / unparsable -> cold boot
        return None


def should_restore(crumb, separated: bool, altitude, cause_is_reset: bool, age_s: int,
                   min_height_m: float = 15.0, max_age_s: int = 600) -> tuple:
    """The warm-start gate — ALL must agree (defense in depth; any doubt -> cold boot):
      1. a breadcrumb exists (we were airborne when the reset hit);
      2. the separation switch reads SEPARATED — the physical latch no software state can fake
         (post-separation it stays LOW for the whole glide; a stack on the pad reads nested);
      3. the baro ABSOLUTE altitude reads at least `min_height_m` above the breadcrumb's pad —
         still clearly in the air (None = baro not up in time -> refuse);
      4. `cause_is_reset` — machine.reset_cause() was WDT/SOFT/HARD. A battery insertion or power
         switch reads PWRON — exactly what a RECOVERY CREW's hands do to a glider that crash-landed
         on a rise above the pad (where gate 3 alone would pass). A mid-air brownout also reads
         PWRON and stays cold: a browning-out battery cannot be trusted to finish the glide;
      5. `age_s` = now - crumb stamp is positive and under `max_age_s`. The RTC survives soft/WDT
         resets, so the arithmetic holds exactly when a warm start is legitimate (even an unsynced
         RTC — continuity matters, not absolute truth); a power cycle restarts the RTC and breaks
         it -> cold.
    Pure function of its inputs (host-testable). Returns (restore, reason)."""
    if crumb is None:
        return False, 'no breadcrumb'
    if not separated:
        return False, 'separation switch reads nested'
    if altitude is None:
        return False, 'no altitude reading'
    height = altitude - crumb['pad_altitude']
    if height < min_height_m:
        return False, 'altitude %.0fm above pad < %.0fm' % (height, min_height_m)
    if not cause_is_reset:
        return False, 'power-on boot (human hands), not a reset'
    if not 0 <= age_s <= max_age_s:
        return False, 'breadcrumb age %ds outside 0..%ds' % (age_s, max_age_s)
    return True, 'airborne %.0fm above pad, reset %ds after boost' % (height, age_s)
