"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Mission -- the per-launch identity the operator sets before a flight: a launch id, the launch site
position (a known origin and a GNSS cold-start seed), and the board clock. Unlike the board config
(hardware; stable across flights, see config.py) the mission changes every launch, so it lives in its
own file, `launch.config`, and is edited live through the Inspector.

Mission is a singleton Inspectable:
  inspect mission -> launch id / site / position + the board clock
  update mission base64:{"launch_id":"t1"} -> set the launch id for this flight
  update mission base64:{"epoch":1750170000} -> set the board RTC (time sync; Unix seconds)
  get-config launch / set-config launch -> read / save (merge + persist) launch.config

Position is metres / decimal degrees; it is a known origin now and seeds the GNSS driver later.
"""

import json
import math
import time

try:
    from machine import RTC
except ImportError:  # CPython tooling / off-board lint+compile only
    RTC = None

import commons
import databoard
import inspector
import navigation
import recorder

LAUNCH_PATH: str = 'launch.config'

# MicroPython's time epoch on the esp32 port is 2000-01-01; Control speaks the Unix (1970) epoch,
# so the wire `epoch` is converted by this many seconds when setting/reading the RTC.
_EPOCH_OFFSET: int = 946684800

_FIELDS: tuple = ('launch_id', 'site', 'latitude', 'longitude', 'altitude')

"""
Default max range (m) from the launch point to any zone point, when the board config omits it. The real
value is `max_range_m` in board.config -- it is a glide-range property of the AIRFRAME (a bigger glider
reaches farther), so it lives in the board config, not the per-launch mission.
"""
_DEFAULT_MAX_RANGE_M: float = 200.0

"""
Default launch site when no launch.config has been set yet: HPRC (Homestead Public Rocketry Club), the
documented test pad + landing zone (doc/plan.md, specs/coludo.md, sim_model.HPRC). So a fresh board already
has a valid pad + zone for HPRC field tests; the operator overrides it live via `update mission` /
`set-config launch` for any other site.
"""
_HPRC_DEFAULT: dict = {
    'site': 'HPRC',
    'latitude': 25.514379,
    'longitude': -80.391795,
    'altitude': 2.0,
    'zone': [[25.514944, -80.392972], [25.514583, -80.391111]],
}


def _load(path: str) -> dict:
    """
    Read launch.config (a JSON object) if present and valid, else the HPRC default mission.

    Never raises -- a missing/corrupt file just falls back to HPRC (the documented test pad), so the
    board stays usable.

    Args:
        path - the launch.config file path to read.

    Returns:
        The parsed mission dict, or a copy of the HPRC default when the file is missing/corrupt.
    """
    try:
        with open(path) as handle:
            data = json.loads(handle.read())
    except (OSError, ValueError):
        return dict(_HPRC_DEFAULT)
    return data if isinstance(data, dict) else dict(_HPRC_DEFAULT)


def _number(value, low: float, high: float):
    """Return value if it is an in-range number (not a bool), else None."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return value if low <= value <= high else None


def _zone(value):
    """
    Validate a landing zone `[[lat_tl, lon_tl], [lat_br, lon_br]]` (top-left + bottom-right corners).

    navigation.py resolves the target (centre) + gates (short-side midpoints) from it. The corner
    choice is an operator SAFETY decision: orient the zone so the short-side entrances face hazard-free
    corridors -- navigation steers to a gate with no hazard awareness (specs/coludo.md "Zone
    orientation").

    Args:
        value - the raw zone value to validate (expected a 2-corner list/tuple).

    Returns:
        ((lat, lon), (lat, lon)) for the two corners, or None when the shape/range is invalid.
    """
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        return None
    corners = []
    for point in value:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            return None
        latitude = _number(point[0], -90.0, 90.0)
        longitude = _number(point[1], -180.0, 180.0)
        if latitude is None or longitude is None:
            return None
        corners.append((latitude, longitude))
    return (corners[0], corners[1])


def _sites(value) -> list:
    """
    Validate launch.config `sites` -- the known launch locations for CC-less field operation.

    From specs/coludo.md "Field operation without CC": [{name, pad: [lat, lon], zone: [[TL], [BR]]},
    ...] becomes [(name, (lat, lon), zone), ...]. Invalid entries are dropped, never raised -- a typo
    in one site must not cost the list.

    Args:
        value - the raw `sites` value to validate (expected a list of site dicts).

    Returns:
        A list of (name, (lat, lon), zone) tuples; empty when value is not a list.
    """
    out = []
    if not isinstance(value, (list, tuple)):
        return out
    for entry in value:
        if not isinstance(entry, dict):
            continue
        pad = entry.get('pad')
        if not isinstance(pad, (list, tuple)) or len(pad) != 2:
            continue
        latitude = _number(pad[0], -90.0, 90.0)
        longitude = _number(pad[1], -180.0, 180.0)
        zone = _zone(entry.get('zone'))
        if latitude is None or longitude is None or zone is None:
            continue
        out.append((str(entry.get('name', 'site%d' % len(out))), (latitude, longitude), zone))
    return out


class Mission(inspector.Inspectable):
    """
    The operator-set launch identity.

    One per board; registers itself so Control can `inspect`/`update mission`. Seeded from
    launch.config at construction.
    """

    name: str = 'mission'
    kind: str = 'mission'

    def __init__(self, path: str = LAUNCH_PATH, max_range_m: float = _DEFAULT_MAX_RANGE_M):
        data = _load(path)
        self.path: str = path
        self.max_range_m: float = max_range_m  # from board config (airframe glide range); zone range gate
        self.launch_id: str = data.get('launch_id', '')
        self.site: str = data.get('site', '')
        self.latitude = _number(data.get('latitude'), -90.0, 90.0)  # decimal degrees, None unset
        self.longitude = _number(data.get('longitude'), -180.0, 180.0)
        self.altitude = data.get('altitude')  # launch-site elevation, metres (None unset)
        self.zone = _zone(data.get('zone'))  # landing zone ((lat,lon) TL, (lat,lon) BR) or None
        self.sites: list = _sites(data.get('sites'))  # known launch locations (CC-less site-by-GPS)
        self._zone_key = None  # zone identity the resolved geometry below was computed from (zone_points)
        self._zone_points = None  # cached navigation.zone() -> (target, gate_a, gate_b) for the hot path
        self._zone_aspect: float = 1.0  # cached long/short ratio -> the auto endgame pattern (o vs oo)
        inspector.Inspector.register(self)

    def set_time(self, epoch) -> bool:
        """
        Set the board RTC from a Unix epoch (seconds, UTC).

        Rejects a value before 2000-01-01: epoch - _EPOCH_OFFSET would go negative and gmtime() would
        set a pre-2000 RTC -- any real flight clock is well past 2000.

        Args:
            epoch - the Unix epoch in seconds (UTC).

        Returns:
            True when the RTC was set; False when RTC is unavailable or the epoch is out of range.
        """
        if RTC is None or isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < _EPOCH_OFFSET:
            return False
        field = time.gmtime(epoch - _EPOCH_OFFSET)  # (year, month, mday, h, m, s, weekday, yday)
        RTC().datetime((field[0], field[1], field[2], field[6], field[3], field[4], field[5], 0))
        return True

    def clock(self) -> str:
        """Current board wall-clock as 'YYYY-MM-DDTHH:MM:SS' (from the RTC)."""
        now = time.localtime()
        return '%04d-%02d-%02dT%02d:%02d:%02d' % (now[0], now[1], now[2], now[3], now[4], now[5])

    def epoch(self) -> int:
        """Current board clock as a Unix epoch (seconds), for Control to compare against its own."""
        return time.time() + _EPOCH_OFFSET

    """Landing zone: the launch origin, the resolved zone geometry, and the endgame pattern."""
    def launch_point(self):
        """
        The launch origin (lat, lon).

        The operator-set position (CC `update mission` / `assist`) if present, else the live on-board
        GNSS fix from the databoard, else None. So it is set by CC unless taken from GPS.

        Returns:
            The origin (lat, lon) from the operator-set position or a fresh GNSS fix; None when neither
            is available.
        """
        if self.latitude is not None and self.longitude is not None:
            return (self.latitude, self.longitude)
        value, source, _age = databoard.Databoard.parameter('position').read()
        return value if source is not None and value is not None else None  # only a FRESH fix

    def freeze_launch(self) -> None:
        """
        Pin the live GNSS fix as the persistent launch point.

        Called at arm -- the last moment the board is known to be ON the pad. Mid-flight the fix can
        drop, and launch_point()'s live-GNSS fallthrough drops with it -- freezing keeps the tier-2
        open-loop heading (and the warm-start crumb's launch field) available for the whole flight. An
        operator-set position always wins; no fix -> stays unset (launch_point() keeps falling
        through).

        Returns:
            None; sets self.latitude/self.longitude from a fresh fix, or leaves them unchanged.
        """
        if self.latitude is not None and self.longitude is not None:
            return  # CC-set -- never overwrite the operator
        value, source, _age = databoard.Databoard.parameter('position').read()
        if source is not None and value is not None:
            self.latitude, self.longitude = value[0], value[1]
            recorder.Recorder.log(self.name, 'launch point frozen (%.5f, %.5f)' % (value[0], value[1]))

    def select_site(self, fix: tuple):
        """
        CC-less site selection (specs/coludo.md "Field operation without CC").

        The nearest known site whose pad is within max_range_m of the live GNSS `fix` becomes the
        mission (site name + zone). The launch POINT stays the LIVE fix -- latitude/longitude are left
        unset so launch_point() keeps falling through to GNSS (freeze_launch pins it at arm).

        Args:
            fix - the live GNSS position (lat, lon) to match against the known sites.

        Returns:
            The selected site name; None when no site is in range (the caller synthesizes the fallback
            zone).
        """
        best = None
        best_distance = self.max_range_m
        for name, pad, zone in self.sites:
            span = navigation.distance(fix[0], fix[1], pad[0], pad[1])
            if span <= best_distance:
                best = (name, zone)
                best_distance = span
        if best is None:
            return None
        self.site, self.zone = best
        return best[0]

    def fallback_zone(self, fix: tuple, bearing_deg: float = 0.0, near_m: float = 50.0,
                      width_m: float = 100.0, depth_m: float = 90.0) -> tuple:
        """
        The spiral-landing fallback: synthesize and ADOPT a GENEROUS box the spiral just lands INSIDE.

        Reached when no known site is in range after ignition is already committed (spec, later
        simplified from the full spiral). We always know the pad, so precise midpoint accuracy is not
        needed here -- we favour landing in-zone over hitting the centre; the ~55 m endgame miss
        (turn-radius-limited) fits a +/-50 m box. The box's WIDE (width_m) side faces the pad -- a broad
        left-to-right entrance -- its NEAR edge `near_m` out (people stand at the pad), running
        `depth_m` away; the CENTRE is `near_m + depth_m/2` from the fix at `bearing_deg` (the operator's
        clear sector). Axis-aligned (an emergency orbit cares about the CENTRE, not orientation; the
        default north clear sector puts the wide side across the pad).

        Args:
            fix - the live GNSS position (lat, lon), the pad the box is placed relative to.
            bearing_deg - the direction from the pad to the box centre (the operator's clear sector).
            near_m - the gap from the pad to the box's near edge (metres).
            width_m - the wide side facing the pad (metres).
            depth_m - how far the box runs away from the pad (metres).

        Returns:
            The adopted zone ((lat, lon) TL, (lat, lon) BR), also stored on self.zone.
        """
        centre = near_m + depth_m / 2.0
        lat = fix[0] + centre * math.cos(math.radians(bearing_deg)) / commons.M_PER_DEG
        lon = fix[1] + centre * math.sin(math.radians(bearing_deg)) / (
            commons.M_PER_DEG * math.cos(math.radians(fix[0])))
        half_lat = (depth_m / 2.0) / commons.M_PER_DEG                                  # along the bearing
        half_lon = (width_m / 2.0) / (commons.M_PER_DEG * math.cos(math.radians(lat)))  # the wide side
        self.site = 'fallback'
        self.zone = ((lat + half_lat, lon - half_lon), (lat - half_lat, lon + half_lon))
        return self.zone

    def zone_points(self) -> tuple:
        """
        (target, gate_a, gate_b) for the current landing zone, memoized by zone identity.

        navigation.zone() is MEMOIZED by zone identity, so the per-flight-CONSTANT geometry resolves
        ONCE instead of every nav tick on the hot guidance path (the zone rectangle never moves in
        flight). Any zone change (a CC update / the CC-less fallback / a reload) reassigns self.zone to
        a new object, so the identity check re-resolves on its own -- no explicit invalidation to
        forget.

        Returns:
            (target, gate_a, gate_b) for the current zone; None when no zone is set.
        """
        zone = self.zone
        if zone is None:
            return None
        if zone is not self._zone_key:  # first call, or the zone was replaced -> resolve + cache
            self._zone_key = zone
            self._zone_points = navigation.zone(zone[0], zone[1])
            self._zone_aspect = navigation.zone_aspect(zone[0], zone[1])
        return self._zone_points

    def zone_aspect(self) -> float:
        """
        The zone's long/short side ratio (>= 1), memoized alongside zone_points.

        endgame_heading() reads it for the o-vs-oo auto decision.

        Returns:
            The ratio (>= 1); 1.0 when no zone is set.
        """
        return 1.0 if self.zone_points() is None else self._zone_aspect

    def endgame_heading(self) -> int:
        """
        The endgame holding pattern (a guidance.Heading id) the AUTO setting resolves to.

        Decided from the zone SHAPE: a strip whose long/short aspect exceeds Heading.OO_ASPECT flies
        FIG_OO (two lobes along the long axis to cover the length), everything squarer flies FIG_O (a
        single circle). The Mission owns the zone, so it owns this decision; guidance only asks when its
        config is AUTO.

        Returns:
            The guidance.Heading id: FIG_OO for an elongated zone, else FIG_O.
        """
        import guidance  # local: the endgame enum lives with the control law, no top-level coupling
        return guidance.Heading.FIG_OO if self.zone_aspect() > guidance.Heading.OO_ASPECT \
            else guidance.Heading.FIG_O

    def geometry(self) -> dict:
        """
        The landing zone resolved against the launch point.

        The target (centre) + both gates (short-side entrances) and the launch-point->point distances,
        with `in_range` False if any of the three exceeds self.max_range_m (specs/coludo.md).

        Returns:
            A dict with origin, target, gates, distances_m, max_distance_m, and in_range; None when the
            zone or the launch point is unset.
        """
        if self.zone is None:
            return None
        origin = self.launch_point()
        if origin is None:
            return None
        target, gate_a, gate_b = self.zone_points()  # the memoized resolve (identity-cached)
        distances = {name: round(navigation.distance(origin[0], origin[1], point[0], point[1]), 1)
                     for name, point in (('target', target), ('gate_a', gate_a), ('gate_b', gate_b))}
        farthest = max(distances.values())
        return {'origin': origin, 'target': target, 'gates': [gate_a, gate_b], 'distances_m': distances,
                'max_distance_m': farthest, 'in_range': farthest <= self.max_range_m}

    async def probe(self) -> str:
        """
        On-demand self-test: a launch position is set and the zone (if any) is in range.

        Checks that a launch position is set (CC or GNSS) and, if a landing zone is set, all three of
        its points are within range of the launch point. Caught pre-flight (arm/verify). Not a hardware
        check -- data.

        Returns:
            None when it passes; else a human-readable error string naming what failed.
        """
        try:
            recorder.Recorder.log(self.name, 'probe: launch position ...')
            origin = self.launch_point()
            if origin is None:
                raise ValueError('launch position not set (CC or GNSS)')
            recorder.Recorder.log(self.name, 'probe: position ok (%.5f, %.5f)' % (origin[0], origin[1]))
            geometry = self.geometry()
            if geometry is not None and not geometry['in_range']:
                raise ValueError('landing zone out of range (%.0f m > %.0f m)' % (
                    geometry['max_distance_m'], self.max_range_m))
        except Exception as error:
            message = 'launch/zone: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    """Inspectable: the operator-facing mission snapshot."""
    def inspect(self) -> dict:
        origin = self.launch_point()
        snapshot = {
            'launch_id': self.launch_id,
            'site': self.site,
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude,
            # the EFFECTIVE origin launch_point() resolves right now: latitude/longitude when
            # CC-set or frozen at arm (then launch_source 'set'), else the live GNSS fix
            # ('gnss'), else None -- so the operator sees the origin the board would fly with
            # even before arm, when latitude/longitude legitimately read null
            'launch_point': origin,
            'launch_source': ('set' if self.latitude is not None and self.longitude is not None
                              else 'gnss' if origin is not None else None),
            'zone': self.zone,
            'clock': self.clock(),
            'epoch': self.epoch(),
        }
        geometry = self.geometry()  # the resolved target + gates + their range from the launch point
        if geometry is not None:
            snapshot['target'] = geometry['target']
            snapshot['gates'] = geometry['gates']
            snapshot['distances_m'] = geometry['distances_m']
            snapshot['in_range'] = geometry['in_range']
        return snapshot

    def update(self, props: dict) -> list:
        """
        Apply the editable mission fields from an update.

        Applies launch_id/site/latitude/longitude/altitude (stored, range-checked) and `epoch` (sets
        the RTC -- not stored). Out-of-range coordinates are ignored (not reported changed).

        Args:
            props - the field name -> value updates to apply.

        Returns:
            The names actually changed (an out-of-range or unchanged field is not listed).
        """
        changed = []
        for key in _FIELDS:
            if key not in props:
                continue
            value = props[key]
            if key == 'latitude':
                value = _number(value, -90.0, 90.0)
            elif key == 'longitude':
                value = _number(value, -180.0, 180.0)
            if value is None and key in ('latitude', 'longitude'):
                continue
            if getattr(self, key) != value:
                setattr(self, key, value)
                changed.append(key)
        if 'zone' in props:  # landing zone: a valid 2-corner rectangle replaces it, invalid is ignored
            zone = _zone(props['zone'])
            if zone is not None and zone != self.zone:
                self.zone = zone
                changed.append('zone')
        if 'epoch' in props and self.set_time(props['epoch']):
            changed.append('epoch')
        return changed

    def persisted(self) -> dict:
        """
        The mission as it is stored in launch.config: the editable launch fields only.

        No computed geometry or clock. `get-config launch` returns this; the dashboard 'launch.config'
        editor saves it back via `set-config launch` (the board config uses the same commands with name
        `board`). Zone is plain lists (JSON has no tuples), or None when unset.

        Returns:
            The persistable mission dict (the _FIELDS values plus `zone` as lists / None).
        """
        data = {key: getattr(self, key) for key in _FIELDS}
        data['zone'] = [list(self.zone[0]), list(self.zone[1])] if self.zone is not None else None
        return data

    def save(self) -> None:
        """
        Persist the stored mission fields to launch.config (atomic temp+rename).

        So the launch identity survives a pre-flight reboot. The clock is never persisted -- it is the
        RTC's.

        Returns:
            None; writes launch.config.
        """
        commons.atomic_write_json(self.path, self.persisted())  #
