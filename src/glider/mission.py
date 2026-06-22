# Mission — the per-launch identity the operator sets before a flight: a launch id, the launch
# site position (a known origin and a GNSS cold-start seed), and the board clock. Unlike the board
# config (hardware; stable across flights, see config.py) the mission changes every launch, so it
# lives in its own file, `launch.config`, and is edited live through the Inspector.
#
# Mission is a singleton Inspectable:
#   inspect mission                              -> launch id / site / position + the board clock
#   update mission base64:{"launch_id":"t1"}     -> set the launch id for this flight
#   update mission base64:{"epoch":1750170000}   -> set the board RTC (time sync; Unix seconds)
#   save-mission                                 -> persist the live mission back to launch.config
#
# Position is metres / decimal degrees; it is a known origin now and seeds the GNSS driver later.

import json
import os
import time

try:
    from machine import RTC
except ImportError:  # CPython tooling / off-board lint+compile only
    RTC = None

import databoard
import inspector
import navigation
import recorder

LAUNCH_PATH: str = 'launch.config'

# MicroPython's time epoch on the esp32 port is 2000-01-01; Control speaks the Unix (1970) epoch,
# so the wire `epoch` is converted by this many seconds when setting/reading the RTC.
_EPOCH_OFFSET: int = 946684800

_FIELDS: tuple = ('launch_id', 'site', 'latitude', 'longitude', 'altitude')

# Default max range (m) from the launch point to any zone point, when the board config omits it. The
# real value is `max_range_m` in board.json -- it is a glide-range property of the AIRFRAME (a bigger
# glider reaches farther), so it lives in the board config, not the per-launch mission.
_DEFAULT_MAX_RANGE_M: float = 200.0


def _load(path: str) -> dict:
    """Read launch.config (a JSON object) if present and valid, else an empty mission. Never
    raises -- a missing/corrupt file just means 'no launch set yet', the board stays usable."""
    try:
        with open(path) as handle:
            data = json.loads(handle.read())
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def _number(value, low: float, high: float):
    """Return value if it is an in-range number (not a bool), else None."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return value if low <= value <= high else None


def _zone(value):
    """Validate a landing zone `[[lat_tl, lon_tl], [lat_br, lon_br]]` (top-left + bottom-right corners)
    -> ((lat, lon), (lat, lon)) or None. navigation.py resolves the target (centre) + gates (short-side
    midpoints) from it. The corner choice is an operator SAFETY decision: orient the zone so the
    short-side entrances face hazard-free corridors -- navigation steers to a gate with no hazard awareness
    (specs/coludo.md "Zone orientation")."""
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


class Mission(inspector.Inspectable):
    """The operator-set launch identity. One per board; registers itself so Control can
    `inspect`/`update mission`. Seeded from launch.config at construction."""

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
        inspector.Inspector.register(self)

    def set_time(self, epoch) -> bool:
        """Set the board RTC from a Unix epoch (seconds, UTC). Returns True if applied."""
        if RTC is None or isinstance(epoch, bool) or not isinstance(epoch, int):
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

    # ------------------------------------------------------------ landing zone
    def launch_point(self):
        """The launch origin (lat, lon): the operator-set position (CC `update mission` / `assist`) if
        present, else the live on-board GNSS fix from the databoard, else None. So it is set by CC
        unless taken from GPS."""
        if self.latitude is not None and self.longitude is not None:
            return (self.latitude, self.longitude)
        value, source, _age = databoard.Databoard.parameter('position').read()
        return value if source is not None and value is not None else None  # only a FRESH fix

    def geometry(self) -> dict:
        """The landing zone resolved against the launch point: the target (centre) + both gates
        (short-side entrances) and the launch-point->point distances, with `in_range` False if any of
        the three exceeds _MAX_RANGE_M (coludo.md). None if the zone or the launch point is unset."""
        if self.zone is None:
            return None
        origin = self.launch_point()
        if origin is None:
            return None
        target, gate_a, gate_b = navigation.zone(self.zone[0], self.zone[1])
        distances = {name: round(navigation.distance(origin[0], origin[1], point[0], point[1]), 1)
                     for name, point in (('target', target), ('gate_a', gate_a), ('gate_b', gate_b))}
        farthest = max(distances.values())
        return {'origin': origin, 'target': target, 'gates': [gate_a, gate_b], 'distances_m': distances,
                'max_distance_m': farthest, 'in_range': farthest <= self.max_range_m}

    async def probe(self) -> str:
        """On-demand self-test: a launch position is set (CC or GNSS) and, if a landing zone is set, all
        three of its points are within range of the launch point. Caught pre-flight (arm/verify). Not a
        hardware check -- data."""
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

    # --- Inspectable ---
    def inspect(self) -> dict:
        snapshot = {
            'launch_id': self.launch_id,
            'site': self.site,
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude,
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
        """Apply launch_id/site/latitude/longitude/altitude (stored, range-checked) and `epoch`
        (sets the RTC -- not stored). Returns the names actually changed; out-of-range coordinates
        are ignored (not reported changed)."""
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

    def save(self) -> None:
        """Persist the stored mission fields to launch.config (atomic temp+rename) so the launch
        identity survives a pre-flight reboot. The clock is never persisted -- it is the RTC's."""
        data = {key: getattr(self, key) for key in _FIELDS}
        if self.zone is not None:  # persist the landing zone as plain lists (JSON has no tuples)
            data['zone'] = [list(self.zone[0]), list(self.zone[1])]
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as handle:
            handle.write(json.dumps(data))
        try:
            os.rename(tmp, self.path)
        except OSError:  # some VFS won't rename onto an existing file
            try:
                os.remove(self.path)
            except OSError:
                pass
            os.rename(tmp, self.path)
