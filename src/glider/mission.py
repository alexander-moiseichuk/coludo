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

import inspector
import recorder

LAUNCH_PATH: str = 'launch.config'

# MicroPython's time epoch on the esp32 port is 2000-01-01; Control speaks the Unix (1970) epoch,
# so the wire `epoch` is converted by this many seconds when setting/reading the RTC.
_EPOCH_OFFSET: int = 946684800

_FIELDS: tuple = ('launch_id', 'site', 'latitude', 'longitude', 'altitude')


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


class Mission(inspector.Inspectable):
    """The operator-set launch identity. One per board; registers itself so Control can
    `inspect`/`update mission`. Seeded from launch.config at construction."""

    name: str = 'mission'
    kind: str = 'mission'

    def __init__(self, path: str = LAUNCH_PATH):
        data = _load(path)
        self.path: str = path
        self.launch_id: str = data.get('launch_id', '')
        self.site: str = data.get('site', '')
        self.latitude = _number(data.get('latitude'), -90.0, 90.0)  # decimal degrees, None unset
        self.longitude = _number(data.get('longitude'), -180.0, 180.0)
        self.altitude = data.get('altitude')  # launch-site elevation, metres (None unset)
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

    async def probe(self) -> str:
        """On-demand self-test: a launch position is set, so an unconfigured site is caught pre-flight
        (the operator sets it via `update mission` / launch.config). Not a hardware check -- data."""
        try:
            recorder.Recorder.log(self.name, 'probe: launch position ...')
            if self.latitude is None or self.longitude is None:
                raise ValueError('launch position not set (lat/lon)')
            recorder.Recorder.log(self.name, 'probe: position ok (%.5f, %.5f)' % (
                self.latitude, self.longitude))
        except Exception as error:
            message = 'launch position: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None

    # --- Inspectable ---
    def inspect(self) -> dict:
        return {
            'launch_id': self.launch_id,
            'site': self.site,
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude,
            'clock': self.clock(),
            'epoch': self.epoch(),
        }

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
        if 'epoch' in props and self.set_time(props['epoch']):
            changed.append('epoch')
        return changed

    def save(self) -> None:
        """Persist the stored mission fields to launch.config (atomic temp+rename) so the launch
        identity survives a pre-flight reboot. The clock is never persisted -- it is the RTC's."""
        data = {key: getattr(self, key) for key in _FIELDS}
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
