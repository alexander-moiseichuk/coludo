"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Render doc/datasources.md: which sensor feeds each databoard channel, on every board revision.

GENERATED, not written, for the reason gen_pinmap.py is: a hand-kept table of providers and priorities
drifts the first time a priority changes, and the drift is invisible -- the doc keeps describing a fallback
order the board stopped using. Here the config IS the table.

Three things the config alone does not say are supplied below and marked as such: how often a driver
polls when its config omits `period_ms` (read from the driver source), when a source is only available
for PART of a flight, and what a channel means. Everything else is read from config_default + layout.

  python3 tools/gen_datasources.py            # write doc/datasources.md
  python3 tools/gen_datasources.py --check    # exit 1 if the file is stale (for `make check`)
"""

import os
import re
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_GLIDER = os.path.join(_HERE, '..', 'src', 'glider')
_OUT = os.path.join(_HERE, '..', 'doc', 'datasources.md')
sys.path.insert(0, _GLIDER)

import config_default  # noqa: E402  -- after sys.path
import layout  # noqa: E402

_REVISIONS: tuple = ('v0.1', 'v1.0', 'v1.1')

"""
When a source is NOT available for the whole flight. The config cannot express this -- a channel with a
provider looks continuously fed -- and it is the difference between a backup and a backup you can rely on
at the moment you need it.
"""
_AVAILABILITY: dict = {
    'laser_agl': 'LAST METRES ONLY -- a ranging sensor, valid inside its range; absent for most of a flight',
    'gnss': 'after a fix, and speed/course only above the course gate; nothing on the pad before lock',
    'airspeed_sdp810': 'needs its pad tare; the reading is a differential pressure, so it is meaningless at rest',
    'power_ina226': 'absent on airframes built without the shunt (TMS-7C)',
}

"""
How GOOD each provider is for what it feeds. The config knows the order, not the quality -- and the order
alone reads as "there is a backup" whether that backup is an equal or a shadow of the primary.
"""
_GRADE: dict = {
    'imu_lsm6dso32': '+/-32 g, +/-2000 dps',
    'imu_bmi323': '+/-16 g (HALF the primary range), +/-2000 dps (equal)',
    'accel_adxl375': '+/-200 g -- the shock backstop, not a general accel',
    'imu_bno055': '+/-16 g; on-chip NDOF fusion',
    'baro_icp10111': 'high grade -- and the part carrying the documented latch-up',
    'baro_bmp581': 'high grade -- EQUAL to the primary',
    'baro_bmp280': 'coarser than the primary',
    'gnss': 'metres, and only after a fix',
    'attitude': 'complementary filter; heading is gyro-integrated, bounded by the GNSS track only above '
                'the course gate',
    'laser_agl': 'ranging, last metres only',
    'airspeed_sdp810': 'differential pressure; needs its pad tare',
    'power_ina226': 'shunt monitor',
}

"""Channels grouped by the QUANTITY they answer, so related ones can be read together."""
_GROUPS: tuple = (
    ('Attitude and motion', ('attitude', 'rate', 'accel')),
    ('Height', ('altitude', 'elevation', 'agl')),
    ('Air data', ('airspeed', 'dynamic_pressure', 'pressure', 'temperature')),
    ('Position', ('position', 'speed', 'course')),
    ('Power', ('voltage', 'current', 'power')),
)

_CHANNEL_NOTES: dict = {
    'attitude': 'roll/pitch/heading -- what the flight loop steers on',
    'rate': 'gyro angular rate, centideg/s fixnum -- the PID D term',
    'accel': 'specific force, float g',
    'agl': 'height above ground from the ranging sensor',
    'elevation': 'height above the per-sensor ground zero',
    'altitude': 'metres AMSL',
}


def _driver_defaults() -> dict:
    """{driver: default period_ms} parsed from each driver's setup() -- the rate when config omits one."""
    defaults = {}
    folder = os.path.join(_GLIDER, 'drivers')
    for name in sorted(os.listdir(folder)):
        if not name.endswith('.py'):
            continue
        with open(os.path.join(folder, name)) as handle:
            found = re.search(r"config\.get\('period_ms',\s*(\d+)\)", handle.read())
        if found:
            defaults[name[:-3]] = int(found.group(1))
    return defaults


def _providers(revision: str, defaults: dict) -> dict:
    """{channel: [(priority, provider, rate_hz, timeout_ms, rate_is_default), ...]} for one revision."""
    cfg = config_default.default()
    layout.apply(cfg, revision)
    channels = {}
    for group in ('sensors', 'components'):
        for device in cfg.get(group, []):
            if not device.get('enabled', True) or not device.get('provides'):
                continue
            driver = device.get('driver') or device.get('activity')
            period = device.get('period_ms')
            fallback = period is None
            if fallback:
                period = defaults.get(driver)
            rate = round(1000.0 / period, 1) if period else None
            where = '%s:%s' % (device.get('bus'), device.get('id')) if device.get('bus') else 'task'
            for channel, spec in device['provides'].items():
                channels.setdefault(channel, []).append(
                    (spec.get('priority', 0), device['name'], rate, spec.get('timeout_ms'), fallback, where))
    for channel in channels:
        channels[channel].sort()
    return channels


def _describe(entry: tuple) -> str:
    """`name` pN @ rate -- one provider in a chain."""
    priority, name, rate, _timeout, fallback, where = entry
    speed = '%s Hz%s' % (rate, '*' if fallback else '') if rate else 'event'
    # the BUS belongs in the chain: a backup sharing its primary's bus shares its bus faults, which is
    # the difference between redundancy and two things that fail together
    return '`%s` p%d @ %s on %s' % (name, priority, speed, where)


def _table(channels: dict) -> str:
    """One revision's channels, grouped by quantity rather than alphabetically."""
    lines = []
    seen = set()
    for title, group in _GROUPS + (('Other', tuple(sorted(set(channels) - _grouped()))),):
        rows = [channel for channel in group if channel in channels]
        if not rows:
            continue
        lines += ['**%s**' % title, '',
                  '| channel | primary (p0) | backups, in order | freshness | availability |',
                  '|---|---|---|---|---|']
        for channel in rows:
            seen.add(channel)
            entries = channels[channel]
            primary = [e for e in entries if e[0] == 0]
            backups = [e for e in entries if e[0] != 0]
            lines.append('| **%s** | %s | %s | %s | %s |' % (
                channel,
                _describe(primary[0]) if primary else '**none — no p0 provider**',
                ' -> '.join(_describe(e) for e in backups) or '**none**',
                '%s ms' % entries[0][3],
                # attribute each caveat to the provider it belongs to: an unqualified note reads as if
                # the whole channel is limited, when it is usually one fallback deep in the chain
                '; '.join('`%s`: %s' % (e[1], _AVAILABILITY[e[1]])
                          for e in entries if e[1] in _AVAILABILITY) or 'continuous'))
        lines.append('')
    return '\n'.join(lines)


def _grouped() -> set:
    """Every channel named in _GROUPS."""
    return {channel for _title, group in _GROUPS for channel in group}


def _alternatives(entries: list) -> list:
    """
    Drop providers that are ALTERNATIVES to the primary rather than backups to it.

    Two drivers for one socket (the VL53L4CX and VL53L1X both sit at 0x29, and `layout._FOLLOWS`
    records the pairing) are one physical part, declared twice so the fitted one can win. Exactly one
    can ever be present, so listing the other as "what you fall back to" invents redundancy that does
    not exist -- the most dangerous kind of entry in a table whose whole job is to say what survives.

    Args:
        entries - the provider chain for one channel, primary first.

    Returns:
        The chain with socket-mates of the primary removed.
    """
    if not entries:
        return entries
    primary = entries[0][1]
    mates = set(layout._FOLLOWS.get(primary, ()))
    for owner, followers in layout._FOLLOWS.items():
        if primary in followers:
            mates.add(owner)
            mates.update(followers)
    mates.discard(primary)
    return [entries[0]] + [e for e in entries[1:] if e[1] not in mates]


def _degradation(channels: dict) -> str:
    """What the primary failing actually costs, per channel -- the question the order alone cannot answer."""
    lines = ['| channel | if the primary fails | consequence |', '|---|---|---|']
    for _title, group in _GROUPS:
        for channel in group:
            entries = _alternatives(channels.get(channel))
            if not entries:
                continue
            rest = entries[1:]
            if not rest:
                lines.append('| `%s` | **nothing** | the channel is LOST -- %s is its only source |' % (
                    channel, '`%s`' % entries[0][1]))
                continue
            lines.append('| `%s` | `%s` (p%d) | %s |' % (
                channel, rest[0][1], rest[0][0], _GRADE.get(rest[0][1], 'see the table above')))
    return '\n'.join(lines)


def _deltas(per_revision: dict) -> str:
    """What changes between consecutive revisions, per channel."""
    lines = []
    for older, newer in zip(_REVISIONS, _REVISIONS[1:]):
        lines.append('### %s → %s\n' % (older, newer))
        before, after = per_revision[older], per_revision[newer]
        rows = []
        for channel in sorted(set(before) | set(after)):
            was = {e[1] for e in before.get(channel, [])}
            now = {e[1] for e in after.get(channel, [])}
            if was == now:
                continue
            gone, added = sorted(was - now), sorted(now - was)
            rows.append('| `%s` | %s | %s | %d → %d |' % (
                channel, ', '.join('`%s`' % g for g in gone) or '—',
                ', '.join('`%s`' % a for a in added) or '—', len(was), len(now)))
        if rows:
            lines.append('| channel | lost | gained | sources |')
            lines.append('|---|---|---|---|')
            lines.extend(rows)
        else:
            lines.append('No channel changes source.')
        lines.append('')
    return '\n'.join(lines)


def _single_points(per_revision: dict) -> str:
    """Channels fed by exactly one provider -- where a single failure takes the channel with it."""
    lines = ['| channel | ' + ' | '.join(_REVISIONS) + ' |', '|---|' + '---|' * len(_REVISIONS)]
    every = sorted({c for channels in per_revision.values() for c in channels})
    for channel in every:
        cells = []
        for revision in _REVISIONS:
            entries = per_revision[revision].get(channel, [])
            cells.append('**1 — %s**' % entries[0][1] if len(entries) == 1 else '%d' % len(entries))
        if any('**' in cell for cell in cells):
            lines.append('| `%s` | %s |' % (channel, ' | '.join(cells)))
    return '\n'.join(lines)


def render() -> str:
    """The whole document."""
    defaults = _driver_defaults()
    per_revision = {revision: _providers(revision, defaults) for revision in _REVISIONS}
    parts = ['# Data sources — which sensor feeds which channel', '',
             'GENERATED by `tools/gen_datasources.py` from `config_default.py` + `layout.py`. Do not edit by',
             'hand: a priority changed in the config and not here would leave this describing a fallback order',
             'the board no longer uses. Re-run after any change to `provides`, and `make check` fails when it',
             'is stale.', '',
             'The databoard hands a consumer the **highest-priority provider that is still fresh** (`p0` first,',
             'then `p1`, ...), where fresh means it pushed within the channel\'s freshness window. A rate marked',
             '`*` comes from the driver default rather than the config.', '']
    for channel, note in sorted(_CHANNEL_NOTES.items()):
        parts.append('* `%s` — %s' % (channel, note))
    parts.append('')
    for revision in _REVISIONS:
        parts += ['## %s' % revision, '', _table(per_revision[revision]),
                  '_Degradation on primary failure:_', '', _degradation(per_revision[revision]), '']
    parts += ['## What changes between revisions', '', _deltas(per_revision),
              '## Single-source channels', '',
              'A channel with one provider has no fallback: that sensor failing takes the channel with it.', '',
              _single_points(per_revision), '']
    return '\n'.join(parts) + '\n'


def main() -> int:
    text = render()
    if '--check' in sys.argv:
        current = open(_OUT).read() if os.path.exists(_OUT) else ''
        if current != text:
            print('doc/datasources.md is STALE -- re-run tools/gen_datasources.py', file=sys.stderr)
            return 1
        print('doc/datasources.md up to date')
        return 0
    with open(_OUT, 'w') as handle:
        handle.write(text)
    print('wrote %s' % _OUT)
    return 0


if __name__ == '__main__':
    sys.exit(main())
