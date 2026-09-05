"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Generate doc/waveshare_esp32p4_pins.md FROM config_default.py, so the pin doc can never drift stale (the
old hand-maintained one nearly sent a solder to phantom codec pins). CPython; reads the MicroPython
config through its const shim.

BOTH revisions are rendered, v1.0 first (the board being ordered), then the transition, then v0.1 as
built. Neither is hand-written: v1.0 is `layout.apply(default, 'v1.0')` and the transition is derived
from the same `layout` tables the firmware votes on at boot, so the doc cannot describe a rewire the
detector does not implement. Run from the repo root:
    python3 tools/gen_pinmap.py            # rewrites doc/waveshare_esp32p4_pins.md
    python3 tools/gen_pinmap.py --check    # non-zero exit if the doc is out of date (CI hook)
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'glider'))

import config as config_mod
import config_default
import layout

_DOC = os.path.join(os.path.dirname(__file__), '..', 'doc', 'waveshare_esp32p4_pins.md')
_PIN_FIELDS = ('pin', 'cs_pin', 'int_pin', 'xshut_pin', 'alert_pin')  # device fields naming a pins entry


def _enabled(gpio) -> bool:
    """A pins value is a live GPIO only when it is a non-negative int; null / any negative means the
    feature is wired off (matches task._pin_gpio and config._validate_pins)."""
    return isinstance(gpio, int) and not isinstance(gpio, bool) and gpio >= 0


def _devices(cfg: dict) -> list:
    return cfg.get('sensors', []) + cfg.get('components', [])


def _pin_users(cfg: dict) -> dict:
    """gpio -> [labels]: every device pin field + bus pin that lands on a GPIO (disabled pins skipped)."""
    pins = cfg.get('pins', {})
    users: dict = {}
    for kind, buses in cfg.get('buses', {}).items():
        for bus_id, spec in buses.items():
            for key, value in spec.items():
                if key in ('sda', 'scl', 'sck', 'mosi', 'miso', 'tx', 'rx') and isinstance(value, int):
                    users.setdefault(value, []).append('%s:%s %s' % (kind, bus_id, key))
    for device in _devices(cfg):
        # A DISABLED device claims nothing. Without this the v1.0 map kept the ADXL375's cs and int
        # pins, so the transition reported GPIO4 and GPIO49 as "unchanged, no re-check needed" when
        # they are in fact freed -- the exact opposite of what someone at the bench needs to read.
        if not device.get('enabled', True):
            continue
        for field in _PIN_FIELDS:
            name = device.get(field)
            gpio = pins.get(name) if name else None
            if _enabled(gpio):
                users.setdefault(gpio, []).append('%s.%s' % (device['name'], field))
    return users


"""
Breakout modules whose SILK LABELS do not match their SPI function -- the ones this project has
mis-soldered before. Rendered per revision so the table only appears where the part is fitted.

`source` is how the GPIO is resolved: ('spi', key) reads the bus spec, ('pin', field) reads the
device's own pin field. Nothing here is a literal GPIO, so the table cannot drift from the config the
firmware actually uses -- which is the whole reason a hand-written copy of this went wrong.
"""
_MODULE_PINOUT: dict = {
    'imu_lsm6dso32': {
        'title': 'LSM6DSO32 breakout — PRIMARY row only',
        'note': ('This breakout carries a **second, AUXILIARY** interface (the sensor-hub / OIS port for '
                 'an external magnetometer), so `SCL`/`SCX` and `DO`/`DO` BOTH appear on the board. The '
                 'auxiliary port is a separate peripheral: clocking it does nothing for the primary bus, '
                 'and a part wired to it goes silent on SPI **and** I²C — reading exactly like a dead chip.'),
        'rows': (('VIN *(bottom 1)*', 'power', None, '3V3'),
                 ('GND *(top 5)*', 'ground', None, 'GND'),
                 ('**SCL** *(bottom 4)*', 'SPI clock (SCK) — **NOT `SCX`**', ('spi', 'sck'), None),
                 ('**SDA** *(bottom 5)*', 'SPI MOSI (SDI)', ('spi', 'mosi'), None),
                 ('**DO** *(bottom 6)*', 'SPI **MISO** (SDO) — **NOT the top-row `DO`**', ('spi', 'miso'), None),
                 ('CS *(bottom 7)*', 'chip-select', ('pin', 'cs_pin'), None),
                 ('I1 *(bottom 8)*', 'INT1 data-ready', ('pin', 'int_pin'), None)),
        'rows_ascii': ('  bottom row (PRIMARY -- use this one):   VIN  3Vo  GND  SCL  SDA  DO  CS  I1  I2\n'
                       '  top row    (AUXILIARY -- do NOT use):   SCX  SDX  CS   DO   GND'),
        'history': ('**v0.1 got this wrong on both built boards**, taking the clock from `SCX` and the '
                    'data-out from the top-row `DO` — both auxiliary. MOSI, CS, INT1, VIN and GND were '
                    'correct, which is why two jumpers repaired TMS-7C and TMS-7D.'),
    },
    'accel_adxl375': {
        'title': 'ADXL375 breakout — SPI pins silk-printed with I²C names',
        'note': ('On the Adafruit board the SPI data pins carry their I²C labels, so the mapping is not '
                 'one-to-one: **SDA = MOSI** and **SDO = MISO**.'),
        'rows': (('VIN', 'power', None, '3V3'),
                 ('GND', 'ground', None, 'GND'),
                 ('SCL', 'SPI clock (SCK)', ('spi', 'sck'), None),
                 ('**SDA**', 'SPI **MOSI** (SDI)', ('spi', 'mosi'), None),
                 ('**SDO**', 'SPI **MISO**', ('spi', 'miso'), None),
                 ('CS', 'chip-select (active low)', ('pin', 'cs_pin'), None),
                 ('INT1', 'DATA_READY', ('pin', 'int_pin'), None)),
        'rows_ascii': None,
        'history': None,
    },
}


def _module_tables(cfg: dict) -> list:
    """Per-module solder tables for the parts whose labels have caused a mis-wire here."""
    pins = cfg.get('pins', {})
    out = []
    for device in _devices(cfg):
        entry = _MODULE_PINOUT.get(device.get('name'))
        if entry is None or not device.get('enabled', True):
            continue
        spec = cfg.get('buses', {}).get(device.get('bus'), {}).get(str(device.get('id')), {})
        out.append('### %s\n' % entry['title'])
        out.append(entry['note'] + '\n')
        if entry['rows_ascii']:
            out.append('```')
            out.append(entry['rows_ascii'])
            out.append('```\n')
        out.append('| module pin | meaning | ESP32-P4 GPIO |')
        out.append('|---|---|---|')
        for label, meaning, source, literal in entry['rows']:
            if literal is not None:
                gpio = literal
            elif source[0] == 'spi':
                gpio = '**%s**' % spec.get(source[1], '?')
            else:
                gpio = '**%s**' % pins.get(device.get(source[1]), '?')
            out.append('| %s | %s | %s |' % (label, meaning, gpio))
        out.append('')
        if entry['history']:
            out.append('> ⚠️ ' + entry['history'] + '\n')
    return out


def _section(cfg: dict, title: str, note: str) -> list:
    """One revision's buses / GPIO / device tables."""
    pins = cfg.get('pins', {})
    reserved = set(config_mod.RESERVED_PINS.get(cfg['board'].get('mcu', ''), ()))
    users = _pin_users(cfg)
    out = []
    out.append('# %s\n' % title)
    out.append(note + '\n')

    out.append('## Buses\n')
    out.append('| Bus | id | Pins |')
    out.append('|---|---|---|')
    for kind in ('i2c', 'spi', 'uart'):
        for bus_id, spec in sorted(cfg.get('buses', {}).get(kind, {}).items()):
            detail = ', '.join('%s %s' % (k, v) for k, v in spec.items() if k != 'freq' and k != 'baud')
            rate = spec.get('freq') or spec.get('baud')
            out.append('| `%s` | %s | %s%s |' % (kind, bus_id, detail, ' @ %s' % rate if rate else ''))
    out.append('')

    out.append('## GPIO assignments\n')
    out.append('| GPIO | Claimed by |')
    out.append('|---|---|')
    for gpio in sorted(users):
        tag = ' *(reserved!)*' if gpio in reserved else ''
        out.append('| %d | %s%s |' % (gpio, ', '.join(users[gpio]), tag))
    out.append('')

    disabled = sorted(n for n, g in pins.items() if not _enabled(g))
    if disabled:
        out.append('**Disabled pins** (declared `null`/`-1` -> feature wired off): '
                   + ', '.join('`%s`' % n for n in disabled) + '.\n')

    out.append('## Device -> pins\n')
    out.append('| Device | Bus | Pin fields |')
    out.append('|---|---|---|')
    # A NOT-FITTED device is listed below the table, not in it. Leaving it in contradicted this same
    # document: the v1.0 map showed accel_adxl375 holding GPIO49 and GPIO4 while the transition section
    # listed both as freed. A reader at the bench has to be able to trust one of those.
    for device in [d for d in _devices(cfg) if d.get('enabled', True)]:
        bus = ('%s:%s' % (device['bus'], device['id'])) if 'bus' in device else '-'
        fields = []
        for field in _PIN_FIELDS:
            name = device.get(field)
            if not name:
                continue
            gpio = pins.get(name)
            shown = 'GPIO%d' % gpio if _enabled(gpio) else 'off'
            fields.append('%s=%s (%s)' % (field, name, shown))
        addr = ' @ 0x%02X' % device['addr'] if isinstance(device.get('addr'), int) else ''
        out.append('| `%s` | %s%s | %s |' % (device['name'], bus, addr, ', '.join(fields) or '-'))
    out.append('')

    # HARDWARE only: a disabled device that names a bus or a pin is a part that is not on the board,
    # while a disabled `flight` / `field` / `watchdog` is a software task switched off. Listing them
    # together under "not fitted" would read as missing silicon.
    out.append('## Module solder maps (label traps)\n')
    out.append('The breakouts whose silk labels do not match their SPI function. Every GPIO below is '
               'read from the same config the firmware uses, so this cannot drift from it.\n')
    out += _module_tables(cfg)

    absent = sorted(d['name'] for d in _devices(cfg)
                    if not d.get('enabled', True)
                    and ('bus' in d or any(d.get(f) for f in _PIN_FIELDS)))
    if absent:
        out.append('**Not fitted on this revision:** ' + ', '.join('`%s`' % n for n in absent)
                   + ' -- physically absent, disabled in config, claiming no pins.\n')

    out.append('## Reserved (never assign)\n')
    out.append(', '.join('GPIO%d' % p for p in sorted(reserved))
               + ' -- ESP32-P4 flash/PSRAM/USB/console straps.\n')
    return out


def _transition(old: dict, new: dict) -> list:
    """
    The v0.1 -> v1.0 worklist, DERIVED by diffing the two configs rather than written by hand.

    Deriving it means the doc cannot describe a rewire the firmware does not implement: both sides come
    out of layout.apply(), which is the same table layout.detect() votes against at boot.
    """
    out = ['# Transition v0.1 -> v1.0\n',
           'What to change on a v0.1 board. Derived by diffing the two configs, so it always matches '
           'what `layout.apply()` actually does.\n']

    moves = []
    for device in _devices(new):
        was = next((d for d in _devices(old) if d['name'] == device['name']), None)
        if was is None or 'bus' not in device:
            continue
        if (was.get('bus'), was.get('id')) != (device.get('bus'), device.get('id')):
            moves.append('| `%s` | %s:%s | **%s:%s** |'
                         % (device['name'], was.get('bus'), was.get('id'), device.get('bus'), device.get('id')))
    if moves:
        out.append('## Devices that change bus (same physical pins)\n')
        out.append('| Device | v0.1 | v1.0 |')
        out.append('|---|---|---|')
        out.extend(moves)
        out.append('')

    rates = []
    for kind in ('i2c', 'spi', 'uart'):
        for bus_id, spec in sorted(new.get('buses', {}).get(kind, {}).items()):
            before = old.get('buses', {}).get(kind, {}).get(bus_id, {})
            for key in ('freq', 'baud'):
                if key in spec and before.get(key) != spec[key]:
                    rates.append('| `%s:%s` %s | %s | **%s** |' % (kind, bus_id, key, before.get(key), spec[key]))
    if rates:
        out.append('## Bus rates\n')
        out.append('| Bus | v0.1 | v1.0 |')
        out.append('|---|---|---|')
        out.extend(rates)
        out.append('')

    freed = sorted(set(_pin_users(old)) - set(_pin_users(new)))
    if freed:
        out.append('## GPIOs freed\n')
        out.append('| GPIO | was |')
        out.append('|---|---|')
        for gpio in freed:
            out.append('| %d | %s |' % (gpio, ', '.join(_pin_users(old)[gpio])))
        out.append('')

    gone = [d['name'] for d in _devices(old)
            if d.get('enabled', True) and not next(
                (n for n in _devices(new) if n['name'] == d['name'] and n.get('enabled', True)), None)]
    if gone:
        out.append('**Not fitted on v1.0:** ' + ', '.join('`%s`' % n for n in gone) + '.\n')

    unchanged = sorted(set(_pin_users(old)) & set(_pin_users(new)))
    out.append('**Unchanged, no re-check needed:** ' +
               ', '.join('GPIO%d' % g for g in unchanged) + '.\n')
    return out


def render(cfg: dict) -> str:
    """The whole doc: v1.0 first (the board being ordered), the transition, then v0.1 as built."""
    v01 = config_default.default()
    layout.apply(v01, 'v0.1')
    v10 = config_default.default()
    layout.apply(v10, 'v1.0')

    banner = ('> **GENERATED from `src/glider/config_default.py` + `src/glider/layout.py` by '
              '`tools/gen_pinmap.py` -- do not hand-edit.** Regenerate after any bus/pin change '
              '(`python3 tools/gen_pinmap.py`); `--check` fails CI if it is stale. `config_default` is '
              'the pin source of truth, validated on hardware by `test/test_pins.py`.')
    out = [banner + '\n']
    out += _section(v10, 'ESP32-P4 pin map -- v1.0 (target layout)',
                    'The board being ordered. Derived by applying the v1.0 revision to the firmware '
                    'defaults, so it matches what `layout.apply()` installs when detection picks v1.0.')
    out += _transition(v01, v10)
    out += _section(v01, 'ESP32-P4 pin map -- v0.1 (as built)',
                    'TMS-7C, TMS-7D and the breadboard until the transition above is done. A config '
                    'with no `layout` key resolves here.')
    return '\n'.join(out)


def main() -> int:
    text = render(config_default.default())
    check = '--check' in sys.argv
    if os.path.exists(_DOC):
        with open(_DOC) as handle:
            existing = handle.read()
    else:
        existing = None
    if check:
        if existing != text:
            print('doc/waveshare_esp32p4_pins.md is STALE -- run: python3 tools/gen_pinmap.py')
            return 1
        print('pin doc up to date')
        return 0
    with open(_DOC, 'w') as handle:
        handle.write(text)
    print('wrote', os.path.relpath(_DOC))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
