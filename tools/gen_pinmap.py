# tools/gen_pinmap.py -- generate doc/waveshare_esp32p4_pins.md FROM config_default.py, so the pin
# doc can never drift stale (the old hand-maintained one nearly sent a solder to phantom codec pins).
# CPython; reads the MicroPython config through its const shim. Run from the repo root:
#   python3 tools/gen_pinmap.py            # rewrites doc/waveshare_esp32p4_pins.md
#   python3 tools/gen_pinmap.py --check    # non-zero exit if the doc is out of date (CI hook)

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'glider'))

import config as config_mod
import config_default

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
        for field in _PIN_FIELDS:
            name = device.get(field)
            gpio = pins.get(name) if name else None
            if _enabled(gpio):
                users.setdefault(gpio, []).append('%s.%s' % (device['name'], field))
    return users


def render(cfg: dict) -> str:
    pins = cfg.get('pins', {})
    reserved = set(config_mod.RESERVED_PINS.get(cfg['board'].get('mcu', ''), ()))
    users = _pin_users(cfg)
    out = []
    out.append('# ESP32-P4 pin map (Coludo)\n')
    out.append('> **GENERATED from `src/glider/config_default.py` by `tools/gen_pinmap.py` -- do not '
               'hand-edit.** Regenerate after any bus/pin change (`python3 tools/gen_pinmap.py`); '
               '`--check` fails CI if it is stale. `config_default` is the pin source of truth, '
               'validated on hardware by `test/test_pins.py`.\n')

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
    for device in _devices(cfg):
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

    out.append('## Reserved (never assign)\n')
    out.append(', '.join('GPIO%d' % p for p in sorted(reserved))
               + ' -- ESP32-P4 flash/PSRAM/USB/console straps.\n')
    return '\n'.join(out)


def main() -> int:
    text = render(config_default.default())
    check = '--check' in sys.argv
    existing = open(_DOC).read() if os.path.exists(_DOC) else None
    if check:
        if existing != text:
            print('doc/waveshare_esp32p4_pins.md is STALE -- run: python3 tools/gen_pinmap.py')
            return 1
        print('pin doc up to date')
        return 0
    open(_DOC, 'w').write(text)
    print('wrote', os.path.relpath(_DOC))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
