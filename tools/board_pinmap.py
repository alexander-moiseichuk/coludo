#!/usr/bin/env python3
# board_pinmap.py — read a board config and print the physical wiring map: which GPIO carries
# which bus line / discrete signal, and which device sits on which bus (with pins + I2C address).
# Helps lay out / trace a PCB without doing it by hand. Host tool (parses config, never imports
# firmware peripherals).
#
#   python3 tools/board_pinmap.py [board.config]    # default: the firmware default config

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src', 'glider'))
import config  # noqa: E402
from config_default import default  # noqa: E402

_BUS_KEYS = ('tx', 'rx', 'sda', 'scl', 'sck', 'mosi', 'miso')


def load(argv: list) -> dict:
    if len(argv) > 1:
        cfg, source, errs = config.load(argv[1], defaults=default())
        if errs:
            print('# warning: %s -> %s; errors: %s' % (argv[1], source, '; '.join(errs)))
        return cfg
    return default()


def main() -> None:
    cfg = load(sys.argv)
    pinmap = {}
    for kind, group in cfg.get('buses', {}).items():
        for ident, spec in group.items():
            for key in _BUS_KEYS:
                if key in spec:
                    pinmap[spec[key]] = '%s:%s.%s' % (kind, ident, key)
    for name, gpio in cfg.get('pins', {}).items():
        pinmap[gpio] = name

    print('# %s wiring map (%s)\n' % (cfg['board']['id'], cfg['board'].get('mcu', '')))
    print('| GPIO | role |')
    print('|------|------|')
    for gpio in sorted(pinmap):
        print('| GPIO%d | %s |' % (gpio, pinmap[gpio]))

    print('\n## Devices\n')
    for label, items in (('sensors', cfg.get('sensors', [])), ('components', cfg.get('components', []))):
        if not items:
            continue
        print('### %s\n' % label)
        for device in items:
            ref = device.get('bus')
            spec = config.bus(cfg, ref) if ref else None
            pins = ', '.join('%s=GPIO%d' % (key.upper(), spec[key]) for key in _BUS_KEYS if spec and key in spec)
            addr = device.get('addr')
            address = ' addr 0x%02X' % addr if isinstance(addr, int) else ''
            disabled = '' if device.get('enabled', True) else ' (disabled)'
            print('- **%s** (`%s`) -> `%s` [%s]%s%s'
                  % (device['name'], device.get('driver'), ref or '-', pins, address, disabled))
        print()


if __name__ == '__main__':
    main()
