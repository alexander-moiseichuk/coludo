# Board configuration loader / validator — the Phase 0 foundation.
#
# Implements the three-layer model from specs/board-config.md:
#   config_default.py  (firmware default / fallback)
#   board.json         (saved active config, a full snapshot)
#   in-memory dict     (validated, what the Controller builds tasks from)
#
# Runs on MicroPython on the board. Validation here is config-file *integrity* (structure,
# types, pin uniqueness, bus refs, reserved pins) — NOT hardware health, which is checked at
# runtime and surfaced to the operator (the strict model).

import json
import os

try:
    import hashlib
    import binascii
    _HAVE_HASH = True
except ImportError:
    _HAVE_HASH = False

KNOWN_MCUS = ('esp32p4', 'esp32c6', 'firebeetle2p4')

# GPIOs that must never be assigned: doing so breaks a core function. For the WaveShare
# ESP32-P4-WIFI6 these are the ESP32-C6 Wi-Fi link (6, 14-19, 54), USB-JTAG (24, 25) and the
# serial console (37, 38). See doc/waveshare_esp32p4_pins.md.
RESERVED_PINS = {
    'esp32p4': (6, 14, 15, 16, 17, 18, 19, 24, 25, 37, 38, 54),
}

_BUS_PIN_KEYS = ('sda', 'scl', 'tx', 'rx', 'sck', 'mosi', 'miso')


def _is_int(x):
    return isinstance(x, int) and not isinstance(x, bool)


# --------------------------------------------------------------------------- validate

def validate(cfg):
    '''Return a list of human-readable error strings (empty list == valid).'''
    errs = []
    if not isinstance(cfg, dict):
        return ['config is not an object']

    # board ----------------------------------------------------------------
    mcu = None
    board = cfg.get('board')
    if not isinstance(board, dict):
        errs.append("missing or invalid 'board' section")
    else:
        bid = board.get('id')
        if not isinstance(bid, str) or not bid:
            errs.append('board.id must be a non-empty string')
        elif any(c in bid for c in ' \t\r\n'):
            errs.append('board.id must not contain whitespace (it is a bare wire token)')
        mcu = board.get('mcu')
        if not isinstance(mcu, str):
            errs.append('board.mcu must be a string')
            mcu = None
        elif mcu not in KNOWN_MCUS:
            errs.append("board.mcu '%s' is not one of %s" % (mcu, ', '.join(KNOWN_MCUS)))

    # wifi (optional) ------------------------------------------------------
    wifi = cfg.get('wifi')
    if wifi is not None:
        if not isinstance(wifi, dict):
            errs.append("'wifi' must be an object")
        else:
            if wifi.get('mode') not in ('sta', None):
                errs.append("wifi.mode must be 'sta'")
            if not isinstance(wifi.get('ssid', ''), str):
                errs.append('wifi.ssid must be a string')

    # buses + pin-uniqueness ----------------------------------------------
    pin_owner = {}      # gpio -> label that claimed it

    def claim(label, pin):
        if not _is_int(pin) or pin < 0:
            errs.append('%s pin must be a non-negative int (got %r)' % (label, pin))
            return
        if pin in pin_owner:
            errs.append('GPIO%d used by both %s and %s' % (pin, pin_owner[pin], label))
        else:
            pin_owner[pin] = label

    bus_names = set()
    buses = cfg.get('buses')
    if not isinstance(buses, dict):
        errs.append("missing or invalid 'buses' section")
    else:
        for bname, spec in buses.items():
            bus_names.add(bname)
            if not isinstance(spec, dict):
                errs.append("bus '%s' must be an object" % bname)
                continue
            claimed = False
            for key in _BUS_PIN_KEYS:
                if key in spec:
                    claim(bname + '.' + key, spec[key])
                    claimed = True
            if not claimed:
                errs.append("bus '%s' declares no pins (need sda/scl, tx/rx or sck/mosi/miso)"
                            % bname)

    # discrete pins --------------------------------------------------------
    pins = cfg.get('pins')
    if not isinstance(pins, dict):
        errs.append("missing or invalid 'pins' section")
    else:
        for pname, pin in pins.items():
            claim('pins.' + pname, pin)

    # reserved pins --------------------------------------------------------
    reserved = RESERVED_PINS.get(mcu)
    if reserved:
        for pin in sorted(pin_owner):
            if pin in reserved:
                errs.append('%s uses reserved GPIO%d (breaks Wi-Fi/USB/console on %s)'
                            % (pin_owner[pin], pin, mcu))

    # recorder (optional) --------------------------------------------------
    rec = cfg.get('recorder')
    if rec is not None:
        if not isinstance(rec, dict):
            errs.append("'recorder' must be an object")
        else:
            for k in ('tel_slots', 'log_slots', 'slot_size', 'drain_ms'):
                if k in rec and not (_is_int(rec[k]) and rec[k] > 0):
                    errs.append('recorder.%s must be a positive int' % k)

    # components -----------------------------------------------------------
    comps = cfg.get('components')
    if not isinstance(comps, list):
        errs.append("missing or invalid 'components' section")
    else:
        seen = set()
        for i, c in enumerate(comps):
            where = 'components[%d]' % i
            if not isinstance(c, dict):
                errs.append('%s must be an object' % where)
                continue
            name = c.get('name')
            if not isinstance(name, str) or not name:
                errs.append('%s.name must be a non-empty string' % where)
            else:
                where = "component '%s'" % name
                if name in seen:
                    errs.append("duplicate component name '%s'" % name)
                seen.add(name)
            if not isinstance(c.get('driver'), str) or not c.get('driver'):
                errs.append('%s.driver must be a non-empty string' % where)
            if 'enabled' in c and not isinstance(c['enabled'], bool):
                errs.append('%s.enabled must be a bool' % where)
            bus = c.get('bus')
            if bus is not None:
                if not isinstance(bus, str):
                    errs.append('%s.bus must be a string' % where)
                elif bus not in bus_names:
                    errs.append("%s.bus '%s' is not a defined bus" % (where, bus))
            addr = c.get('addr')
            if addr is not None and not _is_int(addr):
                errs.append('%s.addr must be an int or null' % where)
            prov = c.get('provides')
            if prov is not None:
                if not isinstance(prov, dict):
                    errs.append('%s.provides must be an object' % where)
                else:
                    for q, qs in prov.items():
                        if not isinstance(qs, dict):
                            errs.append('%s.provides.%s must be an object' % (where, q))
                            continue
                        if not _is_int(qs.get('priority')):
                            errs.append('%s.provides.%s.priority must be an int' % (where, q))
                        if not _is_int(qs.get('timeout_ms')):
                            errs.append('%s.provides.%s.timeout_ms must be an int' % (where, q))
    return errs


# --------------------------------------------------------------------------- config_id

def _canon(o):
    '''Deterministic, sorted-key serialization (no json.dumps options needed).'''
    if isinstance(o, dict):
        return '{' + ','.join(repr(k) + ':' + _canon(o[k]) for k in sorted(o.keys())) + '}'
    if isinstance(o, (list, tuple)):
        return '[' + ','.join(_canon(x) for x in o) + ']'
    return repr(o)


def config_id(cfg):
    '''A stable short hash identifying a config snapshot (for the CC iam/config_id).'''
    s = _canon(cfg)
    if _HAVE_HASH:
        return binascii.hexlify(hashlib.sha256(s.encode()).digest()).decode()[:12]
    acc = 2166136261                                  # FNV-1a fallback
    for ch in s:
        acc = ((acc ^ ord(ch)) * 16777619) & 0xFFFFFFFF
    return '%08x' % acc


# --------------------------------------------------------------------------- load / save

def _builtin_default():
    import config_default
    return config_default.default()


def load(path='board.json', defaults=None):
    '''Layered load: active board.json if present and valid, else defaults.

    Returns (cfg, source, errors). `source` is 'active', 'default', or a fallback reason.
    Never raises — a missing/corrupt/invalid active file degrades to defaults so the board is
    always reachable.
    '''
    if defaults is None:
        defaults = _builtin_default()
    try:
        with open(path) as f:
            text = f.read()
    except OSError:
        return defaults, 'default', validate(defaults)
    try:
        data = json.loads(text)
    except (ValueError, OSError):
        return defaults, 'default(fallback: board.json is not valid JSON)', \
            ['board.json is not valid JSON']
    errs = validate(data)
    if errs:
        return defaults, 'default(fallback: invalid board.json)', errs
    return data, 'active', []


def save(cfg, path='board.json'):
    '''Validate then atomically persist a full config snapshot. Returns its config_id.

    Raises ValueError if invalid (an invalid config is never written).
    '''
    errs = validate(cfg)
    if errs:
        raise ValueError('invalid config: ' + '; '.join(errs))
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        f.write(json.dumps(cfg))
    try:
        os.rename(tmp, path)
    except OSError:                      # some VFS (FAT) won't rename onto an existing file
        try:
            os.remove(path)
        except OSError:
            pass
        os.rename(tmp, path)
    return config_id(cfg)


def reset(path='board.json'):
    '''Delete the active config so the next load uses defaults. Returns True if removed.'''
    try:
        os.remove(path)
        return True
    except OSError:
        return False
