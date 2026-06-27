# Board configuration loader / validator — the Phase 0 foundation.
#
# Implements the three-layer model from specs/board-config.md:
# config_default.py (firmware default / fallback)
# board.config (saved active config, a full snapshot)
# in-memory dict (validated, what the Controller builds tasks from)
#
# Runs on MicroPython on the board. Validation here is config-file *integrity* (structure,
# types, pin uniqueness, bus refs, reserved pins) — NOT hardware health, which is checked at
# runtime and surfaced to the operator (the strict model).

import json
import os

import commons

try:
    import binascii
    import hashlib

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


def _is_int(x) -> bool:
    return isinstance(x, int) and not isinstance(x, bool)


# --------------------------------------------------------------------------- validate
#
# validate() is an orchestrator: it threads three accumulators through one per-section helper each,
# so a section's rules live in one named place. The accumulators are:
#   errs       -- the human-readable error strings (the return value)
#   pin_owner  -- gpio -> label that claimed it (pin-uniqueness + reserved-pin checks)
#   bus_refs   -- (kind, id) pairs the `buses` section defines, that devices may then reference


def _claim(errs: list, pin_owner: dict, label: str, pin) -> None:
    """Record `label`'s claim on a GPIO; flag a non-int/negative pin or a GPIO booked twice."""
    if not _is_int(pin) or pin < 0:
        errs.append('%s pin must be a non-negative int (got %r)' % (label, pin))
        return
    if pin in pin_owner:
        errs.append('GPIO%d used by both %s and %s' % (pin, pin_owner[pin], label))
    else:
        pin_owner[pin] = label


def _validate_board(board, errs: list):
    """Validate the `board` section; return its mcu (a known string), else None."""
    if not isinstance(board, dict):
        errs.append("missing or invalid 'board' section")
        return None
    bid = board.get('id')
    if not isinstance(bid, str) or not bid:
        errs.append('board.id must be a non-empty string')
    elif any(c in bid for c in ' \t\r\n'):
        errs.append('board.id must not contain whitespace (it is a bare wire token)')
    mcu = board.get('mcu')
    if not isinstance(mcu, str):
        errs.append('board.mcu must be a string')
        return None
    if mcu not in KNOWN_MCUS:
        errs.append("board.mcu '%s' is not one of %s" % (mcu, ', '.join(KNOWN_MCUS)))
    return mcu


def _validate_wifi(wifi, errs: list) -> None:
    """Validate the optional `wifi` section (STA-only, string ssid)."""
    if wifi is None:
        return
    if not isinstance(wifi, dict):
        errs.append("'wifi' must be an object")
        return
    if wifi.get('mode') not in ('sta', None):
        errs.append("wifi.mode must be 'sta'")
    if not isinstance(wifi.get('ssid', ''), str):
        errs.append('wifi.ssid must be a string')


def _validate_buses(buses, errs: list, pin_owner: dict, bus_refs: set) -> None:
    """Validate the `buses` section: claim each bus pin into `pin_owner` and collect every
    (kind, id) into `bus_refs` so devices can be checked against the buses they address."""
    if not isinstance(buses, dict):
        errs.append("missing or invalid 'buses' section")
        return
    for kind, group in buses.items():
        if kind not in ('uart', 'i2c', 'spi'):
            errs.append("bus type '%s' is not one of uart/i2c/spi" % kind)
        if not isinstance(group, dict):
            errs.append('buses.%s must be an object' % kind)
            continue
        for ident, spec in group.items():
            bus_refs.add((kind, str(ident)))  # ids are JSON object keys, so always strings
            label = '%s:%s' % (kind, ident)
            if not isinstance(spec, dict):
                errs.append('bus %s must be an object' % label)
                continue
            for key in _BUS_PIN_KEYS:
                if key in spec:
                    _claim(errs, pin_owner, label + '.' + key, spec[key])
            if kind == 'spi' and spec.get('mode', 0) not in (0, 1, 2, 3):  # machine.SPI: polarity/phase in {0,1}
                errs.append('bus %s.mode must be 0..3 (got %r)' % (label, spec.get('mode')))


def _validate_pins(pins, errs: list, pin_owner: dict) -> None:
    """Validate the discrete `pins` map, claiming each into `pin_owner`."""
    if not isinstance(pins, dict):
        errs.append("missing or invalid 'pins' section")
        return
    for pname, pin in pins.items():
        _claim(errs, pin_owner, 'pins.' + pname, pin)


def _validate_reserved(mcu, pin_owner: dict, errs: list) -> None:
    """Flag any claimed GPIO that is reserved for a core function (Wi-Fi/USB/console) on this mcu."""
    reserved = RESERVED_PINS.get(mcu)
    if not reserved:
        return
    for pin in sorted(pin_owner):
        if pin in reserved:
            errs.append('%s uses reserved GPIO%d (breaks Wi-Fi/USB/console on %s)' % (pin_owner[pin], pin, mcu))


def _validate_recorder(rec, errs: list) -> None:
    """Validate the optional `recorder` section (positive-int capacities/sizes)."""
    if rec is None:
        return
    if not isinstance(rec, dict):
        errs.append("'recorder' must be an object")
        return
    for k in ('tlm_capacity', 'log_capacity', 'cell_size', 'stats_ms'):
        if k in rec and not (_is_int(rec[k]) and rec[k] > 0):
            errs.append('recorder.%s must be a positive int' % k)


def _validate_devices(items, label: str, errs: list, bus_refs: set, seen_names: set) -> None:
    """Validate a `sensors`/`components` list: unique names (across both lists, via `seen_names`),
    an implementation named (`driver` or `activity`), any bus ref defined (in `bus_refs`), and any
    `provides` quantities well-formed."""
    if items is None:
        return
    if not isinstance(items, list):
        errs.append("'%s' must be a list" % label)
        return
    for i, dev in enumerate(items):
        where = '%s[%d]' % (label, i)
        if not isinstance(dev, dict):
            errs.append('%s must be an object' % where)
            continue
        name = dev.get('name')
        if not isinstance(name, str) or not name:
            errs.append('%s.name must be a non-empty string' % where)
        else:
            where = "%s '%s'" % (label, name)
            if name in seen_names:
                errs.append("duplicate device name '%s'" % name)
            seen_names.add(name)
        runs = dev.get('driver') or dev.get('activity')  # drivers/ via `driver`, tasks/ via `activity`
        if not isinstance(runs, str) or not runs:
            errs.append('%s must name an implementation: `driver` (drivers/) or `activity` (tasks/)' % where)
        if 'enabled' in dev and not isinstance(dev['enabled'], bool):
            errs.append('%s.enabled must be a bool' % where)
        kind = dev.get('bus')  # a device addresses its bus by kind ('i2c') + id (0)
        if kind is not None:
            ident = dev.get('id')
            if not isinstance(kind, str):
                errs.append('%s.bus must be a string (the bus kind, e.g. "i2c")' % where)
            elif not _is_int(ident):
                errs.append('%s.id must be the int bus id (with bus "%s")' % (where, kind))
            elif (kind, str(ident)) not in bus_refs:
                errs.append("%s addresses bus %s:%s which is not defined" % (where, kind, ident))
        addr = dev.get('addr')
        if addr is not None and not _is_int(addr):
            errs.append('%s.addr must be an int or null' % where)
        provides = dev.get('provides')
        if provides is not None:
            if not isinstance(provides, dict):
                errs.append('%s.provides must be an object' % where)
            else:
                for quantity, spec in provides.items():
                    if not isinstance(spec, dict):
                        errs.append('%s.provides.%s must be an object' % (where, quantity))
                        continue
                    if not _is_int(spec.get('priority')):
                        errs.append('%s.provides.%s.priority must be an int' % (where, quantity))
                    if not _is_int(spec.get('timeout_ms')):
                        errs.append('%s.provides.%s.timeout_ms must be an int' % (where, quantity))


def validate(cfg) -> list:
    """Return a list of human-readable error strings (empty list == valid). Config-file *integrity*
    only -- structure, types, pin uniqueness, bus refs, reserved pins -- NOT hardware health, which
    is checked at runtime and surfaced to the operator (the strict model)."""
    if not isinstance(cfg, dict):
        return ['config is not an object']
    errs = []
    pin_owner = {}  # gpio -> label that claimed it
    bus_refs = set()  # (kind, id) pairs referenced by sensors/components, e.g. ('i2c', '0')
    seen_names = set()  # device names, unique across sensors + components
    mcu = _validate_board(cfg.get('board'), errs)
    _validate_wifi(cfg.get('wifi'), errs)
    _validate_buses(cfg.get('buses'), errs, pin_owner, bus_refs)
    _validate_pins(cfg.get('pins'), errs, pin_owner)
    _validate_reserved(mcu, pin_owner, errs)
    _validate_recorder(cfg.get('recorder'), errs)
    _validate_devices(cfg.get('sensors'), 'sensors', errs, bus_refs, seen_names)
    _validate_devices(cfg.get('components'), 'components', errs, bus_refs, seen_names)
    return errs


# --------------------------------------------------------------------------- config_id


def _canon(o) -> str:
    """Deterministic, sorted-key serialization (no json.dumps options needed)."""
    if isinstance(o, dict):
        return '{' + ','.join(repr(k) + ':' + _canon(o[k]) for k in sorted(o.keys())) + '}'
    if isinstance(o, (list, tuple)):
        return '[' + ','.join(_canon(x) for x in o) + ']'
    return repr(o)


def config_id(cfg) -> str:
    """A stable short hash identifying a config snapshot (for the CC iam/config_id)."""
    s = _canon(cfg)
    if _HAVE_HASH:
        return binascii.hexlify(hashlib.sha256(s.encode()).digest()).decode()[:12]
    acc = 2166136261  # FNV-1a fallback
    for ch in s:
        acc = ((acc ^ ord(ch)) * 16777619) & 0xFFFFFFFF
    return '%08x' % acc


# --------------------------------------------------------------------------- load / save


def _builtin_default() -> dict:
    import config_default

    return config_default.default()


def load(path: str = 'board.config', defaults=None) -> tuple:
    """Layered load: active board.config if present and valid, else defaults.

    Returns (cfg, source, errors). `source` is 'active', 'default', or a fallback reason.
    Never raises — a missing/corrupt/invalid active file degrades to defaults so the board is
    always reachable.
    """
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
        return defaults, 'default(fallback: board.config is not valid JSON)', ['board.config is not valid JSON']
    errs = validate(data)
    if errs:
        return defaults, 'default(fallback: invalid board.config)', errs
    return data, 'active', []


def save(cfg, path: str = 'board.config') -> str:
    """Validate then atomically persist a full config snapshot. Returns its config_id.

    Raises ValueError if invalid (an invalid config is never written).
    """
    errs = validate(cfg)
    if errs:
        raise ValueError('invalid config: ' + '; '.join(errs))
    commons.atomic_write_json(path, cfg)  # validated above -> persist the snapshot atomically
    return config_id(cfg)


def reset(path: str = 'board.config') -> bool:
    """Delete the active config so the next load uses defaults. Returns True if removed."""
    try:
        os.remove(path)
        return True
    except OSError:
        return False


def bus(cfg, kind, ident) -> dict:
    """Resolve a bus addressed by `kind` ('uart'/'i2c'/'spi') + `ident` (its id) to its spec dict,
    or None. Ids are JSON object keys (always strings), so the int id from a component is normalized
    here -- callers pass `device['bus'], device['id']` and never parse a 'type:id' string."""
    return cfg.get('buses', {}).get(kind, {}).get(str(ident))


def device(cfg, name=None, driver=None) -> dict:
    """Find a sensor/component by `name` and/or implementation. `driver` matches the resolved
    implementation -- a component's `driver` (drivers/) or `activity` (tasks/) field. Returns the
    dict or None."""
    for item in cfg.get('sensors', []) + cfg.get('components', []):
        runs = item.get('driver') or item.get('activity')
        if (name is None or item.get('name') == name) and (driver is None or runs == driver):
            return item
    return None
