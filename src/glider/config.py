"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Board configuration loader / validator -- the foundational config layer the rest of the firmware
builds its tasks from.

Implements the three-layer model from specs/board-config.md:
  config_default.py -- firmware default / fallback
  board.config      -- saved active config, a full snapshot
  in-memory dict    -- validated, what the Controller builds tasks from

Runs on MicroPython on the board. Validation here is config-file *integrity* (structure, types, pin
uniqueness, bus refs, reserved pins) -- NOT hardware health, which is checked at runtime and surfaced
to the operator (the strict model).
"""

import json
import os

import commons
import config_default

try:
    import binascii
    import hashlib

    _HAVE_HASH = True
except ImportError:
    _HAVE_HASH = False

KNOWN_MCUS = ('esp32p4', 'esp32c6', 'firebeetle2p4')

"""
GPIOs that must never be assigned: doing so breaks a core function. For the WaveShare ESP32-P4-WIFI6 these
are the ESP32-C6 Wi-Fi link (6, 14-19, 54), USB-JTAG (24, 25) and the serial console (37, 38). See
doc/waveshare_esp32p4_pins.md.
"""
RESERVED_PINS = {
    'esp32p4': (6, 14, 15, 16, 17, 18, 19, 24, 25, 37, 38, 54),
}

_BUS_PIN_KEYS = ('sda', 'scl', 'tx', 'rx', 'sck', 'mosi', 'miso')


def _is_int(x) -> bool:
    return isinstance(x, int) and not isinstance(x, bool)


"""
Validation: validate() is an orchestrator that threads three accumulators through one per-section
helper each, so a section's rules live in one named place. The accumulators are:
  errs      -- the human-readable error strings (the return value)
  pin_owner -- gpio -> label that claimed it (pin-uniqueness + reserved-pin checks)
  bus_refs  -- (kind, id) pairs the `buses` section defines, that devices may then reference
"""


def _claim(errs: list, pin_owner: dict, label: str, pin) -> None:
    """
    Record `label`'s claim on a GPIO into `pin_owner`.

    Args:
        errs - the error-strings accumulator (a bad or double-booked pin appends here).
        pin_owner - the gpio -> owner-label map claims accumulate into.
        label - the human-readable owner of this pin (for the error text).
        pin - the GPIO number being claimed.

    Returns:
        None. Appends to `errs` on a non-int / negative pin or a GPIO booked twice; else records the
        claim in `pin_owner`.
    """
    if not _is_int(pin) or pin < 0:
        errs.append('%s pin must be a non-negative int (got %r)' % (label, pin))
        return
    if pin in pin_owner:
        errs.append('GPIO%d used by both %s and %s' % (pin, pin_owner[pin], label))
    else:
        pin_owner[pin] = label


def _validate_board(board, errs: list):
    """
    Validate the `board` section.

    Args:
        board - the config's `board` object.
        errs - the error-strings accumulator.

    Returns:
        The mcu string when `board.mcu` is present and a string (whether or not it is a KNOWN_MCU),
        else None (missing/invalid section) so the reserved-pin check can be skipped.
    """
    if not isinstance(board, dict):
        errs.append("missing or invalid 'board' section")
        return None
    bid = board.get('id')
    if not isinstance(bid, str) or not bid:
        errs.append('board.id must be a non-empty string')
    elif any(char in bid for char in ' \t\r\n'):
        errs.append('board.id must not contain whitespace (it is a bare wire token)')
    mcu = board.get('mcu')
    if not isinstance(mcu, str):
        errs.append('board.mcu must be a string')
        return None
    if mcu not in KNOWN_MCUS:
        errs.append("board.mcu '%s' is not one of %s" % (mcu, ', '.join(KNOWN_MCUS)))
    return mcu


def _validate_wifi(wifi, errs: list) -> None:
    """
    Validate the optional `wifi` section (STA-only, string ssid) and its `networks` list.

    Network entries are full per-network configs (the top-level shape replicated); a bare string is
    sugar for {'ssid': entry}.

    Args:
        wifi - the config's `wifi` object, or None when the section is absent (a no-op).
        errs - the error-strings accumulator.

    Returns:
        None. Appends one string per malformed field (bad mode, non-string ssid, malformed network).
    """
    if wifi is None:
        return
    if not isinstance(wifi, dict):
        errs.append("'wifi' must be an object")
        return
    if wifi.get('mode') not in ('sta', None):
        errs.append("wifi.mode must be 'sta'")
    if not isinstance(wifi.get('ssid', ''), str):
        errs.append('wifi.ssid must be a string')
    networks = wifi.get('networks')
    if networks is None:
        return
    if not isinstance(networks, (list, tuple)):
        errs.append('wifi.networks must be a list')
        return
    for index, entry in enumerate(networks):
        if isinstance(entry, str):
            continue  # sugar for {'ssid': entry}
        if not isinstance(entry, dict) or not entry.get('ssid') \
                or not isinstance(entry.get('ssid'), str):
            errs.append('wifi.networks[%d] needs a string ssid' % index)
        elif entry.get('mode') not in ('sta', None):
            errs.append("wifi.networks[%d].mode must be 'sta'" % index)


def _validate_buses(buses, errs: list, pin_owner: dict, bus_refs: set) -> None:
    """
    Validate the `buses` section.

    Claims each bus pin into `pin_owner` (so it participates in the pin-uniqueness / reserved checks)
    and collects every (kind, id) into `bus_refs` so a device can be checked against the buses it
    addresses.

    Args:
        buses - the config's `buses` object (grouped by kind then id).
        errs - the error-strings accumulator.
        pin_owner - the gpio -> owner map each bus pin is claimed into.
        bus_refs - the (kind, id) set every defined bus is added to.

    Returns:
        None. Mutates `pin_owner` / `bus_refs` and appends to `errs` on a malformed bus or bad mode.
    """
    if not isinstance(buses, dict):
        errs.append("missing or invalid 'buses' section")
        return
    for kind, group in buses.items():
        if kind not in ('uart', 'i2c', 'spi'):
            errs.append("bus type '%s' is not one of uart/i2c/spi" % kind)
            continue  # skip: don't register an invalid kind's ids (a ref would false-pass then fail at runtime)
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
    """
    Validate the discrete `pins` map, claiming each into `pin_owner`.

    A null / any-negative pin is a DISABLED optional pin (a feature wired off) and is skipped, not
    claimed (see task._pin_gpio).

    Args:
        pins - the config's `pins` object (name -> GPIO).
        errs - the error-strings accumulator.
        pin_owner - the gpio -> owner map each active pin is claimed into.

    Returns:
        None. Mutates `pin_owner` and appends to `errs` on a bad or double-booked pin.
    """
    if not isinstance(pins, dict):
        errs.append("missing or invalid 'pins' section")
        return
    for pname, pin in pins.items():
        if pin is None or (_is_int(pin) and pin < 0):
            continue  # DISABLED optional pin (null / any negative -> feature wired off; see task._pin_gpio)
        _claim(errs, pin_owner, 'pins.' + pname, pin)


def _validate_reserved(mcu, pin_owner: dict, errs: list) -> None:
    """
    Flag any claimed GPIO reserved for a core function (Wi-Fi / USB / console) on this mcu.

    Args:
        mcu - the board's mcu string (None or an mcu with no reserved list is a no-op).
        pin_owner - the gpio -> owner map accumulated by the section validators.
        errs - the error-strings accumulator.

    Returns:
        None. Appends one string per claimed pin that collides with the mcu's reserved set.
    """
    reserved = RESERVED_PINS.get(mcu)
    if not reserved:
        return
    for pin in sorted(pin_owner):
        if pin in reserved:
            errs.append('%s uses reserved GPIO%d (breaks Wi-Fi/USB/console on %s)' % (pin_owner[pin], pin, mcu))


def _validate_recorder(rec, errs: list) -> None:
    """
    Validate the optional `recorder` section (positive-int capacities / sizes).

    Args:
        rec - the config's `recorder` object, or None when absent (a no-op).
        errs - the error-strings accumulator.

    Returns:
        None. Appends one string per capacity/size key that is not a positive int.
    """
    if rec is None:
        return
    if not isinstance(rec, dict):
        errs.append("'recorder' must be an object")
        return
    for key in ('tlm_capacity', 'log_capacity', 'cell_size', 'stats_ms'):
        if key in rec and not (_is_int(rec[key]) and rec[key] > 0):
            errs.append('recorder.%s must be a positive int' % key)


def _validate_devices(items, label: str, errs: list, bus_refs: set, seen_names: set) -> None:
    """
    Validate a `sensors` / `components` list.

    Checks unique names (across both lists, via the shared `seen_names`), that an implementation is
    named (`driver` for drivers/, `activity` for tasks/), that any addressed bus is defined (in
    `bus_refs`), and that any `provides` quantities are well-formed.

    Args:
        items - the list to validate, or None when the section is absent (a no-op).
        label - the section name ('sensors' / 'components'), used in the error text.
        errs - the error-strings accumulator.
        bus_refs - the (kind, id) set of defined buses, to check each device's bus ref against.
        seen_names - the device-name set shared across both lists (for the duplicate check).

    Returns:
        None. Mutates `seen_names` and appends one string per malformed device / field.
    """
    if items is None:
        return
    if not isinstance(items, list):
        errs.append("'%s' must be a list" % label)
        return
    for index, dev in enumerate(items):
        where = '%s[%d]' % (label, index)
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
    """
    Validate a config for FILE INTEGRITY, threading the accumulators through the section helpers.

    Config-file integrity only -- structure, types, pin uniqueness, bus refs, reserved pins -- NOT
    hardware health, which is checked at runtime and surfaced to the operator (the strict model).

    Args:
        cfg - the config object to validate.

    Returns:
        A list of human-readable error strings; the empty list means valid. A non-dict `cfg` returns
        a single 'config is not an object' error.
    """
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


"""Config identity -- a stable short hash of a config snapshot (for the CC iam / config_id)."""


def _canon(o) -> str:
    """
    Deterministic, sorted-key serialisation of a config value (no json.dumps options needed).

    Args:
        o - the value to serialise (dict / list / tuple / scalar).

    Returns:
        A canonical string with dict keys sorted, so two equal configs serialise identically.
    """
    if isinstance(o, dict):
        return '{' + ','.join(repr(key) + ':' + _canon(o[key]) for key in sorted(o.keys())) + '}'
    if isinstance(o, (list, tuple)):
        return '[' + ','.join(_canon(item) for item in o) + ']'
    return repr(o)


def config_id(cfg) -> str:
    """
    A stable short hash identifying a config snapshot (for the CC iam / config_id).

    Args:
        cfg - the config object to hash.

    Returns:
        A 12-hex-char id: the SHA-256 prefix when hashlib is available, else an 8-hex FNV-1a fallback.
    """
    canonical = _canon(cfg)
    if _HAVE_HASH:
        return binascii.hexlify(hashlib.sha256(canonical.encode()).digest()).decode()[:12]
    acc = 2166136261  # FNV-1a fallback
    for char in canonical:
        acc = ((acc ^ ord(char)) * 16777619) & 0xFFFFFFFF
    return '%08x' % acc


"""Load / save -- the layered board.config file lifecycle and the bus / device lookups."""


def _builtin_default() -> dict:
    return config_default.default()


def load(path: str = 'board.config', defaults=None) -> tuple:
    """
    Layered load: the active board.config if present and valid, else the defaults.

    Never raises -- a missing / corrupt / invalid active file degrades to the defaults so the board
    is always reachable.

    Args:
        path - the active config file path (default 'board.config').
        defaults - the fallback config; None builds the firmware default via config_default.

    Returns:
        (cfg, source, errors). `source` is 'active' (the file was loaded), 'default' (no file), or a
        'default(fallback: ...)' reason (the file was bad JSON or failed validation); `errors` is the
        validation error list for whatever config was chosen.
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
    """
    Validate then atomically persist a full config snapshot.

    Args:
        cfg - the full config snapshot to persist.
        path - the destination path (default 'board.config').

    Returns:
        The config_id of the persisted snapshot.

    Raises:
        ValueError - if `cfg` is invalid (an invalid config is never written).
    """
    errs = validate(cfg)
    if errs:
        raise ValueError('invalid config: ' + '; '.join(errs))
    commons.atomic_write_json(path, cfg)  # validated above -> persist the snapshot atomically
    return config_id(cfg)


def reset(path: str = 'board.config') -> bool:
    """
    Delete the active config so the next load falls back to the defaults.

    Args:
        path - the active config file path (default 'board.config').

    Returns:
        True if the file was removed, False if it did not exist.
    """
    try:
        os.remove(path)
        return True
    except OSError:
        return False


def bus(cfg, kind, ident) -> dict:
    """
    Resolve a bus addressed by kind + id to its spec dict.

    Ids are JSON object keys (always strings), so the int id from a component is normalised here --
    callers pass `device['bus'], device['id']` and never parse a 'type:id' string.

    Args:
        cfg - the config to look in.
        kind - the bus kind ('uart' / 'i2c' / 'spi').
        ident - the bus id (int or string; coerced to string for the lookup).

    Returns:
        The bus spec dict, or None if no such bus is defined.
    """
    return cfg.get('buses', {}).get(kind, {}).get(str(ident))


def device(cfg, name=None, driver=None) -> dict:
    """
    Find a sensor / component by name and/or implementation.

    `driver` matches the resolved implementation -- a device's `driver` (drivers/) or `activity`
    (tasks/) field. Both filters are optional; with neither, the first device is returned.

    Args:
        cfg - the config to search.
        name - the device name to match, or None to match any name.
        driver - the implementation to match, or None to match any.

    Returns:
        The first matching device dict (sensors searched before components), or None.
    """
    for item in cfg.get('sensors', []) + cfg.get('components', []):
        runs = item.get('driver') or item.get('activity')
        if (name is None or item.get('name') == name) and (driver is None or runs == driver):
            return item
    return None
