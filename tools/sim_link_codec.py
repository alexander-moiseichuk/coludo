"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The HITL sim-link's frames (doc/specs/sim-link.md) in Python: SENSORS (host -> board) and FINS (board -> host), both
ends of each, as crates/sim_link holds them for the Rust side (src/control/test/test_tools.py pins the bytes). Stdlib
only, so a process running plan B under the golden shims (tools/planb_sitl.py) can import it as tools/hitl_host.py
does.
"""

import binascii
import struct

_MAGIC: bytes = b'CSL1'
_VERSION: int = 1
_SENSORS: int = 1
_FINS: int = 2
_SENSORS_BYTES: int = 104
_FINS_BYTES: int = 32

def encode_sensors(seq: int, sim_time_ms: int, fields: dict) -> bytes:
    """
    One SENSORS frame (doc/specs/sim-link.md): little-endian, presence flags, CRC-32 trailer.

    Args:
        seq - the frame's sequence number (wraps at 2^32).
        sim_time_ms - the physics clock in ms (wraps at 2^32).
        fields - accel, rate, attitude, mag (3 floats), altitude, elevation, agl, airspeed, dynamic_pressure,
            speed, course (a float), position ([lat, lon] in 1e-7 degree ints); a missing or None field is absent.

    Returns:
        The 104-byte frame.
    """
    frame = bytearray(_SENSORS_BYTES)
    flags = 0
    for name, bit, shape, offset in _LAYOUT:
        value = fields.get(name)
        if value is None:
            continue
        flags |= 1 << bit
        struct.pack_into(shape, frame, offset, *(value if isinstance(value, (list, tuple)) else (value,)))
    struct.pack_into('<4sBBHII', frame, 0, _MAGIC, _VERSION, _SENSORS, flags, seq & 0xFFFFFFFF,
                     sim_time_ms & 0xFFFFFFFF)
    struct.pack_into('<I', frame, 100, binascii.crc32(bytes(frame[:100])))
    return bytes(frame)


_LAYOUT: tuple = (('accel', 0, '<3f', 16), ('rate', 1, '<3f', 28), ('attitude', 2, '<3f', 40), ('mag', 3, '<3f', 52),
                  ('altitude', 4, '<f', 64), ('elevation', 5, '<f', 68), ('agl', 6, '<f', 72),
                  ('airspeed', 7, '<f', 76), ('dynamic_pressure', 8, '<f', 80), ('position', 9, '<2i', 84),
                  ('speed', 10, '<f', 92), ('course', 11, '<f', 96))
"""A SENSORS frame's fields: name, presence bit, struct shape, offset (doc/specs/sim-link.md)."""


def decode_sensors(frame: bytes) -> dict | None:
    """
    One SENSORS frame as the board decodes it (tools/planb_sitl.py's end of the link), or None for anything that is
    not one (length, magic, version, kind, CRC).

    Args:
        frame - the datagram.

    Returns:
        {seq, sim_time_ms, fields: {name: a float, a list of 3 floats, or [lat, lon] 1e-7 degree ints}}, a field
        absent from `fields` when its presence bit is clear.
    """
    if len(frame) != _SENSORS_BYTES or frame[:4] != _MAGIC or frame[4] != _VERSION or frame[5] != _SENSORS:
        return None
    if struct.unpack_from('<I', frame, 100)[0] != binascii.crc32(frame[:100]):
        return None
    flags, seq, sim_time_ms = struct.unpack_from('<HII', frame, 6)
    fields = {}
    for name, bit, shape, offset in _LAYOUT:
        if flags & (1 << bit):
            value = struct.unpack_from(shape, frame, offset)
            fields[name] = list(value) if len(value) > 1 else value[0]
    return {'seq': seq, 'sim_time_ms': sim_time_ms, 'fields': fields}


def encode_fins(seq: int, uptime_ms: int, stage: int, armed: bool, launched: bool, fins: list | None) -> bytes:
    """
    One FINS answer (doc/specs/sim-link.md), as a board encodes it.

    Args:
        seq - the answered frame's sequence number.
        uptime_ms - the answering clock, ms.
        stage - the stage id (1 SETTING .. 5 DONE).
        armed, launched - the flight word's flags.
        fins - [left, right, yaw] whole degrees, or None when no fins are driven.

    Returns:
        The 32-byte frame.
    """
    frame = bytearray(_FINS_BYTES)
    struct.pack_into('<4sBBHII', frame, 0, _MAGIC, _VERSION, _FINS, 0 if fins is None else 1, seq & 0xFFFFFFFF,
                     uptime_ms & 0xFFFFFFFF)
    frame[16], frame[17], frame[18] = stage, int(armed), int(launched)
    if fins is not None:
        struct.pack_into('<3h', frame, 20, *fins)
    struct.pack_into('<I', frame, 28, binascii.crc32(bytes(frame[:28])))
    return bytes(frame)


def decode_fins(frame: bytes) -> dict | None:
    """
    One FINS frame, or None for anything that is not one (length, magic, version, kind, CRC).

    Args:
        frame - the datagram.

    Returns:
        {seq, uptime_ms, stage, armed, launched, fins: [left, right, yaw] whole degrees or None}.
    """
    if len(frame) != _FINS_BYTES or frame[:4] != _MAGIC or frame[4] != _VERSION or frame[5] != _FINS:
        return None
    if struct.unpack_from('<I', frame, 28)[0] != binascii.crc32(frame[:28]):
        return None
    flags, seq, uptime_ms = struct.unpack_from('<HII', frame, 6)
    return {'seq': seq, 'uptime_ms': uptime_ms, 'stage': frame[16], 'armed': bool(frame[17]),
            'launched': bool(frame[18]), 'fins': list(struct.unpack_from('<3h', frame, 20)) if flags & 1 else None}
