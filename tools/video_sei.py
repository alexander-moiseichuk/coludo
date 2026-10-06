#!/usr/bin/env python3
"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

The per-frame stamps of a flight video: every access unit the avionics camera recorder writes
(src/avionics/firmware/components/coludo_video) carries one SEI user_data_unregistered NAL -- UUID
"coludo-video-v1", then boot_id u32, frame u32, uptime_microseconds u64, dropped u32 (little-endian). The
uptime is the board's esp_timer at the end of the frame's readout, the same clock as the telemetry's uptime
column, so this is how a frame is placed against the flight log.

    tools/video_sei.py <file.h264>              -> CSV (';'), one row per frame
    tools/video_sei.py <file.h264> --summary    -> frames, exact frame rate, interval spread
"""

import argparse
import struct
import sys

_UUID: bytes = b'coludo-video-v1\x00'


def nal_units(data: bytes):
    """
    The NAL units of a raw Annex-B stream.

    Args:
        data - the whole .h264 file.

    Yields:
        (nal_unit_type, rbsp) with the emulation-prevention bytes removed.
    """
    starts = []
    index = data.find(b'\x00\x00\x01')
    while index >= 0:
        starts.append(index + 3)
        index = data.find(b'\x00\x00\x01', index + 3)
    for number, start in enumerate(starts):
        end = starts[number + 1] - 3 if number + 1 < len(starts) else len(data)
        payload = data[start:end].rstrip(b'\x00')
        yield payload[0] & 0x1F, payload[1:].replace(b'\x00\x00\x03', b'\x00\x00')


def frames(data: bytes) -> list:
    """
    One stamp per encoded frame.

    Args:
        data - the whole .h264 file.

    Returns:
        [(frame, uptime_microseconds, boot_id, dropped, idr)] in stream order; idr is 1 for an IDR frame.
    """
    stamps = []
    pending = None
    for nal_type, rbsp in nal_units(data):
        if nal_type == 6 and rbsp[0] == 5 and rbsp[2:18] == _UUID:
            boot_id, frame, uptime, dropped = struct.unpack_from('<IIQI', rbsp, 18)
            pending = (frame, uptime, boot_id, dropped)
        elif nal_type in (1, 5) and pending is not None:
            stamps.append(pending + (int(nal_type == 5),))
            pending = None
    return stamps


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('video', help='a raw Annex-B .h264 file from the avionics recorder')
    parser.add_argument('--summary', action='store_true', help='frame rate and interval spread instead of rows')
    args = parser.parse_args()
    stamps = frames(open(args.video, 'rb').read())
    if not stamps:
        sys.exit('%s: no coludo SEI stamps' % args.video)
    if not args.summary:
        print('frame;uptime_microseconds;boot_id;dropped;idr')
        for stamp in stamps:
            print(';'.join(str(value) for value in stamp))
        return
    uptimes = [stamp[1] for stamp in stamps]
    intervals = sorted((later - earlier) / 1000 for earlier, later in zip(uptimes, uptimes[1:]))
    span = (uptimes[-1] - uptimes[0]) / 1e6
    print(
        'frames %d, boot %d, span %.2f s, %.3f fps; interval p50 %.2f ms, p99 %.1f ms, max %.1f ms; '
        'sensor frames %d, idr %d'
        % (
            len(stamps),
            stamps[0][2],
            span,
            (len(stamps) - 1) / span if span else 0.0,
            intervals[len(intervals) // 2] if intervals else 0.0,
            intervals[int(len(intervals) * 0.99)] if intervals else 0.0,
            intervals[-1] if intervals else 0.0,
            stamps[-1][0] - stamps[0][0] + 1,
            sum(stamp[4] for stamp in stamps),
        )
    )


if __name__ == '__main__':
    main()
