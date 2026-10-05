"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

GY-NEO6MV2 (u-blox NEO-6M) GNSS on a dedicated UART: a drop-in alternative to the ATGM336H on the SAME
UART -- swap the component `driver` to 'neo6mv2' in config (and lower `hz`; the NEO-6M tops out near
5 Hz). @task.driver('neo6mv2'). NMEA read/parse is the shared gnss.Gnss base; this driver only adds the
u-blox reconfiguration: $PUBX,40 selects RMC (position) + GGA at ~1 Hz (altitude/elevation) on the UART
and silences the rest, UBX-CFG-RATE sets the measurement period, then $PUBX,40 turns GSA + GSV back on
every ~10 s -- the sky diagnostics the base records as `<name>_sky.csv` (no antenna status: the NEO-6M
reports none over NMEA) and keeps to the pad by re-sending the init's GSA + GSV off in flight
(gnss.Gnss._sky_window()). Default link is 9600 8N1, like the ATGM. Graceful: an undefined bus -> setup
False.
"""

import asyncio
import struct

import gnss
import task


def _ubx(class_id: int, msg_id: int, payload: bytes) -> bytes:
    """
    Build a UBX binary frame.

    0xB5 0x62 + class + id + little-endian length + payload + 8-bit Fletcher checksum (over
    class..payload).

    Args:
        class_id - the UBX message class byte.
        msg_id - the UBX message id byte.
        payload - the message payload bytes (may be empty).

    Returns:
        The complete UBX frame as bytes (sync header, body, and the two checksum bytes).
    """
    body = struct.pack('<BBH', class_id, msg_id, len(payload)) + payload
    ck_a = ck_b = 0
    for byte in body:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b'\xb5\x62' + body + bytes((ck_a, ck_b))


def _commands(hz: int) -> tuple:
    """
    The setup frames for `hz` fixes per second, in the order they are sent.

    The flight init first -- the $PUBX,40 selection, then UBX-CFG-RATE -- then the diagnostics: GSA and
    GSV every Nth fix. That is the sequence the 2026-10-04 bench (src/gnss_bench) flew and measured NOT
    to disturb the flight rates: at hz 5, 125 RMC + 25 GGA in 25 s, GSA + GSV every ~10 s.

    Args:
        hz - the fix rate (Hz); below 1 counts as 1.

    Returns:
        The frames (bytes) in send order.
    """
    hz = max(hz, 1)
    period_ms = 1000 // hz
    gga_every = hz  # GGA once every `hz` fixes -> ~1 Hz (bandwidth)
    # 5 Hz -> every 50th fix, ~10 s at any rate; a port's rate is one byte (UBX-CFG-MSG), so 255 caps it --
    # only above 25 Hz, which a module topping out near 5 Hz never runs
    diagnostics_every = min(hz * gnss.DIAGNOSTICS_S, 255)
    # $PUBX,40,<msg>,rddc,rus1,rus2,rusb,rspi,res -> per-port output divider (rus1 = this UART)
    selection = (
        'PUBX,40,RMC,0,1,0,0,0,0',  # RMC every fix (position)
        'PUBX,40,GGA,0,%d,0,0,0,0' % gga_every,  # GGA every Nth fix (altitude/elevation)
        'PUBX,40,GLL,0,0,0,0,0,0',  # silence the rest to stay within 9600 baud
        'PUBX,40,GSA,0,0,0,0,0,0',
        'PUBX,40,GSV,0,0,0,0,0,0',
        'PUBX,40,VTG,0,0,0,0,0,0',
    )
    diagnostics = (
        'PUBX,40,GSA,0,%d,0,0,0,0' % diagnostics_every,  # the sky diagnostics: fix mode + satellites used ...
        'PUBX,40,GSV,0,%d,0,0,0,0' % diagnostics_every,  # ... and satellites in view with their C/N0
    )
    # UBX-CFG-RATE (0x06,0x08): measRate ms (u16), navRate cycles (u16=1), timeRef (u16=1 -> GPS)
    rate = _ubx(0x06, 0x08, struct.pack('<HHH', period_ms, 1, 1))
    return tuple(gnss.nmea(body) for body in selection) + (rate,) + tuple(gnss.nmea(body) for body in diagnostics)


@task.driver('neo6mv2')
class Neo6mv2(gnss.Gnss):
    """u-blox NEO-6M: $PUBX,40 selects RMC + ~1 Hz GGA, UBX-CFG-RATE sets the period, then the sky every ~10 s."""

    async def _configure(self, hz: int) -> tuple:
        """
        Send _commands(hz), 40 ms apart, and keep its GSA + GSV pairs as the sky switch.

        A switch writes each pair at once, its two sentences back to back where the init (and the bench)
        put 40 ms between them: u-blox parses its input sentence by sentence, but this exact pair is not
        bench-checked yet.

        Args:
            hz - the fix rate (Hz).

        Returns:
            (off, on) for gnss.Gnss._sky_window(), the bytes just sent: the flight init's own GSA and GSV
            at 0 (the flight output as it was before the diagnostics) and the diagnostics' GSA and GSV.
        """
        commands = _commands(hz)
        for frame in commands:
            self._writer.write(frame)
            await self._writer.drain()
            await asyncio.sleep_ms(40)
        return commands[3] + commands[4], commands[7] + commands[8]
