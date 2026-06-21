# drivers/neo6mv2.py — GY-NEO6MV2 (u-blox NEO-6M) GNSS on a dedicated UART: a drop-in alternative to
# the ATGM336H on the SAME UART -- swap the component `driver` to 'neo6mv2' in config (and lower `hz`;
# the NEO-6M tops out near 5 Hz). @task.driver('neo6mv2'). NMEA read/parse is the shared gnss.Gnss base;
# this driver only adds the u-blox reconfiguration: $PUBX,40 selects RMC (position) + GGA at ~1 Hz
# (altitude/elevation) on the UART and silences the rest, then UBX-CFG-RATE sets the measurement
# period. Default link is 9600 8N1, like the ATGM. Graceful: an undefined bus -> setup False.

import asyncio
import struct

import gnss
import task


def _ubx(class_id: int, msg_id: int, payload: bytes) -> bytes:
    """A UBX binary frame: 0xB5 0x62 + class + id + little-endian length + payload + 8-bit Fletcher
    checksum (over class..payload)."""
    body = struct.pack('<BBH', class_id, msg_id, len(payload)) + payload
    ck_a = ck_b = 0
    for byte in body:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b'\xb5\x62' + body + bytes((ck_a, ck_b))


@task.driver('neo6mv2')
class Neo6mv2(gnss.Gnss):
    """u-blox NEO-6M: $PUBX,40 selects RMC + ~1 Hz GGA, UBX-CFG-RATE sets the measurement period."""

    async def _configure(self, hz: int) -> None:
        period_ms = 1000 // max(hz, 1)
        gga_every = max(hz, 1)  # GGA once every `hz` fixes -> ~1 Hz (bandwidth)
        writer = asyncio.StreamWriter(self._uart, {})
        # $PUBX,40,<msg>,rddc,rus1,rus2,rusb,rspi,res -> per-port output divider (rus1 = this UART)
        selection = (
            'PUBX,40,RMC,0,1,0,0,0,0',  # RMC every fix (position)
            'PUBX,40,GGA,0,%d,0,0,0,0' % gga_every,  # GGA every Nth fix (altitude/elevation)
            'PUBX,40,GLL,0,0,0,0,0,0',  # silence the rest to stay within 9600 baud
            'PUBX,40,GSA,0,0,0,0,0,0',
            'PUBX,40,GSV,0,0,0,0,0,0',
            'PUBX,40,VTG,0,0,0,0,0,0',
        )
        for body in selection:
            writer.write(gnss.nmea(body))
            await writer.drain()
            await asyncio.sleep_ms(40)
        # UBX-CFG-RATE (0x06,0x08): measRate ms (u16), navRate cycles (u16=1), timeRef (u16=1 -> GPS)
        writer.write(_ubx(0x06, 0x08, struct.pack('<HHH', period_ms, 1, 1)))
        await writer.drain()
        await asyncio.sleep_ms(40)
