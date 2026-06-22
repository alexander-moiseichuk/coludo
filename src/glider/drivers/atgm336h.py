# drivers/atgm336h.py — ATGM336H GNSS (GPS + BDS, CASIC chip) on a dedicated UART. @task.driver(
# 'atgm336h'). All NMEA reading/parsing lives in the shared gnss.Gnss base; this driver only adds the
# CASIC reconfiguration: RMC at `hz` (position) plus GGA at ~1 Hz (altitude/elevation, a baro backup)
# -- both fit 9600 baud (~10 Hz RMC ~700 B/s + ~1 Hz GGA ~70 B/s < 960). PCAS is the CASIC command set;
# the PMTK pair is sent too as a fallback for MTK-variant modules (each side ignores the other's
# sentences). Graceful: an undefined bus -> setup False (the Controller skips it).

import asyncio

import gnss
import task


@task.driver('atgm336h')
class Atgm336h(gnss.Gnss):
    """ATGM336H (CASIC): RMC at `hz` for position + GGA at ~1 Hz for altitude/elevation."""

    async def _configure(self, hz: int) -> None:
        period_ms = 1000 // max(hz, 1)  # 10 Hz -> 100 ms
        gga_every = max(hz, 1)  # GGA once every `hz` fixes -> ~1 Hz at any base rate (bandwidth)
        writer = asyncio.StreamWriter(self._uart, {})
        commands = (
            'PCAS03,%d,0,0,0,1,0,0,0,0,0,,,0,0' % gga_every,  # CASIC: GGA every Nth fix, RMC every fix
            'PCAS02,%d' % period_ms,  # CASIC: measurement period (ms)
            'PMTK314,0,1,0,%d,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0' % min(gga_every, 5),  # MTK fallback: RMC+GGA
            'PMTK220,%d' % period_ms,  # MTK fallback: update period (ms)
        )
        for body in commands:
            writer.write(gnss.nmea(body))
            await writer.drain()
            await asyncio.sleep_ms(80)
