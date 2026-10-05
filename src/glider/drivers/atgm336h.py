"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

ATGM336H GNSS (GPS + BDS, CASIC chip) on a dedicated UART. @task.driver('atgm336h'). All NMEA
reading/parsing lives in the shared gnss.Gnss base; this driver only adds the CASIC reconfiguration:
RMC at `hz` (position) plus GGA at ~1 Hz (altitude/elevation, a baro backup) -- both fit 9600 baud
(~10 Hz RMC ~700 B/s + ~1 Hz GGA ~70 B/s < 960) -- and the sky diagnostics every ~10 s: GSA, GSV and
the antenna text, ~0.5 kB a burst (~50 B/s), which the base records as `<name>_sky.csv` and keeps to the
pad by re-sending the init's own mask in flight (gnss.Gnss._sky_window()). PCAS is the CASIC command
set; the PMTK pair is sent too as a fallback for MTK-variant modules (each side ignores the other's
sentences). Graceful: an undefined bus -> setup False (the Controller skips it).
"""

import asyncio

import gnss
import task


def _commands(hz: int) -> tuple:
    """
    The setup sentences for `hz` fixes per second, framed, in the order they are sent.

    The flight init first -- RMC every fix, GGA every `hz`th, the period, the MTK pair -- then the
    diagnostics: the same PCAS03 with GSA, GSV and the antenna text raised from 0 to every Nth fix.
    That is the sequence the 2026-10-04 bench (src/gnss_bench) flew and measured NOT to disturb the
    flight rates: at hz 10, 250 RMC + 25 GGA in 25 s, the diagnostics at ~9.5 s and ~19.4 s. On every
    fix they would not fit: ~0.5 kB a burst at 10 Hz is 5 kB/s against 960.

    Args:
        hz - the fix rate (Hz); below 1 counts as 1.

    Returns:
        The frames (bytes) in send order.
    """
    hz = max(hz, 1)
    period_ms = 1000 // hz  # 10 Hz -> 100 ms
    gga_every = hz  # GGA once every `hz` fixes -> ~1 Hz at any base rate (bandwidth)
    """
    The diagnostics every ~gnss.DIAGNOSTICS_S at any rate: `hz` fixes a second, times the seconds.
    The CASIC manual gives PCAS03 one digit per field, but the ATGM takes two (measured: the GGA
    divider 10, and 99 on the bench), so 99 is the cap -- 9.9 s at 10 Hz.
    """
    diagnostics_every = min(hz * gnss.DIAGNOSTICS_S, 99)
    bodies = (
        'PCAS03,%d,0,0,0,1,0,0,0,0,0,,,0,0' % gga_every,  # CASIC: GGA every Nth fix, RMC every fix
        'PCAS02,%d' % period_ms,  # CASIC: measurement period (ms)
        'PMTK314,0,1,0,%d,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0' % min(gga_every, 5),  # MTK fallback: RMC+GGA
        'PMTK220,%d' % period_ms,  # MTK fallback: update period (ms)
        # CASIC: the same mask, plus GSA, GSV and the antenna text (ANT) every Nth fix
        'PCAS03,%d,0,%d,%d,1,0,0,%d,0,0,,,0,0' % (gga_every, diagnostics_every, diagnostics_every,
                                                   diagnostics_every),
    )
    return tuple(gnss.nmea(body) for body in bodies)


@task.driver('atgm336h')
class Atgm336h(gnss.Gnss):
    """ATGM336H (CASIC): RMC at `hz` for position + GGA at ~1 Hz for altitude/elevation + the sky."""

    async def _configure(self, hz: int) -> tuple:
        """
        Send _commands(hz), 80 ms apart, and keep its two masks as the sky switch.

        Args:
            hz - the fix rate (Hz).

        Returns:
            (off, on) for gnss.Gnss._sky_window(), the bytes just sent: the flight init's own PCAS03 (RMC
            + GGA, nothing else -- the flight output as it was before the diagnostics) and the
            diagnostics' PCAS03.
        """
        commands = _commands(hz)
        for frame in commands:
            self._writer.write(frame)
            await self._writer.drain()
            await asyncio.sleep_ms(80)
        return commands[0], commands[4]
