"""
Live SDP810 readout: verify the governor's pitot BAND against real breath pressure.

Prints the tared dynamic pressure, the derived airspeed and the verdict the governor would reach --
IGNORED below pitot_min_ms (tare noise / a blocked tube), TRUSTED in band, IGNORED above pitot_max_ms
(a railed cell). Blow into P+ (the RIGHT tube) to drive it through the floor. Ctrl-C / timeout ends it.
"""

import asyncio
import time

import config_default
import governor
import task
from drivers import sdp810  # noqa: F401 -- registers the driver


class Ctrl:
    config = config_default.default()


async def main():
    cfg = {sensor['name']: sensor for sensor in Ctrl.config['sensors']}['airspeed_sdp810']
    driver = task.ACTIVITIES['sdp810']('airspeed_sdp810', cfg, Ctrl())
    if not await driver.setup():
        print('sdp810 setup FAILED -- check wiring (0x25 on i2c:0, SDA 7 / SCL 8)')
        return
    flight = {component['name']: component for component in Ctrl.config['components']}['flight']
    band = governor.GovernorConfig(flight)
    print('pitot band: %.1f .. %.1f m/s   (blow into P+, the RIGHT tube)' % (band.pitot_min_ms, band.pitot_max_ms))
    print('%8s %10s %9s   %s' % ('t', 'q (Pa)', 'v (m/s)', 'governor verdict'))
    asyncio.create_task(driver.run())
    start = time.ticks_ms()
    peak = 0.0
    while time.ticks_diff(time.ticks_ms(), start) < 30000:
        await asyncio.sleep_ms(500)
        status = driver.inspect()
        q = status.get('dynamic_pressure_pa') or 0.0
        v = status.get('airspeed_ms') or 0.0
        peak = max(peak, v)
        if v < band.pitot_min_ms:
            verdict = 'IGNORED  (below floor -> accel backbone carries it)'
        elif v < band.pitot_max_ms:
            verdict = 'TRUSTED  <-- in band, blended into the estimate'
        else:
            verdict = 'IGNORED  (railed -> accel backbone carries it)'
        print('%7.1fs %10.2f %9.2f   %s'
              % (time.ticks_diff(time.ticks_ms(), start) / 1000.0, q, v, verdict))
    print('peak airspeed seen: %.2f m/s (needed > %.1f to cross the floor)' % (peak, band.pitot_min_ms))

asyncio.run(main())
