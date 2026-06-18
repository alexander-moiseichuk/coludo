# drivers/wifi.py — Wi-Fi station driver: joins the configured network and keeps it joined, exposing
# signal/ip to the operator. HAL (it drives the radio), so @task.driver('wifi'). STA only; SSID / CC
# host / TX power come from the `wifi` section of board.json, the password from <ssid>.creds
# (gitignored, deploy.sh-pushed).
#
# Optional + telemetry-first: `network` is imported in setup() so the module still loads on a board
# with no Wi-Fi (e.g. the FireBeetle 2); setup() then returns False, the Controller skips the task,
# and the board runs everything else without CC. run() is a maintain loop -- a failed join is never
# fatal, it just retries.

import asyncio
import time

import task


@task.driver('wifi')
class Wifi(task.Task):
    """Join + maintain the STA link; Inspectable as `wifi`."""

    async def setup(self) -> bool:
        wifi = self.controller.config.get('wifi', {})
        self.ssid: str = wifi.get('ssid', '')
        self.password: str = self._read_password(wifi.get('password', ''))
        self.tx_power = wifi.get('tx_power_dbm')
        try:
            import network

            self.wlan = network.WLAN(network.STA_IF)
            self.wlan.active(True)
        except Exception as error:  # no Wi-Fi interface on this board -> skip the task
            print('wifi :: no Wi-Fi interface (%r)' % error)
            return False
        if self.tx_power is not None:
            try:
                self.wlan.config(txpower=self.tx_power)
            except Exception:
                pass
        self._ok = True
        return True

    async def run(self) -> None:
        """Keep the link up: (re)join whenever disconnected. Never fatal -- the board flies without
        Wi-Fi and this just retries."""
        while True:
            if not self.isconnected():
                await self.connect()
            await asyncio.sleep_ms(5000)

    def _read_password(self, fallback: str) -> str:
        """Read the password from <ssid>.creds (gitignored, deploy.sh-pushed), else `fallback`."""
        try:
            with open('%s.creds' % self.ssid) as creds:
                password = creds.readline().strip()
                return password if password else fallback
        except OSError:
            return fallback

    async def connect(self, timeout_ms: int = 15000) -> bool:
        """Join the configured network. Returns True once connected, False on timeout/error."""
        if self.wlan.isconnected():
            return True
        print('wifi :: connecting to "%s"' % self.ssid)
        try:
            self.wlan.connect(self.ssid, self.password)
        except Exception as error:
            print('wifi :: connect error %r' % error)
            return False
        start = time.ticks_ms()
        while not self.wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                print('wifi :: connect timeout')
                return False
            await asyncio.sleep_ms(200)
        print('wifi :: connected %s' % str(self.ifconfig()))
        return True

    def isconnected(self) -> bool:
        return self.wlan.isconnected()

    def ifconfig(self):
        return self.wlan.ifconfig()

    def ip(self) -> str:
        try:
            return self.wlan.ifconfig()[0]
        except Exception:
            return None

    def rssi(self):
        try:
            return self.wlan.status('rssi')
        except Exception:
            return None

    def set_tx_power(self, dbm: int) -> bool:
        """Adjust the TX power (operator signal-level tuning). Returns True on success."""
        self.tx_power = dbm
        try:
            self.wlan.config(txpower=dbm)
            return True
        except Exception:
            return False

    # --- Inspectable ---
    def inspect(self) -> dict:
        return {
            'ssid': self.ssid,
            'tx_power': self.tx_power,
            'connected': self.isconnected(),
            'rssi': self.rssi(),
            'ip': self.ip(),
        }

    def update(self, props: dict) -> list:
        changed = []
        dbm = props.get('tx_power')
        if dbm is not None and dbm != self.tx_power and self.set_tx_power(dbm):
            changed.append('tx_power')
        return changed

    def stats(self) -> dict:
        return {'connected': self.isconnected(), 'rssi': self.rssi()}
