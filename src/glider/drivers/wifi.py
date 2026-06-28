# drivers/wifi.py — Wi-Fi station driver: joins the configured network and keeps it joined, exposing
# signal/ip to the operator. HAL (it drives the radio), so @task.driver('wifi'). STA only; SSID / CC
# host / TX power come from the `wifi` section of board.config, the password from <ssid>.creds
# (gitignored, deploy.sh-pushed).
#
# Optional + telemetry-first + NON-BLOCKING BOOT: setup() never touches the radio (it only reads
# config), because bringing the STA link up can block and would stall the serial boot -- so the board
# ALWAYS boots and flies, with or without Wi-Fi. The radio comes up lazily in run(), which (re)joins on
# an interval ONLY until ignition (after BOOSTING it idles, never competing with the flight loop). A
# board with no Wi-Fi just logs once and flies standalone -- no Wi-Fi means no CC, nothing more.

import asyncio
import time

import controller as controller_mod
import recorder
import task


@task.driver('wifi')
class Wifi(task.Task):
    """Join + maintain the STA link; Inspectable as `wifi`."""

    async def setup(self) -> bool:
        """NON-BLOCKING: only read config; the radio is brought up lazily in run(). Bringing the
        ESP32-P4 <-> C6 STA link up (network.WLAN().active(True)) can block, and setup() runs serially
        in the single boot coroutine, so doing it here would stall the WHOLE board boot on the radio --
        leaving the flight stack down if Wi-Fi is slow/absent. Always returns True so the run() loop
        exists to (re)try; a board with no Wi-Fi just logs once and flies standalone."""
        wifi = self.controller.config.get('wifi', {})
        self.ssid: str = wifi.get('ssid', '')
        self.password: str = self._read_password(wifi.get('password', ''))
        self.tx_power = wifi.get('tx_power_dbm')
        self._retry_ms: int = wifi.get('retry_ms', 10000)  # join attempt interval before ignition
        self.wlan = None  # the radio object; created on first use in run()
        self._ok = True
        return True

    async def _ensure_radio(self) -> bool:
        """Bring the STA radio up on first use (deferred from setup so boot never blocks on it). Returns
        False -- noted once -- on a board with no Wi-Fi interface."""
        if self.wlan is not None:
            return True
        try:
            import network

            self.wlan = network.WLAN(network.STA_IF)
            self.wlan.active(True)
            if self.tx_power is not None:
                try:
                    self.wlan.config(txpower=self.tx_power)
                except Exception:
                    pass
            self.note(None)
            return True
        except Exception as error:  # no Wi-Fi interface on this board -> stay idle, fly standalone
            self.note('wifi :: no Wi-Fi interface (%r)' % error)
            return False

    async def run(self) -> None:
        """(Re)join every `retry_ms` -- but ONLY until ignition. Once the controller reaches BOOSTING the
        radio work stops: it must not compete with the 100 Hz flight loop, and the link is whatever was
        established on the pad (CC is a pre-flight convenience). Never fatal -- no Wi-Fi just means no CC."""
        while True:
            if self.controller.stage >= controller_mod.Stage.BOOSTING:
                await asyncio.sleep_ms(5000)  # ignition: stop initiating connections, just idle
                continue
            if await self._ensure_radio() and not self.isconnected():
                await self.connect()
            await asyncio.sleep_ms(self._retry_ms)

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
        if self.wlan is None or self.wlan.isconnected():
            return self.wlan is not None and self.wlan.isconnected()
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
        return self.wlan is not None and self.wlan.isconnected()

    def ifconfig(self) -> tuple:
        return self.wlan.ifconfig() if self.wlan is not None else None

    def ip(self) -> str:
        try:
            return self.wlan.ifconfig()[0]
        except Exception:
            return None

    def rssi(self) -> int:
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

    async def diagnose(self) -> str:
        """Dump the Wi-Fi link state to the console AND the recorder log, and return the one-line summary.
        Wi-Fi setup never fails (it is non-blocking and the radio comes up lazily in run()), so this is an
        on-demand link check rather than a setup-failure analysis -- it brings the radio up if needed."""
        if not await self._ensure_radio():
            summary = 'wifi :: no radio -- no Wi-Fi interface on this board (flying standalone)'
        else:
            summary = 'wifi :: ssid=%r connected=%s ip=%s rssi=%s tx_power=%s' % (
                self.ssid, self.isconnected(), self.ip(), self.rssi(), self.tx_power)
        print(summary)
        recorder.Recorder.log(self.name, summary)
        return summary

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
