# Wi-Fi station — joins the Control Center's network as a client (specs/board-config.md,
# cc-protocol.md). STA only; the board never hosts an AP. SSID, CC host/port and the tunable TX
# power come from the `wifi` section of board.json; the password comes from <ssid>.creds (pushed
# by deploy.sh, never committed) so it is not in the repo.

import asyncio
import time

import network
from inspector import Inspectable, Inspector


def _read_password(ssid: str, fallback: str) -> str:
    try:
        with open('%s.creds' % ssid) as creds:
            password = creds.readline().strip()
            return password if password else fallback
    except OSError:
        return fallback


class Wifi(Inspectable):
    name = 'wifi'
    kind = 'wifi'

    def __init__(self, config: dict, log=None):
        wifi = config.get('wifi', {})
        self.ssid: str = wifi.get('ssid', '')
        self.password: str = _read_password(self.ssid, wifi.get('password', ''))
        self.tx_power = wifi.get('tx_power_dbm')
        self.log = log if log is not None else (lambda message: None)
        self.wlan = None
        Inspector.register(self)

    async def connect(self, timeout_ms: int = 15000) -> bool:
        """Join the configured network. Returns True once connected, False on timeout."""
        if self.wlan is None:
            self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)
        if self.tx_power is not None:
            try:
                self.wlan.config(txpower=self.tx_power)
            except Exception:
                pass
        if not self.wlan.isconnected():
            self.log('wifi :: connecting to "%s"' % self.ssid)
            self.wlan.connect(self.ssid, self.password)
            start = time.ticks_ms()
            while not self.wlan.isconnected():
                if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
                    self.log('wifi :: connect timeout')
                    return False
                await asyncio.sleep_ms(200)
        self.log('wifi :: connected %s' % str(self.ifconfig()))
        return True

    def isconnected(self) -> bool:
        return self.wlan is not None and self.wlan.isconnected()

    def ifconfig(self):
        return self.wlan.ifconfig() if self.wlan is not None else None

    def ip(self) -> str:
        config = self.ifconfig()
        return config[0] if config else None

    def rssi(self):
        try:
            return self.wlan.status('rssi')
        except Exception:
            return None

    def set_txpower(self, dbm: int) -> bool:
        """Adjust the TX power (operator signal-level tuning). Returns True on success."""
        self.tx_power = dbm
        if self.wlan is not None:
            try:
                self.wlan.config(txpower=dbm)
                return True
            except Exception:
                return False
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
        if dbm is not None and dbm != self.tx_power and self.set_txpower(dbm):
            changed.append('tx_power')
        return changed

    def stats(self) -> dict:
        return {'connected': self.isconnected(), 'rssi': self.rssi()}
