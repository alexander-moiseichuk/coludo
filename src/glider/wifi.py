# Wi-Fi station — joins the Control Center's network as a client (specs/board-config.md,
# cc-protocol.md). STA only; the board never hosts an AP. Credentials, CC host/port and the
# tunable TX power come from the `wifi` section of board.json.

import network
import time
import asyncio


class Wifi:
    def __init__(self, config, log=None):
        w = config.get('wifi', {})
        self.ssid = w.get('ssid', '')
        self.password = w.get('password', '')
        self.tx_power = w.get('tx_power_dbm')
        self.log = log if log is not None else (lambda m: None)
        self.wlan = None

    async def connect(self, timeout_ms=15000):
        '''Join the configured network. Returns True once connected, False on timeout.'''
        if self.wlan is None:
            self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)
        if self.tx_power is not None:
            try:
                self.wlan.config(txpower=self.tx_power)
            except Exception:
                pass
        if not self.wlan.isconnected():
            self.log("wifi :: connecting to '%s'" % self.ssid)
            self.wlan.connect(self.ssid, self.password)
            t0 = time.ticks_ms()
            while not self.wlan.isconnected():
                if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                    self.log('wifi :: connect timeout')
                    return False
                await asyncio.sleep_ms(200)
        self.log('wifi :: connected %s' % str(self.ifconfig()))
        return True

    def isconnected(self):
        return self.wlan is not None and self.wlan.isconnected()

    def ifconfig(self):
        return self.wlan.ifconfig() if self.wlan is not None else None

    def rssi(self):
        try:
            return self.wlan.status('rssi')
        except Exception:
            return None

    def set_txpower(self, dbm):
        '''Adjust the TX power (operator signal-level tuning). Returns True on success.'''
        self.tx_power = dbm
        if self.wlan is not None:
            try:
                self.wlan.config(txpower=dbm)
                return True
            except Exception:
                return False
        return False
