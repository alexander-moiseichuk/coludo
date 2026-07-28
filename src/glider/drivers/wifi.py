"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Wi-Fi station driver: joins the configured network and keeps it joined, exposing signal/ip to the
operator. HAL (it drives the radio), so @task.driver('wifi'). STA only; SSID / CC host / TX power come
from the 'wifi' section of board.config, the password from <ssid>.creds (gitignored, deploy.sh-pushed).

Optional + telemetry-first + NON-BLOCKING BOOT: setup() never touches the radio (it only reads config),
because bringing the STA link up can block and would stall the serial boot -- so the board ALWAYS boots
and flies, with or without Wi-Fi. The radio comes up lazily in run(), which (re)joins on an interval
ONLY until ignition (after BOOSTING it idles, never competing with the flight loop). A board with no
Wi-Fi just logs once and flies standalone -- no Wi-Fi means no CC, nothing more.
"""

import asyncio
import time

import controller as controller_mod
import recorder
import task

try:
    import network
except ImportError:  # host (CPython): board-only; _ensure_radio() then reports no Wi-Fi interface
    network = None


@task.driver('wifi')
class Wifi(task.Task):
    """Join + maintain the STA link; Inspectable as 'wifi'."""

    async def setup(self) -> bool:
        """
        NON-BLOCKING: only read config; the radio is brought up lazily in run().

        Bringing the ESP32-P4 <-> C6 STA link up (network.WLAN().active(True)) can block, and setup()
        runs serially in the single boot coroutine, so doing it here would stall the WHOLE board boot on
        the radio -- leaving the flight stack down if Wi-Fi is slow/absent. A board with no Wi-Fi just
        logs once and flies standalone.

        Args:
            (none)

        Returns:
            Always True, so the run() loop exists to (re)try even when Wi-Fi is slow or absent.
        """
        wifi = self.controller.config.get('wifi', {})
        """
        policy (CC-less field ops, doc/specs/coludo.md): 'auto' (default) joins/rejoins on the retry
        interval, quiescent while airborne; 'disabled' never touches the radio this session. (Distinct
        from the radio 'mode' key, which stays 'sta'.)
        """
        self._policy: str = wifi.get('policy', 'auto')
        self.ssid: str = wifi.get('ssid', '')
        """
        networks: full per-network configs, each entry ORGANIZED LIKE THE WORKING top-level wifi
        section (the proven panda shape, replicated): ssid + optional enabled/policy/retry_ms/
        tx_power_dbm/password, every missing key INHERITED from the top level. STA is the ONLY
        radio mode -- there is no `mode` knob, the driver hardcodes STA_IF (the validator still
        rejects a stray non-'sta' mode key so a typo cannot silently mean nothing). A bare string
        is sugar for {'ssid': name}. Per-network `retry_ms` is the MINIMAL time between attempts
        of THAT network (its own backoff clock), so a flaky hotspot is polled gently while the
        lab AP retries fast. Passwords: <ssid>.creds wins, the entry/top-level `password` is the
        fallback. `enabled: false` / policy 'disabled' parks an entry.
        """
        defaults = {'policy': 'auto', 'enabled': True,
                    'retry_ms': wifi.get('retry_ms', 10000),
                    'tx_power_dbm': wifi.get('tx_power_dbm'),
                    'password': wifi.get('password', '')}
        raw = wifi.get('networks') or ([{'ssid': self.ssid}] if self.ssid else [])
        self._networks: list = []
        for entry in raw:
            if isinstance(entry, str):
                entry = {'ssid': entry}
            network = dict(defaults)
            network.update(entry)
            if network.get('ssid') and network.get('enabled', True) \
                    and network.get('policy', 'auto') != 'disabled':
                network['last_ms'] = None  # this network's own retry clock (never attempted yet)
                self._networks.append(network)
        self._network_index: int = 0
        self.password: str = self._read_password(wifi.get('password', ''))
        self.tx_power = wifi.get('tx_power_dbm')
        self.wlan = None  # the radio object; created on first use in run()
        self._ok = True
        return True

    def _next_network(self, now: int):
        """
        The next network candidate to attempt (round-robin, per-network backoff).

        Round-robin from the current index over the networks whose OWN retry_ms has elapsed since their
        last attempt (None = never tried -> eligible at once). Stamps the winner's clock and advances
        the rotation.

        Args:
            now - the current time (ticks_ms) to test each network's backoff against.

        Returns:
            The chosen network dict; None while every network is still inside its backoff window.
        """
        count = len(self._networks)
        for step in range(count):
            network = self._networks[(self._network_index + step) % count]
            last = network['last_ms']
            if last is None or time.ticks_diff(now, last) >= network['retry_ms']:
                network['last_ms'] = now
                self._network_index = (self._network_index + step + 1) % count
                return network
        return None

    async def _ensure_radio(self) -> bool:
        """
        Bring the STA radio up on first use (deferred from setup so boot never blocks on it).

        Args:
            (none)

        Returns:
            True once the radio is up; False -- noted once -- on a board with no Wi-Fi interface.
        """
        if self.wlan is not None:
            return True
        try:
            self.wlan = network.WLAN(network.STA_IF)  # AttributeError if network is None (no interface)
            self.wlan.active(True)
            if self.tx_power is not None:
                try:
                    self.wlan.config(txpower=self.tx_power)
                except Exception:
                    pass
            self.note(None)
            return True
        except Exception as error:  # no Wi-Fi interface on this board -> stay idle, fly standalone
            self.note('wifi :: no Wi-Fi interface (%r)', error)
            return False

    async def run(self) -> None:
        """
        (Re)join every retry_ms -- but ONLY on the ground.

        From BOOSTING through LANDING the radio work stops: it must not compete with the 100 Hz flight
        loop or allocate under GC-off; the link is whatever was established on the pad. At DONE scanning
        RESUMES (post-flight recovery telemetry: the crew walks up with the hotspot). Several configured
        networks are tried round-robin, one candidate per retry. 'policy: disabled' never touches the
        radio. Never fatal -- no Wi-Fi just means no CC.

        Args:
            (none)

        Returns:
            None; runs forever (a wedged board reboots rather than exits).
        """
        if self._policy == 'disabled':
            self.note('wifi :: disabled by config (policy)', None)
            while True:
                await asyncio.sleep_ms(60000)  # keep the supervised loop alive, radio untouched
        while True:
            stage = self.controller.stage
            if controller_mod.Stage.BOOSTING <= stage < controller_mod.Stage.DONE:
                await asyncio.sleep_ms(5000)  # airborne: stop initiating connections, just idle
                continue
            if self._networks and await self._ensure_radio() and not self.isconnected():
                chosen = self._next_network(time.ticks_ms())  # per-network backoff decides (not the `network` module)
                if chosen is not None:
                    self.ssid = chosen['ssid']
                    self.password = self._read_password(chosen.get('password', ''))
                    if chosen.get('tx_power_dbm') is not None \
                            and chosen['tx_power_dbm'] != self.tx_power:
                        self.set_tx_power(chosen['tx_power_dbm'])
                    await self.connect()
            """
            CONNECTED -> the hunt stops entirely (no rotation, no attempts): only this status
            poll remains, relaxed to 5 s. Hunting keeps the 1 s tick so each network's retry_ms
            schedules responsively. A dropped link re-enters the hunt with the backoff clocks
            still remembering their last attempts.
            """
            await asyncio.sleep_ms(5000 if self.isconnected() else 1000)

    def _read_password(self, fallback: str) -> str:
        """
        Read the password from <ssid>.creds (gitignored, deploy.sh-pushed), else fallback.

        Args:
            fallback - the password to return when the creds file is missing or empty.

        Returns:
            The creds-file password when present and non-empty, else fallback.
        """
        try:
            with open('%s.creds' % self.ssid) as creds:
                password = creds.readline().strip()
                return password if password else fallback
        except OSError:
            return fallback

    async def connect(self, timeout_ms: int = 15000) -> bool:
        """
        Join the configured network.

        Args:
            timeout_ms - how long to wait for the link before giving up (milliseconds).

        Returns:
            True once connected; False on timeout or error.
        """
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
        """
        Adjust the TX power (operator signal-level tuning).

        Args:
            dbm - the new TX power, in dBm.

        Returns:
            True on success; False when the radio rejects the setting.
        """
        self.tx_power = dbm
        try:
            self.wlan.config(txpower=dbm)
            return True
        except Exception:
            return False

    async def diagnose(self) -> str:
        """
        Dump the Wi-Fi link state to the console AND the recorder log; return the one-line summary.

        Wi-Fi setup never fails (it is non-blocking and the radio comes up lazily in run()), so this is
        an on-demand link check rather than a setup-failure analysis -- it brings the radio up if needed.

        Args:
            (none)

        Returns:
            The one-line link-state summary (also printed and logged).
        """
        if not await self._ensure_radio():
            summary = 'wifi :: no radio -- no Wi-Fi interface on this board (flying standalone)'
        else:
            summary = 'wifi :: ssid=%r connected=%s ip=%s rssi=%s tx_power=%s' % (
                self.ssid, self.isconnected(), self.ip(), self.rssi(), self.tx_power)
        print(summary)
        recorder.Recorder.log(self.name, summary)
        return summary

    """Inspectable: operator-facing view (inspect), live tuning (update), compact status (stats)."""
    def inspect(self) -> dict:
        status = task.Task.inspect(self)
        status.update({
            'ssid': self.ssid,  # the network we are on (or last tried)
            'connected': self.isconnected(),
            'ip': self.ip(),
            'rssi': self.rssi(),
            'tx_power': self.tx_power,
            'networks': [network.get('ssid') for network in self._networks],  # all configured, in order
        })
        return status

    def update(self, props: dict) -> list:
        changed = []
        dbm = props.get('tx_power')
        if dbm is not None and dbm != self.tx_power and self.set_tx_power(dbm):
            changed.append('tx_power')
        return changed

    def stats(self) -> dict:
        return {'connected': self.isconnected(), 'rssi': self.rssi()}
