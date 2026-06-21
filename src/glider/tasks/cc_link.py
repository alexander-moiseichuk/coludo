# tasks/cc_link.py — the Control link task: once Wi-Fi is up it dials the CC hub and serves the
# command dispatcher, reconnecting with backoff. @task.activity('cc'). Telemetry-first: with no Wi-Fi
# up it simply waits, so the board flies fine without CC. The hub address is the configured `cc_host`,
# or -- when unset -- the `.1` of whatever subnet the board joins (the Control hub by convention), so
# a board reaches its hub on any network. The dispatcher is wired to this board's config + Controller.

import asyncio

import cc_client
import recorder
import task


def _network_host(ip: str) -> str:
    """The Control hub for a board with no explicit `cc_host`: the `.1` of its own subnet (the AP /
    gateway by convention). None if the IP is unusable, so the caller waits for a real lease."""
    if not ip or ip == '0.0.0.0':
        return None
    return ip.rsplit('.', 1)[0] + '.1'


@task.activity('cc')
class ControlLink(task.Task):
    """Serve the CC protocol to the hub when the link is available; never fatal. With no `cc_host`
    configured the board dials the `.1` of whatever subnet it joins (the Control hub by convention)."""

    async def setup(self) -> bool:
        dispatcher = cc_client.create_dispatcher(self.controller.config, controller=self.controller)
        self._client = cc_client.Client(self.controller.config, dispatcher, log=print)
        self._ok = True
        return True

    def _host(self, wifi) -> str:
        """The hub address: the explicit `cc_host` if set, else the `.1` of the board's own subnet."""
        return self._client.host or _network_host(wifi.ip())

    async def run(self) -> None:
        """Park until the Wi-Fi dependency is up, then dial CC and serve until the link drops; retry.
        On a board with no Wi-Fi the query never returns, so this just stays idle -- the board keeps
        running its other tasks."""
        wifi, = await self.query(['wifi'])  # block until the wifi task is up (our dependency)
        while True:
            if not wifi.isconnected():
                await asyncio.sleep_ms(2000)
                continue
            host = self._host(wifi)
            if host is None:  # connected but no usable lease yet -> wait for the address
                await asyncio.sleep_ms(2000)
                continue
            try:
                reader, writer = await asyncio.open_connection(host, self._client.port)
                print('cc :: connected %s:%d' % (host, self._client.port))
                await self._client.serve(reader, writer)
            except Exception as error:
                print('cc :: %r' % error)
            await asyncio.sleep_ms(self._client.backoff_ms)

    async def probe(self) -> str:
        """On-demand self-test: the CC hub address resolves (explicit or derived) and the Wi-Fi
        dependency is up. A down link is logged, not failed -- run() dials it on demand with backoff."""
        try:
            recorder.Recorder.log(self.name, 'probe: cc link ...')
            wifi = self.controller.find(['wifi'])[0]
            host = self._host(wifi) if wifi is not None else self._client.host
            host_port = '%s:%d' % (host, self._client.port)
            wifi_up = wifi is not None and wifi.validate()
            recorder.Recorder.log(self.name, 'probe: cc ok (hub %s, wifi up=%s)' % (host_port, wifi_up))
        except Exception as error:
            message = 'cc link: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None
