# tasks/cc_link.py — the Control link task: once Wi-Fi is up it dials the CC hub and serves the
# command dispatcher, reconnecting with backoff. @task.activity('cc'). Optional + telemetry-first:
# with no `cc_host` configured setup() skips it; with no Wi-Fi up it simply waits, so the board
# flies fine without CC. The dispatcher is wired to this board's config + Controller (cc_client.py).

import asyncio

import cc_client
import recorder
import task


@task.activity('cc')
class ControlLink(task.Task):
    """Serve the CC protocol to the hub when the link is available; never fatal."""

    async def setup(self) -> bool:
        wifi = self.controller.config.get('wifi', {})
        if not wifi.get('cc_host'):
            return False  # no CC configured -> the board runs standalone
        dispatcher = cc_client.create_dispatcher(self.controller.config, controller=self.controller)
        self._client = cc_client.Client(self.controller.config, dispatcher, log=print)
        self._ok = True
        return True

    async def run(self) -> None:
        """Park until the Wi-Fi dependency is up, then dial CC and serve until the link drops; retry.
        On a board with no Wi-Fi the query never returns, so this just stays idle -- the board keeps
        running its other tasks."""
        wifi, = await self.query(['wifi'])  # block until the wifi task is up (our dependency)
        while True:
            if not wifi.isconnected():
                await asyncio.sleep_ms(2000)
                continue
            try:
                reader, writer = await asyncio.open_connection(self._client.host, self._client.port)
                print('cc :: connected %s:%d' % (self._client.host, self._client.port))
                await self._client.serve(reader, writer)
            except Exception as error:
                print('cc :: %r' % error)
            await asyncio.sleep_ms(self._client.backoff_ms)

    async def probe(self) -> str:
        """On-demand self-test: the CC client is configured (host:port) and the Wi-Fi dependency is
        up. A down link is logged, not failed -- run() dials it on demand with backoff."""
        try:
            recorder.Recorder.log(self.name, 'probe: cc link ...')
            host_port = '%s:%d' % (self._client.host, self._client.port)
            wifi = self.controller.find(['wifi'])[0]
            wifi_up = wifi is not None and wifi.validate()
            recorder.Recorder.log(self.name, 'probe: cc ok (hub %s, wifi up=%s)' % (host_port, wifi_up))
        except Exception as error:
            message = 'cc link: %s' % error
            recorder.Recorder.log(self.name, 'probe FAILED: ' + message)
            return message
        return None
