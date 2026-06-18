# tasks/cc_link.py — the Control link task: once Wi-Fi is up it dials the CC hub and serves the
# command dispatcher, reconnecting with backoff. @task.activity('cc'). Optional + telemetry-first:
# with no `cc_host` configured setup() skips it; with no Wi-Fi up it simply waits, so the board
# flies fine without CC. The dispatcher is wired to this board's config + Controller (cc_client.py).

import asyncio

import cc_client
import inspector
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
        """Wait for Wi-Fi, dial CC, and serve until the link drops; then retry. Idle (not spinning)
        while there is no Wi-Fi, so a board with no link just keeps running its other tasks."""
        while True:
            wifi = inspector.Inspector.get('wifi')
            if wifi is None or not wifi.isconnected():
                await asyncio.sleep_ms(2000)
                continue
            try:
                reader, writer = await asyncio.open_connection(self._client.host, self._client.port)
                print('cc :: connected %s:%d' % (self._client.host, self._client.port))
                await self._client.serve(reader, writer)
            except Exception as error:
                print('cc :: %r' % error)
            await asyncio.sleep_ms(self._client.backoff_ms)
