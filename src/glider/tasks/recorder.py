# tasks/recorder.py — the Recorder's task adapter. The data path itself is the top-level `recorder`
# singleton (used directly by every module via recorder.Recorder.log/tlm); this thin @task.driver
# plugs it into the Controller's task graph so the `recorder` component (its bus selects the UART)
# is created and supervised like any other task. No 'uart_sink' abstraction -- the Recorder is it.

import recorder
import task


@task.driver('recorder')
class RecorderTask(task.Task):
    """Owns the Recorder's setup + drain loop and surfaces it to the operator; everything else
    keeps logging/telemetering through the global recorder.Recorder."""

    async def setup(self) -> bool:
        recorder.Recorder.setup(self.controller.config)  # resolves the recorder component's UART bus
        self._ok = True
        return True

    async def run(self) -> None:
        await recorder.Recorder.run()

    def inspect(self) -> dict:
        status = recorder.Recorder.inspect()
        status['name'] = self.name
        status['ok'] = self._ok
        return status

    def stats(self) -> dict:
        return recorder.Recorder.stats()

    def update(self, props) -> list:
        return recorder.Recorder.update(props)
