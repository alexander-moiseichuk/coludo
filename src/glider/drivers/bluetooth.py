# drivers/bluetooth.py — set the BLE radio to the state declared in config at boot. The component
# field `radio` (true/false, default false) says whether Bluetooth should be ON; the driver applies
# it -- transparent, so nobody is surprised by an implicit disable. Default false saves power (the
# wireless is the external C6 and BLE is unused on the glider). Setup-only @task.driver('bluetooth')
# plus update() so the operator can toggle it live (`update bluetooth {"radio": true}`).

import task


@task.driver('bluetooth')
class Bluetooth(task.Task):
    """Apply the configured BLE radio state. Inspectable: `radio` requested, `active` actual."""

    async def setup(self) -> bool:
        self.radio = self.config.get('radio', False)  # desired BLE state (default off)
        self.active = self._apply(self.radio)
        self._ok = True
        return True

    def _apply(self, on: bool):
        """Set BLE active to `on`; return the resulting state, or None if there is no BLE here."""
        try:
            import bluetooth

            radio = bluetooth.BLE()
            radio.active(on)
            return radio.active()
        except Exception as error:  # no bluetooth module on this board
            print('bluetooth :: %r' % error)
            return None

    def inspect(self) -> dict:
        return {'radio': self.radio, 'active': self.active}

    def update(self, props) -> list:
        if 'radio' in props and props['radio'] != self.radio:
            self.radio = props['radio']
            self.active = self._apply(self.radio)
            return ['radio']
        return []
