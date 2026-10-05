"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

On-board test for the board bring-up (main.py): the network-free bringup() wires the inspectable
objects + Controller and starts the configured task loops, incl. the Recorder virtual driver,
while skipping config sensors whose drivers are not implemented yet; and the NVS boot counter that names
each boot's capture files. Run by `make test`.
"""

import asyncio

import config_default
import esp32
import inspector
import main


class _FakeNvs:
    """
    Stands in for esp32.NVS('coludo') -- the test must never bump the board's REAL boot count.

    Behaves like the real namespace where it matters: get_i32 of an absent key raises
    OSError(-0x1102, 'ESP_ERR_NVS_NOT_FOUND'), and `failing` names the one call that raises the way a
    corrupt, full or failing NVS partition does ('get', 'set' or 'commit').
    """

    def __init__(self, boot: int = None, failing: str = ''):
        self.values = {} if boot is None else {'boot': boot}
        self.failing = failing
        self.commits = 0

    def get_i32(self, key: str) -> int:
        if self.failing == 'get':
            raise OSError(-0x1103, 'ESP_ERR_NVS_TYPE_MISMATCH')
        if key not in self.values:
            raise OSError(-0x1102, 'ESP_ERR_NVS_NOT_FOUND')
        return self.values[key]

    def set_i32(self, key: str, value: int) -> None:
        if self.failing == 'set':
            raise OSError(-0x1105, 'ESP_ERR_NVS_NOT_ENOUGH_SPACE')
        self.values[key] = value

    def commit(self) -> None:
        if self.failing == 'commit':
            raise OSError(-0x110b, 'ESP_ERR_NVS_INVALID_STATE')
        self.commits += 1


class _BareErrorNvs(_FakeNvs):
    """A store whose read raises an OSError with no code at all: unknown is not 'never counted'."""

    def get_i32(self, key: str) -> int:
        raise OSError()


def test_next_boot():
    # a board that never counted starts at 1; each boot then adds one, committed
    store = _FakeNvs()
    assert main._next_boot(store) == 1 and store.values['boot'] == 1 and store.commits == 1
    assert main._next_boot(store) == 2 and store.values['boot'] == 2 and store.commits == 2
    store = _FakeNvs(boot=122)
    assert main._next_boot(store) == 123 and store.values['boot'] == 123
    """
    NEGATIVE: anything but a missing key -> no id (the Recorder falls back to its legacy prefix) rather
    than reusing one, which would append this boot into an earlier boot's files. A read that fails for
    another reason says nothing about how far the count got, so it must not restart at 1, and nothing
    is written; a refused write or commit leaves the old count.
    """
    for unreadable in (_FakeNvs(boot=41, failing='get'), _BareErrorNvs(boot=41)):
        assert main._next_boot(unreadable) is None
        assert unreadable.values['boot'] == 41 and unreadable.commits == 0
    refused = _FakeNvs(boot=41, failing='set')
    assert main._next_boot(refused) is None and refused.values['boot'] == 41 and refused.commits == 0
    uncommitted = _FakeNvs(boot=41, failing='commit')
    assert main._next_boot(uncommitted) is None and uncommitted.commits == 0


def test_nvs_not_found():
    """
    The REAL NVS reports a never-written key as -0x1102, the one code _next_boot reads as "never counted".

    Read-only: it reads a key nothing writes, so the board's boot count and crumb are untouched. Pins
    the fake above to the port, which raises OSError(-0x1102, 'ESP_ERR_NVS_NOT_FOUND').
    """
    code = None
    try:
        esp32.NVS('coludo').get_i32('test-absent')
    except OSError as error:
        code = error.args[0] if error.args else None
    assert code == -0x1102, code


async def amain():
    # a board with no Wi-Fi (e.g. FireBeetle 2): drop the connectivity components -- it must still
    # bring up everything else and run without CC.
    cfg = config_default.default()
    cfg['components'] = [c for c in cfg['components']
                         if (c.get('driver') or c.get('activity')) not in ('wifi', 'cc')]

    flight = await main.bringup(cfg, log=lambda message: None)

    # Mission (explicit) + the Controller and its enabled tasks all registered with the Inspector
    assert {'mission', 'controller', 'recorder', 'health', 'bluetooth'} <= set(inspector.Inspector.names())

    # every enabled component with a registered driver came up healthy (built purely from config)
    for name in ('recorder', 'health', 'bluetooth'):
        assert name in flight.tasks and flight.tasks[name].validate()

    # disabled-by-default + stripped components are absent; the board still runs standalone
    assert 'led' not in flight.tasks  # enabled: False by default
    assert 'wifi' not in flight.tasks and 'cc' not in flight.tasks

    # the gnss driver (atgm336h) is implemented and builds whenever enabled -- a UART has no
    # presence check, unlike the i2c/spi sensors which build only if their device answers on the bus
    assert 'gnss' in flight.tasks and flight.tasks['gnss'].validate()

    """
    the failures mechanism (hardware-independent invariants): a task that came up is never also
    listed failed, and the always-set-up tasks (no bus presence check) never appear in failures.
    The strict "every device connected" gate is the OPERATOR's check -- `verify` /
    tools/board_check.py -- not the unit suite, which must run without every i2c/spi sensor wired.
    """
    assert set(flight.failures) & set(flight.tasks) == set()  # up and failed are disjoint
    assert not (set(flight.failures) & {'gnss', 'recorder', 'health', 'bluetooth'})  # always set up

    await asyncio.sleep_ms(30)  # let the loops tick (the recorder drains)
    await flight.finish()
    assert flight.stage_name() == 'done' and flight.tasks == {}

    print('ok: main.bringup wires Mission/BoardHealth/Controller + Recorder driver, skips driverless sensors; '
          'NVS boot counter +/- (and the real NVS not-found code)')


test_next_boot()
test_nvs_not_found()
asyncio.run(amain())
