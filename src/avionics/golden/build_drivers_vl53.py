"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture drivers_vl53.json: plan B's two laser rangers (src/glider/drivers/vl53l1x.py, vl53l4cx.py), one
socket at 0x29, as the Rust port (crates/drivers, modules vl53l1x and vl53l4cx) must answer them. Sections:
  * default_configuration -- each silicon's 91-byte block for registers 0x2D..0x87, byte for byte (the L1X block
                             plan B carried once was wrong in three places and the part never ranged);
  * timing_budget         -- the L4CX's RANGE_CONFIG A and B words for an oscillator reading and a budget: the ULD's
                             macro period ((2304 x 2^30 // osc) >> 6) and its mantissa/exponent split, written by
                             plan B's _set_timing_budget (nothing for an oscillator of 0);
  * status                -- _range_locked over a scripted part: the raw RESULT__RANGE_STATUS (its low 5 bits
                             through _STATUS_RTN) and the distance -> the AGL in metres, or none for an invalid range;
  * setup                 -- setup()'s whole I2C transcript per silicon: the XSHUT pulse when wired, the boot polls,
                             the model id (the other silicon refused), the block, the polarity, the VHV calibration
                             cycle and its polls, the L4CX's timing budget, the start;
  * probe                 -- probe()'s transcript and verdict;
  * diagnose              -- diagnose(): the model id's high byte through i2cbus._Device and commons.id_classify.

Two deliberate divergences (avionics-port.md §6), each a case's `divergence` member, which the port owes instead:
  * status: the port checks data-ready (GPIO__TIO_HV_STATUS) before every range read, so a range already published
    is never read again as fresh; its transcript is plan B's behind that one read;
  * probe: the port reads the L4CX's full 16-bit model id where plan B read its high byte alone (the L1X probe
    bug's leftover); the verdicts are the same for both silicons.

Every value comes from running plan B's own code over driver_bus's scripted part; nothing here re-derives what the
port is checked on, save the two divergences' transcripts, built from plan B's own steps.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import random

import config_default
import databoard
import driver_bus
import i2cbus
import recorder
from drivers import vl53l1x, vl53l4cx

_SEED: int = 20261006  # the sweeps' random.Random seed: the fixture is a function of the code alone
_ADDRESS: int = 0x29
_XSHUT_PIN: str = 'laser_xshut'  # the XSHUT pin's name in the stand-in board's pin map
_XSHUT_GPIO: int = 6
_MODULES: dict = {'vl53l1x': (vl53l1x, vl53l1x.Vl53l1x), 'vl53l4cx': (vl53l4cx, vl53l4cx.Vl53l4cx)}
_MODEL: dict = {'vl53l1x': 0xEACC, 'vl53l4cx': 0xEBAA}
_AGL_TOLERANCE: float = 1e-5  # m: the port's AGL is the f32 quotient mm / 1000, half an ulp at 65.535 m is 3.8e-6


class _Controller:
    """The board the driver reads its pins from: plan B's default config, with an XSHUT pin mapped."""

    def __init__(self) -> None:
        self.config = config_default.default()
        self.config['pins'][_XSHUT_PIN] = _XSHUT_GPIO


class _Channel:
    """A databoard channel stand-in."""

    def push(self, value) -> None:
        pass

    def value(self):
        return None


def _driver(silicon: str, config: dict = None):
    """A plan B driver for `silicon` that has not run setup()."""
    module, driver_class = _MODULES[silicon]
    return driver_class('laser_agl', dict(config or {}), _Controller())


def _configurations() -> dict:
    """The default_configuration section."""
    return {'compare': {'kind': 'exact'},
            'note': '_DEFAULT_CONFIG, registers 0x2D..0x87, as hex',
            'cases': [{'name': silicon, 'input': {'silicon': silicon},
                       'output': bytes(module._DEFAULT_CONFIG).hex()} for silicon, (module, _) in _MODULES.items()]}


def _budget_case(oscillator: int, budget_ms: int) -> dict:
    """One timing_budget case: the RANGE_CONFIG writes plan B makes after reading this oscillator word."""
    bus = driver_bus.Bus(driver_bus.sequence([oscillator.to_bytes(2, 'big')] + [None, None] * (oscillator != 0)))
    driver = _driver('vl53l4cx')
    driver._bus, driver._addr = bus, _ADDRESS
    driver_bus.run(bus, driver._set_timing_budget(budget_ms))
    writes = [[int(step['bytes'][:4], 16), int(step['bytes'][4:], 16)] for step in bus.transcript[1:]]
    return {'name': 'osc_%d_budget_%d' % (oscillator, budget_ms),
            'input': {'oscillator': oscillator, 'budget_ms': budget_ms}, 'output': writes}


def _budgets() -> dict:
    """The timing_budget section."""
    cases = [_budget_case(oscillator, budget) for oscillator, budget in (
        (0x0B9A, 50), (0x0B9A, 20), (0x0B9A, 100), (0x0B9A, 200), (0x0B9A, 1000), (0x0BC0, 50), (0x0C00, 33),
        (0, 50), (1, 50), (65535, 50), (0x0B9A, 3), (0x0B9A, 2), (0x0B9A, 0), (0x0B9A, -5), (0x0B9A, 10 ** 9))]
    generator = random.Random(_SEED)
    for _ in range(40):
        cases.append(_budget_case(generator.randint(0x0800, 0x1000), generator.randint(10, 500)))
    return {'compare': {'kind': 'exact'},
            'note': ('_set_timing_budget(budget_ms) after the oscillator word reads `oscillator`: the [register, '
                     'word] writes in order, none for an oscillator of 0'),
            'cases': cases}


def _status_case(raw: int, distance: int) -> dict:
    """One status case: _range_locked over a part answering this status byte and distance."""
    bus = driver_bus.Bus(driver_bus.sequence([bytes((raw,)), distance.to_bytes(2, 'big'), None]))
    driver = _driver('vl53l1x')
    driver._bus, driver._addr = bus, _ADDRESS
    agl = driver_bus.run(bus, driver._range_locked())
    ready = {'op': 'write_read', 'address': _ADDRESS, 'write': '0031', 'read': '01'}
    return {'name': 'raw_0x%02x_%d_mm' % (raw, distance), 'input': {'script': bus.transcript},
            'output': {'agl': agl},
            'divergence': {'note': ('the port reads GPIO__TIO_HV_STATUS first and ranges only on data-ready '
                                    '(polarity 1, bit 0 set here)'),
                           'script': [ready] + bus.transcript, 'agl': agl}}


def _statuses() -> dict:
    """The status section: every raw status code, with the bits above the low five set too."""
    cases = [_status_case(raw, 1234) for raw in range(32)]
    cases += [_status_case(0x09 | 0xE0, 1500), _status_case(0x09, 0), _status_case(0x09, 65535),
              _status_case(0x09, 3999), _status_case(0x13, 800)]
    return {'compare': {'kind': 'abs', 'tol': _AGL_TOLERANCE},
            'note': ('_range_locked(): the AGL in metres (distance_mm / 1000.0) for status 0, null otherwise; the '
                     'port owes each case its divergence (the data-ready read first)'),
            'cases': cases}


def _setup_case(name: str, silicon: str, answers: list, config: dict = None) -> dict:
    """One setup case: plan B's setup() over a part answering `answers` in order."""
    part = driver_bus.sequence(answers)
    bus = driver_bus.Bus(part)
    module = _MODULES[silicon][0]
    driver = _driver(silicon, config)
    originals = i2cbus.bind, databoard.Databoard.provide, module.Pin
    i2cbus.bind = lambda board, device, default: (bus, device.get('addr', default))
    databoard.Databoard.provide = staticmethod(lambda source, provides, *want: _Channel())
    module.Pin = bus.pin_class()
    try:
        ok = driver_bus.run(bus, driver.setup())
    finally:
        i2cbus.bind, databoard.Databoard.provide, module.Pin = originals
    if not part.exhausted():
        raise ValueError('drivers_vl53: setup case %s left scripted answers unused' % name)
    return {'name': name, 'input': {'silicon': silicon, 'config': config or {}, 'script': bus.transcript},
            'output': {'ok': ok}}


def _bring_up(silicon: str, model: int, boot_polls: int = 1, mux: int = 0x01, ready_polls: int = 1,
              oscillator: int = 0x0B9A) -> list:
    """
    The answers a part gives a whole setup() of `silicon`: the boot polls (the last one booted), the model id, the
    block ack, GPIO_HV_MUX__CTRL, the start ack, the data-ready polls (the last one ready, for its polarity), the
    clear, the stop, the two VHV writes, the L4CX's 0x0024 write, oscillator read and two budget writes, the start.
    """
    polarity = 0 if mux & 0x10 else 1
    answers = [b'\x00'] * (boot_polls - 1) + [b'\x03', model.to_bytes(2, 'big'), None, bytes((mux,)), None]
    answers += [bytes((polarity ^ 1,))] * (ready_polls - 1) + [bytes((polarity,))]
    answers += [None, None, None, None]
    if silicon == 'vl53l4cx':
        answers += [None, oscillator.to_bytes(2, 'big')] + [None, None] * (oscillator != 0)
    return answers + [None]


def _setups() -> dict:
    """The setup section."""
    nak = driver_bus.NAK
    xshut = {'xshut_pin': _XSHUT_PIN}
    cases = []
    for silicon in ('vl53l1x', 'vl53l4cx'):
        model, other = _MODEL[silicon], _MODEL['vl53l4cx' if silicon == 'vl53l1x' else 'vl53l1x']
        cases += [
            _setup_case('%s_healthy' % silicon, silicon, _bring_up(silicon, model, boot_polls=3, ready_polls=7)),
            _setup_case('%s_with_xshut' % silicon, silicon, _bring_up(silicon, model), config=xshut),
            _setup_case('%s_active_low_data_ready' % silicon, silicon,
                        _bring_up(silicon, model, mux=0x11, ready_polls=4)),
            _setup_case('%s_refuses_the_other_silicon' % silicon, silicon, [b'\x03', other.to_bytes(2, 'big')]),
            _setup_case('%s_firmware_never_boots' % silicon, silicon, [b'\x00'] * 100),
            _setup_case('%s_calibration_never_ready' % silicon, silicon,
                        _bring_up(silicon, model)[:5] + [b'\x00'] * 200 + _bring_up(silicon, model)[6:]),
            _setup_case('%s_nothing_acks' % silicon, silicon, [nak]),
            _setup_case('%s_the_block_naks' % silicon, silicon, [b'\x03', model.to_bytes(2, 'big'), nak]),
            _setup_case('%s_another_address' % silicon, silicon, _bring_up(silicon, model), config={'addr': 0x30}),
        ]
    cases += [
        _setup_case('vl53l4cx_oscillator_0_sets_no_budget', 'vl53l4cx', _bring_up('vl53l4cx', 0xEBAA, oscillator=0)),
        _setup_case('vl53l4cx_a_100_ms_budget', 'vl53l4cx', _bring_up('vl53l4cx', 0xEBAA),
                    config={'timing_budget_ms': 100}),
        _setup_case('vl53l4cx_the_oscillator_read_naks', 'vl53l4cx', _bring_up('vl53l4cx', 0xEBAA)[:11] + [nak]),
    ]
    return {'compare': {'kind': 'exact'},
            'note': ('setup() over a scripted part: input the silicon (the driver), the component block (xshut_pin '
                     'names a wired XSHUT) and the transcript; output ok'),
            'cases': cases}


def _probe_case(name: str, silicon: str, model: int) -> dict:
    """One probe case: probe() over a part whose model id is `model`."""
    width = 2 if silicon == 'vl53l1x' else 1
    bus = driver_bus.Bus(driver_bus.sequence([model.to_bytes(2, 'big')[:width]]))
    driver = _driver(silicon)
    driver._bus, driver._addr = bus, _ADDRESS
    original = recorder.Recorder.log
    recorder.Recorder.log = staticmethod(lambda descriptor, message: None)
    try:
        verdict = driver_bus.run(bus, driver.probe())
    finally:
        recorder.Recorder.log = original
    case = {'name': name, 'input': {'silicon': silicon, 'script': bus.transcript}, 'output': {'ok': verdict is None}}
    if width == 1:
        case['divergence'] = {'note': 'the port reads the full 16-bit model id (plan B: the high byte alone)',
                              'script': [dict(bus.transcript[0], read=model.to_bytes(2, 'big').hex())],
                              'ok': verdict is None}
    return case


def _probes() -> dict:
    """The probe section."""
    return {'compare': {'kind': 'exact'},
            'note': "probe(): the model id read and whether it passed; a divergence's transcript is owed instead",
            'cases': [_probe_case('vl53l1x_on_its_silicon', 'vl53l1x', 0xEACC),
                      _probe_case('vl53l1x_on_the_l4cx', 'vl53l1x', 0xEBAA),
                      _probe_case('vl53l1x_on_a_shared_high_byte', 'vl53l1x', 0xEA00),
                      _probe_case('vl53l4cx_on_its_silicon', 'vl53l4cx', 0xEBAA),
                      _probe_case('vl53l4cx_on_the_l1x', 'vl53l4cx', 0xEACC)]}


def _diagnose_case(silicon: str, name: str, answer) -> dict:
    """One diagnose case: the model id's high byte, or a refusal."""
    bus = driver_bus.Bus(driver_bus.sequence([answer]))
    driver = _driver(silicon)
    driver._bus, driver._addr = bus, _ADDRESS
    verdict = driver_bus.run(bus, driver.diagnose())
    return {'name': '%s_%s' % (silicon, name), 'input': {'silicon': silicon, 'script': bus.transcript},
            'output': verdict}


def _diagnoses() -> dict:
    """The diagnose section."""
    answers = (('l1x_silicon', b'\xea'), ('l4cx_silicon', b'\xeb'), ('unpowered', b'\x00'), ('idle_high', b'\xff'),
               ('absent', driver_bus.NAK))
    return {'compare': {'kind': 'exact'},
            'note': 'diagnose(): one byte of the model id; the verdict text (the other silicon is a wrong device)',
            'cases': [_diagnose_case(silicon, name, answer) for silicon in _MODULES for name, answer in answers]}


def build() -> dict:
    """
    The drivers_vl53 fixture document.

    Returns:
        {meta, default_configuration, timing_budget, status, setup, probe, diagnose}.
    """
    return {
        'meta': {'source': ('src/glider/drivers/vl53l1x.py, src/glider/drivers/vl53l4cx.py, src/glider/i2cbus.py, '
                            'src/glider/test/test_vl53l1x.py')},
        'default_configuration': _configurations(),
        'timing_budget': _budgets(),
        'status': _statuses(),
        'setup': _setups(),
        'probe': _probes(),
        'diagnose': _diagnoses(),
    }
