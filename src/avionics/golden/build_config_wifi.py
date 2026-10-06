"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Golden fixture config_wifi.json: the Wi-Fi a board config sets up, as plan B's own code reads it -- the networks
and the policy from drivers/wifi.py's Wifi.setup(), the hub from tasks/cc_link.py's ControlLink (setup() and
_host()) over cc_client.Client -- in the shape of the port's typed view (board_config::wifi).

Members:
  * bases -- the generated documents the cases name (config_corpus.py);
  * cases -- {name, input: a document named as config_corpus.py describes, output: {policy_enabled, networks:
    [{ssid, retry_ms, password}], hub, hub_port}}. `hub` is "gateway" (the board dials the .1 of the subnet it
    joins), "off" (CC disabled) or {"host": h}.

How each output member is read from plan B, with no rule re-derived here:
  * networks -- Wifi(...).setup() runs on the document, and its `_networks` list is mapped member by member;
  * policy_enabled -- the setup's `_policy` is not 'disabled' (run()'s test for leaving the radio alone);
  * hub -- ControlLink(...).setup() runs (cc_client.create_dispatcher stood in: the dispatcher is not under
    test); a False answer is "off". Otherwise _host() is asked under two different leases: the .1 of each
    lease both times is "gateway", one fixed text both times is that host;
  * hub_port -- the Client's `port`.
A document whose setups raise (a wifi section that is null) has `plan_b` saying so, and the output the port
gives for it: the radio untouched and CC off (no plan B task runs at all).

Credentials: plan B prefers `<ssid>.creds` from the working directory over the config's password. The builder
replaces Wifi._read_password so no file is ever opened, and runs inside an empty scratch directory besides, so
`password` is the config's fallback, as the port's view reports it. Every password in the inputs is made up.

Values the view converts rather than carries (a text retry_ms, a non-text password or host, a float port) stay
in the Rust unit tests: the builder refuses an output it could not compare exactly.

Run through generate.py, which installs the shims first: the imports below need src/glider on sys.path.
"""

import asyncio
import os
import tempfile

import cc_client
import config_corpus
from drivers import wifi as wifi_driver
from tasks import cc_link

_BASE_NAMES: list = ['default', 'flight_ready_new']  # the generated documents the cases may name (config_corpus)
_LEASES: tuple = ('192.168.4.23', '10.7.0.42')  # two board addresses on different subnets, for the hub reading
_U32_END: int = 1 << 32  # retry_ms must fit the view's u32
"""The `plan_b` note of a case whose document makes both setups raise (the output is then the port's reading)."""
_SETUP_RAISED: str = 'the wifi and cc setups raise AttributeError on it: neither task runs'


class _Controller:
    """The one thing the wifi and cc setups read from their Controller: the board config."""

    def __init__(self, document: object) -> None:
        self.config: object = document


class _Lease:
    """A joined wifi driver as cc_link._host() asks it: only its IP address."""

    def __init__(self, address: str) -> None:
        self._address: str = address

    def ip(self) -> str:
        """The board's own address."""
        return self._address


def _config_password_only(driver: object, fallback: str) -> str:
    """Wifi._read_password without the file: the config's password, which is what the view reports."""
    return fallback


def _no_dispatcher(config: dict, controller: object = None) -> None:
    """cc_client.create_dispatcher stood in: the hub reading needs the Client, not a dispatcher."""
    return None


def _network(entry: dict) -> dict:
    """
    One of plan B's network dicts in the view's shape.

    Raises:
        ValueError - a member the view converts instead of carrying (it belongs in the Rust unit tests).
    """
    ssid, retry_ms, password = entry['ssid'], entry['retry_ms'], entry['password']
    exact = (isinstance(ssid, str) and ssid and isinstance(password, str) and isinstance(retry_ms, int)
             and not isinstance(retry_ms, bool) and 0 <= retry_ms < _U32_END)
    if not exact:
        raise ValueError('not comparable exactly: %r' % entry)
    return {'ssid': ssid, 'retry_ms': retry_ms, 'password': password}


def _hub(document: object) -> tuple:
    """
    Where plan B's CC link dials, and on which port.

    Returns:
        (hub, port): hub is 'off', 'gateway' or {'host': text}.

    Raises:
        ValueError - a hub the view cannot carry exactly (a host that is not text, a port that is not an int).
    """
    controller = _Controller(document)
    port = cc_client.Client(document, None).port
    if not isinstance(port, int) or isinstance(port, bool) or not 0 <= port <= 0xFFFF:
        raise ValueError('cc_port %r is not comparable exactly' % port)
    link = cc_link.ControlLink('cc', {'name': 'cc', 'activity': 'cc'}, controller)
    if not asyncio.run(link.setup()):
        return 'off', port
    dialled = [link._host(_Lease(address)) for address in _LEASES]
    if dialled == [cc_link._network_host(address) for address in _LEASES]:
        return 'gateway', port
    if isinstance(dialled[0], str) and dialled[0] == dialled[1]:
        return {'host': dialled[0]}, port
    raise ValueError('cc_host dials %r: not comparable exactly' % dialled)


def _plan_b(document: object) -> dict:
    """
    The Wi-Fi plan B sets up for `document`, in the view's shape.

    Returns:
        {output} or {output, plan_b} when the setups raise.
    """
    driver = wifi_driver.Wifi('wifi', {'name': 'wifi', 'driver': 'wifi'}, _Controller(document))
    try:
        asyncio.run(driver.setup())
    except AttributeError:
        return {'output': {'policy_enabled': False, 'networks': [], 'hub': 'off', 'hub_port': 1234},
                'plan_b': _SETUP_RAISED}
    hub, port = _hub(document)
    return {'output': {'policy_enabled': driver._policy != 'disabled',
                       'networks': [_network(entry) for entry in driver._networks], 'hub': hub, 'hub_port': port}}


def _wifi(section: object) -> dict:
    """A document holding just this wifi section."""
    return {'document': {'wifi': section}}


def _cases() -> list:
    """The inputs: (name, document specification)."""
    cases = [(path.rsplit('/', 1)[1][:-len('.config')], {'file': path}) for path in config_corpus.LAUNCH_FILES]
    cases += [(name, {'base': name}) for name in _BASE_NAMES]
    mixed = ['lab', {'ssid': 'field', 'retry_ms': 500, 'password': 'made-up-field'}, {'ssid': 'car'},
             {'ssid': 'phone', 'password': ''}]
    parked = ['', {'ssid': 'off', 'enabled': False}, {'ssid': 'zero', 'enabled': 0}, {'ssid': 'null', 'enabled': None},
              {'ssid': 'blank', 'enabled': ''}, {'ssid': 'parked', 'policy': 'disabled'}, {'ssid': ''},
              {'ssid': 'yes', 'enabled': 'yes'}, {'ssid': 'auto', 'policy': 'auto'}, {'ssid': 'other', 'policy': 'x'},
              {'ssid': 'on', 'enabled': True, 'mode': 'sta', 'tx_power_dbm': 8}]
    cases += [
        ('no_wifi_section', {'document': {}}),
        ('empty_wifi_section', _wifi({})),
        ('wifi_null', _wifi(None)),
        ('ssid_only', _wifi({'ssid': 'field'})),
        ('ssid_with_section_defaults', _wifi({'ssid': 'field', 'retry_ms': 2500, 'password': 'made-up-shared'})),
        ('ssid_with_empty_networks', _wifi({'ssid': 'field', 'networks': []})),
        ('ssid_with_null_networks', _wifi({'ssid': 'field', 'networks': None})),
        ('empty_ssid_no_networks', _wifi({'ssid': ''})),
        ('networks_win_over_ssid', _wifi({'ssid': 'field', 'networks': ['lab']})),
        ('entries_inherit_and_override', _wifi({'retry_ms': 2500, 'password': 'made-up-shared', 'networks': mixed})),
        ('parked_and_unnamed_entries', _wifi({'networks': parked})),
        ('retry_ms_ends', _wifi({'networks': [{'ssid': 'a', 'retry_ms': 0}, {'ssid': 'b', 'retry_ms': _U32_END - 1}]})),
        ('policy_disabled', _wifi({'policy': 'disabled', 'networks': ['lab']})),
        ('policy_auto', _wifi({'policy': 'auto', 'networks': ['lab']})),
        ('policy_null', _wifi({'policy': None, 'networks': ['lab']})),
        ('policy_does_not_reach_entries', _wifi({'policy': 'disabled', 'enabled': False, 'networks': ['lab']})),
        ('hub_null', _wifi({'cc_host': None})),
        ('hub_off', _wifi({'cc_host': ''})),
        ('hub_off_keeps_the_port', _wifi({'cc_host': '', 'cc_port': 999})),
        ('hub_address', _wifi({'cc_host': '192.168.102.1', 'cc_port': 4321})),
        ('hub_name', _wifi({'cc_host': 'hub.local'})),
        ('hub_named_like_a_gateway', _wifi({'cc_host': '192.168.4.1'})),
        ('hub_port_only', _wifi({'cc_port': 1235})),
    ]
    return cases


def build() -> dict:
    """
    The config_wifi fixture document.

    Args:
        (none)

    Returns:
        {meta, compare, bases, cases}.
    """
    wifi_driver.Wifi._read_password = _config_password_only
    cc_client.create_dispatcher = _no_dispatcher
    known = config_corpus.bases(_BASE_NAMES)
    cases = []
    here = os.getcwd()
    with tempfile.TemporaryDirectory() as directory:
        os.chdir(directory)  # no <ssid>.creds can be in reach, whatever the generator's working directory
        try:
            for name, specification in _cases():
                answer = _plan_b(config_corpus.resolve(specification, known))
                cases.append(dict(answer, name=name, input=specification))
        finally:
            os.chdir(here)
    return {
        'meta': {'source': 'src/glider/drivers/wifi.py (setup), src/glider/tasks/cc_link.py, src/glider/cc_client.py '
                           '(Client); ' + config_corpus.SOURCES},
        'compare': {'kind': 'exact'},
        'bases': known,
        'cases': cases,
    }
