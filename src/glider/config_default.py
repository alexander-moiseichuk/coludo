# Baked-in default board configuration for the WaveShare ESP32-P4-WIFI6 controller.
#
# Human-edited firmware default and the safe fallback when no valid board.json exists (see
# specs/board-config.md). Pins come from doc/waveshare_esp32p4_pins.md (validated on hardware by
# test/test_pins.py). `default()` returns a FRESH dict each call so callers may mutate it freely.
#
# Topology: buses are grouped by type then id; a sensor/component addresses one by `bus` (the kind,
# e.g. 'i2c') + `id` (its int id), so nothing parses a 'type:id' string. `sensors` are data
# providers fused by quantity + priority (several may provide the same quantity with different
# drivers/priorities); `components` are the consumers/actuators (recorder, ...).

try:
    from version import VERSION as _FIRMWARE_VERSION  # generated at deploy/install (git commit sha)
except ImportError:
    _FIRMWARE_VERSION = 'dev'


def default() -> dict:
    return {
        'board': {'id': 'taster', 'mcu': 'esp32p4', 'rev': 1, 'firmware_version': _FIRMWARE_VERSION},
        'wifi': {  # STA — the board joins the Control network
            'mode': 'sta',
            'ssid': 'panda',
            'password': '',
            'cc_host': '192.168.10.1',
            'cc_port': 1234,
            'tx_power_dbm': 11,
        },
        'buses': {
            'uart': {
                '1': {'tx': 20, 'rx': 21, 'baud': 921600},  # recorder
                '2': {'tx': 22, 'rx': 23, 'baud': 9600},  # gnss
            },
            'i2c': {
                '0': {'sda': 7, 'scl': 8, 'freq': 400000},
            },
            'spi': {},
        },
        'pins': {
            'led_status': 2,  # external LED (board has no user LED)
            'separation_switch': 33,  # copper pads: HIGH=nested (3v3 routed), LOW=separated
            'adxl375_int': 4,  # ADXL375 INT1 (free spare) — DATA_READY drives the accel sampling
            'servo_yaw': 26,
            'servo_eleron_left': 27,
            'servo_eleron_right': 32,
        },
        'recorder': {  # PSRAM ring sizes + stats cadence (Recorder)
            'tlm_capacity': 512,
            'log_capacity': 512,
            'cell_size': 256,  # power-of-two cell; ~128 KB/ring, nothing on 32 MB PSRAM
            'stats_ms': 1000,
        },
        # Data providers. Fusion groups by quantity and orders providers by priority (lower
        # first); several providers per quantity is normal (different drivers, redundancy).
        'sensors': [
            {
                'name': 'accel_adxl375',
                'driver': 'adxl375',
                'bus': 'i2c', 'id': 0,
                'addr': 0x53,
                'int_pin': 'adxl375_int',  # INT1 (data-ready / boost-detect) — polled for now
                'telemetry_us': 20000,  # ~100 Hz sampling decimated to 50 Hz in accel_adxl375.csv
                'enabled': True,
                'provides': {'accel': {'priority': 0, 'timeout_ms': 50}},
            },
            {
                'name': 'imu_bno055',
                'driver': 'bno055',
                'bus': 'i2c', 'id': 0,
                'addr': 0x28,
                'telemetry_us': 40000,  # ~50 Hz sampling decimated to 25 Hz in imu_bno055.csv
                'enabled': True,
                'provides': {'attitude': {'priority': 0, 'timeout_ms': 100},
                             'accel': {'priority': 1, 'timeout_ms': 50}},
            },
            {
                'name': 'baro_icp10111',
                'driver': 'icp10111',
                'bus': 'i2c', 'id': 0,
                'addr': 0x63,
                'enabled': True,
                'provides': {'altitude': {'priority': 0, 'timeout_ms': 300},
                             'temperature': {'priority': 0, 'timeout_ms': 2000}},
            },
            {
                'name': 'baro_bmp280',
                'driver': 'bmp280',
                'bus': 'i2c', 'id': 0,
                'addr': 0x76,
                'enabled': True,
                'provides': {'altitude': {'priority': 1, 'timeout_ms': 500},
                             'temperature': {'priority': 1, 'timeout_ms': 2000}},
            },
            {
                'name': 'laser_agl',
                'driver': 'vl53l4cx',
                'bus': 'i2c', 'id': 0,
                'addr': 0x29,
                'enabled': True,
                'provides': {'agl': {'priority': 0, 'timeout_ms': 20}, 'altitude': {'priority': 2, 'timeout_ms': 20}},
            },
            {
                'name': 'gnss',
                'driver': 'atgm336h',
                'bus': 'uart', 'id': 2,
                'addr': None,
                'hz': 10,
                'enabled': True,
                'provides': {
                    'position': {'priority': 0, 'timeout_ms': 150},
                    'altitude': {'priority': 3, 'timeout_ms': 1000},
                },
            },
        ],
        # Consumers / actuators / system tasks. `driver` runs from drivers/ (HAL), `activity` from
        # tasks/ (higher-level subsystems); both resolve through the same registry.
        'components': [
            # Recorder drain loop: a thin activity over the global Recorder, using uart:1.
            {'name': 'recorder', 'activity': 'recorder', 'bus': 'uart', 'id': 1, 'enabled': True},
            # Status LED on the led_status pin: blinks the board state (error/standby/flying).
            # Disabled by default -- not every board has the external LED wired; enable per board.
            {'name': 'led', 'driver': 'led', 'pin': 'led_status', 'enabled': False},
            # Stage-separation switch (copper pads): HIGH=nested, LOW=separated -> Boosting->Gliding.
            {'name': 'separation', 'driver': 'separation', 'pin': 'separation_switch', 'enabled': True},
            # Board vitals (temperature/memory/load) -> telemetry every period_ms.
            {'name': 'health', 'activity': 'health', 'period_ms': 1000, 'enabled': True},
            # Apply the BLE radio state at boot: off by default to save power (BLE is unused).
            {'name': 'bluetooth', 'driver': 'bluetooth', 'radio': False, 'enabled': True},
            # Connectivity (optional): join Wi-Fi (HAL driver), then serve the CC hub (activity). A
            # board with no Wi-Fi (e.g. FireBeetle 2) skips these at setup and runs standalone.
            {'name': 'wifi', 'driver': 'wifi', 'enabled': True},
            {'name': 'cc', 'activity': 'cc', 'enabled': True},
        ],
    }
