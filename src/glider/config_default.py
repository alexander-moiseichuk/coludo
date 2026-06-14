# Baked-in default board configuration for the WaveShare ESP32-P4-WIFI6 controller.
#
# This is the human-edited firmware default and the safe fallback used when no valid
# board.json exists (see specs/board-config.md). Pin numbers come from
# doc/waveshare_esp32p4_pins.md and are validated on hardware by test/test_pins.py.
#
# `default()` returns a FRESH dict each call so callers may mutate it freely.


def default():
    return {
        'board': {'id': 'glider1', 'mcu': 'esp32p4', 'rev': 1},

        'wifi': {                       # STA — the board joins the Control Center's network
            'mode': 'sta',
            'ssid': 'panda',
            'password': '',
            'cc_host': '192.168.10.1',
            'cc_port': 1234,
            'tx_power_dbm': 11,
        },

        'buses': {
            'i2c0':          {'sda': 7, 'scl': 8, 'freq': 400000},
            'uart_recorder': {'tx': 20, 'rx': 21, 'baud': 921600},
            'uart_gnss':     {'tx': 22, 'rx': 23, 'baud': 9600},
        },

        'pins': {
            'led_status': 2,            # external LED (board has no user LED)
            'separation_switch': 33,    # pull-up: LOW=nested, HIGH=separated
            'servo_yaw': 26,
            'servo_elevon_l': 27,
            'servo_elevon_r': 32,
        },

        'components': [
            {'name': 'accel_adxl375', 'driver': 'adxl375', 'bus': 'i2c0', 'addr': 0x53,
             'enabled': True,
             'provides': {'accel': {'priority': 0, 'timeout_ms': 5}}},

            {'name': 'imu_bno055', 'driver': 'bno055', 'bus': 'i2c0', 'addr': 0x28,
             'enabled': True,
             'provides': {'attitude': {'priority': 0, 'timeout_ms': 5},
                          'accel':    {'priority': 1, 'timeout_ms': 5}}},

            {'name': 'baro_icp10111', 'driver': 'icp10111', 'bus': 'i2c0', 'addr': 0x63,
             'enabled': True,
             'provides': {'altitude': {'priority': 0, 'timeout_ms': 100}}},

            {'name': 'baro_bmp280', 'driver': 'bmp280', 'bus': 'i2c0', 'addr': 0x76,
             'enabled': True,
             'provides': {'altitude': {'priority': 1, 'timeout_ms': 200}}},

            {'name': 'gnss', 'driver': 'atgm336h', 'bus': 'uart_gnss', 'addr': None,
             'enabled': True,
             'provides': {'position': {'priority': 0, 'timeout_ms': 150},
                          'altitude': {'priority': 3, 'timeout_ms': 1000}}},

            {'name': 'laser_agl', 'driver': 'sen0648', 'bus': 'i2c0', 'addr': 0x50,
             'enabled': True,
             'provides': {'agl': {'priority': 0, 'timeout_ms': 20}}},

            {'name': 'recorder', 'driver': 'uart_sink', 'bus': 'uart_recorder', 'addr': None,
             'enabled': True},
        ],
    }
