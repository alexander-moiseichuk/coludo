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
        'setup_retries': 3,  # re-attempt a flaky device setup at boot (breadboard contacts; 1 = no retry)
        'wifi': {  # STA — the board joins the Control network
            'mode': 'sta',
            'ssid': 'panda',
            'password': '',
            # no cc_host -> the board dials the `.1` of whatever subnet it joins (the hub by
            # convention); set an explicit address to override, or '' to disable CC (fly standalone).
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
            'spi': {
                # ADXL375 on its own SPI bus (mode 3, 5 MHz). NOTE the Adafruit 5374 breakout labels
                # its SPI data pins by their I2C names: SDA = MOSI (->47), SDO = MISO (->46). See
                # doc/hardware.md "ADXL375 -> SPI wiring".
                '1': {'sck': 48, 'mosi': 47, 'miso': 46, 'baud': 5000000, 'mode': 3},
            },
        },
        'pins': {
            'led_status': 2,  # external LED (board has no user LED)
            'separation_switch': 33,  # copper pads: HIGH=nested (3v3 routed), LOW=separated
            'adxl375_int': 4,  # ADXL375 INT1 (free spare) — DATA_READY drives the accel sampling
            'adxl375_cs': 49,  # ADXL375 SPI chip-select (free spare)
            'laser_xshut': 5,  # VL53L4CX XSHUT enable/reset (free spare)
            'laser_int': 3,  # VL53L4CX GPIO1 data-ready interrupt (free spare)
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
        # Max fin servos allowed to SLEW at once via servo.move() -- caps the boost-rail current
        # transient. 3 (== fin count) = no limit; drop to 2/1 if the rail sags on the built airframe.
        'servo_concurrency': 3,
        # Control-surface mixing (Phase 3): elevons (the two elerons move together for pitch,
        # differentially for roll) + a rudder (the yaw fin). Flip a gain sign if a surface deflects the
        # wrong way; set `trim` (deg) for mechanical neutral. limit_deg bounds control deflection.
        'mixer': {
            'neutral_deg': 90,
            'limit_deg': 45,
            'surfaces': {
                'servo_yaw': {'yaw': 1},
                'servo_eleron_left': {'pitch': 1, 'roll': 1},
                'servo_eleron_right': {'pitch': 1, 'roll': -1},
            },
            'trim': {},  # per-fin neutral offset (deg), set during bench alignment
        },
        # Data providers. Fusion groups by quantity and orders providers by priority (lower
        # first); several providers per quantity is normal (different drivers, redundancy).
        'sensors': [
            {
                'name': 'accel_adxl375',
                'driver': 'adxl375',
                'bus': 'spi', 'id': 1,  # moved off i2c:0 to its own SPI bus for clean high-rate reads
                'addr': 0x53,  # kept for an i2c fallback (set bus 'i2c', id 0)
                'cs_pin': 'adxl375_cs',  # SPI chip-select
                'int_pin': 'adxl375_int',  # INT1 (data-ready / boost-detect) — drives the sampling
                'telemetry_us': 20000,  # ~100 Hz sampling decimated to 50 Hz in accel_adxl375.csv
                'enabled': True,
                'provides': {'accel': {'priority': 0, 'timeout_ms': 20}},
            },
            {
                'name': 'imu_bno055',
                'driver': 'bno055',
                'bus': 'i2c', 'id': 0,
                'addr': 0x28,
                'telemetry_us': 40000,  # ~50 Hz sampling decimated to 25 Hz in imu_bno055.csv
                'enabled': True,
                'provides': {'attitude': {'priority': 0, 'timeout_ms': 40},
                             'accel': {'priority': 1, 'timeout_ms': 40}},
            },
            {
                'name': 'baro_icp10111',
                'driver': 'icp10111',
                'bus': 'i2c', 'id': 0,
                'addr': 0x63,
                'enabled': True,
                # reconcile altitude/pressure: ICP-10111 is the rank-0 primary, so BMP280 (and any
                # GNSS/laser) is bias-corrected against it on a fallback handover (additive scalars).
                'provides': {'altitude': {'priority': 0, 'timeout_ms': 200, 'reconcile': True},
                             'elevation': {'priority': 0, 'timeout_ms': 200},
                             'pressure': {'priority': 0, 'timeout_ms': 200, 'reconcile': True},
                             'temperature': {'priority': 0, 'timeout_ms': 500}},  # slow quantity, capped ≤1000
            },
            {
                'name': 'baro_bmp280',
                'driver': 'bmp280',
                'bus': 'i2c', 'id': 0,
                'addr': 0x76,
                'enabled': True,
                'provides': {'altitude': {'priority': 1, 'timeout_ms': 200},
                             'elevation': {'priority': 1, 'timeout_ms': 200},
                             'pressure': {'priority': 1, 'timeout_ms': 200},
                             'temperature': {'priority': 1, 'timeout_ms': 500}},  # slow quantity, capped ≤1000
            },
            {
                'name': 'laser_agl',
                'driver': 'vl53l4cx',
                'bus': 'i2c', 'id': 0,
                'addr': 0x29,
                'xshut_pin': 'laser_xshut',  # enable/reset
                'int_pin': 'laser_int',  # GPIO1 data-ready
                'timing_budget_ms': 100,  # ranging integration (10..200); higher = lower sigma, slower
                'enabled': True,
                # laser gives AGL (ground distance), not AMSL altitude, so it provides 'agl' only;
                # ~30 Hz continuous ranging -> 100 ms freshness (tune the timing budget on the bench).
                'provides': {'agl': {'priority': 0, 'timeout_ms': 100}},
            },
            # GNSS. Primary: ATGM336H (CASIC) at 10 Hz. Backup: GY-NEO6MV2 (u-blox NEO-6M) -- same
            # uart:2, both share gnss.py. To run the NEO-6M instead:
            #   1. power the board off, swap the module onto uart:2 (keep board-TX -> module-RX wired:
            #      without it the NEO ignores config and free-runs all sentences at 1 Hz),
            #   2. set 'driver' to 'neo6mv2' and 'hz' to 5 (the NEO-6M tops out near 5 Hz).
            # Live-verified on the NEO: RMC 5 Hz (position) + GGA ~1 Hz (altitude/elevation).
            {
                'name': 'gnss',
                'driver': 'atgm336h',  # ATGM336H (CASIC), 10 Hz -- or 'neo6mv2' (see note above)
                'bus': 'uart', 'id': 2,
                'addr': None,
                'hz': 10,  # set 5 for 'neo6mv2' (NEO-6M caps ~5 Hz)
                'enabled': True,
                'provides': {
                    'position': {'priority': 0, 'timeout_ms': 200},  # 10 Hz -> 2x period
                    # altitude/elevation are a deep baro backup: high priority number (low rank), and a
                    # generous window since GGA runs at ~1 Hz to stay within 9600 baud.
                    'altitude': {'priority': 3, 'timeout_ms': 2000},
                    'elevation': {'priority': 3, 'timeout_ms': 2000},
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
            # Fin servos (SG90) on their PWM pins, commanded in INTEGER degrees via `update {"angle":
            # d}` / move(); neutral (mid-range) at boot. Open-loop -- no position feedback. Powered
            # from a separate boost rail (the board drives signal only). Set min_deg/max_deg per fin to
            # limit throw (e.g. the 60deg geared -> +-30deg). Other servo types = their own `driver`.
            {'name': 'servo_yaw', 'driver': 'sg90', 'pin': 'servo_yaw', 'enabled': True},
            {'name': 'servo_eleron_left', 'driver': 'sg90', 'pin': 'servo_eleron_left', 'enabled': True},
            {'name': 'servo_eleron_right', 'driver': 'sg90', 'pin': 'servo_eleron_right', 'enabled': True},
            # Flight-stage automation (Phase 3): launch-detect -> separation/burnout -> agl-landing ->
            # on-ground. Drives the stage machine the control loop gates on; logs every transition
            # (+ sequencer.csv). Enabled -- safe on the passive flights (the flight task is the actuator
            # and stays disabled), and it captures the stage timeline. Tune launch_g/launch_ms from the
            # first powered flights.
            {'name': 'sequencer', 'activity': 'sequencer', 'enabled': True, 'period_ms': 50,
             'launch_g': 3.0, 'launch_ms': 100, 'boost_timeout_ms': 6000,
             'land_agl_m': 5.0, 'still_g': 0.3, 'ground_ms': 3000},
            # Phase 3 stabilization loop (off by default -- no actuation until enabled + tuned on the
            # airframe). schedule_hz > 0 -> machine.Timer (deterministic slice, ~1 m/step at 100 Hz/100 m/s);
            # schedule_hz 0 -> asyncio at period_ms. Gains/setpoint are airframe tuning; gates to GLIDING.
            {'name': 'flight', 'activity': 'flight', 'schedule_hz': 100, 'period_ms': 20, 'enabled': False,
             'gains': {'roll': {}, 'pitch': {}, 'yaw': {}}, 'setpoint': {'roll': 0, 'pitch': 0}},
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
