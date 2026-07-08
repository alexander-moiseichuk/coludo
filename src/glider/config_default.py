# Baked-in default board configuration for the WaveShare ESP32-P4-WIFI6 controller.
#
# Human-edited firmware default and the safe fallback when no valid board.config exists (see
# specs/board-config.md). Pins come from doc/waveshare_esp32p4_pins.md (validated on hardware by
# test/test_pins.py). `default()` returns a FRESH dict each call so callers may mutate it freely.
#
# Topology: buses are grouped by type then id; a sensor/component addresses one by `bus` (the kind,
# e.g. 'i2c') + `id` (its int id), so nothing parses a 'type:id' string. `sensors` are data
# providers fused by quantity + priority (several may provide the same quantity with different
# drivers/priorities); `components` are the consumers/actuators (recorder, ...).

import commons

try:
    from version import VERSION as _FIRMWARE_VERSION  # generated at deploy/install (git commit sha)
except ImportError:
    _FIRMWARE_VERSION = 'dev'


def default() -> dict:
    return {
        'board': {'id': 'taster', 'mcu': 'esp32p4', 'rev': 1, 'firmware_version': _FIRMWARE_VERSION},
        'setup_retries': 3,  # re-attempt a flaky device setup at boot (breadboard contacts; 1 = no retry)
        # warm start (specs/coludo.md "In-flight reboot & warm start"): a mid-air reset restores
        # GLIDING when the NVS breadcrumb + the separation latch + baro-above-pad all agree.
        # False -> every boot is cold (bench work with a separated stack on the desk).
        'warm_start': True,
        'max_range_m': 200,  # landing zone must be within this of the launch point -- the AIRFRAME glide
        # range (a bigger glider reaches farther); the mission range-gate uses it
        'wifi': {  # STA-only (the sole supported radio mode; no `mode` knob exists)
            # policy (CC-less field ops, specs/coludo.md "Field operation without CC"): 'auto'
            # joins/rejoins every retry_ms on the ground, goes SILENT from BOOSTING through LANDING
            # (no reconnect churn under GC-off) and resumes at DONE (recovery-crew hotspot);
            # 'disabled' never touches the radio this session.
            'policy': 'auto',
            'ssid': 'panda',
            # networks: FULL per-network configs, each entry the proven panda shape replicated --
            # ssid + optional enabled (default true) / policy / retry_ms / tx_power_dbm /
            # password, missing keys inherited from this top-level section. retry_ms is THAT
            # network's own minimal re-attempt interval (per-network backoff). Scanning runs
            # through SETTING, silent BOOSTING->LANDING, resumes at DONE. Passwords per SSID from
            # <ssid>.creds (panda.creds, coludo.creds); `password` is the shared fallback.
            'networks': [
                {'ssid': 'panda', 'enabled': True, 'retry_ms': 10000},
                {'ssid': 'coludo', 'enabled': True, 'retry_ms': 10000},
            ],
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
                # i2c:1 -- the ESP32-P4's second I2C (audio-codec bus, codec unused). Hosts the
                # INA226 on the aft power board on its own short local bus, so i2c:0 (the forward
                # sensor cluster) stays short/fast (see doc/board_layout.md). GPIOs 30/31 are real
                # header pins (the codec's own 9-13 are not broken out on this board).
                '1': {'sda': 31, 'scl': 30, 'freq': 400000},
            },
            'spi': {
                # ADXL375 on its own SPI bus (mode 3, 5 MHz). NOTE the Adafruit 5374 breakout labels
                # its SPI data pins by their I2C names: SDA = MOSI (->47), SDO = MISO (->46). See
                # doc/hardware.md "ADXL375 -> SPI wiring".
                '1': {'sck': 48, 'mosi': 47, 'miso': 46, 'baud': 5000000, 'mode': 3},
            },
        },
        'pins': {
            'separation_switch': 33,  # copper pads: HIGH=nested (3v3 routed), LOW=separated
            'adxl375_int': 4,  # ADXL375 INT1 (free spare) — DATA_READY drives the accel sampling
            'adxl375_cs': 49,  # ADXL375 SPI chip-select (free spare)
            'lsm6dso32_cs': 50,  # LSM6DSO32 SPI chip-select (shares SPI1 with the ADXL375)
            'lsm6dso32_int1': 28,  # LSM6DSO32 INT1 accel data-ready (INT2/GPIO29 not wired)
            'laser_xshut': 5,  # VL53L4CX XSHUT enable/reset (free spare)
            'laser_int': 3,  # VL53L4CX GPIO1 data-ready interrupt (free spare)
            'servo_yaw': 26,
            'servo_eleron_left': 27,
            'servo_eleron_right': 32,
            'ina226_alert': 29,  # INA226 ALERT (open-drain, active-low) -- hardware over-current trip
            # (GPIO30 became i2c:1 SCL when the INA226 moved to the aft power bus; ALERT -> 29)
        },
        'recorder': {  # PSRAM ring sizes + stats cadence (Recorder)
            'tlm_capacity': 256,  # measured peak ~16 buffered records -> 256 is ~16x headroom
            'log_capacity': 256,
            'cell_size': 256,  # power-of-two cell; ~64 KB/ring, nothing on 32 MB PSRAM
            'stats_ms': 1000,
        },
        # Max fin servos allowed to SLEW at once via servo.move() -- caps the boost-rail current
        # transient. 3 (== fin count) = no limit; drop to 2/1 if the rail sags on the built airframe.
        'servo_concurrency': 3,
        # Dynamic-pressure fin governor (coludo.md "Fin authority"): the flight loop caps fin control
        # deflection by airspeed (commons.fin_deflection_limit, ∝ 1/v²); this scales the whole schedule.
        # 1.0 = nominal; drop (e.g. 0.5) if a flight starts losing fins / control authority in the air.
        'fin_limit_multiplier': 1.0,
        # Control-surface mixing (Phase 3): elevons (the two elerons move together for pitch,
        # differentially for roll) + a rudder (the yaw fin). Flip a gain sign if a surface deflects the
        # wrong way; set `trim` (deg) for mechanical neutral. limit_deg bounds control deflection.
        'mixer': {
            'neutral_deg': commons.SERVO_NEUTRAL_DEG,
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
                'int_pin': 'adxl375_int',  # INT1 (data-ready / boost-detect) — drives the sampling;
                # fallback_ms 500 (default): a timed safety sample keeps data flowing if INT goes silent
                'telemetry_us': 0,  # 0 -> the Recorder global rate (recorder.telemetry_us, 50 Hz)
                'enabled': True,
                'provides': {'accel': {'priority': 1, 'timeout_ms': 20}},  # >32 g backstop behind lsm6dso32
            },
            {
                'name': 'imu_lsm6dso32',
                'driver': 'lsm6dso32',
                'bus': 'spi', 'id': 1,  # shares the ADXL375 SPI1, own chip-select (mode 3, 5 MHz)
                'addr': 0x6A,  # kept for an i2c fallback (set bus 'i2c', id 0)
                'cs_pin': 'lsm6dso32_cs',  # SPI chip-select (GPIO50)
                'int_pin': 'lsm6dso32_int1',  # INT1 accel data-ready drives the sampling (fallback_ms 500 if silent)
                'telemetry_us': 0,  # 0 -> the Recorder global rate (recorder.telemetry_us, 50 Hz)
                'enabled': True,
                # `rate` has NO backup source: on LSM6DSO32 loss the PID D term degrades to
                # d(error)/dt (noisier, no setpoint-kick immunity) -- flies, but less crisply. A
                # BNO055 raw-gyro backup provider is planned with the IMU-redundancy work.
                'provides': {'accel': {'priority': 0, 'timeout_ms': 20},   # PRIMARY accel (±32 g)
                             'rate': {'priority': 0, 'timeout_ms': 20}},    # sole gyro `rate` source
            },
            {
                'name': 'imu_bno055',
                'driver': 'bno055',
                'bus': 'i2c', 'id': 0,
                'addr': 0x28,
                'telemetry_us': 0,  # 0 -> the Recorder global rate (recorder.telemetry_us, 50 Hz)
                'enabled': True,
                'provides': {'attitude': {'priority': 0, 'timeout_ms': 40},
                             'accel': {'priority': 2, 'timeout_ms': 40}},  # fused fallback behind lsm/adxl
            },
            # Attitude REDUNDANCY (tasks/attitude.py): a complementary-filter backup that derives
            # (heading, roll, pitch) from the LSM6DSO32 gyro `rate` + accel gravity vector and provides
            # it at PRIORITY 1, so the databoard swaps to it if the BNO055 (priority 0) stops -- losing
            # the sole attitude source would otherwise go ballistic. Mirrors the BNO055 while it is fresh
            # (warm handoff, no math); free-runs the filter only once it is lost. corr_shift = the accel
            # pull strength (err >> shift); grav band = the |accel| window (g) where the gravity vector
            # is trusted (reject thrust/manoeuvre). Cheap while the BNO055 is alive; enable by default.
            {
                'name': 'attitude', 'activity': 'attitude', 'enabled': True,
                'period_ms': 20, 'accel_period_ms': 50, 'corr_shift': 4,
                'grav_low_g': 0.7, 'grav_high_g': 1.3,
                # turn_gate: suppress the accel gravity-vector correction above this yaw rate (deg/s) --
                # in a coordinated turn the accel points down the body axis (looks level at any bank),
                # so past the gate the filter trusts the gyro alone; below it, straight-ish flight lets
                # the gravity vector re-anchor roll/pitch and cancel gyro drift.
                'turn_gate_deg_s': 4,
                'provides': {'attitude': {'priority': 1, 'timeout_ms': 40}},
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
                'int_pin': 'laser_int',  # GPIO1 data-ready (fallback_ms 500: timed safety sample if INT goes silent)
                'timing_budget_ms': 100,  # ranging integration (10..200); higher = lower sigma, slower
                'enabled': True,
                # laser gives AGL (ground distance), not AMSL altitude, so it provides 'agl' only;
                # ~30 Hz continuous ranging -> 100 ms freshness (tune the timing budget on the bench).
                'provides': {'agl': {'priority': 0, 'timeout_ms': 100}},
            },
            {
                'name': 'power_ina226',
                'driver': 'ina226',
                'bus': 'i2c', 'id': 1,  # aft power bus (i2c:1, sda 31 / scl 30); off the forward i2c:0
                'addr': 0x40,  # INA226, A0=A1=GND (scan-confirmed on i2c:1: mfr 'TI', Vbus ~5 V)
                'shunt_mohms': 10,  # installed 2512 R010 (10 mΩ); calibrate vs a known current for <1% absolute
                'max_current_ma': 5000,  # Current_LSB = 5000mA/2^15 ≈ 153 µA -> CAL = 167772160//(mA·mΩ) ≈ 3355
                'period_ms': 100,  # 10 Hz poll (conversion ~9 ms at 4-sample averaging)
                'alert_pin': 'ina226_alert',  # INA226 ALERT (open-drain) -> GPIO29: hardware over-current trip
                'alert_ma': 3000,  # ALERT fires above this (mA) -- over the ~2.4 A 3-servo peak: a stall/short flag
                'enabled': True,
                'provides': {'voltage': {'priority': 0, 'timeout_ms': 500},
                             'current': {'priority': 0, 'timeout_ms': 500},
                             'power': {'priority': 0, 'timeout_ms': 500}},
            },
            # GNSS. Primary: ATGM336H (CASIC) at 10 Hz. Backup: GY-NEO6MV2 (u-blox NEO-6M) -- same
            # uart:2, both share gnss.py. To run the NEO-6M instead:
            # 1. power the board off, swap the module onto uart:2 (keep board-TX -> module-RX wired:
            # without it the NEO ignores config and free-runs all sentences at 1 Hz),
            # 2. set 'driver' to 'neo6mv2' and 'hz' to 5 (the NEO-6M tops out near 5 Hz).
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
                    'speed': {'priority': 0, 'timeout_ms': 500},  # GNSS ground speed (m/s) -> airspeed governor
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
            # Stage-separation switch (copper pads): HIGH=nested, LOW=separated -> Boosting->Gliding.
            # debounce_ms: the LOW must hold this long -- a contact bounce on the pads never deploys.
            {'name': 'separation', 'driver': 'separation', 'pin': 'separation_switch', 'enabled': True,
             'debounce_ms': 20},
            # Fin servos (SG90) on their PWM pins, commanded in INTEGER degrees via `update {"angle":
            # d}` / move(); neutral (mid-range) at boot. Open-loop -- no position feedback. Powered
            # from a separate boost rail (the board drives signal only). Other servo types = their own
            # `driver`. Per-fin knobs (driver defaults; set per LINKAGE in the board config):
            #   min_us 500 / max_us 2500 -- the PWM pulse endpoints (SG90 datasheet range);
            #   min_deg 0 / max_deg 180  -- the PHYSICAL clamp (a horn that binds at 135 -> max_deg
            #     135); the mixer's limit_deg caps control AUTHORITY, this caps absolute travel;
            #   angle -- boot angle (default: neutral mid-range);
            #   engine_min_mw 500 / engine_max_mw 3500 -- the INA226 servo-rail draw window the probe
            #     sweep must stay inside (stall / no-load detection during the pre-flight self-test).
            {'name': 'servo_yaw', 'driver': 'sg90', 'pin': 'servo_yaw', 'enabled': True},
            {'name': 'servo_eleron_left', 'driver': 'sg90', 'pin': 'servo_eleron_left', 'enabled': True},
            {'name': 'servo_eleron_right', 'driver': 'sg90', 'pin': 'servo_eleron_right', 'enabled': True},
            # Flight-stage automation (Phase 3): launch-detect -> separation/burnout -> agl-landing ->
            # on-ground. Drives the stage machine the control loop gates on; logs every transition
            # (+ sequencer.csv). Enabled -- safe on the passive flights (the flight task is the actuator
            # and stays disabled), and it captures the stage timeline. Tune launch_g/launch_ms from the
            # first powered flights.
            # launch_g 2.5: the measured v2 F15 stack (517 g) boosts at only ~2.84 g (specific force =
            # thrust/mass), so the old 3.0 g would MISS launch on the real airframe. launch_alt_m 10 is an
            # independent backup -- the baro climbing 10 m off the pad trips BOOSTING regardless of the accel
            # threshold (a heavy/marginal boost, or a dropped accel window, still detects).
            # deploy at apogee: apogee_drop_m (baro fell 5 m off its peak) is the primary boost->glide
            # trigger after the separation switch; boost_timeout_ms is the LAST-RESORT fallback, set past
            # the slowest v2 apogee (~11 s, F15 half-glider) so it never pre-empts the apogee detect.
            # disable_gc_flight: compact the heap at BOOSTING and hold GC OFF for the WHOLE airborne phase
            # (re-enable + collect only at DONE, stationary on the ground) so no GC pause can blow a
            # 100 Hz control slice (sequencer._gc_transition). False keeps GC on -- ground benches.
            # apogee_arm_ms: the apogee detector (peak tracking included) is blind this long after
            # BOOSTING entry -- the motor exhaust pressure wave corrupts the in-airframe baro during
            # burn (a false peak could deploy GLIDING under thrust, wings still folded). Covers the
            # longest burn (F15 3.45 s) + margin; no motor reaches apogee before burnout.
            # flight_timeout_ms: the RSO backstop -- this long after BOOSTING entry the stage forces
            # DONE (GC + neutral fins) even with every landing sensor dead, so a blind glider cannot
            # circle until the battery dies. 5 min >> any physically possible TMS flight.
            {'name': 'sequencer', 'activity': 'sequencer', 'enabled': True, 'period_ms': 50,
             'launch_g': 2.5, 'launch_ms': 100, 'launch_alt_m': 10.0,
             'apogee_drop_m': 5.0, 'apogee_arm_ms': 4000, 'boost_timeout_ms': 12000,
             'land_agl_m': 5.0, 'land_ms': 300, 'still_g': 0.3, 'ground_ms': 3000,
             'flight_timeout_ms': 300000, 'disable_gc_flight': True},
            # Phase 3 stabilization loop (off by default -- no actuation until enabled + tuned on the
            # airframe). schedule_hz > 0 -> machine.Timer (deterministic slice, ~1 m/step at 100 Hz/100 m/s);
            # schedule_hz 0 -> asyncio at period_ms. Gains/setpoint are airframe tuning; gates to GLIDING.
            {'name': 'flight', 'activity': 'flight', 'schedule_hz': 100, 'period_ms': 20, 'enabled': False,
             'gains': {'roll': {}, 'pitch': {}, 'yaw': {}},
             # bank-to-turn: GLIDING steers by banking (roll setpoint = nav_bank_gain * heading_error,
             # capped at bank_limit deg) so the turn is tight and orbits the zone to bleed altitude
             # rather than over-ranging it on a flat rudder skid. nav_bank_gain 0 -> rudder-only.
             'nav_bank_gain': 1.5, 'bank_limit': 30,
             # final approach: below final_approach_agl, track the strip CENTRELINE (not the centre
             # point) using the FULL fin authority (45 deg) to crab a crosswind out -- keep it gliding,
             # not rolling-and-dropping. final_approach_agl 0 -> off.
             # land_bank_gain 3.0: the endgame spiral must actually REACH the 45-deg bank -- at
             # 1.5 the rotating-target P-loop saturated near 25 deg and the orbit froze at ~44 m
             # (the 25-deg turn radius); 45 deg gives the R_min ~20 m the <=30 m miss target needs.
             'land_bank_gain': 3.0, 'land_bank_limit': 45,
             # the LOITER orbit (tuned 7/06, coludo.md "Gliding"): within loiter_capture_m of
             # the zone centre the heading command becomes the circle tangent + loiter_gain deg
             # of inward cut per metre off the loiter_radius_m circle -- a stable constant-radius
             # orbit that bleeds altitude through the banked turn (objective #1's energy
             # management). The radius must stay ABOVE the cruise-bank minimum (~34 m at 30 deg)
             # or the orbit destabilizes.
             'loiter_radius_m': 30, 'loiter_capture_m': 120, 'loiter_gain': 3.0,
             # the ENDGAME band: below this baro elevation the glide steering opens the full
             # land-bank authority (turn radius halves) so the last seconds SPIRAL around the zone
             # instead of racetracking past it -- objective #2/#3 tightening that leaves #1 (fly
             # long) untouched up high. 0 -> off.
             'endgame_alt_m': 50,
             'final_approach_agl': 8, 'final_cross_gain': 3.0, 'final_intercept_deg': 45,
             # boost: BOOSTING holds the rod-vertical attitude captured at stage entry, but only once
             # PAST THE ROD (airspeed > boost_engage_speed m/s) -- the 3-point rod keeps it vertical and the
             # fins have no authority below that. The speed governor caps the throw the whole way up.
             'boost_engage_speed': 15.0,
             # airspeed governor (governor.py): the estimator runs FULL RATE pre-glide and on a
             # steep dive (pitch <= airspeed_dive_pitch deg -- a LEADING indicator, the dive
             # precedes the overspeed); otherwise the DISTANCE-CONSTANT throttle updates it at
             # clamp(speed, floor, ceiling) Hz = one update per ~1 m of travel at any speed (the
             # floor bounds time-staleness near stall, the ceiling caps the float-path cost at
             # speed). Staleness self-scales -- an overspeed shrinks its own next interval.
             'airspeed_floor_hz': 5, 'airspeed_ceiling_hz': 50, 'airspeed_dive_pitch': -45.0,
             # GNSS corrector gate: at |pitch| >= this (deg) the receiver's 2D GROUND speed cannot
             # represent airspeed (vertical boost climb, steep dive) -- blending it would loosen the
             # fin cap at high q. Past the gate the accel integrator flies alone (over-read = safe).
             'gnss_steep_pitch': 45.0,
             # navigation cadence: steer()/approach() float trig recomputes at most every
             # nav_period_ms; the loop reads the cached heading between (GNSS fixes ~10 Hz anyway).
             # Omitted-but-supported knobs (their defaults are DERIVED, do not pin them here):
             #   position_age_max_ms -- tier-1 freshness gate, defaults to the GNSS channels' own
             #   databoard windows (set TIGHTER to distrust a stale fix sooner);
             #   integral_limit -- PID anti-windup, defaults to the mixer deflection limit.
             # steering noise filter: integer EMA on the heading error (shift 3 = alpha 1/8,
             # tau ~80 ms @ 100 Hz) -- smooths per-step sensor jitter out of the bank command
             # without touching genuine target changes (>90 deg jumps follow at once). 0 -> off.
             'steer_filter_shift': 3,
             'nav_period_ms': 100,
             'timer_id': 0,  # the machine.Timer instance the schedule_hz timer mode claims
             # per-stage attitude setpoint; stages absent here hold the fins neutral. BOOSTING = hold the
             # captured rod-vertical attitude (no config setpoint); GLIDING = bank-to-turn heading hold.
             # pitch = the TRIM glide attitude: hold it and the glider flies as LONG as possible,
             # bleeding altitude through the orbit's bank, not through a forced off-trim descent (a
             # pitch-0 'level attitude' setpoint fought the trim and tripled the sink to ~10 m/s).
             # -6 is the SIM's trim, NOT the airframe's: FIELD-TUNE it -- set the mixer `trim` so the
             # elevator neutral matches the built airframe, then read the hands-off steady-glide pitch
             # from the walk-test / passive-flight telemetry and put THAT number here (at true trim the
             # pitch PID rides ~zero and the fins stay near neutral). LANDING keeps trim; the flare is
             # a field-tuning knob on top.
             'stages': {'boosting': {}, 'gliding': {'roll': 0, 'pitch': -6}, 'landing': {'roll': 0, 'pitch': -6}}},
            # Watchdog + heartbeat (Phase 3): feeds a hardware WDT (a total event-loop wedge -> hard
            # reset) and supervises the control loop (stall in a control stage -> full reset; boot
            # re-centres the fins). Disabled by default -- a live WDT also resets the board when you
            # drop the running firmware to the REPL for bench work; enable it for flight.
            # wdt_timeout 1000: the frozen-fin bound at a hard OOM (7/06 soak: ~1.4 s to reset).
            # NOT lower: a rescue gc.collect() is atomic (nothing can feed mid-sweep) and its cost
            # scales with heap FILL -- ~65-260 ms on a mostly-free heap (the real-anomaly rescue
            # case) but measured 3.4 s on a ballast-full one; 500 ms killed the rescue in HITL.
            # Fast control-loop-death detection is stall_ms below, independent of this timeout.
            {'name': 'watchdog', 'activity': 'watchdog', 'enabled': False,
             'wdt_timeout_ms': 1000, 'period_ms': 200, 'stall_ms': 500},
            # Board vitals (temperature/memory/load) -> telemetry every period_ms. probe_ms is the
            # load probe's sleep slice (measures wake-up lateness; the core idles between probes).
            # Memory rescue (the pre-OOM safety net, measured 7/06): with GC off in flight the
            # leak is garbage, an emergency gc.collect() (fins hold through the pause)
            # reclaims the flight -- vastly cheaper than the OOM chain (~1.4 s frozen fins +
            # ~7 s reboot) -- and it re-fires EVERY health period for as long as the trigger
            # holds (a fast leak gets a collect per second, altitude allowing; the HITL soak
            # logged 8). One knob, the rest is physics: collect when the predicted
            # time-to-OOM < 2x the time left to sink to the rescue floor (memory-decay vs
            # elevation-decay slopes), with PROVEN safe altitude (elevation > rescue_agl_m ~ 2x
            # the 5 m landing gate: a 0.2 s pause costs ~2 m), in BOOSTING/GLIDING only (never
            # LANDING; no descent trend yet -> no rescue -- boost is 3.5 s of peak dynamics).
            # rescue_agl_m 0 disables. oom_s + land_s ride health.csv + `inspect health`.
            {'name': 'health', 'activity': 'health', 'period_ms': 1000, 'probe_ms': 10, 'enabled': True,
             'rescue_agl_m': 10},
            # CC-less field agent (specs/coludo.md "Field operation without CC"), OFF by default:
            # on the pad it selects the mission site by the first GNSS fix (nearest launch.config
            # site within max_range_m; none -> the spiral-landing fallback zone fallback_offset_m
            # from the fix at fallback_bearing_deg -- point that bearing at the CLEAR sector), and
            # optionally AUTO-ARMS after auto_arm_dwell_s stationary with a live fix. Enable for a
            # field day with no hub; every decision stays operator-overridable via CC.
            {'name': 'field', 'activity': 'field', 'enabled': False, 'period_ms': 1000,
             'site_select': True, 'fallback_bearing_deg': 0.0, 'fallback_offset_m': 50.0,
             'auto_arm': False, 'auto_arm_dwell_s': 60, 'still_g': 0.3},
            # Apply the BLE radio state at boot: off by default to save power (BLE is unused).
            {'name': 'bluetooth', 'driver': 'bluetooth', 'radio': False, 'enabled': True},
            # Connectivity (optional): join Wi-Fi (HAL driver), then serve the CC hub (activity). A
            # board with no Wi-Fi (e.g. FireBeetle 2) skips these at setup and runs standalone.
            {'name': 'wifi', 'driver': 'wifi', 'enabled': True},
            {'name': 'cc', 'activity': 'cc', 'enabled': True},
        ],
    }
