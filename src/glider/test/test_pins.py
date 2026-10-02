"""
Coludo project, copyright under MIT license, Alexander Moiseichuk

Validates that Coludo's recommended WaveShare ESP32-P4-WIFI6 pin assignment actually constructs on
real hardware, and prints each peripheral. Raises (-> runner reports FAIL) if any pin cannot be
configured. See doc/waveshare_esp32p4_pins.md.

Deliberately does NOT construct the firmware *default* buses: I2C(0) defaults onto the C6 Wi-Fi pins
(GPIO18/19) and SPI(2) onto the microSD pins, so touching the defaults can disrupt Wi-Fi / the SD
slot. It also never calls I2C(2), which hard-crashes this build.
"""

import config_default
from machine import I2C, PWM, SPI, UART, Pin

"""
THE PIN MAP COMES FROM THE CONFIG, not from a copy kept here.

This file used to carry its own literal map "kept in sync with doc/waveshare_esp32p4_pins.md" -- a
document its own notes mark as OUTDATED, with config_default.py as the truth. The copy drifted: it
described GPIO49 as "ADXL375 on SPI(1)" and asserted that pin on boards that have no ADXL375 at all
(every v1.0 and v1.1 build), so the test could pass while describing hardware that does not exist.
Reading the config means this test checks the map the FIRMWARE will actually use.
"""
_CFG = config_default.default()
_PINS = _CFG['pins']
_I2C0 = _CFG['buses']['i2c']['0']
_SPI1 = _CFG['buses']['spi']['1']

_UART1, _UART2 = _CFG['buses']['uart']['1'], _CFG['buses']['uart']['2']

I2C_SDA, I2C_SCL = _I2C0['sda'], _I2C0['scl']
# The recorder link is TX-ONLY -- the old literal map here paired it with rx=21, a pin that has since
# been reassigned. Taking it from the config is exactly how that kind of drift stops mattering.
REC_TX, REC_RX = _UART1['tx'], _UART1.get('rx')
GNSS_TX, GNSS_RX = _UART2['tx'], _UART2['rx']
SPI_SCK, SPI_MOSI, SPI_MISO = _SPI1['sck'], _SPI1['mosi'], _SPI1['miso']
PIN_ADXL_CS = _PINS['adxl375_cs']   # not fitted on v1.0/v1.1, but the pin is still deselected
PIN_LSM_CS = _PINS['lsm6dso32_cs']  # the device that is actually on this bus here
_LSM_WHO_AM_I, _LSM_ID = 0x0F, 0x6C
_LSM_CTRL3_C, _LSM_CFG_C = 0x12, 0x44  # BDU + IF_INC, as drivers/lsm6dso32.py writes at setup
SERVOS = tuple((name, _PINS['servo_' + key]) for name, key in
               (('yaw', 'yaw'), ('elevon_l', 'eleron_left'), ('elevon_r', 'eleron_right')))
PIN_SEPARATION = _PINS['separation_switch']


def _repin_imu():
    """
    Resynchronise the LSM6DSO32 after this test has created and destroyed SPI(1) underneath it.

    Reads its id until it answers, then writes CTRL3_C exactly as the driver's setup() does -- the write
    is what actually pins the interface, and the reads are what get the part back in step to accept it.
    Budgeted well past the ~30 measured here, because the cost is microseconds and the cost of being
    wrong is the next test (or the next boot) finding no gyro.

    Args:
        (none)

    Returns:
        None; prints how many reads it took, which is the number worth watching if this ever regresses.
    """
    spi = SPI(1, baudrate=5_000_000, polarity=1, phase=1,
              sck=Pin(SPI_SCK), mosi=Pin(SPI_MOSI), miso=Pin(SPI_MISO))
    cs = Pin(PIN_LSM_CS, Pin.OUT, value=1)
    frame = bytearray(2)
    found = -1
    for attempt in range(64):
        cs.value(0)
        spi.write_readinto(bytes([0x80 | _LSM_WHO_AM_I, 0x00]), frame)  # 0x80 = read
        cs.value(1)
        if frame[1] == _LSM_ID:
            found = attempt
            break
    if found >= 0:
        cs.value(0)
        spi.write(bytes([_LSM_CTRL3_C, _LSM_CFG_C]))  # the write is what pins the interface
        cs.value(1)
    spi.deinit()
    cs.value(1)
    print('imu re-pinned : WHO_AM_I after %d reads%s' % (found, '' if found >= 0 else ' -- NOT FOUND'))
    assert found >= 0, 'LSM6DSO32 did not recover after the SPI(1) deinit -- the bus was left broken'


def main():
    i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=_I2C0.get('freq', 400000))
    print('I2C0 sensors  :', i2c)

    rec = (UART(1, tx=REC_TX, rx=REC_RX, baudrate=_UART1['baud']) if REC_RX is not None
           else UART(1, tx=REC_TX, baudrate=_UART1['baud']))
    print('UART1 recorder:', rec)
    rec.deinit()

    gnss = UART(2, tx=GNSS_TX, rx=GNSS_RX, baudrate=_UART2['baud'])
    print('UART2 gnss    :', gnss)
    gnss.deinit()

    """
    SPI(1) carries a FLIGHT SENSOR, so this test has to put it back.

    Constructing and deinitialising this peripheral leaves the LSM6DSO32 (cs 50) out of step with it:
    measured here, WHO_AM_I returns 0x00 and STAYS there -- 30 reads, 60 reads, it does not matter,
    because reading is not what recovers it. Writing CTRL3_C pins the interface and fixes it at once.
    An MCU reset does NOT clear it, because the reset never reaches the chip, so the damage outlived
    every `mpremote reset` and made test_spibus fail in three consecutive full suite runs while passing
    standalone. Bisected to this file.

    This is bench-only: a warm start (12 tries) and a bus retune (4) were both measured and neither
    reproduces it. Creating and destroying the peripheral is what does, and only this test does that.

    Both chip-selects are held high while the peripheral exists (the ADXL375 is the other device on
    this bus), and `_repin_imu()` below resynchronises the part afterwards. A test that disturbs a
    shared resource restores it.
    """
    deselect = [Pin(PIN_ADXL_CS, Pin.OUT, value=1), Pin(PIN_LSM_CS, Pin.OUT, value=1)]
    spi = SPI(1, baudrate=5_000_000, polarity=1, phase=1,
              sck=Pin(SPI_SCK), mosi=Pin(SPI_MOSI), miso=Pin(SPI_MISO))  # ADXL375 + LSM6DSO32, mode 3
    print('SPI1 imu bus  :', spi)
    spi.deinit()
    for pin in deselect:
        pin.value(1)  # still deselected after the deinit -- a floating CS is what this guards against
    print('CS held high  : GPIO%d adxl375, GPIO%d lsm6dso32' % (PIN_ADXL_CS, PIN_LSM_CS))
    _repin_imu()

    for name, g in SERVOS:
        pwm = PWM(Pin(g), freq=50, duty_u16=0)
        print('servo %-9s: GPIO%d %s' % (name, g, pwm))
        pwm.deinit()

    sw = Pin(PIN_SEPARATION, Pin.IN, Pin.PULL_UP)
    val = sw.value()
    assert val in (0, 1)
    print('separation sw : GPIO%d pull-up = %d (1=separated)' % (PIN_SEPARATION, val))

    print('ok: all recommended pins constructed')


main()
