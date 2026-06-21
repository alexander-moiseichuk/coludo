# Validates that Coludo's recommended WaveShare ESP32-P4-WIFI6 pin assignment actually
# constructs on real hardware, and prints each peripheral. Raises (-> runner reports FAIL) if
# any pin cannot be configured. See doc/waveshare_esp32p4_pins.md.
#
# Deliberately does NOT construct the firmware *default* buses: I2C(0) defaults onto the C6
# Wi-Fi pins (GPIO18/19) and SPI(2) onto the microSD pins, so touching the defaults can disrupt
# Wi-Fi / the SD slot. It also never calls I2C(2), which hard-crashes this build.

from machine import I2C, PWM, SPI, UART, Pin

# Recommended map (keep in sync with doc/waveshare_esp32p4_pins.md until board.json drives it).
I2C_SDA, I2C_SCL = 7, 8
REC_TX, REC_RX = 20, 21
GNSS_TX, GNSS_RX = 22, 23
SPI_SCK, SPI_MOSI, SPI_MISO = 48, 47, 46  # ADXL375 on SPI(1)
PIN_ADXL_CS, PIN_ADXL_INT = 49, 4
SERVOS = (('yaw', 26), ('elevon_l', 27), ('elevon_r', 32))
PIN_SEPARATION = 33
PIN_LED = 2


def main():
    i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA), freq=400000)
    print('I2C0 sensors  :', i2c)

    rec = UART(1, tx=REC_TX, rx=REC_RX, baudrate=921600)
    print('UART1 recorder:', rec)
    rec.deinit()

    gnss = UART(2, tx=GNSS_TX, rx=GNSS_RX, baudrate=9600)
    print('UART2 gnss    :', gnss)
    gnss.deinit()

    spi = SPI(1, baudrate=5_000_000, polarity=1, phase=1,
              sck=Pin(SPI_SCK), mosi=Pin(SPI_MOSI), miso=Pin(SPI_MISO))  # ADXL375, mode 3
    print('SPI1 adxl375  :', spi)
    spi.deinit()
    cs = Pin(PIN_ADXL_CS, Pin.OUT, value=1)
    cs.value(1)
    print('adxl375 CS/INT: GPIO%d cs, GPIO%d int' % (PIN_ADXL_CS, PIN_ADXL_INT))

    for name, g in SERVOS:
        pwm = PWM(Pin(g), freq=50, duty_u16=0)
        print('servo %-9s: GPIO%d %s' % (name, g, pwm))
        pwm.deinit()

    sw = Pin(PIN_SEPARATION, Pin.IN, Pin.PULL_UP)
    val = sw.value()
    assert val in (0, 1)
    print('separation sw : GPIO%d pull-up = %d (1=separated)' % (PIN_SEPARATION, val))

    led = Pin(PIN_LED, Pin.OUT)
    led.value(0)
    print('status LED    : GPIO%d out ok' % PIN_LED)

    print('ok: all recommended pins constructed')


main()
