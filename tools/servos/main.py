# main.py — a handheld servo bench tester for an ESP32-C3 + 0.42" SSD1306 OLED + two capacitive
# buttons (see servos.md). Standalone: it does NOT import the glider firmware. Dial one of three views
# of the same PWM state (angle / pulse width / raw duty) up or down with the buttons, watch the live
# set-vs-get read-back on the OLED, and get the full state over the USB console. Named main.py so the
# C3 auto-starts it on power-up -- a self-contained gadget; nothing else lives on that board.
#
# Controls: left (pin 0) = '-', right (pin 1) = '+', both = switch parameter, hold = auto-repeat.
#
# Install (the C3 enumerates as /dev/ttyACM1 here; adjust the port to taste):
#   pip install mpremote                                   # once, on the host
#   mpremote connect /dev/ttyACM1 cp ssd1306.py :          # the OLED driver this imports
#   mpremote connect /dev/ttyACM1 cp main.py :             # this tool (auto-runs on the next reset)
#   mpremote connect /dev/ttyACM1 reset                    # reset -> it starts on its own
# Watch the console (or iterate before relying on auto-run):
#   mpremote connect /dev/ttyACM1 run main.py              # run + stream the console (Ctrl-C to stop)
# Tweak if the hardware differs: _BUTTON_ACTIVE (active-high/low) and _X_OFFSET/_Y_OFFSET (OLED centring).

import asyncio
import time

import framebuf
import ssd1306
from machine import I2C, PWM, Pin

try:
    from micropython import const
except ImportError:  # CPython (host syntax check only)

    def const(value):
        return value


# --- hardware wiring (servos.md) -------------------------------------------------------------------
_I2C_ID = const(0)
_PIN_SDA = const(5)
_PIN_SCL = const(6)
_PIN_SERVO = const(2)
_PIN_LEFT = const(0)   # '-' button
_PIN_RIGHT = const(1)  # '+' button
_BUTTON_ACTIVE = const(1)  # capacitive module reads 1 when touched; set 0 for an active-low module

# --- OLED geometry: a 72x40 visible window centred in the SSD1306's 128x64 buffer ------------------
_BUFFER_WIDTH = const(128)
_BUFFER_HEIGHT = const(64)
_SCREEN_WIDTH = const(72)
_SCREEN_HEIGHT = const(40)
_X_OFFSET = const((_BUFFER_WIDTH - _SCREEN_WIDTH) // 2)   # 28
_Y_OFFSET = const((_BUFFER_HEIGHT - _SCREEN_HEIGHT) // 2)  # 12

# --- PWM / servo envelope --------------------------------------------------------------------------
_FREQ_HZ = const(50)
_PERIOD_US = const(1000000 // _FREQ_HZ)  # 20000 us
_U16 = const(65535)
_ANGLE_MIN = const(0)
_ANGLE_MAX = const(360)
_ANGLE_STEP = const(5)
_PULSE_MIN_US = const(500)
_PULSE_MAX_US = const(2500)
_PULSE_STEP_US = const(25)
_DUTY_STEP = const(100)

# --- button timing (in poll ticks of _POLL_MS) -----------------------------------------------------
_POLL_MS = const(20)
_CONFIRM_TICKS = const(2)  # a lone press must hold this long before the first step (filters a both-press race)
_REPEAT_TICKS = const(8)   # auto-repeat period while held (~160 ms)

_ANGLE, _PULSE, _DUTY = const(0), const(1), const(2)
_PARAM_NAMES = ('angle', 'pulse', 'duty')


def _clamp(value: float, low: float, high: float) -> float:
    """value clamped to [low, high]."""
    return low if value < low else (high if value > high else value)


def _scaled_text(display, text: str, x: int, y: int, scale: int) -> None:
    """Draw `text` at integer `scale` (the FrameBuffer 8x8 font has no native scaling): render it once
    into a temporary 1-bpp buffer, then stamp each lit pixel as a scale*scale block. Cheap enough for a
    few characters on an event-driven redraw."""
    width = len(text) * 8
    temp = framebuf.FrameBuffer(bytearray((width // 8) * 8), width, 8, framebuf.MONO_HLSB)
    temp.fill(0)
    temp.text(text, 0, 0, 1)
    for row in range(8):
        for col in range(width):
            if temp.pixel(col, row):
                display.fill_rect(x + col * scale, y + row * scale, scale, scale, 1)


class Servo:
    """One hobby servo on a PWM pin. The canonical state is the pulse width (us); angle and duty are
    just linear views of it. apply() writes the duty and the peripheral read-back gives the 'get'."""

    def __init__(self, pin: int):
        self._pwm = PWM(Pin(pin), freq=_FREQ_HZ)
        self.pulse_us: float = _PULSE_MIN_US  # canonical commanded state
        self.apply(_PULSE_MIN_US)

    # --- conversions (all linear over the pulse envelope) ---
    @staticmethod
    def angle_to_pulse(angle: float) -> float:
        span = (angle - _ANGLE_MIN) / (_ANGLE_MAX - _ANGLE_MIN)
        return _PULSE_MIN_US + span * (_PULSE_MAX_US - _PULSE_MIN_US)

    @staticmethod
    def pulse_to_angle(pulse: float) -> float:
        span = (pulse - _PULSE_MIN_US) / (_PULSE_MAX_US - _PULSE_MIN_US)
        return _ANGLE_MIN + span * (_ANGLE_MAX - _ANGLE_MIN)

    @staticmethod
    def pulse_to_duty(pulse: float) -> int:
        return round(pulse / _PERIOD_US * _U16)

    @staticmethod
    def duty_to_pulse(duty: float) -> float:
        return duty / _U16 * _PERIOD_US

    def apply(self, pulse_us: float) -> None:
        """Command a pulse width (clamped) and write it to the timer."""
        self.pulse_us = _clamp(pulse_us, _PULSE_MIN_US, _PULSE_MAX_US)
        self._pwm.duty_u16(self.pulse_to_duty(self.pulse_us))

    def duty_get(self) -> int:
        """The duty the PWM peripheral reports back (proves the write, shows quantisation)."""
        return self._pwm.duty_u16()

    # --- set values (what we commanded) ---
    def angle_set(self) -> int:
        return round(self.pulse_to_angle(self.pulse_us))

    def pulse_set(self) -> int:
        return round(self.pulse_us)

    def duty_set(self) -> int:
        return self.pulse_to_duty(self.pulse_us)


class Hud:
    """The 72x40 OLED view: a big 2x read-out of the selected parameter over three compact rows
    (angle/pulse/duty set values) with a '>' cursor on the selection."""

    def __init__(self):
        i2c = I2C(_I2C_ID, sda=Pin(_PIN_SDA), scl=Pin(_PIN_SCL), freq=400000)
        self._display = ssd1306.SSD1306_I2C(_BUFFER_WIDTH, _BUFFER_HEIGHT, i2c)
        self._display.contrast(255)

    def _text(self, text: str, col: int, row: int) -> None:
        """1x text at character cell (col, row) inside the visible window."""
        self._display.text(text, _X_OFFSET + col * 8, _Y_OFFSET + row * 8, 1)

    def render(self, servo: Servo, selected: int) -> None:
        display = self._display
        display.fill(0)
        values = (servo.angle_set(), servo.pulse_set(), servo.duty_set())
        # big read-out of the selected value, centred over the visible width
        big = '%d' % values[selected]
        big_x = _X_OFFSET + max(0, (_SCREEN_WIDTH - len(big) * 16) // 2)
        _scaled_text(display, big, big_x, _Y_OFFSET, 2)
        # three compact rows under the big read-out (rows 2/3/4 of the 5-row window)
        labels = ('ang', 'pul', 'dut')
        for index in range(3):
            cursor = '>' if index == selected else ' '
            self._text('%s%s %d' % (cursor, labels[index], values[index]), 0, 2 + index)
        display.show()


class Buttons:
    """Poll the two buttons: edge-triggered single press with hold-to-repeat, and a both-pressed combo
    that fires once. The _CONFIRM_TICKS delay before the first single step swallows the brief moment one
    button of an intended combo lands before the other."""

    def __init__(self):
        self._left = Pin(_PIN_LEFT, Pin.IN, Pin.PULL_DOWN if _BUTTON_ACTIVE else Pin.PULL_UP)
        self._right = Pin(_PIN_RIGHT, Pin.IN, Pin.PULL_DOWN if _BUTTON_ACTIVE else Pin.PULL_UP)
        self._held_side: int = 0   # -1 left, +1 right, 0 none
        self._held_ticks: int = 0
        self._combo: bool = False

    def _down(self, pin) -> bool:
        return pin.value() == _BUTTON_ACTIVE

    def poll(self) -> str:
        """One sample -> an event: '-', '+', 'switch', or '' (nothing). Call every _POLL_MS."""
        left, right = self._down(self._left), self._down(self._right)
        if left and right:
            self._held_side, self._held_ticks = 0, 0
            if not self._combo:
                self._combo = True
                return 'switch'
            return ''
        if left or right:
            side = -1 if left else 1
            if self._combo:
                return ''  # wait for full release after a combo
            if side != self._held_side:
                self._held_side, self._held_ticks = side, 1  # new press: start the confirm window
                return ''
            self._held_ticks += 1
            first = self._held_ticks == _CONFIRM_TICKS
            repeat = self._held_ticks > _CONFIRM_TICKS and (self._held_ticks - _CONFIRM_TICKS) % _REPEAT_TICKS == 0
            if first or repeat:
                return '-' if side < 0 else '+'
            return ''
        self._held_side, self._held_ticks, self._combo = 0, 0, False
        return ''


class Tester:
    """Glue: hold the servo + selection, apply button events, drive the HUD, and report to the console."""

    def __init__(self):
        self.servo = Servo(_PIN_SERVO)
        self.hud = Hud()
        self.selected: int = _ANGLE
        self.moves: int = 0
        self.angle_min_seen: int = self.servo.angle_set()
        self.angle_max_seen: int = self.servo.angle_set()
        self._last_angle: int = self.servo.angle_set()

    def _step(self, direction: int) -> None:
        """Nudge the selected parameter by its step in `direction` (-1/+1) and re-apply the servo."""
        if self.selected == _ANGLE:
            angle = _clamp(self.servo.angle_set() + direction * _ANGLE_STEP, _ANGLE_MIN, _ANGLE_MAX)
            self.servo.apply(self.servo.angle_to_pulse(angle))
        elif self.selected == _PULSE:
            self.servo.apply(self.servo.pulse_us + direction * _PULSE_STEP_US)
        else:  # _DUTY
            duty = _clamp(self.servo.duty_set() + direction * _DUTY_STEP,
                          self.servo.pulse_to_duty(_PULSE_MIN_US), self.servo.pulse_to_duty(_PULSE_MAX_US))
            self.servo.apply(self.servo.duty_to_pulse(duty))
        self.moves += 1

    def step(self) -> int:
        """The active parameter's step size (for reporting)."""
        return (_ANGLE_STEP, _PULSE_STEP_US, _DUTY_STEP)[self.selected]

    def apply_event(self, event: str) -> None:
        if event == 'switch':
            self.selected = (self.selected + 1) % 3
        elif event == '-':
            self._step(-1)
        elif event == '+':
            self._step(1)
        angle = self.servo.angle_set()
        self.angle_min_seen = min(self.angle_min_seen, angle)
        self.angle_max_seen = max(self.angle_max_seen, angle)
        self.report(event)
        self._last_angle = angle

    def report(self, event: str) -> None:
        """One key=value line per event to the USB console -- the rich data that does not fit the OLED.
        Human-readable and trivially parseable; capture it while you sweep the servo."""
        servo = self.servo
        duty_set, duty_get = servo.duty_set(), servo.duty_get()
        pulse_set = servo.pulse_set()
        pulse_get = round(servo.duty_to_pulse(duty_get))
        travel = (servo.pulse_us - _PULSE_MIN_US) / (_PULSE_MAX_US - _PULSE_MIN_US) * 100
        slew_ms = round(abs(servo.angle_set() - self._last_angle) / 60.0 * 150 + 60)  # SG90 ~0.15s/60deg + settle
        print('t=%d ev=%s sel=%s angle=%d/%d pulse=%dus/%dus duty=%d/%d duty_pct=%.1f '
              'freq=%dHz period=%dus step=%d travel=%.1f%% quant_err=%dus slew_est=%dms moves=%d '
              'angle_span=%d..%d' % (
                  time.ticks_ms(), event or 'tick', _PARAM_NAMES[self.selected],
                  servo.angle_set(), round(servo.pulse_to_angle(servo.duty_to_pulse(duty_get))),
                  pulse_set, pulse_get, duty_set, duty_get, duty_get / _U16 * 100,
                  _FREQ_HZ, _PERIOD_US, self.step(), travel, pulse_set - pulse_get, slew_ms,
                  self.moves, self.angle_min_seen, self.angle_max_seen))

    def banner(self) -> None:
        print('# servo_test -- servo=pin%d left=pin%d right=pin%d i2c=(sda%d,scl%d)' % (
            _PIN_SERVO, _PIN_LEFT, _PIN_RIGHT, _PIN_SDA, _PIN_SCL))
        print('# ranges: angle[%d..%d]/%d pulse[%d..%d]us/%d duty[%d..%d]/%d at %dHz' % (
            _ANGLE_MIN, _ANGLE_MAX, _ANGLE_STEP, _PULSE_MIN_US, _PULSE_MAX_US, _PULSE_STEP_US,
            self.servo.pulse_to_duty(_PULSE_MIN_US), self.servo.pulse_to_duty(_PULSE_MAX_US), _DUTY_STEP, _FREQ_HZ))


async def main() -> None:
    tester = Tester()
    tester.banner()
    tester.report('init')
    tester.hud.render(tester.servo, tester.selected)
    buttons = Buttons()
    while True:
        event = buttons.poll()
        if event:
            tester.apply_event(event)
            tester.hud.render(tester.servo, tester.selected)
        await asyncio.sleep_ms(_POLL_MS)


if __name__ == '__main__':
    asyncio.run(main())
