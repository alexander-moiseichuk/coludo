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
_PIN_SERVO = const(3)  # GPIO3 (lastminuteengineers ESP32-C3 super-mini pinout: a safe, free PWM pin)
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

# --- PWM / servo timing ----------------------------------------------------------------------------
_FREQ_HZ = const(50)
_PERIOD_US = const(1000000 // _FREQ_HZ)  # 20000 us
_U16 = const(65535)

# --- button timing (in poll ticks of _POLL_MS) -----------------------------------------------------
_POLL_MS = const(20)
_CONFIRM_TICKS = const(2)        # a lone press must hold this long before the first step (filters a both-press race)
_REPEAT_DELAY_TICKS = const(25)  # then keep holding this long (~0.5 s) before auto-repeat begins -> a tap = 1 step
_REPEAT_TICKS = const(8)         # auto-repeat period once it has begun (sweep while held)
_TRACE_BUTTONS = const(1)        # 1 -> print a 'BTN ...' trace line per button event (bench validation); 0 -> off

# --- the three editable parameters: indices into _HUD_ROWS (defined below, after Servo) -------------
_ANG: int = const(0)
_PUL: int = const(1)
_DUT: int = const(2)


def _clamp(value: int, low: int, high: int) -> int:
    """value clamped to [low, high]."""
    return low if value < low else (high if value > high else value)


def _div_round(numerator: int, denominator: int) -> int:
    """Rounded integer division (both operands >= 0). Used everywhere instead of float math -- the C3
    has no FPU, and rounding (not floor) keeps the angle<->pulse<->duty round-trips from drifting."""
    return (numerator + denominator // 2) // denominator


def _convert(value: int, source: int, target: int) -> int:
    """Map `value` from row `source`'s [min, max] linearly onto row `target`'s [min, max] (integer).
    The three parameters are views of ONE servo position, so this is how a change to one becomes the
    others. The duty row's range is exactly the pulse range expressed as duty, so _convert(pulse->duty)
    equals duty_u16 = pulse * 65535 / period -- no separate formula needed."""
    src, dst = _HUD_ROWS[source], _HUD_ROWS[target]
    return dst['min'] + _div_round((value - src['min']) * (dst['max'] - dst['min']), src['max'] - src['min'])


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
    """Thin PWM wrapper: it holds the canonical pulse width (us, int) and writes / reads back the raw
    duty. All the parameter conversions live in _convert + the _HUD_ROWS callbacks, so this stays
    minimal. Float-free -- the C3 has no FPU."""

    def __init__(self, pin: int):
        self._pwm = PWM(Pin(pin), freq=_FREQ_HZ)
        self.pulse_us: int = _HUD_ROWS[_PUL]['min']  # canonical commanded state
        self.apply_pulse(self.pulse_us)

    def apply_pulse(self, pulse: int) -> None:
        """Command a pulse width (clamped to the pulse row's range) and write it to the timer."""
        self.pulse_us = _clamp(pulse, _HUD_ROWS[_PUL]['min'], _HUD_ROWS[_PUL]['max'])
        self._pwm.duty_u16(_convert(self.pulse_us, _PUL, _DUT))

    def duty_get(self) -> int:
        """The duty the PWM peripheral reports back (proves the write, shows quantisation)."""
        return self._pwm.duty_u16()

    # --- the three parameter views: get/set per parameter, referenced directly by _HUD_ROWS as
    # unbound methods (row['get'](servo) / row['set'](servo, value)). All keyed off the canonical pulse.
    def angle(self) -> int:
        return _convert(self.pulse_us, _PUL, _ANG)

    def set_angle(self, value: int) -> None:
        self.apply_pulse(_convert(value, _ANG, _PUL))

    def pulse(self) -> int:
        return self.pulse_us

    def set_pulse(self, value: int) -> None:
        self.apply_pulse(value)

    def duty(self) -> int:
        return _convert(self.pulse_us, _PUL, _DUT)

    def set_duty(self, value: int) -> None:
        self.apply_pulse(_convert(value, _DUT, _PUL))


# Every per-parameter knob in ONE table (these were scattered across constants + three _step branches):
# the OLED position (x, y in the visible window), the label, the editable [min, max], the step, and the
# get / set callbacks. The three parameters are LINKED views of one position, so a row carries no stored
# value -- get(servo) computes the live value and set(servo, value) applies it through the servo, which
# keeps all three consistent. Indexed by _ANG / _PUL / _DUT. To add a parameter, add a row.
# `text` carries a leading space (the cursor column): render draws the label as-is and overlays a '>'
# on the selected row, so the draw loop needs no per-row cursor branch.
_HUD_ROWS = [
    {'x': 0, 'y': 16, 'text': ' ang', 'min': 0, 'max': 360, 'step': 5,
     'get': Servo.angle, 'set': Servo.set_angle},
    {'x': 0, 'y': 24, 'text': ' pul', 'min': 500, 'max': 2500, 'step': 25,  # microseconds
     'get': Servo.pulse, 'set': Servo.set_pulse},
    {'x': 0, 'y': 32, 'text': ' dut', 'min': 1638, 'max': 8192, 'step': 100,  # duty_u16 == pulse 500..2500us @50Hz
     'get': Servo.duty, 'set': Servo.set_duty},
]


class Hud:
    """The 72x40 OLED view: a big 2x read-out of the selected parameter over the _HUD_ROWS rows
    (angle/pulse/duty live values) with a '>' cursor on the selection."""

    def __init__(self):
        i2c = I2C(_I2C_ID, sda=Pin(_PIN_SDA), scl=Pin(_PIN_SCL), freq=400000)
        self._display = ssd1306.SSD1306_I2C(_BUFFER_WIDTH, _BUFFER_HEIGHT, i2c)
        self._display.contrast(255)

    def render(self, servo: Servo, selected: int) -> None:
        display = self._display
        display.fill(0)
        # big read-out of the selected value, centred over the visible width
        big = '%d' % _HUD_ROWS[selected]['get'](servo)
        big_x = _X_OFFSET + max(0, (_SCREEN_WIDTH - len(big) * 16) // 2)
        _scaled_text(display, big, big_x, _Y_OFFSET, 2)
        # one compact row per parameter (the label already carries the leading cursor column)
        for row in _HUD_ROWS:
            display.text('%s %d' % (row['text'], row['get'](servo)), _X_OFFSET + row['x'], _Y_OFFSET + row['y'], 1)
        # overlay the cursor on the selected row's leading column
        chosen = _HUD_ROWS[selected]
        display.text('>', _X_OFFSET + chosen['x'], _Y_OFFSET + chosen['y'], 1)
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
        self._counts: dict = {'+': 0, '-': 0, 'switch': 0}  # bench instrumentation (see _emit)

    def _down(self, pin) -> bool:
        return pin.value() == _BUTTON_ACTIVE

    def _emit(self, event: str, left: bool, right: bool) -> str:
        """Tally an event and (when _TRACE_BUTTONS) print a 'BTN' trace line: the event, the raw pin
        levels (confirms wiring/polarity), and the running +/-/switch counts -- so a hand-pressed
        sequence is verifiable from the console."""
        self._counts[event] += 1
        if _TRACE_BUTTONS:
            print('BTN ev=%-6s L=%d R=%d +=%d -=%d switch=%d' % (
                event, left, right, self._counts['+'], self._counts['-'], self._counts['switch']))
        return event

    def poll(self) -> str:
        """One sample -> an event: '-', '+', 'switch', or '' (nothing). Call every _POLL_MS."""
        left, right = self._down(self._left), self._down(self._right)
        if left and right:
            self._held_side, self._held_ticks = 0, 0
            if not self._combo:
                self._combo = True
                return self._emit('switch', left, right)
            return ''
        if left or right:
            side = -1 if left else 1
            if self._combo:
                return ''  # wait for full release after a combo
            if side != self._held_side:
                self._held_side, self._held_ticks = side, 1  # new press: start the confirm window
                return ''
            self._held_ticks += 1
            elapsed = self._held_ticks - _CONFIRM_TICKS  # ticks since the first step fired
            first = elapsed == 0  # the initial step
            # auto-repeat only after holding _REPEAT_DELAY_TICKS past the first step, then every _REPEAT_TICKS
            repeat = elapsed >= _REPEAT_DELAY_TICKS and (elapsed - _REPEAT_DELAY_TICKS) % _REPEAT_TICKS == 0
            if first or repeat:
                return self._emit('-' if side < 0 else '+', left, right)
            return ''
        self._held_side, self._held_ticks, self._combo = 0, 0, False
        return ''


class Tester:
    """Glue: hold the servo + selection, apply button events through the _HUD_ROWS callbacks, drive the
    HUD, and report to the console."""

    def __init__(self):
        self.servo = Servo(_PIN_SERVO)
        self.hud = Hud()
        self.selected: int = _ANG
        self.moves: int = 0
        angle = _HUD_ROWS[_ANG]['get'](self.servo)
        self.angle_min_seen: int = angle
        self.angle_max_seen: int = angle
        self._last_angle: int = angle

    def _step(self, direction: int) -> None:
        """Nudge the selected parameter by its step in `direction` (-1/+1), clamped to the row's range,
        and apply it through the row's set() callback. Fully table-driven -- no per-parameter branch."""
        row = _HUD_ROWS[self.selected]
        value = _clamp(row['get'](self.servo) + direction * row['step'], row['min'], row['max'])
        row['set'](self.servo, value)
        self.moves += 1

    def apply_event(self, event: str) -> None:
        if event == 'switch':
            self.selected = (self.selected + 1) % len(_HUD_ROWS)
        elif event == '-':
            self._step(-1)
        elif event == '+':
            self._step(1)
        angle = _HUD_ROWS[_ANG]['get'](self.servo)
        self.angle_min_seen = min(self.angle_min_seen, angle)
        self.angle_max_seen = max(self.angle_max_seen, angle)
        self.report(event)
        self._last_angle = angle

    def report(self, event: str) -> None:
        """One key=value line per event to the USB console -- the rich data that does not fit the OLED.
        Human-readable and trivially parseable; capture it while you sweep the servo. Float-free: the
        percentages are integer tenths (per-mille // formatting)."""
        servo = self.servo
        row, pulse = _HUD_ROWS[self.selected], _HUD_ROWS[_PUL]
        duty_get = servo.duty_get()
        pulse_set, pulse_get = servo.pulse_us, _convert(duty_get, _DUT, _PUL)
        angle_set, angle_get = _HUD_ROWS[_ANG]['get'](servo), _convert(pulse_get, _PUL, _ANG)
        duty_set = _HUD_ROWS[_DUT]['get'](servo)
        duty_pct = _div_round(duty_get * 1000, _U16)  # tenths of a percent
        travel = _div_round((pulse_set - pulse['min']) * 1000, pulse['max'] - pulse['min'])
        slew_ms = _div_round(abs(angle_set - self._last_angle) * 150, 60) + 60  # SG90 ~0.15s/60deg + settle
        print('t=%d ev=%s sel=%s angle=%d/%d pulse=%dus/%dus duty=%d/%d duty_pct=%d.%d '
              'freq=%dHz period=%dus step=%d travel=%d.%d%% quant_err=%dus slew_est=%dms moves=%d '
              'angle_span=%d..%d' % (
                  time.ticks_ms(), event or 'tick', row['text'].strip(),
                  angle_set, angle_get, pulse_set, pulse_get, duty_set, duty_get,
                  duty_pct // 10, duty_pct % 10, _FREQ_HZ, _PERIOD_US, row['step'],
                  travel // 10, travel % 10, pulse_set - pulse_get, slew_ms,
                  self.moves, self.angle_min_seen, self.angle_max_seen))

    def banner(self) -> None:
        print('# servo tester -- servo=pin%d left=pin%d right=pin%d i2c=(sda%d,scl%d) at %dHz' % (
            _PIN_SERVO, _PIN_LEFT, _PIN_RIGHT, _PIN_SDA, _PIN_SCL, _FREQ_HZ))
        for row in _HUD_ROWS:
            print('# %s [%d..%d] step %d' % (row['text'].strip(), row['min'], row['max'], row['step']))


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
