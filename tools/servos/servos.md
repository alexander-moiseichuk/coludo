# Servos — a handheld servo bench tester

A small standalone MicroPython gadget (its own ESP32-C3 + OLED, **not** the glider board) for
exercising a standard hobby servo (SG90 and friends) by hand: dial a parameter up/down with two
buttons, watch the live read-back on the OLED, and capture the full state over the USB console.

It is deliberately self-contained — it does **not** import the glider firmware (no `task`/`databoard`
coupling) — so it can be flashed onto the C3 on its own.

## What it does

Three views of the **same** physical PWM state, each editable:

- **angle** — a logical dial `[0, 360]°`, mapped linearly onto the pulse range (so it is not
  necessarily mechanical degrees on a 180° servo; it is "where in the travel"), step **5°**
- **pulse** — the servo pulse width in microseconds, `[500, 2500] µs`, step **25 µs**
- **duty**  — the raw 16-bit PWM duty (`duty_u16`) actually written to the timer, step **100**

Editing any one recomputes and applies the other two, so the three rows always agree. Each row shows
**set vs get**: `set` is what we commanded, `get` is read back from the PWM peripheral
(`PWM.duty_u16()`) — confirming the write landed and exposing the timer's quantisation.

### Controls

- **left button (`-`)** — decrement the selected parameter by its step (clamped at the minimum)
- **right button (`+`)** — increment the selected parameter by its step (clamped at the maximum)
- **both buttons together** — switch the selection: `angle → pulse → duty → angle`
- **hold** a single button — auto-repeat after a short delay (sweep the value)

A `>` cursor on the OLED marks the selected row; its big read-out is drawn at 2× for a glance-able
value. On boot the servo is driven to angle 0 and every parameter is printed.

### OLED layout (72×40 visible window)

```
  090          <- selected parameter value, 2x font
>ang  090      <- cursor + the three rows (set values), zero-padded
 pul 1500
 dut 4915
```

## Console reporting

Everything that does not fit the tiny screen goes to the USB serial console (115200, the REPL port) —
one `key=value` line per event (button press, switch, boot), easy to read or pipe/parse. See
`servo_test.py` `report()` for the field list; it includes set **and** get for angle/pulse/duty, the
duty percentage, PWM frequency/period, the active step, travel percentage, the set-vs-get
quantisation error in µs, an SG90 slew-time estimate for the last move, a move counter, and the
min/max angle touched this session. A `# ` banner with the pin map and ranges is printed at startup.

## Hardware

### OLED — 0.42" SSD1306
- board: an ESP32-C3 with an onboard 0.42" OLED
  ([example](https://www.amazon.com/dp/B0F37JWCDW))
- quirk: the panel is a 72×40 window inside the SSD1306's 128×64 buffer, so everything is drawn at an
  offset of `x = (128-72)//2 = 28`, `y = (64-40)//2 = 12` (`demo.py` uses 30/12 — either centres
  acceptably; the tool uses the computed values)
- I2C: `I2C(0, sda=Pin(5), scl=Pin(6), freq=400000)`
- driver: `ssd1306.py` (bundled); `demo.py` is the original bouncing-ball example

### Buttons
- two capacitive buttons: **pin 0 = left (`-`)**, **pin 1 = right (`+`)** (active-high; flip
  `_BUTTON_ACTIVE` if your module is active-low)

### Servo
- signal on **pin 2**, driven at 50 Hz (`PWM(Pin(2), freq=50)`)

## Running

Copy `ssd1306.py` and `servo_test.py` to the C3 and run `servo_test.py` (or save it as `main.py` to
auto-start as a kiosk on boot):

```
mpremote connect /dev/ttyACM1 cp ssd1306.py : + cp servo_test.py : + run servo_test.py
```
