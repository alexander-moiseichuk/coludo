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

The OLED is tiny (it only shows the live `get_*` values), so the **USB serial console** (115200 over
`/dev/ttyACMx`, the REPL port) is the detailed tracking channel — **always on**, three streams:

- a `# ` **banner** at startup: the pin map, and for each parameter its range/step and its `api()`
  (what it maps to in the platform PWM API — see s5);
- a **`BTN`** line per button event: the event, the raw `L`/`R` pin levels (confirms wiring/polarity)
  and the running +/-/switch counts — a hand-pressed sequence is fully verifiable from the console;
- a **`t=` `report()`** line per event: set **and** get for angle/pulse/duty, the duty percentage, PWM
  frequency/period, the active step, travel percentage, the set-vs-get quantisation error in µs, an
  SG90 slew-time estimate for the last move, a move counter, and the min/max angle touched this session.

Watch it live with `mpremote connect /dev/ttyACM1 run main.py` (or attach any serial monitor at 115200).

## Hardware

### OLED — 0.42" SSD1306
- board: an ESP32-C3 with an onboard 0.42" OLED
  ([example](https://www.amazon.com/dp/B0F37JWCDW))
- quirk: the panel is a 72×40 window inside the SSD1306's 128×64 buffer, so everything is drawn at an
  offset of `x = (128-72)//2 = 28`, `y = (64-40)//2 = 12` (`demo.py` uses 30/12 — either centres
  acceptably; the tool uses the computed values)
- I2C: `I2C(0, sda=Pin(5), scl=Pin(6), freq=400000)`
- driver: `ssd1306.py` (bundled, stock MicroPython SSD1306 framebuffer driver)

**Note:** the original code of ssd1306.py available under MIT license https://github.com/PerfecXX/MicroPython-SSD1306/
**Copyright (c) 2024 Teeraphat Kullanankanjana**

### Buttons
- two capacitive buttons: **pin 0 = left (`-`)**, **pin 1 = right (`+`)** (active-high; flip
  `_BUTTON_ACTIVE` if your module is active-low)

### Servo
- signal on **pin 3** (GPIO3), driven at 50 Hz (`PWM(Pin(3), freq=50)`) — pin 3 is a safe, free PWM
  pin per the [ESP32-C3 super-mini pinout](https://lastminuteengineers.com/esp32-c3-super-mini-pinout-reference/#esp32c3-super-mini-uart-pins)

## Running

The tool is `main.py`, so once copied the C3 auto-starts it on power-up. Copy it and the driver, then
reset (or `run` it to stream the console while iterating):

```
mpremote connect /dev/ttyACM1 cp ssd1306.py : + cp main.py : + reset
mpremote connect /dev/ttyACM1 run main.py      # stream the console (Ctrl-C to stop)
```
