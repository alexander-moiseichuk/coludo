# Board Configuration

This document is the **authoritative reference** for how a Coludo Main Controller is
configured, persisted, changed, and re-activated. Where `coludo.md` describes hardware,
Wi-Fi, or storage behaviour that conflicts with this document, **this document wins**.

## Why a config file exists

The controller hardware is deliberately variable:

- Different MCUs: WaveShare ESP32-P4-WIFI, FireBeetle 2 ESP32-P4 (Wi-Fi not yet working),
  or an ESP32-C6 low-end variant.
- Redundant sensors may be present for fusion (e.g. BNO055 **and** ADXL375, plus GNSS).
- Pin assignments differ per board revision.

So the firmware must not hardcode the hardware set. A single configuration file describes
*what this board is*, and the controller reads it at boot to decide which tasks and
component drivers to instantiate.

## Design principles

1. **Immutable for the duration of a run.** There is no live-edited running state that can
   drift from disk. To change anything, the operator saves a new config and re-activates the
   board. The board always boots from the saved config, so *running options always equal
   saved options*. "Restart from the same conditions" is therefore a property of the design,
   not a feature bolted on.
2. **Operator authority (strict model).** The board runs exactly what the operator
   configured. It never second-guesses the operator by silently enabling or disabling
   components. If reality does not match the config (a sensor is unplugged), that is surfaced
   as a health signal — see [Validation vs Health](#validation-vs-health) — and a human makes
   the go/no-go decision.
3. **Never brick, never boot-loop.** A corrupt or invalid saved config falls back to the
   firmware defaults in a flagged, degraded state and reports it to the Control Center (CC).
   The board is always reachable.

## The three layers

```
config_default.py   baked into firmware — human-edited, the safe fallback / floor
        │  loaded first
        ▼
board.json          saved active config — JSON, a FULL snapshot of what the board runs
        │  replaces defaults, then VALIDATED
        ▼
in-memory objects   tasks + component drivers built from the validated active config
```

- **`config_default.py`** — a Python module shipped with the firmware. Human-authored, so it
  may use comments, hex addresses (`0x28`), and per-MCU variants. It is the fallback used
  when no valid `board.json` exists.
- **`board.json`** — the active config the board actually runs. It is a **full snapshot**, not
  a delta against the defaults. A snapshot reproduces the exact same conditions even if the
  firmware defaults later change; a delta would silently drift.
- **in-memory objects** — never persisted, never the source of truth. They are rebuilt from
  the active config on every activation.

**Power-on and re-activation use the same code path:** load defaults → if a valid `board.json`
exists, use it instead → validate → build tasks. One activation path to write, one to test,
identical behaviour every time.

## Schema

`board.json` (and the dict produced by `config_default.py`) has these top-level sections.

```json
{
  "board": { "id": "glider-01", "mcu": "esp32p4", "rev": 1 },

  "wifi": {
    "mode": "sta",
    "ssid": "coludo-cc",
    "password": "...",
    "cc_host": "192.168.10.1",
    "cc_port": 1234,
    "tx_power_dbm": 11
  },

  "buses": {
    "i2c0":          { "sda": 21, "scl": 22, "freq": 400000 },
    "uart_recorder": { "tx": 17, "rx": 18, "baud": 921600 },
    "uart_console":  { "tx": 43, "rx": 44, "baud": 115200 }
  },

  "pins": {
    "led_status": 2,
    "separation_switch": 4,
    "servo_yaw": 5,
    "servo_elevon_l": 6,
    "servo_elevon_r": 7
  },

  "components": [
    { "name": "accel_adxl375", "driver": "adxl375", "bus": "i2c0", "addr": 83,
      "enabled": true,
      "provides": { "accel": { "priority": 0, "timeout_ms": 5 } } },

    { "name": "imu_bno055", "driver": "bno055", "bus": "i2c0", "addr": 40,
      "enabled": true,
      "provides": { "attitude": { "priority": 0, "timeout_ms": 5 },
                    "accel":    { "priority": 1, "timeout_ms": 5 } } },

    { "name": "baro_icp10111", "driver": "icp10111", "bus": "i2c0", "addr": 99,
      "enabled": true,
      "provides": { "altitude": { "priority": 0, "timeout_ms": 100 } } },

    { "name": "baro_bmp280", "driver": "bmp280", "bus": "i2c0", "addr": 118,
      "enabled": true,
      "provides": { "altitude": { "priority": 1, "timeout_ms": 200 } } },

    { "name": "gnss", "driver": "atgm336h", "bus": "uart_console", "addr": null,
      "enabled": true,
      "provides": { "position": { "priority": 0, "timeout_ms": 150 },
                    "altitude": { "priority": 3, "timeout_ms": 1000 } } },

    { "name": "laser_agl", "driver": "sen0648", "bus": "i2c0", "addr": 80,
      "enabled": true,
      "provides": { "agl": { "priority": 0, "timeout_ms": 20 } } },

    { "name": "recorder", "driver": "uart_sink", "bus": "uart_recorder", "addr": null,
      "enabled": true }
  ]
}
```

### Section reference

- **`board`** — identity and MCU type. `mcu` is one of `esp32p4`, `esp32c6`, `firebeetle2p4`
  and lets the firmware select MCU-specific behaviour.
- **`wifi`** — the board is a **station (STA)** that joins the network hosted by **CC**.
  (The earlier "glider hosts an AP" idea in `coludo.md` is superseded.) `cc_host`/`cc_port`
  point at the CC service; `tx_power_dbm` is the operator-tunable signal level.
- **`buses`** — named buses (I²C / UART / SPI) with their pins and parameters. Components
  reference a bus by name.
- **`pins`** — discrete signals (LED, separation switch, servo PWM lines).
- **`components`** — the declarative hardware list. Each entry has a `name`, a `driver`, a
  `bus` reference, an optional `addr`, an `enabled` flag, and a `provides` map. JSON has no
  hex literals, so addresses are decimal (`0x28` → `40`); `config_default.py` may use hex.

### Fusion is derived, not duplicated

Each component declares what measured quantities it `provides`, each with a `priority`
(lower = preferred) and a `timeout_ms`. The fusion layer groups all enabled components by
quantity and orders them by priority, e.g. from the example above:

- `altitude` → `[icp10111 (100ms), bmp280 (200ms), gnss (1000ms)]`
- `accel`    → `[adxl375 (5ms), bno055 (5ms)]`
- `position` → `[gnss (150ms)]`
- `agl`      → `[laser_agl (20ms)]`

Adding or disabling a sensor updates fusion automatically. There is no separate fusion table
to keep in sync.

### What is NOT in board config

**Mission config** (landing-zone coordinates, altitude thresholds, target point) is *per
launch*, not *per board*, and lives elsewhere. Board config describes the vehicle's hardware;
mission config describes a specific flight.

## Lifecycle and activation

All configuration changes happen in **prestart mode only**. There are **no config changes
during flight** — from ignition onward the board is autonomous and the config is frozen.

**Save and reboot are two separate operator actions:**

```
1. Operator edits settings in the CC browser UI (enable/disable components, tx power, ...)
2. Operator requests SAVE
       → board validates the resulting config
            ├─ invalid → reject, keep running current config, report error to CC
            └─ valid   → atomically write board.json (temp file + rename)
       → board KEEPS RUNNING the previous config (now "pending reactivation":
         saved config differs from running config; CC shows this state)
3. Operator requests REBOOT (when ready)
       → hard reset (machine.reset())
       → board boots from the saved board.json via the normal activation path
       → reconnects to CC; saved == running again
```

Decoupling save from reboot lets the operator batch several edits, save once, and pick the
moment for the disruptive reboot. Re-activation is a **hard reset** specifically because it is
the *same* path as power-on — there is exactly one activation path to trust, with no risk of
half-deinitialized drivers or lingering peripheral state. The cost is a few seconds of Wi-Fi
reconnect, which is negligible inside the ~15-minute pad window.

## Validation vs Health

These are two different checks and must not be confused.

- **Validation** is about the **integrity of the config file itself**, checked *before* a save
  is persisted and *again* at boot:
  - every `pins`/`buses` pin number is unique (no pin used twice),
  - every component's `bus` reference names a bus that exists,
  - required fields are present and well-typed.
  An invalid config is **never written** (save is rejected) and **never booted** (boot falls
  back to `config_default.py`, flagged degraded, reported to CC). This makes it impossible for
  CC to brick a board with a bad config.

- **Health** is about whether the **physical hardware matches the config** at runtime:
  an enabled component that does not respond on its bus, high temperature, low memory, stalled
  data flow. Health is **reported to CC**; the board does **not** auto-disable anything the
  operator enabled. If a sensor the operator declared is disconnected, that is a **go/no-go
  decision for the operator** — typically "cancel launch on pad XYZ" — not an automatic
  reconfiguration. This is the strict / operator-authority principle in action.

## Storage note

The Main Controller has **no SD card**. Logs and telemetry are streamed over the
`uart_recorder` link to the Recorder module (Luckfox Pico), which owns the only SD storage and
also records video. See [the recorder module](../src/camera/README.md). Any path beginning
`/sd/...` in older parts of `coludo.md` refers to behaviour that has moved to the Recorder.
