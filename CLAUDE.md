# Coludo — working notes for Claude

Rocket-powered autonomous glider. Firmware in `src/glider/` (**MicroPython** on a WaveShare
ESP32-P4-WIFI6); ground station "Control" in `src/control/` (**CPython 3.12**). Architecture:
`specs/` (`coludo.md`, `board-config.md`, `cc-protocol.md`). Status + roadmap: `doc/plan.md`.
Hardware/dev guide + the full conventions: `doc/skills.md`.

## Read before coding: `doc/skills.md` → "Coding conventions". In short:

- Single-quote strings; **no abbreviations**; `_`-prefixed module internals (PEP8).
- **Type-annotate** all non-locals (constants, class vars, args, returns).
- `micropython.const` for constants (with the CPython shim).
- Docstring/comment per entry; **slim classes, YAGNI** (no unused params/flexibility).
- **Async I/O**: blocking peripherals via `asyncio.StreamWriter`/`drain()`; loops run forever
  (no `stop` flags — a wedged board reboots).
- **Error policy**: logs best-effort (drop); **telemetry raises** on overflow.
- **Inspectable** mixin (`inspect()`/`update()`/`stats()` + `type`/`name`) for operator-facing objects.
- Tests cover **positive and negative**, on the board.

## Workflow

- `src/glider/` is MicroPython-only — verify on the board: `cd src/glider/test && make test`.
- `cc_protocol.py` is **shared** with Control (symlinked); keep it CPython+MicroPython portable.
- Lint + compile + push via `./deploy.sh` (ruff + `mpy-cross`). Never commit secrets
  (`src/glider/ssid.creds` is gitignored).
- Commit only when asked; this repo commits to `main`.
