# Recorder wire format: boot ids, integrity wrapper, session index

The board streams its logs and telemetry to the Recorder module (the Luckfox) over `uart_recorder`. This
spec defines what goes on that wire: how lines are routed, how each line proves its own integrity, how a
boot names its files, and how a boot is tied to wall-clock time. It exists because the 2026-10-03 dumps
showed the link corrupting ~10 % of rows, mangling file names into thousands of junk files, and naming
every session 2000-01-01, so flights had to be found by their boost rather than their date.

**The Luckfox daemon (`src/camera/recorded/recorderd.cpp`) does not change.** Everything here is
produced by the board and checked offline. The wire stays readable by the deployed `recorderd`:
the wrapper sits inside the payload it already writes verbatim.

## Routing (unchanged)

Every line is `[@<routing>@]<payload>\n`. The routing is optional:

| line | the Luckfox writes it to |
|---|---|
| with `@<routing>@` (telemetry) | the file named `<routing>`, payload only |
| without it (a log line) | `recorder.log` |

`<routing>` is `<session>_<stream>.csv` for a boot's streams, or a bare `session.csv` for the shared
session index (below).

## Integrity wrapper

Every line the board sends to the Luckfox carries two checks around its payload:

```
[@<routing>@]{OPEN};<payload>;<CLOSE>\n
```

- **OPEN** is the CRC-32 (IEEE 802.3, the `binascii.crc32` polynomial, initial value 0) of the raw
  bytes of `[<routing>@]<payload>`, exactly as sent (UTF-8 for any non-ASCII text). It covers the
  routing and its closing `@` when the line has a routing (even an empty one), and the payload alone
  when it does not. It is written as **8 lowercase hex digits**. Because it covers the routing, a row
  proves which file it belongs to.
- **CLOSE** is `(~OPEN XOR U) & 0xFFFFFFFF`, also 8 lowercase hex digits. `U` is the payload's leading
  unsigned decimal number: the `uptime` of a telemetry row, or the ticks of a log line. It is `0` when the
  payload does not start with a digit, as with a CSV header. CLOSE chains the two checks: it proves OPEN
  and the uptime, so a truncated line, a damaged tail and a damaged uptime all fail it. A header line,
  with `U = 0`, closes with plain `~OPEN`.

Example: the row `884029;0.98;0.05;0.01` for routing `000123_imu_lsm6dso32.csv`:

```
@000123_imu_lsm6dso32.csv@{OPEN};884029;0.98;0.05;0.01;<CLOSE>
   OPEN  = crc32(b'000123_imu_lsm6dso32.csv@884029;0.98;0.05;0.01')
   CLOSE = ~OPEN ^ 884029
```

### Verifying a line offline

A line is **good** only when both checks pass. In a Luckfox file the routing is the file's own name, so
the check for a row in file `F` is `crc32(F + '@' + payload) == OPEN`.

Lines that lost or corrupted their routing on the wire are **salvaged** offline, because every routing
of a boot is known: its `<session>_<stream>.csv` names, plus `session.csv`.
- **A corrupted routing** puts the row in a junk-named file. Its OPEN fails against that name; the right
  file is the known routing that makes OPEN match.
- **A lost routing**, such as a dropped leading `@`, sends the row to `recorder.log`, where it fails as a
  log line. It is then tried against every known routing, and against any `<name>@` left at its start;
  the one that makes OPEN match is its file.
- **A lost second `@`** (`@<name>{…`) leaves a routing the daemon cannot split. Every known routing is
  tried from the wrapper's `{` head, ignoring whatever precedes it.
- **Two merged lines** (a lost `\n` between them) fail as one. They are split at each seam, where a
  `>` is followed by `{` or `@`, and each piece is verified on its own.

A salvaged row has no trustworthy position in its file, so it is **placed by time**. Its uptime, unwrapped
next to the good rows around it, puts it among its stream's rows: never at the end, and never across a
`ticks_us` wrap. A row that matches no routing, or a time that can't be placed unambiguously, is dropped
and counted, never guessed.

Captures from before this spec carry no wrapper. They stay readable, and the parsers accept both
formats.

### Where the board adds it

The wrapper is added **at drain time, on the UART path only**.
- The PSRAM ring cells and the CC `log`/`tlm` tee keep the raw line, so the dashboard and the tests that
  read the rings see what they always have.
- The wrapper costs 22 bytes per line on the wire. The shorter boot-id prefix below saves ~16, so a
  row grows ~5 B, about +1.2 KB/s at 2026-10-03 rates on a ~92 KB/s link.
- **The GC is off from BOOSTING to DONE**, so the wrapper must not allocate per line. CRC-32 values
  above 2³⁰ would be boxed integers on this 32-bit port. The CRC and the hex digits are therefore
  produced by a `@micropython.viper` routine into a pre-allocated buffer, which is handed to the
  `StreamWriter` as a memoryview.

## Session prefix: the boot id

Each boot names its files `<session>_<stream>.csv`, with `<session>` taken from the first of:

1. **`recorder.session`** from the config, verbatim: set by the test system for a labelled run
   ([`board-config.md`](board-config.md)).
2. **The boot id** as `%06u`, for example `000123`. It is an unsigned counter in NVS (`coludo` / `boot`),
   incremented once per boot by `main.py` before the Recorder starts. It is unique per board for the
   life of its NVS, which survives deploys. **Unique, not consecutive:** every reset counts, including
   the ~80 a board test run causes. A key that is missing starts the count at 1; any other NVS error
   falls back to the legacy prefix rather than risk reusing an id.
3. **The legacy `YYYYMMDD_HHMMSS_<6-digit random>`**, when no boot id is available: test and HITL
   bring-ups that do not go through `main.py`.

The board name is not in the prefix: one recorder serves one board. A **label** must be letters, digits
and `-`, containing at least one letter and at most 32 characters, so that it can neither look like a
boot id nor be split ambiguously from a stream name at a `_`, and leaves every line room in its ring
cell. An invalid label is logged and ignored: the boot id names the files.

A reset in flight (a warm start) is a new boot, so the flight continues under a new boot id. Both
boots' `session.csv` anchor rows tie them to UTC, so they can be joined by time.

## Clock and the session index

The board RTC runs on **UTC**, as before; `mission.set_time(epoch)` takes Unix seconds. It reads
2000-01-01 at power-on.

**CC sets it on connect.** `whoami` reports the board's `epoch`, `boot_id` and `session`. When CC
registers a board whose `epoch` lies between 2000-01-01 and 2001-01-01 (a clock never set) and whose
stage is **SETTING**, it sends:

```
update mission {"epoch": <utc seconds>, "utc_offset": <CC's offset from UTC, minutes>,
                "cc_position": [lat, lon] | null, "source": "cc-auto"}
```

Boards that report no `epoch` (older firmware, test fakes) are left alone, and so is everything when CC's
own clock reads before 2020. DONE is excluded on purpose: a 26-year clock jump there makes the
post-landing warm-start crumb look stale. The dashboard's sync button sends the same fields with
`"source": "dashboard"`, from the browser's clock. CC's GPS position goes in `cc_position`, never in
`latitude`/`longitude`: those are the launch pad.

**Rows are appended to `session.csv`**, a single file shared by all boots, routed without a session
prefix. **Every boot is listed, whether or not anyone sets its clock:**

- **`boot`**, as soon as the Recorder is up;
- **on every successful time set**, with the `source` the setter gave (`cc-auto`, `dashboard`);
- **`anchor`**, once, when uptime passes 60 s. This covers what the earlier rows can miss:
  - a cold boot's `boot` row and first sync go out before the Luckfox is listening (it starts ~29 s
    after power-on);
  - a soft or watchdog reset, or a warm start, keeps the RTC and so never prompts a sync.

`utc` and `utc_offset` are **empty while the clock is unset** (before 2001). An offset outside
-720..+840 minutes is left empty too, and a text cell (`board`, `firmware`, `source`) that is not
printable ASCII without `;`, or is longer than 32 characters, is written empty. Such a boot is still
identified by its boot id, board, firmware and config, and since boot ids only grow on a board, it sits
between its dated neighbours. A row that cannot be queued (a full ring) is logged and lost, and a time
set still counts.

```
uptime;boot;session;utc;utc_offset;board;firmware;config_id;source;cc_lat;cc_lon
3104552;123;000123;;;taster;2026.09.23.55c494a0c659;3f2a91;boot;;
17884029;123;000123;2026-10-03T14:21:07Z;-240;taster;2026.09.23.55c494a0c659;3f2a91;cc-auto;25.5144;-80.3918
60000117;123;000123;2026-10-03T14:21:49Z;;taster;2026.09.23.55c494a0c659;3f2a91;anchor;;
```

The header goes out before a boot's `boot` row, and again before its `anchor` row, since the first may
have been sent before anyone was listening. The rows carry the wrapper too.
**A boot may have several rows**: its `boot` and `anchor` rows plus one per time set (a first sync on
connect, then a later re-sync from the dashboard or a reconnect). Analysis combines them per boot.
Only rows with a `utc` date the boot. Each such row pairs an `uptime` with a `utc`,
so every uptime of that boot converts to wall-clock time as `utc + (uptime_row - uptime_set) / 1e6`,
unwrapping `ticks_us` every 2³⁰ µs (17.9 min). Use the nearest set, preferring the latest one before the
uptime, since a later set corrects an earlier one.

## The Luckfox

- **`recorderd` is unchanged** and writes the wrapped payloads verbatim.
- `/userdata` is mounted with `commit=1` (`src/camera/opt/recorder/recorderd.sh`), so the kernel commits
  writes every second.
- The camera stays **off** while the recorder runs. In a closed bay it overheats within ~10 min (the
  video distorts), and video plus logging is more than the module handles.
