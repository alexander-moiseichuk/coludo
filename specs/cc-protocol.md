# Control Center ↔ Board Protocol

This document specifies the wire protocol between the **Control Center (CC)** and the
**Main Controller** boards, and how the browser UI is bridged onto it. It is authoritative
for connection direction, framing, and the command set. It complements
[`board-config.md`](board-config.md) (which owns the config schema and activation lifecycle)
and is only relevant in **prestart mode** — from ignition onward the board is autonomous and
no CC link is expected.

## Topology & roles

- **Boards are TCP clients.** On boot a board dials out to CC (host/port from the `wifi`
  section of its `board.json`). A board hosts no server and knows only CC's address.
- **CC is the single server / hub.** One CC instance accepts every board connection, keeps a
  registry of who is online, and brokers traffic between boards and operators.
- **Browser** talks HTTP + SSE to CC; **telnet/dev** talks the raw line protocol to CC. CC
  adapts both onto the same internal command set.

```
   browser ──HTTP + SSE──┐
                         ├──►  CC (single hub) ──TCP──►  board glider1
   telnet  ──raw TCP─────┘                       └─TCP──►  board glider7a
```

## CC drives everything (poll model)

**CC initiates every exchange; boards only ever answer.** There are no unsolicited messages
from a board — no async push, no subscriptions, no out-of-band events. The consequences:

- **Timing is controlled by CC.** It decides when to poll health, telemetry, or logs, and it
  never overlaps requests (see [Heartbeat](#heartbeat--liveness)).
- **One outstanding request per board (lockstep).** CC sends a command and waits for the
  single response before sending the next to that board. No request IDs are needed to match
  responses. (Different boards are polled concurrently; only per-board is serial.)
- **Data is pulled, not pushed.** The board maintains bounded ring buffers (in PSRAM) for log
  and telemetry records. CC retrieves slices on demand (`log`, `tel`). The same buffers feed
  the Recorder over UART, so there is one producer and two drains.
- **"Events" are just log lines.** Since nothing is pushed, notable occurrences (validation
  failures, fallbacks) surface either as the `err` response to the command that caused them,
  or in the log buffer that CC polls.

## Transport & ports

Plain TCP, no encryption (trusted LAN; encryption is explicitly out of scope for now).

| Port | Peer | Protocol |
|------|------|----------|
| 1234 | boards | line protocol — boards dial in as clients |
| 1235 | operator / dev | line protocol + operator sugar (`list`, `select`) — telnet-friendly |
| 8080 | browser UI | HTTP for the page and commands, **SSE** for live streams |

## Framing

**Newline-delimited messages. Each line is one message.** All tokens are **lowercase**.
The general form is:

```
<command> <board-id> [params...]
```

- The **first word** is the command.
- The **second word** is the target **board id**, taken verbatim from the board's config
  (`board.id`, e.g. `glider1`, `glider7a`).
- The **rest** is parameters, **space-separated**, in an old-school shell style:
  - **positional** — a bare token: `log glider1 3000`
  - **named** — `key=value`: `tel glider1 ms=3000`
  - **quoted** — wrap a value in double quotes when it contains spaces:
    `note glider1 msg="pad 7, gusty"` (use `\"` for a literal quote inside).

So an operator can type any command by hand over telnet. Parsing is a simple tokenizer:
split on spaces (respecting quotes), then a token with `=` is a named param, otherwise
positional.

**JSON is used only when the parameter itself is a structured document** — currently just the
full config in `save-config <json>`, which CC or the browser sends and no operator hand-types.
Everything else a human types is positional or `key=value`. (Response payloads are also JSON,
but those are machine-generated and only read, never typed — see
[Responses](#responses--error-codes).)

Responses use the same framing, with a status word first:

```
<status> <board-id> [payload]
```

where `status` is `ok`, `err`, `pong`, or `iam`. Any structured payload is compact JSON on the
same line; multi-record results (e.g. several log lines) are returned as a **JSON array on one
line**, never as multiple physical lines. This keeps the invariant: exactly one message per
`\n`. Plain text inside JSON is escaped normally.

Two exceptions to "`command board-id ...`":
- **`whoami`** carries no board id — at that moment CC has not yet learned the id (see below).
- **Operator/CC commands** (`list`, `select`) are addressed to the hub, answered as `ok cc …`.

## Connection & identification

Because CC drives everything, a freshly accepted connection is identified by CC prompting it:

```
1. Board boots, dials CC:1234.
2. CC accepts and sends:        whoami
3. Board answers:               iam glider1 {"mcu":"esp32p4","fw":"0.1","config_id":"<hash>","state":"setting","uptime":812}
4. CC registers socket ⇄ glider1 and begins its 2 s poll loop.
```

`config_id` is a hash/version of the running `board.json`, so CC can tell whether its cached
view of the board's config is current. If a board reconnects with an id already registered, CC
drops the older socket and keeps the newest (a board only re-dials after a reboot or link loss).

At **ignition** the Wi-Fi link drops; the TCP connection breaks and CC marks the board
**offline**. This is expected — the board is now autonomous. The board may keep retrying to
connect; CC simply shows it offline until/unless it returns.

## Heartbeat / liveness

CC polls each online board roughly every **2 seconds**. The heartbeat is just a normal
command (`ping`, or a `health` poll that also returns vitals). Because CC owns all timing:

- If any command/response for a board completed within the heartbeat window, CC **skips** the
  redundant ping — a successful exchange already proves liveness.
- CC never sends a heartbeat while another request to that board is outstanding (lockstep); it
  waits or skips.
- After ~5 s of silence (a couple of missed beats), CC marks the board **offline** and
  surfaces it to operators. The board is never auto-reconfigured — go/no-go stays with the
  operator (the strict model from `board-config.md`).

## Command catalog (CC → board)

All board commands take the board id as their second token.

| Command | Params | Response | Meaning |
|---------|--------|----------|---------|
| `whoami` | *(none — connection-level)* | `iam <id> {json}` | identify a new connection |
| `ping` | — | `pong <id>` | liveness |
| `health` | — | `ok <id> {temp,mem_free,load,uptime,links,components[]}` | vitals; `components[]` carries `{name, ok}` presence |
| `state` | — | `ok <id> {state,uptime}` | current flight phase |
| `tel` | `[ms]` | `ok <id> {samples:[...]}` | latest telemetry sample, or all within the last `ms` |
| `log` | `<ms>` | `ok <id> {lines:[...], truncated}` | log lines from the last `ms` (relative window) |
| `report` | `[task]` | `ok <id> {...}` | `task.report()` dump; all tasks if none named |
| `get-config` | `[running\|default]` | `ok <id> {config}` | fetch a config (`running` if omitted) |
| `save-config` | `<json>` | `ok <id> {config_id}` / `err <id> invalid <msg>` | validate + persist full snapshot to `board.json`; **running config is unchanged** |
| `reset-config` | — | `ok <id>` | delete `board.json`; next boot uses `config_default.py` |
| `reboot` | — | `ok <id>` then disconnect | ack, then hard reset → boots from saved config |

### Log / telemetry retrieval

`log glider1 5000` means "the log records from the **last 5000 ms**." The board keeps a bounded
ring buffer; if the requested window is older than the buffer holds, it returns what it has and
sets `"truncated": true`. For continuous tailing, CC polls with a window at least as wide as
its poll interval and de-duplicates by record uptime (each record carries its uptime, per the
`coludo.md` logging format). `tel` behaves the same way for telemetry samples.

### Config commands map to the activation model

`save-config` and `reboot` are deliberately **separate** commands, matching the
save/reboot-separated model in [`board-config.md`](board-config.md). The editable *draft*
config lives on **CC** (it holds the UI/operator state, seeded by `get-config`); only the full
snapshot is pushed via `save-config`. The board firmware therefore needs no per-field config
mutation handlers — just "validate-and-persist a whole config," "delete config," and "reboot."
A failed validation comes straight back as `err <id> invalid <msg>` and the board keeps running
its previous config.

## Responses & error codes

| Status | Form | Meaning |
|--------|------|---------|
| `ok` | `ok <id> [json]` | success, optional result payload |
| `pong` | `pong <id>` | reply to `ping` |
| `iam` | `iam <id> {json}` | reply to `whoami` |
| `err` | `err <id> <code> <msg>` | failure |

Error `code`s (short, lowercase): `badcmd` (unknown command), `badboard` (id does not match
this board / unknown board at CC), `badargs` (malformed params), `invalid` (config failed
validation), `busy` (a request is already in flight), `unsupported` (capability absent on this
board), `internal` (unexpected fault).

## Operator / telnet sugar (operator ↔ CC only)

These commands are handled by CC and never forwarded verbatim to a board.

| Command | Meaning |
|---------|---------|
| `help` | `ok cc {commands:[...]}` — list all commands; `help <command>` returns usage for one |
| `list` | `ok cc [{id, online, state, config_id}]` — connected boards |
| `select <board-id>` | set this operator session's **sticky** target; `ok cc {selected}` |
| `who` | `ok cc {selected, since}` — current selection |

`help` is answered by CC from its command catalog, so usage stays discoverable over a bare
telnet session (`help log` → the `log` syntax and params).

**Sticky select:** after `select glider1`, the operator may omit the board id and CC injects
the selected one — `health` becomes `health glider1`. An explicit id always overrides the
selection for that one line. CC tags everything it relays back from a board with the source id
(`ok glider1 …`), so a telnet operator always sees which board answered.

Example telnet session:

```
> help log
ok cc {"usage":"log <board-id> <ms>","desc":"log records from the last <ms> milliseconds"}
> list
ok cc [{"id":"glider1","online":true,"state":"setting","config_id":"a1b2"},
       {"id":"glider7a","online":true,"state":"setting","config_id":"c3d4"}]
> select glider1
ok cc {"selected":"glider1"}
> health
ok glider1 {"temp":54,"mem_free":812000,"load":31,"uptime":90422,
            "components":[{"name":"gnss","ok":false},{"name":"baro_icp10111","ok":true}]}
> tel ms=3000
ok glider1 {"samples":[{"t":90100,"alt":0.2,"yaw":1.1},{"t":90200,"alt":0.2,"yaw":1.0}]}
> log 3000
ok glider1 {"lines":["90011 Controller :: gnss timeout","90250 Fusion :: altitude -> bmp280"],"truncated":false}
> reboot glider1
ok glider1
```

## Browser bridge (HTTP + SSE)

CC exposes the same capabilities to the browser without the browser ever speaking raw TCP:

- **`GET /`** — the UI page (static assets).
- **`GET /api/boards`** — JSON list (same data as `list`).
- **`POST /api/cmd`** — body `{board, command, params}`; CC runs the command against the board
  (respecting per-board lockstep) and returns the response as JSON. Used for one-off actions
  (`save-config`, `reboot`, `get-config`, …).
- **`GET /events`** (optionally `?board=glider1`) — a **Server-Sent Events** stream. CC's
  per-board poll loop publishes `health`, `tel`, and `log` results to subscribed browsers as
  SSE events. SSE is chosen over WebSocket because the live need is server→browser streaming,
  it is plain HTTP (no extra dependency), and browser→board actions are ordinary POSTs.

The browser is thus a thin view over CC's polling: CC is the only component that polls boards,
and it relays results both to telnet operators and to SSE subscribers.

## Python implementation notes (non-binding)

Targeted at stdlib-only `asyncio`:

- `asyncio.start_server` for the two TCP listeners (1234 boards, 1235 operators).
- One coroutine per board connection drives the poll loop and serializes commands (lockstep).
- A small in-memory registry maps board id ⇄ connection, last-seen time, and cached
  `config_id`/health.
- HTTP + SSE on 8080 via `asyncio`/stdlib `http.server`; SSE is just a long-lived
  `text/event-stream` response fed from the registry's update fan-out.
