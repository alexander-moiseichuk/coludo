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

**Newline-delimited messages, one per line, all lowercase tokens.** The operator / Control form is
**target first**:

```
<board> <command> [params...]
```

- The **first token is the target**: a board id (from `board.id`, e.g. `glider1`, always a bare
  whitespace-free token) or `all` / `*` to broadcast.
- **Control routes by that token and strips it**, forwarding `<command> [params...]` to the matched
  board socket(s). **A board receives `<command> [params...]` with no id** and never deals with
  routing — board-id handling lives entirely in Control. (`all`/`*` is a Control-side fan-out to
  every connected board; boards never see it.)
- The **rest** is parameters, whitespace-separated, with **no quoting or escaping** — both sides
  know each command's schema. A value is a **bare token** (no spaces) or **`base64:<data>`** (spaces,
  quotes, JSON, binary). **named** params are `key=value`; everything else is positional. Parsing
  is a trivial `line.split()`. JSON has no special case — it rides as a `base64:` value (e.g.
  `glider1 save-config base64:<encoded-json>`).

**Responses** use the same framing minus the routing token: a board replies `<status> [params...]`
(`status` = `ok` / `err` / `pong` / `iam`), any structured payload being one `base64:`-encoded JSON
token. Control tags the reply back to the operator as `from <board> <status> [params...]`. The only
id a board ever emits is in **`iam`** (so Control can learn it on a new socket — see below).

## Connection & identification

On a fresh socket Control has not yet learned the board id, so it sends `whoami` **directly** (no
routing token); the board replies with the one message that carries its own id:

```
Control → board:   whoami
board → Control:   iam glider1 base64:<{"mcu":"esp32p4","fw":"0.1","config_id":"<hash>","state":"setting","uptime":812}>
```

Control registers socket ⇄ `glider1`, begins its ~2 s poll loop, and thereafter routes operator
traffic to it by id. `config_id` is a hash/version of the running `board.json`, so Control can tell
whether its cached view of the board's config is current. If a board reconnects with an id already registered, CC
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

## Command catalog (Control → board)

These are the **board-facing** commands — Control has already stripped the routing id, so neither
the command nor its response carries one. Structured payloads are `base64:`-encoded JSON (shown
here decoded). `whoami` is the connection-level exception that returns the id.

| Command | Params | Response | Meaning |
|---------|--------|----------|---------|
| `whoami` | — | `iam <id> {json}` | identify a new socket (the one reply carrying the id) |
| `ping` | — | `pong` | liveness |
| `health` | — | `ok {temp,mem_free,load,uptime,components[]}` | vitals; `components[]` carries `{name, ok}` |
| `state` | — | `ok {state,uptime}` | current flight phase |
| `tel` | `[ms]` | `ok {samples:[...]}` | telemetry samples within the last `ms` |
| `log` | `<ms>` | `ok {lines:[...], truncated}` | log lines from the last `ms` |
| `report` | — | `ok {state, tasks:{...}}` | the Controller's aggregated task status (`controller.stats()`) |
| `objects` | — | `ok [name, ...]` | names of all `Inspectable` objects (for the `inspect`/`update`/`stats` targets) |
| `inspect` | `<object>` | `ok {props}` | `Inspectable.inspect()` of a named object |
| `update` | `<object> <json>` | `ok {changed:[...]}` | `Inspectable.update()` — names of properties actually changed |
| `stats` | `<object>` | `ok {stats}` | `Inspectable.stats()` of a named object |
| `get-config` | `[running\|default]` | `ok {config}` | fetch a config (`running` if omitted) |
| `save-config` | `<json>` | `ok {config_id}` / `err invalid <msg>` | validate + persist full snapshot; **running config unchanged** |
| `reset-config` | — | `ok` | delete `board.json`; next boot uses `config_default.py` |
| `save-mission` | — | `ok` / `err unsupported` | persist the live mission (set via `update mission`) to `launch.config` |
| `reboot` | — | `ok` then disconnect | ack, then hard reset → boots from saved config |

`inspect`/`update`/`stats` address an object by name (`inspect wifi`, `update servo_yaw <json>`);
the board resolves it from the registry of `Inspectable`s. `update` applies only supported,
changed properties and returns their names — saving/rebooting stays an explicit operator step.

### The mission object (launch identity + clock)

The per-launch identity — **launch id**, launch-**site** name, launch **position**
(`latitude`/`longitude`/`altitude`), and the board **clock** — is the `mission` Inspectable, so it
rides the generic `inspect`/`update` surface rather than bespoke commands:

```
> inspect mission
from glider1 ok {"launch_id":"hprc-t1","site":"pad-a","latitude":45.5,"longitude":-73.5,
                 "altitude":120,"clock":"2026-06-17T14:32:05","epoch":1781000000}
> update mission base64:{"epoch":1781000000}    (time sync — Unix seconds, sets the board RTC)
from glider1 ok {"changed":["epoch"]}
> update mission base64:{"launch_id":"hprc-t2","latitude":45.51}
from glider1 ok {"changed":["launch_id","latitude"]}
```

`epoch` is a momentary action (it sets the RTC, never stored); `inspect` reports the live clock as
both an ISO string and a Unix `epoch` for CC to compare against its own. A broadcast
`all update mission base64:{"epoch":...}` time-syncs the whole fleet. Unlike the board config
(whose draft lives on CC), the mission is small and edited live on the board, so **`save-mission`**
persists the current values to `launch.config` — the per-launch counterpart to `board.json` — to
survive a pre-flight reboot. `err unsupported` means the board has no mission object.

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
A failed validation comes straight back as `err invalid <msg>` and the board keeps running its
previous config.

## Responses & error codes

A board reply carries no id (Control re-tags per socket); only `iam` carries one.

| Status | Form | Meaning |
|--------|------|---------|
| `ok` | `ok [base64:json]` | success, optional result payload |
| `pong` | `pong` | reply to `ping` |
| `iam` | `iam <id> base64:json` | reply to `whoami` (carries the id) |
| `err` | `err <code> <msg>` | failure |

Error `code`s (short, lowercase): `badcmd` (unknown command), `badargs` (malformed params),
`invalid` (config failed validation), `busy` (a request is already in flight), `unsupported`
(capability absent on this board), `internal` (unexpected fault). Routing failures (unknown
target board) are Control's concern, returned to the operator as `from cc err noboard <id>`.

## Operator commands (operator ↔ Control only)

A first token that is a known board id (or `all`/`*`) routes to a board; otherwise it is a
**Control command**, handled by Control and never sent to a board:

| Command | Meaning |
|---------|---------|
| `help` | `from cc ok {commands:[...]}` — all commands; `help <command>` for one |
| `list` | `from cc ok [{id, online, state, config_id}]` — connected boards |
| `select <board>` | set this session's **sticky** target; afterwards a bare `<command>` is routed to it |
| `who` | `from cc ok {selected, since}` — current selection |

**Sticky select / broadcast:** after `select glider1`, typing `health` is routed as `glider1
health`; an explicit `<board>`/`all`/`*` first token overrides it for that line. Control tags every
relayed reply with its source (`from glider1 ok …`), so the operator always sees who answered.
`all`/`*` fans out to every connected board and yields one tagged reply per board.

Example telnet session (response JSON shown **decoded for readability**; on the wire each is one
`base64:` token):

```
> list
from cc ok [{"id":"glider1","online":true,"state":"setting","config_id":"a1b2"},
            {"id":"glider7a","online":true,"state":"setting","config_id":"c3d4"}]
> select glider1
from cc ok {"selected":"glider1"}
> health                         (routed as: glider1 health)
from glider1 ok {"temp":54,"mem_free":812000,"load":31,"uptime":90422,
                 "components":[{"name":"gnss","ok":false},{"name":"baro_icp10111","ok":true}]}
> inspect wifi
from glider1 ok {"ssid":"panda","rssi":-52,"tx_power_dbm":11}
> all ping
from glider1 pong
from glider7a pong
> glider1 reboot
from glider1 ok
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
