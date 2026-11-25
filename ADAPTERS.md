# Robocraft network adapter layer

A design sketch for a shared robot network that any robot can join. Optimus can be the most common robot on it, but it is not a special case in the protocol. One small adapter per robot type turns that robot's own commands and sensors into the network's common messages.

The public repo is [fitzyracing1/robocraft-network](https://github.com/fitzyracing1/robocraft-network). `main` holds only the short README and the MIT license; this design and the reference code live on a feature branch with an open PR.

A tiny zero-dependency Python reference is in `robocraft/` (interface, in-process gateway, mock adapters). Run the tests with:

```bash
cd /workspace/robocraft-network
python3 -m unittest discover -s tests -v
```

Copyright (c) 2026 Joshua Almeida

---

## 1. What this builds on

There is no existing work named "robocraft" on the box, in durable memory, or in GitHub under `fitzyracing1` or `Earthenware-Computer`. The name is new. The sketch reuses patterns already in Joshua's robot projects rather than inventing a second stack.

| Existing work | Where | What the adapter layer takes from it |
|---|---|---|
| phone-robot | [fitzyracing1/phone-robot](https://github.com/fitzyracing1/phone-robot), box: `/workspace/phone-robot` | Per-device bearer tokens, ingest telemetry (`loc`, `activity`, `motion`, `battery`), owner commands `start` / `stop` / `ping`, code-level safety, hourly rate limits, append-only audit, command ack so a command never runs twice |
| Opti (Axb\*1c/d predictive-echo) | Private [fitzyracing1/opti](https://github.com/fitzyracing1/opti), [opti-v1](https://github.com/fitzyracing1/opti-v1), [opti-v2](https://github.com/fitzyracing1/opti-v2); public logs [opti-steps](https://github.com/fitzyracing1/opti-steps), [opti-run-log](https://github.com/fitzyracing1/opti-run-log). Box notes: `/workspace/fitzy/readmes/opti.md`, `/workspace/d/opti-steps` | Cycle: predict echo → decide → push path → do → hear real echo → correct. No world map. Modes: Navigate, Wait-and-see, Release and drag, Follow questions, Scout |
| axb-optimus | [fitzyracing1/axb-optimus](https://github.com/fitzyracing1/axb-optimus), box: `/workspace/d/axb-optimus` | Optimus-class body / joint telemetry, mission waypoints, JSONL telemetry the ground station can pull, restart on every echo. This is Joshua's Optimus-*class* software, not a Tesla API |
| lex-loop | [fitzyracing1/lex-loop](https://github.com/fitzyracing1/lex-loop), box: `/workspace/d/lex-loop` | ROM beside the planner: READ → TRANSCRIBE → ISSUE → TEST → DRIVE → LEAN → ESCAPE → RESTART. The network should fail a cycle if TEST is skipped after ISSUE, or if ESCAPE never runs on a stuck pose |
| robot-ros / service-ros | [fitzyracing1/robot-ros](https://github.com/fitzyracing1/robot-ros), [fitzyracing1/service-ros](https://github.com/fitzyracing1/service-ros), box: `/workspace/robot-ros` | ROS 2 topics `/robot/sensors` → limiter → `/robot/lit_actions` → brain → `/robot/motion`. Prechecks before uncertain movement |
| munch | [fitzyracing1/munch](https://github.com/fitzyracing1/munch) | An existing predictive-echo *fleet* (several named robots), which is the right shape for many adapters on one network |
| x-robot "target adapters" | [Almeidafly/x-robot](https://github.com/Almeidafly/x-robot), box: `/workspace/x-robot/DESIGN.md` | Same idea in a different domain: a root controller talks to adapters, not to each target's private API. x-robot itself is browser automation, not a physical robot |
| drive-guard | [fitzyracing1/drive-guard](https://github.com/fitzyracing1/drive-guard), box: `/workspace/drive-guard` | One gate, pluggable checks, rate limits, audit. Analogous safety posture, even though it guards files rather than actuators |
| uiltiol / Teslabotspacecruiser | earlier Optimus-class interest | Confirms Optimus is a likely large fleet, not a reason to make the protocol Optimus-only |

A private biped workstream exists and can get its own adapter later without changing the network. That work stays out of this sketch.

---

## 2. Shape of the network

```
 operator / planner / lex-loop
              |
              v
     Robocraft gateway
     (auth, rate limit, discovery,
      e-stop policy, audit)
        |         |          |
        v         v          v
   phone-robot  Opti    ROS 2 adapter
   adapter    adapter   (robot-ros)
        |         |          |
        v         v          v
     Android    Axb loop   /cmd_vel etc.
     app+server

   ...same gateway...
        |         |
        v         v
   MAVLink     Optimus placeholder
   drone/rover  (no public Tesla API)
```

The gateway never imports a vendor SDK. It only speaks the common schema to adapters. Each adapter is the only place that knows MAVLink, ROS 2, the phone-robot HTTP API, or a future humanoid SDK.

---

## 3. Common message schema

Every message is an envelope. Bodies depend on `type`. Timestamps are UTC with a `Z` suffix; display them in America/New_York when showing them to the operator.

```json
{
  "v": 1,
  "id": "9f3c2a6e-2c1a-4b0f-9d3a-0a1b2c3d4e5f",
  "type": "hello",
  "ts": "2026-11-25T02:30:00.000Z",
  "robot_id": "opti-01",
  "src": "adapter",
  "auth": { "scheme": "token", "kid": "opti-01" },
  "body": {}
}
```

`type` is one of: `hello`, `caps`, `telemetry`, `command`, `command_result`, `estop`, `heartbeat`, `error`.

`src` is one of: `adapter`, `network`, `operator`, `robot`.

Unknown fields are ignored for validation and must be dropped before an adapter acts (same rule as phone-robot ingest).

### 3.1 Identity and capabilities (`hello` / `caps`)

Sent when a robot attaches, and again whenever capabilities change.

```json
{
  "identity": {
    "robot_id": "opti-01",
    "kind": "opti",
    "make": "fitzyracing1",
    "model": "opti",
    "name": "Opti",
    "firmware": "",
    "tags": ["predictive-echo", "opti-fleet"]
  },
  "capabilities": {
    "locomotion": ["navigate", "hold"],
    "sensing": ["echo", "predicted_echo"],
    "commands": ["goto", "set_mode", "step"],
    "modes": ["navigate", "wait_and_see", "release_and_drag", "follow_questions", "scout"],
    "safety": ["local_estop", "rate_limit"]
  }
}
```

`kind` is a type name (`humanoid`, `opti`, `phone`, `ros2`, `mavlink`, …), not a brand privilege. Discovery keys off capability lists, not `make`.

Every adapter is treated as if it also supports the common commands `estop`, `ping`, and `hold`, even when they are omitted from `commands`.

### 3.2 Telemetry

Fill only what the robot has. A phone has `pose` as lat/lon; a sim humanoid may use `{x, y, z, frame}`; Opti can put predicted vs real echo in `extras` and should leave any world map field empty.

```json
{
  "pose": { "lat": 40.7829, "lon": -73.9654, "frame": "wgs84" },
  "velocity": { "speed_mps": 1.2 },
  "battery_pct": 81.0,
  "activity": "walking",
  "mode": "logging",
  "extras": {}
}
```

### 3.3 Commands and results

```json
{
  "command_id": "c1",
  "name": "goto",
  "params": { "target": { "x": 2.0, "y": 0.0, "frame": "map" } }
}
```

Result `status` is one of: `accepted`, `rejected`, `in_progress`, `done`, `failed`, plus a `reason` and optional `payload`. Adapters should ack like phone-robot does, so a retried `command_id` does not run twice.

### 3.4 Safety / e-stop

An `estop` message or command engages the stop. The body records `source` (`local` or `network`) and `reason`. A local hardware or on-device switch is also reflected in heartbeat `status: "estop"` and `detail: "local estop"`.

`clear_estop` is a network request to clear a *network* e-stop only. If a local e-stop is active, the gateway rejects the request. Someone at the robot has to release the local switch.

### 3.5 Heartbeat

```json
{ "status": "ok", "period_s": 1.0, "detail": "" }
```

`status` is `ok`, `degraded`, `estop`, `hold`, or `offline`. If the gateway misses several periods, it marks the robot offline and asks the adapter to `hold`. Missed heartbeats are not a clearance to keep moving.

---

## 4. Adapter interface

Every robot type implements the same interface. The reference ABC is `robocraft.adapter.RobotAdapter`:

```
identity() -> Identity
capabilities() -> Capabilities
heartbeat() -> Heartbeat
telemetry() -> Telemetry
local_estop_active() -> bool
apply_estop(source, reason) -> None
clear_network_estop() -> bool          # False if a local e-stop is still set
handle_command(command) -> CommandResult   # already authorized by the gateway
```

The gateway (`RobotNetwork` in the reference) is the only path for operator commands:

1. Resolve `robot_id`.
2. Check the token belongs to *that* robot.
3. Reject commands not in advertised capabilities (plus the common safety commands).
4. Apply the per-robot rate limit.
5. Enforce e-stop rules.
6. Call `handle_command`, or handle `estop` / `clear_estop` itself.

Lex-loop sits beside this, not inside the adapter: ISSUE a command, TEST against the expected echo or ack, DRIVE only if the test passes, ESCAPE if the pose is stuck, RESTART back at READ. The adapter should still refuse a move if TEST was skipped; the gateway can later require a test result on motion commands.

---

## 5. Example adapters

None of these need a special message type. Each one is a translation layer.

### 5.1 Humanoid / Optimus (placeholder)

Tesla does not publish a public Optimus API, so this adapter is a placeholder. It would map common commands onto a future official SDK (walk, stand, go-to, battery, cameras, proprioception). Until that exists, the mock uses the same telemetry shape already in `axb-optimus` (`pose`, leg joints, `torso_height_m`, `battery_pct`, `thermal_c`, mission waypoints).

`kind` is `humanoid`, `make` is `Tesla`, `model` is `Optimus`. That is identity metadata, not a second protocol.

### 5.2 ROS 2 (via a ROS bridge)

Talks to the robot over ROS 2, then to the gateway over the common schema.

A first mapping onto [robot-ros](https://github.com/fitzyracing1/robot-ros):

| Robocraft | ROS 2 |
|---|---|
| telemetry / sensing | `/robot/sensors` (and later standard `sensor_msgs`, `nav_msgs/Odometry`, `sensor_msgs/BatteryState`) |
| motion commands | `/robot/motion`, or a later `/cmd_vel` |
| precheck before uncertain motion | [service-ros](https://github.com/fitzyracing1/service-ros) `.service` files |
| lit/dark plan gate | limiter node; only lit plans become motion |

The adapter subscribes and publishes on the robot's graph. The gateway never runs `rclpy`.

### 5.3 phone-robot

Map the existing HTTP API rather than replacing it.

| Robocraft | phone-robot |
|---|---|
| `robot_id` | ingest `device` |
| auth token | that device's `INGEST_TOKEN` / owner `VIEW_TOKEN` (keep them distinct) |
| telemetry | `POST /ingest` samples: `loc`, `activity`, `motion`, `battery` |
| `start` / `stop` / `ping` | existing owner commands |
| heartbeat | last poll / last ingest time |
| rate limits | existing `COMMANDS_PER_HOUR` (60), `TASKS_PER_HOUR`, `NOTIFY_PER_HOUR` |
| audit | existing hash-chained `audit.ndjson` |

The phone still only talks to its own server. The Robocraft adapter sits next to that server (or in front of it) and does not open a path to anyone else's phone, number, or email — the same "one recipient" rule as the current safety layer.

### 5.4 Opti

Opti's invariants stay inside the adapter:

- No world map is published or stored (`extras.map` stays empty).
- Path is pushed before the robot moves.
- Risk divisor `d` comes only from the predicted echo.
- `set_mode` is limited to advertised modes: `navigate`, `wait_and_see`, `release_and_drag`, `follow_questions`, `scout`.
- `step` runs one predict → decide → push → do → hear → correct cycle.
- Telemetry can include predicted echo, real echo, and the correction, plus the current lex-loop step if Lex is running beside it.

The adapter wraps the existing Opti / `axb-predictive-echo` loop. It does not reimplement it.

### 5.5 Generic drone or rover (MAVLink)

A thin MAVLink translator, useful for common autopilots. Public message names only; the adapter is the only component that speaks the dialect.

| Robocraft | MAVLink (typical) |
|---|---|
| heartbeat | `HEARTBEAT` |
| pose / velocity | `GLOBAL_POSITION_INT`, `ATTITUDE` |
| battery | `SYS_STATUS` or `BATTERY_STATUS` |
| `goto` / `hold` / `rtl` | `COMMAND_LONG` / `SET_POSITION_TARGET_*` |
| local e-stop | hold or disarm, whichever the vehicle's safety policy already uses |

Arming stays an advertised capability, not a default. A rover and a copter can share this adapter kind and differ only in `capabilities`.

---

## 6. Capability discovery

On attach, the adapter sends `hello` with identity and capabilities. The gateway keeps an inventory keyed by `robot_id`.

To find robots that can do something, query the capability lists:

- "Who can `goto`?" → every robot that advertises `goto` under `commands` or `locomotion`, regardless of brand.
- "Who has GPS?" → `sensing` contains `gps` (phones and MAVLink vehicles, not necessarily a humanoid).
- "Show the humanoid fleet" → `kind == "humanoid"` or a tag. That may be mostly Optimus units. The query does not change the protocol.

Capabilities can be refreshed with a later `caps` message (a tool is removed, a mode is disabled, a limb is folded). If a command is no longer advertised, the gateway rejects it.

The reference helper is `robots_with_capability(inventory, group, name)`, also exposed as `RobotNetwork.discover(group, name)`.

---

## 7. Safety rules

These are gateway rules. An adapter may be stricter, never weaker.

1. **Local e-stop always wins.** An on-robot switch, a local process flag, or a hardware breaker cannot be cleared by the network. `clear_estop` from the operator is rejected while `local_estop_active()` is true. Ping may still run so the operator can see status. Motion commands do not.

2. **Network e-stop is additive.** The gateway can engage e-stop on one robot or many. Clearing it is allowed only when no local e-stop is set. A missed heartbeat leads to `hold` or e-stop, not to continued motion.

3. **Commands are rate-limited per robot.** Default in the reference is 60 commands per hour, matching phone-robot's `COMMANDS_PER_HOUR`. Adapters may set a lower limit. Burst windows can be tighter for motion commands later.

4. **Auth is per robot.** Each robot has its own token (or later, its own key). A token for `opti-01` cannot command `pixel-8`. Tokens are at least 8 characters in the reference; production should use long random secrets, like phone-robot's ingest tokens, and never log them.

5. **No implied capabilities.** A drone does not receive humanoid joint commands. If it is not advertised, it is rejected.

6. **Audit.** Every attach, verdict, command, ack, and e-stop should be appended to an audit log (phone-robot already has a hash-chained NDJSON log; drive-guard audits allowed and blocked actions). This sketch's reference does not yet persist an audit file.

7. **Planners propose, code decides.** Same split as phone-robot's safety layer and robot-ros's limiter: a model or planner may suggest `goto`, but the gateway and adapter can refuse it.

---

## 8. How Optimus fits as the largest fleet

Optimus uses the same envelope, the same adapter interface, and the same safety rules as a phone, a rover, or Opti. If there are thousands of Optimus units and a handful of everything else, the inventory just has more `kind=humanoid` rows.

What scales with a large Optimus fleet is operations, not the protocol:

- Many attachments of the same adapter implementation, each with its own `robot_id` and token.
- Discovery queries like "all humanoids with `walk` and battery above 20%".
- Per-robot rate limits and e-stop, plus a later optional group e-stop that still cannot override a local switch.
- Telemetry volume: batch and subscribe, the way phone-robot already batches samples.

What does *not* happen: an Optimus-only message type, a bypass around auth, or a path that lets the largest fleet clear another robot's local e-stop.

Joshua's existing Optimus-class software (`axb-optimus`, Opti, munch) can join immediately through the Opti / humanoid placeholder adapters. A real Tesla Optimus would join later through the same placeholder once a public or partner API exists. Until then, the mock is explicit that it is not talking to Tesla.

---

## 9. What the reference implements

| File | Role |
|---|---|
| `robocraft/messages.py` | Envelope and body schema, validation, discovery helper |
| `robocraft/adapter.py` | `RobotAdapter` interface and `RobotNetwork` gateway (auth, rate limit, e-stop) |
| `robocraft/mock.py` | `MockAdapter` plus thin fleet stand-ins for Optimus, Opti, phone-robot, ROS 2, and MAVLink |
| `tests/test_adapter.py` | Hello/caps, telemetry, command allow/deny, per-robot auth, rate limit, local vs network e-stop, mixed-fleet discovery |

It does not open sockets, talk to ROS or MAVLink, call Tesla, or deploy anything.

## 10. Later

Natural next steps, only with an explicit OK, would be a phone-robot adapter on top of the real HTTP API, an Opti adapter around the existing loop, a ROS 2 bridge beside robot-ros, and a persisted audit log. Do not overwrite existing robot repos.
