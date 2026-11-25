#!/usr/bin/env python3
"""Common Robocraft message schema (stdlib only).

An envelope wraps every message. Body shapes depend on `type`.
Unknown fields are ignored by the reference validator; adapters must
still drop them before acting, as phone-robot already does on ingest.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, is_dataclass
from datetime import datetime, timezone
from typing import Any, Dict, List, Mapping, Optional, Sequence
from uuid import uuid4

SCHEMA_VERSION = 1
MESSAGE_TYPES = (
    "hello",
    "caps",
    "telemetry",
    "command",
    "command_result",
    "estop",
    "heartbeat",
    "error",
)
SOURCES = ("adapter", "network", "operator", "robot")

# Commands every adapter must understand, even if some only acknowledge them.
COMMON_COMMANDS = ("estop", "ping", "hold")
# Network may request this, but it cannot clear a *local* e-stop.
CLEAR_ESTOP = "clear_estop"


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def _as_plain(value: Any) -> Any:
    if is_dataclass(value) and not isinstance(value, type):
        return {k: _as_plain(v) for k, v in asdict(value).items()}
    if isinstance(value, dict):
        return {k: _as_plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_as_plain(v) for v in value]
    return value


@dataclass(frozen=True)
class Identity:
    robot_id: str
    kind: str
    make: str = ""
    model: str = ""
    name: str = ""
    firmware: str = ""
    # Free-form tags the network can group on (e.g. "humanoid", "opti-fleet").
    tags: List[str] = field(default_factory=list)


@dataclass(frozen=True)
class Capabilities:
    """What this robot can do. Discovery keys off these lists, not the brand."""

    locomotion: List[str] = field(default_factory=list)
    sensing: List[str] = field(default_factory=list)
    commands: List[str] = field(default_factory=list)
    modes: List[str] = field(default_factory=list)
    safety: List[str] = field(default_factory=lambda: ["local_estop", "rate_limit"])

    def all_commands(self) -> List[str]:
        seen = []
        for name in list(COMMON_COMMANDS) + list(self.commands):
            if name not in seen:
                seen.append(name)
        return seen

    def allows(self, command: str) -> bool:
        if command == CLEAR_ESTOP:
            return True
        return command in self.all_commands()


@dataclass
class Telemetry:
    pose: Optional[Dict[str, Any]] = None  # {lat, lon, alt_m?} or {x, y, z, frame}
    velocity: Optional[Dict[str, Any]] = None
    battery_pct: Optional[float] = None
    activity: Optional[str] = None
    mode: Optional[str] = None
    extras: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Heartbeat:
    status: str  # ok | degraded | estop | offline | hold
    period_s: float = 1.0
    detail: str = ""


@dataclass
class Command:
    name: str
    params: Dict[str, Any] = field(default_factory=dict)
    command_id: str = field(default_factory=lambda: str(uuid4()))


@dataclass
class CommandResult:
    command_id: str
    name: str
    status: str  # accepted | rejected | in_progress | done | failed
    reason: str = ""
    payload: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Envelope:
    type: str
    robot_id: str
    body: Dict[str, Any] = field(default_factory=dict)
    src: str = "adapter"
    v: int = SCHEMA_VERSION
    id: str = field(default_factory=lambda: str(uuid4()))
    ts: str = field(default_factory=utc_now_iso)
    auth: Dict[str, str] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return _as_plain(self)


def make_envelope(
    typ: str,
    robot_id: str,
    body: Optional[Mapping[str, Any]] = None,
    *,
    src: str = "adapter",
    auth: Optional[Mapping[str, str]] = None,
) -> Envelope:
    return Envelope(
        type=typ,
        robot_id=robot_id,
        body=dict(body or {}),
        src=src,
        auth=dict(auth or {}),
    )


def validate_envelope(msg: Mapping[str, Any]) -> List[str]:
    """Return a list of problems. Empty means the envelope is usable."""
    errors: List[str] = []
    if not isinstance(msg, Mapping):
        return ["message must be an object"]
    if msg.get("v") != SCHEMA_VERSION:
        errors.append(f"v must be {SCHEMA_VERSION}")
    typ = msg.get("type")
    if typ not in MESSAGE_TYPES:
        errors.append(f"type must be one of {', '.join(MESSAGE_TYPES)}")
    if not msg.get("robot_id"):
        errors.append("robot_id is required")
    if not msg.get("id"):
        errors.append("id is required")
    if not msg.get("ts"):
        errors.append("ts is required")
    src = msg.get("src")
    if src is not None and src not in SOURCES:
        errors.append(f"src must be one of {', '.join(SOURCES)}")
    body = msg.get("body", {})
    if not isinstance(body, Mapping):
        errors.append("body must be an object")
    return errors


def identity_body(identity: Identity, caps: Capabilities) -> Dict[str, Any]:
    return {
        "identity": _as_plain(identity),
        "capabilities": _as_plain(caps),
    }


def robots_with_capability(
    inventory: Sequence[Mapping[str, Any]],
    group: str,
    name: str,
) -> List[str]:
    """Return robot_ids that advertise `name` in capability group `group`."""
    found = []
    for item in inventory:
        caps = item.get("capabilities") or item.get("caps") or {}
        values = caps.get(group) or []
        if name in values or (group == "commands" and name in COMMON_COMMANDS):
            rid = item.get("robot_id") or (item.get("identity") or {}).get("robot_id")
            if rid:
                found.append(rid)
    return found
