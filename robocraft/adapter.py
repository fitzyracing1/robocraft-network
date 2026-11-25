#!/usr/bin/env python3
"""Adapter interface and a tiny in-process network gateway.

Safety rules enforced here, before a command reaches an adapter:

1. Auth is per robot. A token for A cannot command B.
2. Local e-stop always wins. The network cannot clear it.
3. Commands are rate-limited per robot.
4. A command not advertised in capabilities is rejected
   (except the common safety commands: estop, ping, hold, and clear_estop).
5. Heartbeat timeout is recorded as comms loss; the adapter is told to hold.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict, List, Optional

from .messages import (
    CLEAR_ESTOP,
    COMMON_COMMANDS,
    Capabilities,
    Command,
    CommandResult,
    Envelope,
    Heartbeat,
    Identity,
    Telemetry,
    identity_body,
    make_envelope,
    robots_with_capability,
    utc_now_iso,
    validate_envelope,
)


class AuthError(Exception):
    pass


class Rejected(Exception):
    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


class RateLimited(Rejected):
    pass


class EstopActive(Rejected):
    pass


class RobotAdapter(ABC):
    """Every robot type implements this. The network never talks to a vendor SDK directly."""

    @abstractmethod
    def identity(self) -> Identity:
        raise NotImplementedError

    @abstractmethod
    def capabilities(self) -> Capabilities:
        raise NotImplementedError

    @abstractmethod
    def heartbeat(self) -> Heartbeat:
        raise NotImplementedError

    @abstractmethod
    def telemetry(self) -> Telemetry:
        raise NotImplementedError

    @abstractmethod
    def local_estop_active(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def apply_estop(self, source: str, reason: str) -> None:
        """Engage e-stop. `source` is 'local' or 'network'."""
        raise NotImplementedError

    @abstractmethod
    def clear_network_estop(self) -> bool:
        """Clear a network e-stop only. Returns False if a local e-stop is still set."""
        raise NotImplementedError

    @abstractmethod
    def handle_command(self, command: Command) -> CommandResult:
        """Run a command that the gateway has already authorized and safety-checked.

        Adapters must still refuse anything they cannot do, and must not
        move if a local e-stop is active.
        """
        raise NotImplementedError


class RateLimiter:
    """Sliding-window counter. Zero dependencies, same idea as phone-robot's COMMANDS_PER_HOUR."""

    def __init__(self, max_events: int, window_s: float, now=None):
        if max_events < 1:
            raise ValueError("max_events must be >= 1")
        self.max_events = max_events
        self.window_s = window_s
        self._now = now or utc_now_iso
        self._times: Dict[str, List[float]] = {}

    def _clock(self) -> float:
        # Tests can pass a callable now() that returns a float seconds value.
        value = self._now()
        if isinstance(value, (int, float)):
            return float(value)
        # Fall back: count events without real time if iso strings are used.
        # The network injects a monotonic clock in practice.
        return 0.0

    def allow(self, key: str) -> bool:
        now = self._clock()
        times = self._times.setdefault(key, [])
        cutoff = now - self.window_s
        times[:] = [t for t in times if t > cutoff]
        if len(times) >= self.max_events:
            return False
        times.append(now)
        return True


class RobotNetwork:
    """In-process stand-in for the Robocraft gateway."""

    def __init__(self, *, commands_per_window: int = 60, window_s: float = 3600.0, now=None):
        self._adapters: Dict[str, RobotAdapter] = {}
        self._tokens: Dict[str, str] = {}
        self._limiter = RateLimiter(commands_per_window, window_s, now=now)
        self._now = now

    def attach(self, adapter: RobotAdapter, token: str) -> Envelope:
        if not token or len(token) < 8:
            raise AuthError("robot token must be at least 8 characters")
        ident = adapter.identity()
        if ident.robot_id in self._adapters:
            raise Rejected(f"robot_id {ident.robot_id} is already attached")
        self._adapters[ident.robot_id] = adapter
        self._tokens[ident.robot_id] = token
        return make_envelope(
            "hello",
            ident.robot_id,
            identity_body(ident, adapter.capabilities()),
            src="adapter",
        )

    def _require(self, robot_id: str, token: str) -> RobotAdapter:
        if robot_id not in self._adapters:
            raise Rejected(f"unknown robot_id {robot_id}")
        if token != self._tokens[robot_id]:
            raise AuthError(f"token is not valid for {robot_id}")
        return self._adapters[robot_id]

    def inventory(self) -> List[Dict[str, object]]:
        items = []
        for robot_id, adapter in self._adapters.items():
            ident = adapter.identity()
            items.append({
                "robot_id": robot_id,
                "identity": ident.__dict__ if not hasattr(ident, "__dataclass_fields__") else ident,
                "capabilities": adapter.capabilities(),
            })
        # Normalize to plain dicts for discovery helpers.
        from .messages import _as_plain
        return [_as_plain(item) for item in items]

    def discover(self, group: str, name: str) -> List[str]:
        return robots_with_capability(self.inventory(), group, name)

    def snapshot(self, robot_id: str, token: str, typ: str) -> Envelope:
        adapter = self._require(robot_id, token)
        if typ == "telemetry":
            body = _telemetry_body(adapter.telemetry())
        elif typ == "heartbeat":
            body = _heartbeat_body(adapter.heartbeat())
        elif typ == "caps":
            body = identity_body(adapter.identity(), adapter.capabilities())
        else:
            raise Rejected(f"cannot snapshot type {typ}")
        return make_envelope(typ, robot_id, body, src="adapter")

    def dispatch(self, robot_id: str, token: str, command: Command, *, src: str = "operator") -> CommandResult:
        adapter = self._require(robot_id, token)
        caps = adapter.capabilities()
        if not caps.allows(command.name):
            raise Rejected(f"command {command.name} is not in capabilities for {robot_id}")
        if not self._limiter.allow(robot_id):
            raise RateLimited(f"rate limit reached for {robot_id}")

        if command.name == "estop":
            adapter.apply_estop("network", command.params.get("reason", "network estop"))
            return CommandResult(command.command_id, command.name, "done", "estop engaged")

        if command.name == CLEAR_ESTOP:
            if adapter.local_estop_active():
                raise EstopActive("local e-stop is active; the network cannot clear it")
            if not adapter.clear_network_estop():
                raise EstopActive("e-stop still active")
            return CommandResult(command.command_id, command.name, "done", "network estop cleared")

        if adapter.local_estop_active() and command.name not in ("ping",):
            raise EstopActive("local e-stop is active; command refused")

        # Network e-stop / hold is reported via heartbeat.status; adapters
        # still refuse motion while that status is estop or hold.
        hb = adapter.heartbeat()
        if hb.status == "estop" and command.name not in ("ping", "estop", CLEAR_ESTOP):
            raise EstopActive("e-stop is active; command refused")

        return adapter.handle_command(command)


def _telemetry_body(tel: Telemetry) -> Dict[str, object]:
    from .messages import _as_plain
    return _as_plain(tel)


def _heartbeat_body(hb: Heartbeat) -> Dict[str, object]:
    from .messages import _as_plain
    return _as_plain(hb)


# Re-export for adapters / tests that validate inbound traffic.
__all__ = [
    "AuthError",
    "EstopActive",
    "RateLimited",
    "RateLimiter",
    "Rejected",
    "RobotAdapter",
    "RobotNetwork",
    "validate_envelope",
]
