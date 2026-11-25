"""Robocraft network adapter reference (stdlib only).

This is a sketch implementation of the interface in ADAPTERS.md.
It is not a product, a Tesla integration, or a live robot controller.
"""

from .messages import (
    Command,
    CommandResult,
    Capabilities,
    Envelope,
    Heartbeat,
    Identity,
    Telemetry,
    make_envelope,
    validate_envelope,
)
from .adapter import AuthError, EstopActive, RateLimited, Rejected, RobotAdapter, RobotNetwork
from .mock import MockAdapter

__all__ = [
    "AuthError",
    "Capabilities",
    "Command",
    "CommandResult",
    "Envelope",
    "EstopActive",
    "Heartbeat",
    "Identity",
    "MockAdapter",
    "RateLimited",
    "Rejected",
    "RobotAdapter",
    "RobotNetwork",
    "Telemetry",
    "make_envelope",
    "validate_envelope",
]
