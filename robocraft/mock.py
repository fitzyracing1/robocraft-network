#!/usr/bin/env python3
"""In-memory adapters used by the reference tests.

These stand in for real robot types. They do not talk to hardware,
ROS, MAVLink, Tesla, or any network service.
"""

from __future__ import annotations

from typing import Dict, Optional

from .adapter import RobotAdapter
from .messages import (
    Capabilities,
    Command,
    CommandResult,
    Heartbeat,
    Identity,
    Telemetry,
)


class MockAdapter(RobotAdapter):
    """A configurable fake robot with a local e-stop switch."""

    def __init__(
        self,
        identity: Identity,
        capabilities: Optional[Capabilities] = None,
        telemetry: Optional[Telemetry] = None,
    ):
        self._identity = identity
        self._caps = capabilities or Capabilities(
            locomotion=["hold"],
            sensing=[],
            commands=["goto"],
            modes=[],
        )
        self._telemetry = telemetry or Telemetry(battery_pct=100.0, mode="idle")
        self._local_estop = False
        self._network_estop = False
        self.last_command: Optional[Command] = None
        self.commands_handled = 0

    def identity(self) -> Identity:
        return self._identity

    def capabilities(self) -> Capabilities:
        return self._caps

    def heartbeat(self) -> Heartbeat:
        if self._local_estop or self._network_estop:
            source = "local" if self._local_estop else "network"
            return Heartbeat(status="estop", period_s=1.0, detail=f"{source} estop")
        return Heartbeat(status="ok", period_s=1.0)

    def telemetry(self) -> Telemetry:
        return self._telemetry

    def local_estop_active(self) -> bool:
        return self._local_estop

    def press_local_estop(self, reason: str = "operator hit the local switch") -> None:
        """Simulate the on-robot switch. The network cannot undo this."""
        self._local_estop = True
        self._telemetry.activity = "estop"
        self._telemetry.extras = dict(self._telemetry.extras, estop_reason=reason)

    def release_local_estop(self) -> None:
        """Only a person at the robot can do this (tests call it directly)."""
        self._local_estop = False
        if not self._network_estop:
            self._telemetry.activity = "idle"
            self._telemetry.extras.pop("estop_reason", None)

    def apply_estop(self, source: str, reason: str) -> None:
        if source == "local":
            self.press_local_estop(reason)
        else:
            self._network_estop = True
            self._telemetry.activity = "estop"
            self._telemetry.extras = dict(self._telemetry.extras, estop_reason=reason)

    def clear_network_estop(self) -> bool:
        if self._local_estop:
            return False
        self._network_estop = False
        self._telemetry.activity = "idle"
        self._telemetry.extras.pop("estop_reason", None)
        return True

    def handle_command(self, command: Command) -> CommandResult:
        if command.name == "ping":
            self.last_command = command
            self.commands_handled += 1
            return CommandResult(command.command_id, command.name, "done", "pong")
        if self._local_estop or self._network_estop:
            return CommandResult(
                command.command_id,
                command.name,
                "rejected",
                "e-stop is active",
            )
        if not self._caps.allows(command.name):
            return CommandResult(
                command.command_id,
                command.name,
                "rejected",
                f"{command.name} is not supported",
            )
        self.last_command = command
        self.commands_handled += 1
        if command.name == "hold":
            self._telemetry.mode = "hold"
            return CommandResult(command.command_id, command.name, "done", "holding")
        if command.name == "goto":
            target = command.params.get("target")
            self._telemetry.mode = "goto"
            self._telemetry.extras = dict(self._telemetry.extras, last_target=target)
            return CommandResult(
                command.command_id,
                command.name,
                "accepted",
                "moving",
                {"target": target},
            )
        if command.name == "set_mode":
            mode = command.params.get("mode")
            if mode and mode not in self._caps.modes:
                return CommandResult(
                    command.command_id,
                    command.name,
                    "rejected",
                    f"mode {mode} is not advertised",
                )
            self._telemetry.mode = mode
            return CommandResult(command.command_id, command.name, "done", f"mode={mode}")
        # Accept any other advertised command so example fleet adapters can stay thin.
        return CommandResult(command.command_id, command.name, "accepted", "ok")


def mock_optimus(robot_id: str = "optimus-001") -> MockAdapter:
    """Placeholder for a Tesla Optimus-class humanoid. There is no public Optimus API."""
    return MockAdapter(
        Identity(
            robot_id=robot_id,
            kind="humanoid",
            make="Tesla",
            model="Optimus",
            name="Optimus",
            tags=["humanoid", "optimus-class"],
        ),
        Capabilities(
            locomotion=["walk", "goto", "hold", "stand"],
            sensing=["cameras", "proprioception", "battery"],
            commands=["goto", "set_mode", "stand"],
            modes=["walk", "stand", "hold"],
        ),
        Telemetry(battery_pct=98.0, mode="stand", extras={"torso_height_m": 1.65}),
    )


def mock_opti(robot_id: str = "opti-01") -> MockAdapter:
    return MockAdapter(
        Identity(
            robot_id=robot_id,
            kind="opti",
            make="fitzyracing1",
            model="opti",
            name="Opti",
            tags=["predictive-echo", "opti-fleet"],
        ),
        Capabilities(
            locomotion=["navigate", "hold"],
            sensing=["echo", "predicted_echo"],
            commands=["goto", "set_mode", "step"],
            modes=["navigate", "wait_and_see", "release_and_drag", "follow_questions", "scout"],
        ),
        Telemetry(mode="navigate", extras={"map": None, "path_pushed": False}),
    )


def mock_phone_robot(robot_id: str = "pixel-8") -> MockAdapter:
    return MockAdapter(
        Identity(
            robot_id=robot_id,
            kind="phone",
            make="generic",
            model="android",
            name="phone-robot",
            tags=["phone-robot"],
        ),
        Capabilities(
            locomotion=[],
            sensing=["gps", "imu", "activity", "battery"],
            commands=["start", "stop"],  # ping/estop/hold are common
            modes=["logging", "stopped"],
        ),
        Telemetry(
            pose={"lat": 40.7829, "lon": -73.9654, "frame": "wgs84"},
            battery_pct=81.0,
            activity="walking",
            mode="logging",
        ),
    )


def mock_ros2(robot_id: str = "ros-base-01") -> MockAdapter:
    return MockAdapter(
        Identity(
            robot_id=robot_id,
            kind="ros2",
            make="generic",
            model="robot-ros",
            name="ROS 2 base",
            tags=["ros2", "wheeled"],
        ),
        Capabilities(
            locomotion=["cmd_vel", "goto", "hold"],
            sensing=["sensors", "battery"],
            commands=["goto", "set_velocity"],
            modes=["nav", "hold"],
            safety=["local_estop", "rate_limit", "precheck"],
        ),
        Telemetry(mode="hold", extras={"topics": ["/robot/sensors", "/robot/motion"]}),
    )


def mock_mavlink(robot_id: str = "rover-01") -> MockAdapter:
    return MockAdapter(
        Identity(
            robot_id=robot_id,
            kind="mavlink",
            make="generic",
            model="rover",
            name="MAVLink rover",
            tags=["mavlink", "rover"],
        ),
        Capabilities(
            locomotion=["goto", "hold", "rtl"],
            sensing=["gps", "attitude", "battery"],
            commands=["goto", "rtl", "arm", "disarm"],
            modes=["hold", "guided", "rtl"],
        ),
        Telemetry(battery_pct=90.0, mode="hold", extras={"armed": False, "sysid": 1}),
    )


EXAMPLE_FLEET = {
    "optimus": mock_optimus,
    "opti": mock_opti,
    "phone": mock_phone_robot,
    "ros2": mock_ros2,
    "mavlink": mock_mavlink,
}
