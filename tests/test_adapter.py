#!/usr/bin/env python3
"""Reference tests for the Robocraft adapter interface. Stdlib only."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

# Allow `python3 -m unittest` from the repo root without installing a package.
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from robocraft import (  # noqa: E402
    AuthError,
    Command,
    EstopActive,
    MockAdapter,
    RateLimited,
    Rejected,
    RobotNetwork,
    make_envelope,
    validate_envelope,
)
from robocraft.mock import (  # noqa: E402
    mock_mavlink,
    mock_opti,
    mock_optimus,
    mock_phone_robot,
    mock_ros2,
)
from robocraft.messages import SCHEMA_VERSION  # noqa: E402


class Clock:
    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, seconds: float) -> None:
        self.t += seconds


class EnvelopeTests(unittest.TestCase):
    def test_valid_hello(self):
        env = make_envelope("hello", "opti-01", {"ok": True})
        self.assertEqual(validate_envelope(env.to_dict()), [])
        self.assertEqual(env.v, SCHEMA_VERSION)

    def test_rejects_unknown_type(self):
        errors = validate_envelope({"v": 1, "type": "launch", "robot_id": "x", "id": "1", "ts": "t"})
        self.assertTrue(any("type" in e for e in errors))


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.net = RobotNetwork(commands_per_window=60, window_s=60.0, now=self.clock)
        self.opti = mock_opti()
        self.hello = self.net.attach(self.opti, "token-opti-01")

    def test_hello_advertises_identity_and_caps(self):
        self.assertEqual(self.hello.type, "hello")
        ident = self.hello.body["identity"]
        caps = self.hello.body["capabilities"]
        self.assertEqual(ident["robot_id"], "opti-01")
        self.assertEqual(ident["kind"], "opti")
        self.assertIn("navigate", caps["modes"])
        self.assertIn("echo", caps["sensing"])

    def test_telemetry_snapshot(self):
        snap = self.net.snapshot("opti-01", "token-opti-01", "telemetry")
        self.assertEqual(snap.type, "telemetry")
        self.assertEqual(snap.body["mode"], "navigate")
        self.assertIsNone(snap.body["extras"]["map"])

    def test_advertised_command_is_accepted(self):
        result = self.net.dispatch(
            "opti-01",
            "token-opti-01",
            Command(name="set_mode", params={"mode": "scout"}),
        )
        self.assertEqual(result.status, "done")
        self.assertEqual(self.opti.telemetry().mode, "scout")
        self.assertEqual(self.opti.commands_handled, 1)

    def test_unknown_command_is_rejected(self):
        with self.assertRaises(Rejected) as ctx:
            self.net.dispatch("opti-01", "token-opti-01", Command(name="arm"))
        self.assertIn("not in capabilities", str(ctx.exception))
        self.assertEqual(self.opti.commands_handled, 0)

    def test_wrong_token_is_rejected(self):
        with self.assertRaises(AuthError):
            self.net.dispatch("opti-01", "token-someone-else", Command(name="ping"))

    def test_token_does_not_work_on_another_robot(self):
        phone = mock_phone_robot()
        self.net.attach(phone, "token-phone-01")
        with self.assertRaises(AuthError):
            self.net.dispatch("pixel-8", "token-opti-01", Command(name="ping"))
        result = self.net.dispatch("pixel-8", "token-phone-01", Command(name="ping"))
        self.assertEqual(result.status, "done")

    def test_rate_limit(self):
        clock = Clock()
        net = RobotNetwork(commands_per_window=3, window_s=60.0, now=clock)
        net.attach(mock_opti(), "token-opti-01")
        for _ in range(3):
            net.dispatch("opti-01", "token-opti-01", Command(name="ping"))
        with self.assertRaises(RateLimited):
            net.dispatch("opti-01", "token-opti-01", Command(name="ping"))
        clock.advance(61)
        result = net.dispatch("opti-01", "token-opti-01", Command(name="ping"))
        self.assertEqual(result.status, "done")

    def test_local_estop_blocks_motion_and_cannot_be_cleared_by_network(self):
        self.opti.press_local_estop()
        with self.assertRaises(EstopActive):
            self.net.dispatch(
                "opti-01",
                "token-opti-01",
                Command(name="goto", params={"target": {"x": 1, "y": 0}}),
            )
        # ping is still allowed so the operator can check the robot
        pong = self.net.dispatch("opti-01", "token-opti-01", Command(name="ping"))
        self.assertEqual(pong.status, "done")
        with self.assertRaises(EstopActive) as ctx:
            self.net.dispatch("opti-01", "token-opti-01", Command(name="clear_estop"))
        self.assertIn("local e-stop", str(ctx.exception))
        # After someone at the robot releases the switch, motion works again.
        self.opti.release_local_estop()
        result = self.net.dispatch(
            "opti-01",
            "token-opti-01",
            Command(name="goto", params={"target": {"x": 1, "y": 0}}),
        )
        self.assertEqual(result.status, "accepted")

    def test_network_estop_can_be_cleared_when_local_is_not_set(self):
        self.net.dispatch("opti-01", "token-opti-01", Command(name="estop", params={"reason": "test"}))
        self.assertEqual(self.net.snapshot("opti-01", "token-opti-01", "heartbeat").body["status"], "estop")
        with self.assertRaises(EstopActive):
            self.net.dispatch("opti-01", "token-opti-01", Command(name="goto", params={"target": "home"}))
        cleared = self.net.dispatch("opti-01", "token-opti-01", Command(name="clear_estop"))
        self.assertEqual(cleared.status, "done")
        result = self.net.dispatch("opti-01", "token-opti-01", Command(name="goto", params={"target": "home"}))
        self.assertEqual(result.status, "accepted")

    def test_capability_discovery_treats_optimus_as_one_kind_among_many(self):
        fleet = RobotNetwork()
        adapters = [
            mock_optimus("optimus-001"),
            mock_optimus("optimus-002"),
            mock_opti("opti-01"),
            mock_phone_robot("pixel-8"),
            mock_ros2("ros-base-01"),
            mock_mavlink("rover-01"),
        ]
        for i, adapter in enumerate(adapters):
            fleet.attach(adapter, f"token-{adapter.identity().robot_id}-{i:02d}xx")

        inventory = fleet.inventory()
        self.assertEqual(len(inventory), 6)
        goto = fleet.discover("commands", "goto")
        self.assertEqual(
            set(goto),
            {"optimus-001", "optimus-002", "opti-01", "ros-base-01", "rover-01"},
        )
        self.assertNotIn("pixel-8", goto)
        phones = fleet.discover("sensing", "gps")
        self.assertIn("pixel-8", phones)
        self.assertIn("rover-01", phones)
        self.assertNotIn("optimus-001", phones)
        humanoids = [item["robot_id"] for item in inventory if item["identity"]["kind"] == "humanoid"]
        self.assertEqual(set(humanoids), {"optimus-001", "optimus-002"})
        # Optimus can be the largest group without getting a special protocol.
        counts = {}
        for item in inventory:
            kind = item["identity"]["kind"]
            counts[kind] = counts.get(kind, 0) + 1
        self.assertEqual(max(counts, key=counts.get), "humanoid")
        self.assertEqual(counts["humanoid"], 2)

    def test_short_token_refused_on_attach(self):
        net = RobotNetwork()
        with self.assertRaises(AuthError):
            net.attach(mock_opti("opti-02"), "short")


if __name__ == "__main__":
    unittest.main()
