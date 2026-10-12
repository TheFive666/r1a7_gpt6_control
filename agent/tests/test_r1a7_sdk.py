import json
import tempfile
import unittest

from agent.r1a7_sdk import Action, ControlLedger, EpisodeTrace, Evidence, R1A7PickPlaceSDK, SDKError


class FakeBridge:
    def __init__(self):
        self.capture = 0
        self.closed = False
        self.calls = []

    def __call__(self, command, args):
        self.calls.append((command, args))
        state = {"right_closed_ratio_command": float(self.closed), "motion_enabled": False}
        if command == "frames":
            self.capture += 1
            return {"ok": True, "capture": self.capture, "images": {"top": "/top.jpg", "wrist": "/wrist.jpg"},
                    "camera_mapping": {"top": "head_right", "wrist": "wrist_right"}, "state": state}
        if command == "gripper":
            self.closed = args["action"] == "close"
        return {"ok": True, "after": {**state, "right_closed_ratio_command": float(self.closed)}}


def evidence(**overrides):
    values = dict(confidence=.95, uncertain=False, path_clear=True)
    values.update(overrides)
    return Evidence(**values)


class SDKTests(unittest.TestCase):
    def setUp(self):
        self.bridge = FakeBridge()
        self.sdk = R1A7PickPlaceSDK("/unused", transport=self.bridge)

    def test_move_binds_capture_and_returns_fresh_observation(self):
        first = self.sdk.observe()
        result = self.sdk.step(Action("move", "approach", evidence(), dpos_m=(.01, 0, 0)))
        self.assertTrue(result.executed)
        self.assertGreater(result.observation.capture, first.capture)
        move = [call for call in self.bridge.calls if call[0] == "move_delta"][0]
        self.assertEqual(move[1]["capture"], first.capture)

    def test_close_requires_alignment_then_first_motion_is_trial_lift(self):
        self.sdk.observe()
        with self.assertRaises(SDKError):
            self.sdk.step(Action("close", "close", evidence()))
        self.sdk.step(Action("close", "close", evidence(cake_between_fingers=True, tips_at_grasp_height=True)))
        with self.assertRaises(SDKError):
            self.sdk.step(Action("move", "raise", evidence(), dpos_m=(.01, 0, 0)))
        result = self.sdk.step(Action("move", "trial_lift", evidence(), dpos_m=(0, 0, .01)))
        self.assertTrue(result.executed)

    def test_transfer_needs_following_evidence_after_trial(self):
        self.sdk.observe()
        self.sdk.step(Action("close", "close", evidence(cake_between_fingers=True, tips_at_grasp_height=True)))
        self.sdk.step(Action("move", "trial_lift", evidence(), dpos_m=(0, 0, .01)))
        with self.assertRaises(SDKError):
            self.sdk.step(Action("move", "raise", evidence(), dpos_m=(0, 0, .01)))
        result = self.sdk.step(Action("move", "raise", evidence(object_follows_hand=True), dpos_m=(0, 0, .01)))
        self.assertTrue(result.executed)

    def test_compact_ledger_and_aligned_episode_trace(self):
        before = self.sdk.observe()
        action = Action("move", "approach", evidence(), dpos_m=(.01, 0, 0), observation="cake visible")
        result = self.sdk.step(action)
        ledger = ControlLedger("cake to plate", 60)
        ledger.update(action, result, self.sdk.task_state)
        with tempfile.TemporaryDirectory() as root:
            trace = EpisodeTrace(root)
            trace.append(step=1, before=before, action=action, result=result, ledger=ledger, elapsed_s=.2)
            row = json.loads(trace.trace.read_text().strip())
            self.assertEqual(row["before"]["capture"], before.capture)
            self.assertEqual(row["after"]["capture"], result.observation.capture)
            self.assertEqual(ledger.context()["remaining_steps"], 59)
            self.assertNotIn("robot_result", ledger.context())

    def test_uncertainty_allows_only_stop(self):
        self.sdk.observe()
        with self.assertRaises(SDKError):
            self.sdk.step(Action("move", "approach", evidence(uncertain=True), dpos_m=(.001, 0, 0)))
        stopped = self.sdk.step(Action("stop", "approach", evidence(uncertain=True), reason="target lost"))
        self.assertEqual(stopped.status, "stopped")

    def test_subcentimeter_moves_never_reach_bridge(self):
        self.sdk.observe()
        for delta in ((0, 0, 0), (.009, 0, 0), (0, 0, .003)):
            with self.subTest(delta=delta), self.assertRaises(SDKError):
                self.sdk.step(Action("move", "approach", evidence(), dpos_m=delta))
        self.assertFalse(any(cmd == "move_delta" for cmd, _ in self.bridge.calls))


if __name__ == "__main__":
    unittest.main()
