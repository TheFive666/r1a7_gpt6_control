"""High-level pick/place SDK over the R1-A7 server's file bridge."""
from __future__ import annotations

import fcntl
import json
import math
import os
import time
from pathlib import Path
from typing import Any, Callable

from .types import ACTIONS, STAGES, Action, Observation, StepResult


class SDKError(RuntimeError):
    """An action was invalid, refused, or the robot bridge failed."""


class FileBridgeTransport:
    def __init__(self, session: str | Path, timeout_s: float = 240.0):
        self.session = Path(session).expanduser().resolve()
        self.bridge = self.session / "bridge_right"
        self.timeout_s = timeout_s

    def call(self, command: str, args: dict[str, Any] | None = None) -> dict[str, Any]:
        if not self.bridge.is_dir():
            raise SDKError(f"R1-A7 server is not running: missing {self.bridge}")
        lock_path = self.bridge / ".sdk_client.lock"
        with lock_path.open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            numbers = [0]
            for path in self.bridge.glob("req_*.json"):
                try:
                    numbers.append(int(path.stem[4:]))
                except ValueError:
                    pass
            sequence = max(numbers) + 1
            request = self.bridge / f"req_{sequence:06d}.json"
            response = self.bridge / f"resp_{sequence:06d}.json"
            temporary = request.with_suffix(".tmp")
            temporary.write_text(json.dumps({"id": sequence, "cmd": command, "args": args or {}, "t": time.time()}))
            os.replace(temporary, request)
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            if response.exists():
                result = json.loads(response.read_text())
                if not result.get("ok"):
                    raise SDKError(str(result.get("error", "R1-A7 bridge rejected command")))
                return result
            time.sleep(0.05)
        raise SDKError(f"timeout waiting for R1-A7 response to {command}")


class R1A7PickPlaceSDK:
    """One-observation/one-action interface with deterministic task gates.

    The model decides only a neighboring Cartesian action and visual evidence.
    This class owns captures, bridge parameters and pick/place phase memory.
    """
    def __init__(self, session: str | Path, *, transport: Callable[[str, dict[str, Any]], dict[str, Any]] | None = None):
        self.session = Path(session).expanduser().resolve()
        bridge = FileBridgeTransport(self.session)
        self._call = transport or bridge.call
        self._observation: Observation | None = None
        self._grasp_verified = False
        self._trial_taken = False
        self._released = False
        self._grasp_z = None
        self._lift_clear = False

    @staticmethod
    def _measured_z(state):
        try:
            z = float(state['ee_pose']['position'][2])
        except (KeyError, TypeError, ValueError, IndexError):
            raise SDKError('缺少实测末端高度，无法确认抓取后抬升') from None
        if not math.isfinite(z):
            raise SDKError('实测末端高度无效')
        return z

    @property
    def lift_state(self):
        height = None
        if self._grasp_z is not None and self._observation is not None:
            height = self._measured_z(self._observation.state) - self._grasp_z
        return dict(required_lift_m=0.15, measured_lift_m=height,
                    transfer_allowed=self._lift_clear)

    def status(self) -> dict[str, Any]:
        return self._call("status", {})

    @property
    def task_state(self) -> dict[str, bool]:
        return {"grasp_verified": self._grasp_verified,
                "trial_taken": self._trial_taken, "released": self._released}

    def observe(self) -> Observation:
        raw = self._call("frames", {"cams": ["top", "wrist"]})
        self._observation = Observation(
            capture=int(raw["capture"]), images=dict(raw["images"]), state=dict(raw["state"]),
            camera_mapping=dict(raw.get("camera_mapping", {})))
        if (self._grasp_z is not None and
                self._observation.state['right_closed_ratio_command'] > 0.5 and
                self._measured_z(self._observation.state) - self._grasp_z >= 0.15):
            self._lift_clear = True
        return self._observation

    @staticmethod
    def _norm(values: tuple[float, float, float]) -> float:
        if len(values) != 3 or any(type(x) not in (int, float) or not math.isfinite(x) for x in values):
            raise SDKError("delta must contain three finite numbers")
        return math.sqrt(sum(float(x) ** 2 for x in values))

    def _validate(self, action: Action) -> None:
        if action.action not in ACTIONS or action.stage not in STAGES:
            raise SDKError("unknown action or stage")
        e = action.evidence
        for name, value in vars(e).items():
            if name != "confidence" and type(value) is not bool:
                raise SDKError(f"evidence.{name} must be boolean")
        if type(e.confidence) not in (int, float) or not math.isfinite(e.confidence) or not 0.8 <= e.confidence <= 1:
            raise SDKError("visual confidence must be within [0.8, 1.0]")
        if e.uncertain and action.action != "stop":
            raise SDKError("uncertain visual evidence: use stop")
        distance = self._norm(action.dpos_m)
        rotation_deg = self._norm(action.drot_deg)
        if distance > 0.020000001 or math.radians(rotation_deg) > 0.025000001:
            raise SDKError("single action exceeds 20 mm / 0.025 rad")
        if action.action != "move" and (distance > 1e-9 or rotation_deg > 1e-9):
            raise SDKError("non-motion action must have zero deltas")
        # Every translation must be at least 20 mm so the real actuator can
        # realize it, including the post-close vertical trial.
        if action.action == "move" and distance < 0.020 - 1e-9:
            raise SDKError("every planning move must translate at least 20 mm")
        if action.action in ("move", "close", "open") and self._observation is None:
            raise SDKError("observe() is required immediately before an executable action")
        closed = bool(self._observation and self._observation.state["right_closed_ratio_command"] > 0.5)
        if action.action == "move" and not e.path_clear:
            raise SDKError("motion path is not visibly clear")
        if action.action == "close" and (closed or not (e.path_clear and e.cake_between_fingers and e.tips_at_grasp_height)):
            raise SDKError("close requires open gripper and visible grasp alignment")
        if action.action == 'close':
            self._measured_z(self._observation.state)
        if action.action == 'move' and closed and not self._lift_clear:
            pure_up = (abs(action.dpos_m[0]) < 1e-9 and abs(action.dpos_m[1]) < 1e-9
                       and abs(action.dpos_m[2] - .02) < 1e-9 and rotation_deg < 1e-9)
            if self._grasp_z is None:
                raise SDKError('缺少抓取高度基准，不能恢复搬运')
            if not pure_up or action.stage not in ('trial_lift', 'raise'):
                raise SDKError('抓取后实测抬升未达到15cm，只允许纯向上2cm')
        if action.action == "move" and closed and not self._grasp_verified:
            pure_trial = 0.020 <= action.dpos_m[2] <= 0.02000001 and self._norm((action.dpos_m[0], action.dpos_m[1], 0.0)) < 1e-9 and rotation_deg < 1e-9
            if not self._trial_taken and not pure_trial:
                raise SDKError("first motion after close must be a 20 mm pure +Z trial lift")
            if self._trial_taken and not e.object_follows_hand:
                raise SDKError("trial lift needs fresh object-follows-hand evidence")
        if action.action == "open" and not (closed and e.object_supported and ((self._grasp_verified and e.inside_plate) or e.empty_gripper)):
            raise SDKError("open requires supported object in plate, or visibly empty failed grasp")
        if action.action == "done" and not (self._released and not closed and e.placement_verified and e.inside_plate and e.object_supported):
            raise SDKError("done requires fresh visual placement evidence")

    def step(self, action: Action | dict[str, Any], *, observe_after: bool = True) -> StepResult:
        if isinstance(action, dict):
            try:
                action = Action.from_dict(action)
            except (KeyError, TypeError, ValueError) as exc:
                raise SDKError(f"invalid action object: {exc}") from None
        self._validate(action)
        if action.action == "stop":
            self._observation = None
            return StepResult("stopped", False, "stop", None, message=action.reason)
        if action.action == "done":
            return StepResult("done", False, "done", self._observation, message=action.reason)
        if action.action == "observe":
            return StepResult("observed", False, "observe", self.observe(), message=action.reason)

        assert self._observation is not None
        evidence = vars(action.evidence).copy()
        args: dict[str, Any] = {"capture": self._observation.capture, "evidence": evidence}
        command = "move_delta" if action.action == "move" else "gripper"
        if action.action == "move":
            args.update(dpos=list(action.dpos_m), drot_deg=list(action.drot_deg))
        else:
            args["action"] = action.action
        was_closed = self._observation.state["right_closed_ratio_command"] > 0.5
        grasp_z = self._measured_z(self._observation.state) if action.action == 'close' else None
        result = self._call(command, args)
        self._observation = None
        if action.action == "close":
            self._grasp_z = grasp_z
            self._lift_clear = False
            self._grasp_verified = self._trial_taken = self._released = False
        elif action.action == "move" and was_closed:
            if self._trial_taken:
                self._grasp_verified = True
            else:
                self._trial_taken = True
        elif action.action == "open":
            self._grasp_z = None
            self._lift_clear = False
            self._released = not action.evidence.empty_gripper
            if action.evidence.empty_gripper:
                self._grasp_verified = self._trial_taken = False
        next_observation = self.observe() if observe_after else None
        return StepResult("executed", True, action.action, next_observation, result, action.reason)
