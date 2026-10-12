"""Public value types and the only action schema an agent needs to know."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


STAGES = ("approach", "capture", "close", "trial_lift", "raise", "transfer", "lower", "release", "verify")
ACTIONS = ("move", "close", "open", "observe", "done", "stop")


@dataclass(frozen=True)
class Evidence:
    confidence: float
    uncertain: bool
    path_clear: bool
    cake_between_fingers: bool = False
    tips_at_grasp_height: bool = False
    object_follows_hand: bool = False
    object_supported: bool = False
    inside_plate: bool = False
    placement_verified: bool = False
    empty_gripper: bool = False


@dataclass(frozen=True)
class Action:
    action: str
    stage: str
    evidence: Evidence
    dpos_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    drot_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    observation: str = ""
    reason: str = ""

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "Action":
        value = dict(value)
        value["evidence"] = Evidence(**value["evidence"])
        for name in ("dpos_m", "drot_deg"):
            value[name] = tuple(value.get(name, (0.0, 0.0, 0.0)))
        return cls(**value)


@dataclass(frozen=True)
class Observation:
    capture: int
    images: dict[str, str]
    state: dict[str, Any]
    camera_mapping: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class StepResult:
    status: str
    executed: bool
    action: str
    observation: Observation | None
    robot_result: dict[str, Any] = field(default_factory=dict)
    message: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


ACTION_SCHEMA = {
    "type": "object",
    "additionalProperties": False,
    "required": ["action", "stage", "dpos_m", "drot_deg", "evidence", "observation", "reason"],
    "properties": {
        "action": {"type": "string", "enum": list(ACTIONS)},
        "stage": {"type": "string", "enum": list(STAGES)},
        "dpos_m": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
        "drot_deg": {"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3},
        "evidence": {
            "type": "object", "additionalProperties": False,
            "required": ["confidence", "uncertain", "path_clear", "cake_between_fingers",
                         "tips_at_grasp_height", "object_follows_hand", "object_supported",
                         "inside_plate", "placement_verified", "empty_gripper"],
            "properties": {
                "confidence": {"type": "number", "minimum": 0.8, "maximum": 1.0},
                **{name: {"type": "boolean"} for name in (
                    "uncertain", "path_clear", "cake_between_fingers", "tips_at_grasp_height",
                    "object_follows_hand", "object_supported", "inside_plate",
                    "placement_verified", "empty_gripper")},
            },
        },
        "observation": {"type": "string"},
        "reason": {"type": "string"},
    },
}
