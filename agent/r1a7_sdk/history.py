"""Compact rollout history inspired by RPent's step-state/recipe split."""
from __future__ import annotations

import json
import os
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .types import Action, Observation, StepResult


@dataclass
class ControlLedger:
    task: str
    max_steps: int
    step: int = 0
    phase: str = "approach"
    held_object: str | None = None
    grasp_verified: bool = False
    released: bool = False
    first_unmet_gate: str = "align cake between fingers at grasp height"
    blocker: str | None = None
    recent: list[dict[str, Any]] = field(default_factory=list)

    def update(self, action: Action, result: StepResult, task_state: dict[str, bool]) -> None:
        self.step += 1
        self.phase = action.stage
        self.grasp_verified = task_state["grasp_verified"]
        self.released = task_state["released"]
        if action.action == "close":
            self.held_object = "cake_unverified"
        if self.grasp_verified and not self.released:
            self.held_object = "cake"
        if action.action == "open":
            self.held_object = None
        self.blocker = action.reason if action.action == "stop" else None
        self.first_unmet_gate = self._next_gate(action, task_state)
        self.recent.append({
            "step": self.step, "action": action.action, "stage": action.stage,
            "dpos_m": list(action.dpos_m), "result": result.status,
            "observed": action.observation[:160],
            "feedback": result.robot_result.get("timing", {}),
        })
        self.recent = self.recent[-4:]

    @staticmethod
    def _next_gate(action: Action, state: dict[str, bool]) -> str:
        if state["released"]:
            return "verify cake remains supported inside plate and fingers are clear"
        if state["grasp_verified"]:
            return "raise above rim, transfer over plate, then verify support"
        if state["trial_taken"]:
            return "verify cake follows hand in fresh head and wrist views"
        if action.action == "close":
            return "execute pure +Z trial lift of 20 mm"
        return "align cake between fingers at grasp height"

    def context(self) -> dict[str, Any]:
        """Bounded context for the planner; raw server logs stay out of prompts."""
        return {
            "task": self.task,
            "step": self.step,
            "remaining_steps": max(0, self.max_steps - self.step),
            "phase": self.phase,
            "held_object": self.held_object,
            "grasp_verified": self.grasp_verified,
            "released": self.released,
            "first_unmet_gate": self.first_unmet_gate,
            "blocker": self.blocker,
            "recent": list(self.recent),
        }


class EpisodeTrace:
    """Append-only, crash-tolerant action trace plus replayable successful recipe."""
    def __init__(self, session: str | Path):
        self.root = Path(session).expanduser().resolve() / "sdk_episode"
        self.root.mkdir(parents=True, exist_ok=True)
        self.trace = self.root / "steps.jsonl"
        self.recipe = self.root / "recipe.jsonl"

    @staticmethod
    def _observation(value: Observation | None) -> dict[str, Any] | None:
        if value is None:
            return None
        return {"capture": value.capture, "images": value.images, "state": value.state}

    def append(self, *, step: int, before: Observation, action: Action, result: StepResult,
               ledger: ControlLedger, elapsed_s: float) -> None:
        record = {
            "time_ns": time.time_ns(), "step": step,
            "before": self._observation(before), "proposal": asdict(action),
            "execution": {"status": result.status, "executed": result.executed,
                          "robot_result": result.robot_result, "elapsed_s": elapsed_s},
            "after": self._observation(result.observation), "ledger": ledger.context(),
        }
        self._append_atomic(self.trace, record)
        if result.executed:
            self._append_atomic(self.recipe, asdict(action))

    @staticmethod
    def _append_atomic(path: Path, value: dict[str, Any]) -> None:
        # One O_APPEND write prevents partial interleaving with monitoring tools.
        payload = (json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n").encode()
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
        try:
            os.write(fd, payload)
            os.fsync(fd)
        finally:
            os.close(fd)
