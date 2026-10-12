"""Small, stable Agent-as-Policy interface for the R1-A7 right arm."""

from .client import R1A7PickPlaceSDK, SDKError
from .history import ControlLedger, EpisodeTrace
from .external_video import ExternalVideoRecorder
from .trajectory_reference import summarize_episodes, write_summary
from .reference_policy import load_reference, write_reference
from .types import ACTION_SCHEMA, Action, Evidence, Observation, StepResult

__all__ = [
    "ACTION_SCHEMA", "Action", "Evidence", "Observation",
    "ControlLedger", "EpisodeTrace", "ExternalVideoRecorder", "R1A7PickPlaceSDK",
    "SDKError", "StepResult",
    "summarize_episodes", "write_summary",
    "load_reference", "write_reference",
]
