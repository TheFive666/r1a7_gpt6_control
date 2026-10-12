"""Reference-guided Cartesian step hints from successful teleoperation data."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np


def load_reference(root: str | Path) -> dict[str, Any]:
    """Return bounded phase-independent motion statistics, never joint commands."""
    root = Path(root)
    rule = "use as motion scale prior; resolve current direction from fresh RGB and SDK FK"
    if not root.is_dir():
        return {"episodes": [], "rule": rule, "missing_root": str(root)}
    try:
        from scripts.replay_mcap import load_episode
    except Exception as exc:  # soft-fail when data tooling / deps unavailable
        return {"episodes": [], "rule": rule, "load_error": str(exc), "root": str(root)}
    episodes = []
    for path in sorted(root.glob("episode_000[1-3].mcap")):
        try:
            frames, _, _ = load_episode(path)
            q = np.asarray([frame["states"]["robot_q_current"] for frame in frames], dtype=float)
            # Dataset robot_q_current is body(4)+arms(14)+hand(4); right arm is 11:18.
            dq = np.diff(q[:, 11:18], axis=0)
            episodes.append({
                "episode": path.name,
                "frames": len(frames),
                "right_joint_step_p95_rad": np.percentile(np.abs(dq), 95, axis=0).round(5).tolist(),
                "right_joint_total_range_rad": np.ptp(q[:, 11:18], axis=0).round(4).tolist(),
            })
        except Exception as exc:
            episodes.append({"episode": path.name, "error": str(exc)})
    return {"episodes": episodes, "rule": rule}


def write_reference(root: str | Path, output: str | Path) -> Path:
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(load_reference(root), ensure_ascii=False, indent=2) + "\n")
    return output
