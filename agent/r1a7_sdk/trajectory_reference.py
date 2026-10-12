"""Extract compact action-scale references from successful teleop episodes."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np


def summarize_episodes(root: str | Path, episodes=("episode_0001.mcap", "episode_0002.mcap", "episode_0003.mcap")) -> dict[str, Any]:
    root = Path(root)
    if not root.is_dir():
        return {"source": str(root), "episodes": [], "missing_root": str(root),
                "usage": "reference action scale and phase timing; never replay joint values"}
    try:
        from scripts.replay_mcap import load_episode
    except Exception as exc:
        return {"source": str(root.resolve()), "episodes": [], "load_error": str(exc),
                "usage": "reference action scale and phase timing; never replay joint values"}
    summaries = []
    for name in episodes:
        path = root / name
        if not path.is_file():
            summaries.append({"episode": name, "missing": True})
            continue
        try:
            frames, _, _ = load_episode(path)
            q = np.asarray([f["states"]["robot_q_current"] for f in frames], dtype=float)
            hand = np.asarray([f["actions"]["hand_cmd"] for f in frames], dtype=float)
            summaries.append({"episode": name, "frames": len(frames),
                              "right_arm_range_rad": np.ptp(q[:, 11:18], axis=0).round(4).tolist(),
                              "right_hand_range": np.ptp(hand[:, 2:4], axis=0).round(4).tolist(),
                              "right_hand_start": hand[0, 2:4].round(4).tolist(),
                              "right_hand_end": hand[-1, 2:4].round(4).tolist()})
        except Exception as exc:
            summaries.append({"episode": name, "error": str(exc)})
    return {"source": str(root.resolve()), "episodes": summaries,
            "usage": "reference action scale and phase timing; never replay joint values"}


def write_summary(root: str | Path, output: str | Path) -> Path:
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summarize_episodes(root), ensure_ascii=False, indent=2) + "\n")
    return path
