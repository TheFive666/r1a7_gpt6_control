"""LeRobot Dataset v2.1 writer for four-camera R1-A7 recordings.

This module intentionally targets ``lerobot==0.3.3``.  That release writes
LeRobot Dataset v2.1 (one parquet and one video per camera per episode).
LeRobot 0.4 and newer write the incompatible Dataset v3 format.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from robot.config import ARM_JOINTS


class FourCameraLeRobotV21Recorder:
    """Record R1-A7 samples through the official LeRobot v2.1 API."""

    IMAGE_WRITER_THREADS = 4
    CAMERA_ORDER = ("head_left", "head_right", "wrist_left", "wrist_right")
    CAMERA_KEYS = tuple(f"observation.images.{name}" for name in CAMERA_ORDER)
    STATE_NAMES = (
        "waist_yaw_joint",
        *ARM_JOINTS,
        "left_dex1_gripper_joint",
        "right_dex1_gripper_joint",
    )
    # record_real.py also feeds the legacy MCAP-compatible vector layout into
    # this writer: body(4), arms(14), left Dex1 URDF joints(2), right(2).
    # Only waist, the 14 actuated arm joints and one DoF per physical Dex1 are
    # useful for LeRobot.  The second URDF finger on each side is coupled to the
    # first one and therefore carries the same command/state.
    LEGACY_VECTOR_SIZE = 22
    LEGACY_TO_COMPACT = (0, *range(4, 18), 18, 20)

    def __init__(self, dataset_root: str, repo_id: str, task_goal: str, frequency: float) -> None:
        rounded_fps = int(round(float(frequency)))
        if rounded_fps <= 0 or abs(float(frequency) - rounded_fps) > 1e-6:
            raise ValueError("LeRobot v2.1 requires --frequency to be a positive integer")

        self.root = Path(dataset_root).expanduser().resolve()
        self.repo_id = str(repo_id)
        self.task_goal = str(task_goal)
        self.frequency = rounded_fps
        self.dataset = None
        self.episode_id = -1
        self.item_id = -1
        self.current_path: Optional[str] = None
        self._active = False
        self._delete_backup = self.root.parent / f".{self.root.name}.last_episode_meta"

        self._LeRobotDataset, codebase_version = self._load_api()
        if codebase_version != "v2.1":
            raise RuntimeError(
                "LeRobot Dataset v2.1 recording requires lerobot==0.3.3; "
                f"the installed package writes {codebase_version}"
            )

        info_path = self.root / "meta" / "info.json"
        if info_path.is_file():
            self.dataset = self._LeRobotDataset(repo_id=self.repo_id, root=self.root)
            self._validate_existing_dataset()
            self._start_image_writer()
            self.episode_id = int(self.dataset.meta.total_episodes) - 1
        elif self.root.exists() and any(self.root.iterdir()):
            raise RuntimeError(
                f"LeRobot root is not empty and has no v2.1 metadata: {self.root}"
            )

        self._remove_backup()

    def _start_image_writer(self) -> None:
        """Enable asynchronous image I/O after loading an existing dataset.

        ``LeRobotDataset.create(..., image_writer_threads=N)`` starts this
        automatically, while ``LeRobotDataset(repo_id=..., root=...)`` does
        not. Without restarting it, four PNG files are written synchronously
        for every sample and recording falls from 30 Hz to roughly 6 Hz.
        """
        assert self.dataset is not None
        self.dataset.start_image_writer(
            num_processes=0,
            num_threads=self.IMAGE_WRITER_THREADS,
        )
        print(
            "[Record] LeRobot asynchronous image writer enabled "
            f"({self.IMAGE_WRITER_THREADS} threads)"
        )

    @staticmethod
    def _load_api():
        try:
            from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset
        except ImportError as exc:
            raise RuntimeError(
                "LeRobot v2.1 dependencies are missing. Run `bash setup.sh --skip-apt --skip-xrt` "
                "inside the r1_teleop environment."
            ) from exc
        return LeRobotDataset, CODEBASE_VERSION

    def _features(self, frames: tuple[np.ndarray, ...]) -> dict:
        if len(frames) != len(self.CAMERA_KEYS):
            raise ValueError(f"Expected four camera frames, got {len(frames)}")
        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (len(self.STATE_NAMES),),
                "names": list(self.STATE_NAMES),
            },
            "action": {
                "dtype": "float32",
                "shape": (len(self.STATE_NAMES),),
                "names": list(self.STATE_NAMES),
            },
            "source_frame_id": {
                "dtype": "int64",
                "shape": (1,),
                "names": ["source_frame_id"],
            },
        }
        for key, frame in zip(self.CAMERA_KEYS, frames):
            if frame.ndim != 3 or frame.shape[2] != 3:
                raise ValueError(f"{key} must be an HWC three-channel image, got {frame.shape}")
            features[key] = {
                "dtype": "video",
                "shape": tuple(int(value) for value in frame.shape),
                "names": ["height", "width", "channels"],
            }
        return features

    def _ensure_dataset(self, frames: tuple[np.ndarray, ...]) -> None:
        if self.dataset is None:
            # LeRobotDataset.create requires the dataset root not to exist.
            if self.root.exists():
                self.root.rmdir()
            self.dataset = self._LeRobotDataset.create(
                repo_id=self.repo_id,
                fps=self.frequency,
                features=self._features(frames),
                root=self.root,
                robot_type="r1_a7_dex1",
                use_videos=True,
                image_writer_threads=self.IMAGE_WRITER_THREADS,
            )
            print(
                "[Record] LeRobot asynchronous image writer enabled "
                f"({self.IMAGE_WRITER_THREADS} threads)"
            )
            self.episode_id = -1
        self._validate_frame_shapes(frames)

    def _validate_existing_dataset(self) -> None:
        assert self.dataset is not None
        if int(self.dataset.fps) != self.frequency:
            raise ValueError(
                f"Existing dataset fps is {self.dataset.fps}, requested {self.frequency}"
            )
        required = {"observation.state", "action", "source_frame_id", *self.CAMERA_KEYS}
        missing = required.difference(self.dataset.features)
        if missing:
            raise ValueError(f"Existing LeRobot dataset is missing features: {sorted(missing)}")
        for key in ("observation.state", "action"):
            feature = self.dataset.features[key]
            shape = tuple(feature["shape"])
            if shape != (len(self.STATE_NAMES),):
                raise ValueError(
                    f"Existing feature {key} has incompatible shape {shape}; "
                    f"the compact R1-A7/Dex1 schema requires ({len(self.STATE_NAMES)},). "
                    "Use a new --task-dir-name or archive the old 22-DoF dataset "
                    "before recording."
                )
            names = feature.get("names")
            if names is not None and list(names) != list(self.STATE_NAMES):
                raise ValueError(f"Existing feature {key} has incompatible joint names {names}")

    def _validate_frame_shapes(self, frames: tuple[np.ndarray, ...]) -> None:
        assert self.dataset is not None
        if len(frames) != len(self.CAMERA_KEYS):
            raise ValueError(f"Expected four camera frames, got {len(frames)}")
        for key, frame in zip(self.CAMERA_KEYS, frames):
            expected = tuple(self.dataset.features[key]["shape"])
            if tuple(frame.shape) != expected:
                raise ValueError(f"{key} shape changed from {expected} to {tuple(frame.shape)}")

    def start_episode(self, metadata: dict, frames: tuple[np.ndarray, ...]) -> str:
        """Start buffering a new episode and return its future parquet path."""
        del metadata  # Dataset v2.1 has no per-episode arbitrary metadata field.
        if self._active:
            raise RuntimeError("A LeRobot episode is already active")
        self._remove_backup()
        self._ensure_dataset(frames)
        assert self.dataset is not None
        self.episode_id = int(self.dataset.meta.total_episodes)
        self.item_id = -1
        relative = self.dataset.meta.get_data_file_path(self.episode_id)
        self.current_path = str(self.root / relative)
        self._active = True
        return self.current_path

    @staticmethod
    def _vector(data: dict, key: str) -> np.ndarray:
        vector = np.asarray(data[key], dtype=np.float32).reshape(-1)
        if vector.shape == (FourCameraLeRobotV21Recorder.LEGACY_VECTOR_SIZE,):
            vector = vector[list(FourCameraLeRobotV21Recorder.LEGACY_TO_COMPACT)]
        expected = len(FourCameraLeRobotV21Recorder.STATE_NAMES)
        if vector.shape != (expected,):
            raise ValueError(
                f"{key} must have {expected} compact values or "
                f"{FourCameraLeRobotV21Recorder.LEGACY_VECTOR_SIZE} legacy values, "
                f"got {vector.shape}"
            )
        return vector

    def add_frame(
        self,
        frames: tuple[np.ndarray, ...],
        states: dict,
        actions: dict,
        image_source_frame_id: int,
    ) -> None:
        """Append one synchronized state/action/four-camera sample."""
        if not self._active or self.dataset is None:
            return
        self._validate_frame_shapes(frames)
        frame = {
            "observation.state": self._vector(states, "robot_q_current"),
            "action": self._vector(actions, "robot_q_desired"),
            "source_frame_id": np.asarray([image_source_frame_id], dtype=np.int64),
        }
        for key, image in zip(self.CAMERA_KEYS, frames):
            frame[key] = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.dataset.add_frame(frame, task=self.task_goal)
        self.item_id += 1

    def _remove_backup(self) -> None:
        if self._delete_backup.is_dir():
            shutil.rmtree(self._delete_backup)

    def _snapshot_metadata(self) -> None:
        self._remove_backup()
        meta = self.root / "meta"
        if meta.is_dir():
            shutil.copytree(meta, self._delete_backup)

    def _episode_files(self, episode_id: int) -> list[Path]:
        assert self.dataset is not None
        files = [self.root / self.dataset.meta.get_data_file_path(episode_id)]
        files.extend(
            self.root / self.dataset.meta.get_video_file_path(episode_id, key)
            for key in self.CAMERA_KEYS
        )
        return files

    def _restore_metadata(self) -> None:
        if not self._delete_backup.is_dir():
            raise RuntimeError("No metadata backup is available")
        previous_info = json.loads((self._delete_backup / "info.json").read_text(encoding="utf-8"))
        if self.dataset is not None:
            self.dataset.stop_image_writer()
        if int(previous_info["total_episodes"]) == 0:
            # LeRobot v2.1 cannot construct an empty on-disk dataset: its
            # loader mistakes the absence of parquet files for a missing local
            # dataset and contacts the Hub. Recreate the original empty writer
            # state directly instead.
            if self.root.is_dir():
                shutil.rmtree(self.root)
            features = {}
            for key, value in previous_info["features"].items():
                if key in {"timestamp", "frame_index", "episode_index", "index", "task_index"}:
                    continue
                value = dict(value)
                value["shape"] = tuple(value["shape"])
                features[key] = value
            self.dataset = self._LeRobotDataset.create(
                repo_id=self.repo_id,
                fps=int(previous_info["fps"]),
                features=features,
                root=self.root,
                robot_type=previous_info.get("robot_type"),
                use_videos=True,
                image_writer_threads=self.IMAGE_WRITER_THREADS,
            )
            print(
                "[Record] LeRobot asynchronous image writer enabled "
                f"({self.IMAGE_WRITER_THREADS} threads)"
            )
            return
        meta = self.root / "meta"
        if meta.is_dir():
            shutil.rmtree(meta)
        shutil.copytree(self._delete_backup, meta)
        self.dataset = self._LeRobotDataset(repo_id=self.repo_id, root=self.root)
        self._start_image_writer()

    def stop_episode(self) -> None:
        """Encode videos, write parquet/metadata and finalize the active episode."""
        if not self._active or self.dataset is None:
            return
        if self.item_id < 0:
            self.dataset.clear_episode_buffer()
            self._active = False
            raise RuntimeError("Cannot save an empty LeRobot episode")

        episode_id = self.episode_id
        self._snapshot_metadata()
        try:
            self.dataset.save_episode()
        except Exception:
            for path in self._episode_files(episode_id):
                path.unlink(missing_ok=True)
            self._restore_metadata()
            self._remove_backup()
            self._active = False
            raise
        self._active = False

    def delete_last_episode(self):
        """Delete the just-saved episode and restore the previous v2.1 metadata."""
        if self._active or self.dataset is None or not self._delete_backup.is_dir():
            return None
        episode_id = self.episode_id
        episode_path = self.current_path
        for path in self._episode_files(episode_id):
            path.unlink(missing_ok=True)
        self._restore_metadata()
        self._remove_backup()
        self.episode_id = int(self.dataset.meta.total_episodes) - 1
        self.current_path = None
        return episode_id, episode_path

    def close(self) -> None:
        """Stop background image writers and discard the temporary delete snapshot."""
        if self.dataset is not None:
            self.dataset.stop_image_writer()
        self._remove_backup()


def read_codebase_version(dataset_root: str | Path) -> str:
    """Small verification helper used by smoke tests and operators."""
    info_path = Path(dataset_root) / "meta" / "info.json"
    return str(json.loads(info_path.read_text(encoding="utf-8"))["codebase_version"])
