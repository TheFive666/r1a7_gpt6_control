"""Lifecycle-owned recording for a non-policy USB overview camera."""
from __future__ import annotations

import json
import subprocess
import time
from pathlib import Path

from .client import SDKError


class ExternalVideoRecorder:
    """Record /dev/video0 without exposing frames to the policy."""

    def __init__(self, output_dir: str | Path, *, device: str = "/dev/video0",
                 fps: float = 5.0, size: str = "1920x1080"):
        if fps <= 0 or fps > 15:
            raise SDKError("external camera fps must be within (0, 15]")
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.device = device
        self.fps = float(fps)
        self.size = size
        self.path = self.output_dir / "external_camera.mp4"
        self.log_path = self.output_dir / "external_camera_ffmpeg.log"
        self.metadata_path = self.output_dir / "external_camera.json"
        self._process: subprocess.Popen | None = None
        self._log = None
        self._started_ns: int | None = None

    def start(self) -> Path:
        if self._process is not None:
            return self.path
        if not Path(self.device).exists():
            raise SDKError(f"external camera does not exist: {self.device}")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._log = self.log_path.open("w")
        command = [
            "ffmpeg", "-hide_banner", "-loglevel", "warning", "-y",
            "-f", "v4l2", "-input_format", "mjpeg", "-video_size", self.size,
            "-framerate", "30", "-i", self.device,
            "-vf", f"fps={self.fps:g}", "-an", "-c:v", "libx264",
            "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p",
            "-movflags", "+faststart", str(self.path),
        ]
        try:
            self._process = subprocess.Popen(
                command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                stderr=self._log, text=True)
        except (OSError, ValueError) as exc:
            self._close_log()
            raise SDKError(f"cannot start external camera recorder: {exc}") from None
        time.sleep(0.35)
        if self._process.poll() is not None:
            code = self._process.returncode
            self._process = None
            self._close_log()
            detail = self.log_path.read_text(errors="replace")[-500:]
            raise SDKError(f"external camera recorder exited ({code}): {detail}")
        self._started_ns = time.time_ns()
        self._write_metadata(stopped_ns=None, returncode=None)
        return self.path

    def stop(self) -> Path | None:
        process = self._process
        if process is None:
            return self.path if self.path.exists() else None
        if process.poll() is None:
            try:
                assert process.stdin is not None
                process.stdin.write("q\n")
                process.stdin.flush()
                process.wait(timeout=8)
            except (BrokenPipeError, subprocess.TimeoutExpired):
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
        returncode = process.returncode
        self._process = None
        self._close_log()
        self._write_metadata(stopped_ns=time.time_ns(), returncode=returncode)
        if returncode not in (0, 255):
            raise SDKError(f"external camera recorder failed with exit code {returncode}")
        return self.path

    def _close_log(self) -> None:
        if self._log is not None:
            self._log.close()
            self._log = None

    def _write_metadata(self, *, stopped_ns: int | None, returncode: int | None) -> None:
        self.metadata_path.write_text(json.dumps({
            "device": self.device, "policy_input": False, "output": str(self.path),
            "fps": self.fps, "size": self.size, "started_ns": self._started_ns,
            "stopped_ns": stopped_ns, "returncode": returncode,
        }, indent=2) + "\n")

    def __enter__(self) -> "ExternalVideoRecorder":
        self.start()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.stop()
