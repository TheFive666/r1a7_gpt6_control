#!/usr/bin/env python3
"""Inspect and preview the live R1-A7 four-camera ZMQ stream.

The packet rate can be higher than a camera's real source rate when the image
bridge republishes a cached frame. This tool therefore reports both the packet
rate and the content-change rate of every logical camera.

Press ``q`` or Escape in the preview window to exit.
"""

from __future__ import annotations

import argparse
import hashlib
import sys
import time
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from teleop.image_client import CAMERA_NAMES, FourCameraSubscriber  # noqa: E402


WINDOW_NAME = "R1-A7 camera stream (q/Esc to quit)"


class PreviewError(RuntimeError):
    """Raised when no requested GUI preview backend can be initialized."""


def opencv_gui_available() -> bool:
    for line in cv2.getBuildInformation().splitlines():
        if line.strip().startswith("GUI:"):
            return line.split(":", 1)[1].strip().upper() != "NONE"
    return False


class PreviewWindow:
    """Display BGR images through OpenCV or a Matplotlib fallback."""

    def __init__(self, requested_backend: str) -> None:
        self.backend = requested_backend
        self.closed = False
        self.figure = None
        self.axes = None
        self.image_artist = None
        self.plt = None

        if self.backend == "auto":
            self.backend = "opencv" if opencv_gui_available() else "matplotlib"
        if self.backend == "opencv" and not opencv_gui_available():
            raise PreviewError(
                "OpenCV was built without GUI support; use "
                "--preview-backend matplotlib or --no-show"
            )
        if self.backend == "matplotlib":
            try:
                import matplotlib.pyplot as plt
            except Exception as exc:
                raise PreviewError(f"failed to import Matplotlib: {exc}") from exc
            self.plt = plt
            self.plt.ion()
        print(f"[Check] preview backend: {self.backend}")

    def _on_key(self, event) -> None:
        if event.key in {"q", "escape"}:
            self.closed = True

    def _on_close(self, _event) -> None:
        self.closed = True

    def show(self, bgr: np.ndarray) -> bool:
        """Show one image and return True when the user requested exit."""
        if self.backend == "opencv":
            try:
                cv2.imshow(WINDOW_NAME, bgr)
                return (cv2.waitKey(1) & 0xFF) in (27, ord("q"))
            except cv2.error as exc:
                raise PreviewError(f"OpenCV preview failed: {exc}") from exc

        assert self.plt is not None
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        try:
            if self.figure is None:
                self.figure, self.axes = self.plt.subplots(num=WINDOW_NAME)
                self.axes.axis("off")
                self.image_artist = self.axes.imshow(rgb)
                self.figure.canvas.mpl_connect("key_press_event", self._on_key)
                self.figure.canvas.mpl_connect("close_event", self._on_close)
                self.plt.show(block=False)
            else:
                self.image_artist.set_data(rgb)
                self.figure.canvas.draw_idle()
            self.plt.pause(0.001)
            return self.closed or not self.plt.fignum_exists(self.figure.number)
        except Exception as exc:
            raise PreviewError(f"Matplotlib preview failed: {exc}") from exc

    def poll(self) -> bool:
        """Process window events while waiting for a new camera packet."""
        if self.backend == "opencv":
            try:
                return (cv2.waitKey(1) & 0xFF) in (27, ord("q"))
            except cv2.error as exc:
                raise PreviewError(f"OpenCV preview failed: {exc}") from exc
        assert self.plt is not None
        try:
            self.plt.pause(0.001)
            return self.closed or (
                self.figure is not None and not self.plt.fignum_exists(self.figure.number)
            )
        except Exception as exc:
            raise PreviewError(f"Matplotlib preview failed: {exc}") from exc

    def close(self) -> None:
        if self.backend == "opencv":
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        elif self.plt is not None and self.figure is not None:
            self.plt.close(self.figure)


def frame_signature(frame: np.ndarray) -> bytes:
    """Return a compact exact-content signature for duplicate detection."""
    return hashlib.blake2b(frame.tobytes(), digest_size=8).digest()


def fit_frame(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """Resize with preserved aspect ratio and letterbox to a fixed cell."""
    source_height, source_width = frame.shape[:2]
    scale = min(width / source_width, height / source_height)
    resized_width = max(1, int(round(source_width * scale)))
    resized_height = max(1, int(round(source_height * scale)))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(frame, (resized_width, resized_height), interpolation=interpolation)
    cell = np.zeros((height, width, 3), dtype=np.uint8)
    x = (width - resized_width) // 2
    y = (height - resized_height) // 2
    cell[y : y + resized_height, x : x + resized_width] = resized
    return cell


def label_cell(cell: np.ndarray, name: str, shape: tuple[int, ...], fps: float) -> None:
    text = f"{name}  {shape[1]}x{shape[0]}  effective {fps:.1f} Hz"
    cv2.rectangle(cell, (0, 0), (cell.shape[1], 34), (0, 0, 0), -1)
    cv2.putText(
        cell,
        text,
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )


def make_preview(
    frames: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray],
    effective_fps: dict[str, float],
    cell_width: int,
    cell_height: int,
) -> np.ndarray:
    cells = []
    for name, frame in zip(CAMERA_NAMES, frames):
        cell = fit_frame(frame, cell_width, cell_height)
        label_cell(cell, name, frame.shape, effective_fps[name])
        cells.append(cell)
    return np.vstack((np.hstack(cells[:2]), np.hstack(cells[2:])))


def inspect(args: argparse.Namespace) -> int:
    subscriber = FourCameraSubscriber(args.address, args.port)
    preview_window = None
    try:
        subscriber.start()
        if not args.no_show:
            preview_window = PreviewWindow(args.preview_backend)
        if not subscriber.wait_first_frame(args.wait_timeout):
            print(
                f"[Check] no image packet received from {subscriber.connect} "
                f"within {args.wait_timeout:.1f}s"
            )
            print(
                "[Check] the bridge publishes only after all required streams are ready; "
                "check the PC2 head_left/head_right counters first."
            )
            return 2

        if preview_window is None:
            print("[Check] stream received; measuring without a preview window")
        else:
            print("[Check] stream received; press q or Escape to exit the preview")
        started = time.monotonic()
        stats_started = started
        last_frame_id = -1
        packet_count = 0
        window_packets = 0
        total_changes = {name: 0 for name in CAMERA_NAMES}
        window_changes = {name: 0 for name in CAMERA_NAMES}
        previous_signatures: dict[str, bytes | None] = {name: None for name in CAMERA_NAMES}
        effective_fps = {name: 0.0 for name in CAMERA_NAMES}

        while True:
            now = time.monotonic()
            if args.duration > 0 and now - started >= args.duration:
                break

            frames, frame_id, _received_at_ns = subscriber.get_if_new(last_frame_id)
            if frames is None:
                if preview_window is not None and preview_window.poll():
                    break
                time.sleep(0.002)
                continue

            last_frame_id = frame_id
            packet_count += 1
            window_packets += 1

            for name, frame in zip(CAMERA_NAMES, frames):
                signature = frame_signature(frame)
                if signature != previous_signatures[name]:
                    total_changes[name] += 1
                    window_changes[name] += 1
                    previous_signatures[name] = signature

            now = time.monotonic()
            stats_elapsed = now - stats_started
            if stats_elapsed >= args.stats_period:
                packet_fps = window_packets / stats_elapsed
                for name in CAMERA_NAMES:
                    effective_fps[name] = window_changes[name] / stats_elapsed
                rates = " ".join(
                    f"{name}={effective_fps[name]:.1f}" for name in CAMERA_NAMES
                )
                print(f"[Check] packet={packet_fps:.1f} Hz | effective: {rates}")
                stats_started = now
                window_packets = 0
                window_changes = {name: 0 for name in CAMERA_NAMES}

            if preview_window is not None:
                preview = make_preview(
                    frames,
                    effective_fps,
                    args.cell_width,
                    args.cell_height,
                )
                if preview_window.show(preview):
                    break

        elapsed = max(time.monotonic() - started, 1e-6)
        print(f"[Check] total packets={packet_count}, average packet rate={packet_count / elapsed:.2f} Hz")
        for name in CAMERA_NAMES:
            print(
                f"[Check] {name}: content changes={total_changes[name]}, "
                f"average effective rate={total_changes[name] / elapsed:.2f} Hz"
            )
        return 0
    except PreviewError as exc:
        print(f"[Check] preview failed: {exc}", file=sys.stderr)
        print("[Check] use --no-show when running without a desktop display", file=sys.stderr)
        return 3
    finally:
        subscriber.close()
        if preview_window is not None:
            preview_window.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--address", default="192.168.123.164", help="PC2 image-server address")
    parser.add_argument("--port", type=int, default=5555, help="PC2 ZMQ image-server port")
    parser.add_argument("--wait-timeout", type=float, default=5.0)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="measurement duration in seconds; 0 runs until q/Escape/Ctrl-C",
    )
    parser.add_argument("--stats-period", type=float, default=2.0)
    parser.add_argument("--cell-width", type=int, default=480)
    parser.add_argument("--cell-height", type=int, default=360)
    parser.add_argument("--no-show", action="store_true", help="measure FPS without opening a window")
    parser.add_argument(
        "--preview-backend",
        choices=("auto", "opencv", "matplotlib"),
        default="auto",
        help="preview GUI backend; auto falls back to Matplotlib when OpenCV is headless",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    try:
        raise SystemExit(inspect(args))
    except KeyboardInterrupt:
        print("\n[Check] stopped")


if __name__ == "__main__":
    main()
