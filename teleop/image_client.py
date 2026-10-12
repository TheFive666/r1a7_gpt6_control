"""Four-camera image subscriber for the R1-A7 image service.

Migrated from ``record/r1_a7_record.py`` in the original ``xr_teleoperate_r1a7``
repository. Receives the PC2 Dex1 wire format over ZMQ and splits it into the
four camera views used by the recorder.

Both deployed wire formats are accepted:

* Legacy PC2 Dex1: ``struct("II")`` header followed by one JPEG containing
  all four views concatenated horizontally and an optional ignored IR payload.
* R1 image_server multipart: topic, JSON metadata, then one JPEG per name in
  ``metadata["streams"]``. A single ``head`` stream is duplicated to the two
  logical head-camera slots expected by the recorder.

The split is fixed as: head_left, head_right, wrist_left, wrist_right.
"""

from __future__ import annotations

import json
import struct
import threading
import time
from typing import Optional

import cv2
import numpy as np
import zmq


CAMERA_NAMES = ("head_left", "head_right", "wrist_left", "wrist_right")


class FourCameraSubscriber:
    """Receive either supported image-server protocol as four camera views."""

    def __init__(self, address: str, port: int) -> None:
        self.connect = f"tcp://{address}:{int(port)}"
        self.lock = threading.Lock()
        self.frames: Optional[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = None
        self.frame_id = 0
        self.received_at_ns = 0
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.context: Optional[zmq.Context] = None
        self.socket: Optional[zmq.Socket] = None

    def start(self) -> None:
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")
        # Keep only the newest frames: recording wants live images, not a backlog.
        self.socket.setsockopt(zmq.RCVHWM, 2)
        self.socket.setsockopt(zmq.RCVTIMEO, 500)
        self.socket.connect(self.connect)
        self.thread = threading.Thread(target=self._loop, name="r1-a7-four-camera-sub", daemon=True)
        self.thread.start()
        print(f"[Image] receiving camera stream from {self.connect}")

    @staticmethod
    def _decode(message: bytes) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Decode the legacy single-message PC2 Dex1 wire format."""
        header_size = struct.calcsize("II")
        if len(message) < header_size:
            raise ValueError("message is shorter than the Dex1 image header")
        color_size, ir_size = struct.unpack("II", message[:header_size])
        color_end = header_size + color_size
        if color_size <= 0 or color_end > len(message):
            raise ValueError("invalid JPEG size in Dex1 image header")
        if ir_size and color_end + ir_size > len(message):
            raise ValueError("invalid IR size in Dex1 image header")

        combined = cv2.imdecode(np.frombuffer(message[header_size:color_end], dtype=np.uint8), cv2.IMREAD_COLOR)
        if combined is None:
            raise ValueError("failed to decode PC2 JPEG")
        if combined.shape[1] % 4:
            raise ValueError(f"combined image width {combined.shape[1]} cannot be divided into four cameras")
        head_left, head_right, wrist_left, wrist_right = np.hsplit(combined, 4)
        return head_left, head_right, wrist_left, wrist_right

    @staticmethod
    def _decode_jpeg(payload: bytes, stream: str) -> np.ndarray:
        frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError(f"failed to decode JPEG stream {stream!r}")
        return frame

    @classmethod
    def _decode_parts(
        cls, parts: list[bytes]
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Decode either a legacy message or the newer multipart protocol."""
        if len(parts) == 1:
            return cls._decode(parts[0])
        if len(parts) < 3:
            raise ValueError(f"multipart image message has only {len(parts)} parts")

        try:
            meta = json.loads(parts[1].decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError, TypeError) as exc:
            raise ValueError("invalid multipart image metadata") from exc
        streams = meta.get("streams")
        if not isinstance(streams, list) or not all(isinstance(name, str) for name in streams):
            raise ValueError("multipart image metadata has no valid streams list")
        payloads = parts[2:]
        if len(streams) != len(payloads):
            raise ValueError(
                f"multipart stream/payload count differs: {len(streams)} != {len(payloads)}"
            )

        decoded = {
            name: cls._decode_jpeg(payload, name)
            for name, payload in zip(streams, payloads)
        }
        if "head" in decoded:
            # /frontvideostream is a monocular head stream. Preserve the
            # recorder's stable four-camera schema by exposing it in both
            # logical head slots.
            decoded.setdefault("head_left", decoded["head"])
            decoded.setdefault("head_right", decoded["head"])

        missing = [name for name in CAMERA_NAMES if name not in decoded]
        if missing:
            raise ValueError(f"multipart image message is missing streams: {', '.join(missing)}")
        return tuple(decoded[name] for name in CAMERA_NAMES)  # type: ignore[return-value]

    def _loop(self) -> None:
        assert self.socket is not None
        last_error = ""
        while not self.stop_event.is_set():
            try:
                parts = self.socket.recv_multipart()
                frames = self._decode_parts(parts)
            except zmq.Again:
                continue
            except zmq.ZMQError:
                if self.stop_event.is_set():
                    break
                continue
            except Exception as exc:
                # A malformed stream would otherwise flood the console at frame
                # rate; print each distinct error only once.
                error = str(exc)
                if error != last_error:
                    print(f"[Image] skipped malformed frame: {error}")
                    last_error = error
                continue
            with self.lock:
                self.frames = frames
                self.frame_id += 1
                self.received_at_ns = time.time_ns()

    def wait_first_frame(self, timeout: float) -> bool:
        """Block until the first frame arrives or ``timeout`` seconds elapse."""
        deadline = time.monotonic() + max(float(timeout), 0.0)
        while time.monotonic() < deadline:
            with self.lock:
                if self.frames is not None:
                    return True
            time.sleep(0.05)
        with self.lock:
            return self.frames is not None

    def get(self) -> tuple[Optional[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]], int, int]:
        """Return ``(frames, frame_id, received_at_ns)``; frames is None if idle."""
        with self.lock:
            if self.frames is None:
                return None, self.frame_id, self.received_at_ns
            return tuple(frame.copy() for frame in self.frames), self.frame_id, self.received_at_ns

    def get_if_new(
        self, last_frame_id: int
    ) -> tuple[Optional[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]], int, int]:
        """Return copied frames only when a packet newer than ``last_frame_id`` exists."""
        with self.lock:
            if self.frames is None or self.frame_id == last_frame_id:
                return None, self.frame_id, self.received_at_ns
            return tuple(frame.copy() for frame in self.frames), self.frame_id, self.received_at_ns

    def close(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.socket is not None:
            self.socket.close(0)
            self.socket = None
        if self.context is not None:
            self.context.term()
            self.context = None
