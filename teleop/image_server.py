#!/usr/bin/env python3
"""Bridge four R1 camera streams from PC2 to the operator PC.

R1 network layout used by this script:
  PC1 192.168.123.161: sends the official RTP/JPEG head streams
  PC2 192.168.123.164: receives head UDP plus two local wrist cameras
  User PC 192.168.123.120: receives all four images over ZMQ

The upgraded robot defaults to UDP 5004/5005, RTP/JPEG payload type 26.
The legacy UDP 5002/5003 RTP/H264 relay remains available through CLI options.
"""

from __future__ import annotations

import argparse
import json
import queue
import signal
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import BinaryIO, Optional

import cv2
import numpy as np

try:
    import zmq
except ImportError as exc:
    raise SystemExit("Missing pyzmq: python -m pip install pyzmq") from exc


DEFAULT_TOPIC = "r1a7/head_stereo"


def read_exact(stream: BinaryIO, size: int) -> Optional[bytes]:
    chunks: list[bytes] = []
    remaining = size
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def gst_pipeline(port: int, args: argparse.Namespace) -> list[str]:
    encoding_name = "JPEG" if args.head_codec == "jpeg" else "H264"
    caps = (
        "application/x-rtp,"
        "media=video,"
        f"clock-rate={args.clock_rate},"
        f"encoding-name={encoding_name},"
        f"payload={args.payload}"
    )
    command = [
        args.gst_bin,
        "-q",
        "udpsrc",
        "address=0.0.0.0",
        f"port={port}",
        "!",
        caps,
    ]
    if args.latency > 0:
        command.extend(
            [
                "!",
                "rtpjitterbuffer",
                f"latency={args.latency}",
                "drop-on-latency=true",
            ]
        )
    if args.head_codec == "jpeg":
        command.extend(["!", "rtpjpegdepay", "!", "jpegparse", "!", "jpegdec"])
    else:
        command.extend(["!", "rtph264depay", "!", "avdec_h264"])
    command.extend(
        [
            "!",
            "videoconvert",
            "!",
            "videoscale",
            "!",
            f"video/x-raw,format=BGR,width={args.width},height={args.height}",
            "!",
            "fdsink",
            "fd=1",
            "sync=false",
        ]
    )
    return command


class RtpReader:
    def __init__(self, name: str, port: int, args: argparse.Namespace):
        self.name = name
        self.port = port
        self.width = args.width
        self.height = args.height
        self.codec = args.head_codec.upper()
        self.frame_size = self.width * self.height * 3
        self.command = gst_pipeline(port, args)
        self.process: Optional[subprocess.Popen[bytes]] = None
        self.frame: Optional[np.ndarray] = None
        self.sequence = 0
        self.error_lines: list[str] = []
        self.lock = threading.Lock()
        self.stopping = threading.Event()

    def start(self) -> None:
        print(f"[{self.name}] listening for RTP/{self.codec} on UDP {self.port}")
        print(f"[{self.name}] {' '.join(self.command)}")
        try:
            self.process = subprocess.Popen(
                self.command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except FileNotFoundError as exc:
            raise RuntimeError(f"GStreamer executable not found: {self.command[0]}") from exc
        threading.Thread(target=self._read_frames, name=f"{self.name}-frames", daemon=True).start()
        threading.Thread(target=self._read_errors, name=f"{self.name}-stderr", daemon=True).start()

    def _read_frames(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        while not self.stopping.is_set():
            raw = read_exact(self.process.stdout, self.frame_size)
            if raw is None:
                return
            frame = np.frombuffer(raw, dtype=np.uint8).reshape(self.height, self.width, 3).copy()
            with self.lock:
                self.frame = frame
                self.sequence += 1

    def _read_errors(self) -> None:
        assert self.process is not None and self.process.stderr is not None
        for raw in iter(self.process.stderr.readline, b""):
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            self.error_lines.append(line)
            self.error_lines = self.error_lines[-10:]
            print(f"[{self.name} gstreamer] {line}", file=sys.stderr)

    def peek_sequence(self) -> int:
        with self.lock:
            return self.sequence

    def latest(self, copy: bool = True) -> tuple[int, Optional[np.ndarray]]:
        with self.lock:
            if self.frame is None:
                return self.sequence, None
            frame = self.frame.copy() if copy else self.frame
            return self.sequence, frame

    def check_alive(self) -> None:
        if self.process is None:
            raise RuntimeError(f"{self.name} GStreamer was not started")
        code = self.process.poll()
        if code is not None and not self.stopping.is_set():
            detail = self.error_lines[-1] if self.error_lines else "no stderr output"
            raise RuntimeError(f"{self.name} GStreamer exited with code {code}: {detail}")

    def close(self) -> None:
        self.stopping.set()
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()


def parse_camera(value: str) -> int | str:
    value = value.strip()
    return int(value) if value.isdigit() else value


def _v4l2_index_from_path(path: str) -> Optional[int]:
    """Map ``/dev/videoN`` (or a symlink resolving to it) to device number N."""
    try:
        resolved = str(Path(path).resolve())
    except OSError:
        resolved = path
    name = Path(resolved).name
    if name.startswith("video") and name[5:].isdigit():
        return int(name[5:])
    return None


def _sysfs_video_attr(video_name: str, attr: str) -> Optional[str]:
    path = Path("/sys/class/video4linux") / video_name / attr
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return None


def _same_usb_interface_siblings(video_idx: int) -> list[int]:
    """Return videoN indices that share the same USB interface as ``video_idx``."""
    link = Path("/sys/class/video4linux") / f"video{video_idx}"
    try:
        resolved = link.resolve()
    except OSError:
        return [video_idx]
    # .../1-2.2:1.0/video4linux/video1 -> parent interface dir
    interface_dir = resolved.parent.parent
    siblings: list[int] = []
    try:
        for child in interface_dir.joinpath("video4linux").iterdir():
            name = child.name
            if name.startswith("video") and name[5:].isdigit():
                siblings.append(int(name[5:]))
    except OSError:
        return [video_idx]
    return sorted(set(siblings)) or [video_idx]


def _is_capture_node(video_idx: int) -> bool:
    """Best-effort: UVC capture nodes usually have sysfs index 0."""
    idx_attr = _sysfs_video_attr(f"video{video_idx}", "index")
    if idx_attr is not None:
        return idx_attr == "0"
    # Fall back to v4l2-ctl if available.
    try:
        out = subprocess.run(
            ["v4l2-ctl", f"--device=/dev/video{video_idx}", "--all"],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        text = (out.stdout or "") + (out.stderr or "")
        return "Video Capture" in text
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        return True


def _candidate_video_devices(target: int | str) -> list[str]:
    """Prefer real capture nodes over UVC metadata siblings.

    udev symlinks like ``/dev/cam_wrist_left -> video1`` often point at
    ``ATTR{index}==1`` (metadata). Capture is usually ``video0`` on the same
    USB interface (``ATTR{index}==0``).
    """
    devices: list[str] = []
    if isinstance(target, int):
        seed = target
        devices.append(f"/dev/video{seed}")
    else:
        target_s = str(target)
        devices.append(target_s)
        seed = _v4l2_index_from_path(target_s)
        if seed is not None:
            devices.append(f"/dev/video{seed}")
        else:
            return devices

    siblings = _same_usb_interface_siblings(seed)
    capture = [i for i in siblings if _is_capture_node(i)]
    # Prefer index-0 capture nodes first, then other siblings, then seed.
    ordered: list[int] = []
    for i in capture:
        if i not in ordered:
            ordered.append(i)
    for i in siblings:
        if i not in ordered:
            ordered.append(i)
    if seed not in ordered:
        ordered.append(seed)

    for i in ordered:
        path = f"/dev/video{i}"
        if path not in devices:
            devices.append(path)
    return devices


def _try_open_capture(
    device: str | int,
    width: int,
    height: int,
    fps: int,
    fourcc: str,
) -> Optional[tuple[cv2.VideoCapture, str, np.ndarray]]:
    """Try GStreamer v4l2src first, then OpenCV CAP_V4L2. No obsensor fallback."""
    device_str = f"/dev/video{device}" if isinstance(device, int) else str(device)
    pipelines: list[str] = []
    if fourcc and fourcc.upper() == "MJPG":
        pipelines.append(
            f"v4l2src device={device_str} ! "
            f"image/jpeg,width={width},height={height},framerate={int(fps)}/1 ! "
            f"jpegdec ! videoconvert ! video/x-raw,format=BGR ! "
            f"appsink drop=1 max-buffers=1 sync=false"
        )
        pipelines.append(
            f"v4l2src device={device_str} ! image/jpeg ! jpegdec ! videoconvert ! "
            f"video/x-raw,format=BGR ! appsink drop=1 max-buffers=1 sync=false"
        )
    pipelines.append(
        f"v4l2src device={device_str} ! videoconvert ! "
        f"video/x-raw,format=BGR,width={width},height={height} ! "
        f"appsink drop=1 max-buffers=1 sync=false"
    )
    pipelines.append(
        f"v4l2src device={device_str} ! videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=1 max-buffers=1 sync=false"
    )

    for pipeline in pipelines:
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            cap.release()
            continue
        ok, frame = cap.read()
        if ok and frame is not None:
            return cap, f"gst:{device_str}", frame
        cap.release()

    # Strict V4L2 path open (avoid default backend / obsensor false opens).
    cap = cv2.VideoCapture(device_str, cv2.CAP_V4L2)
    if cap.isOpened():
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc[:4]))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        ok, frame = cap.read()
        if ok and frame is not None:
            return cap, device_str, frame
    cap.release()
    return None


def _open_v4l2_capture(
    target: int | str,
    width: int,
    height: int,
    fps: int,
    fourcc: str,
) -> tuple[cv2.VideoCapture, str, np.ndarray]:
    """Open a real capture node for ``target``, probing USB siblings if needed."""
    tried: list[str] = []
    for device in _candidate_video_devices(target):
        tried.append(str(device))
        result = _try_open_capture(device, width, height, fps, fourcc)
        if result is not None:
            return result

    raise RuntimeError(
        f"Cannot open camera {target!r}. Tried: {tried}. "
        "UVC cams expose multiple /dev/video*; ensure udev symlink uses "
        'ATTR{index}=="0" (capture), not metadata. '
        "Debug: v4l2-ctl --list-devices && v4l2-ctl -d /dev/videoX --all"
    )


class OpenCvReader:
    """Continuously read a V4L2 camera while the head RTP loop runs."""

    def __init__(
        self,
        name: str,
        target: int | str,
        width: int,
        height: int,
        fps: int,
        fourcc: str,
        exposure: float | None = None,
        gain: float | None = None,
    ) -> None:
        self.name = name
        self.target = target
        try:
            self.cap, self.open_target, frame = _open_v4l2_capture(
                target, width, height, fps, fourcc
            )
        except RuntimeError as exc:
            raise RuntimeError(f"Cannot open {name} camera {target!r}: {exc}") from exc
        # Exposure/gain only apply to native V4L2 opens (not gst pipelines).
        if not str(self.open_target).startswith("gst:"):
            if exposure is not None:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                self.cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure))
            if gain is not None:
                self.cap.set(cv2.CAP_PROP_GAIN, float(gain))
        self.frame: Optional[np.ndarray] = frame.copy()
        self.sequence = 1
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._run, name=f"{name}-camera", daemon=True)
        print(
            f"[{name}] {target!r} -> open={self.open_target!r}: "
            f"{frame.shape[1]}x{frame.shape[0]}"
        )

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        failure_logged = False
        while not self.stopping.is_set():
            ok, frame = self.cap.read()
            if not ok or frame is None:
                if not failure_logged:
                    print(f"WARNING: failed to read {self.name} camera {self.target!r}")
                    failure_logged = True
                time.sleep(0.05)
                continue
            failure_logged = False
            with self.lock:
                # OpenCV may reuse the capture buffer; keep a private copy once.
                self.frame = frame.copy()
                self.sequence += 1

    def peek_sequence(self) -> int:
        with self.lock:
            return self.sequence

    def latest(self, copy: bool = True) -> tuple[int, Optional[np.ndarray]]:
        with self.lock:
            if self.frame is None:
                return self.sequence, None
            frame = self.frame.copy() if copy else self.frame
            return self.sequence, frame

    def close(self) -> None:
        self.stopping.set()
        self.cap.release()
        if self.thread.is_alive():
            self.thread.join(timeout=1)


def encode_jpeg(frame: np.ndarray, quality: int) -> bytes:
    ok, result = cv2.imencode(
        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    )
    if not ok:
        raise RuntimeError("JPEG encoding failed")
    return result.tobytes()


def decode_jpeg(data: bytes) -> np.ndarray:
    frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError("JPEG decoding failed")
    return frame


def add_label(frame: np.ndarray, text: str) -> np.ndarray:
    result = frame.copy()
    cv2.putText(result, text, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(result, text, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1, cv2.LINE_AA)
    return result


def stereo_preview(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    if left.shape[0] != right.shape[0]:
        height = min(left.shape[0], right.shape[0])
        left = cv2.resize(left, (round(left.shape[1] * height / left.shape[0]), height))
        right = cv2.resize(right, (round(right.shape[1] * height / right.shape[0]), height))
    return np.hstack((add_label(left, "head left"), add_label(right, "head right")))


def rotate_frame(frame: np.ndarray, rotation: str) -> np.ndarray:
    if rotation == "180":
        return cv2.rotate(frame, cv2.ROTATE_180)
    if rotation == "cw":
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if rotation == "ccw":
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame


def resize_camera(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    if frame.shape[1] == width and frame.shape[0] == height:
        return frame
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_LINEAR)


def four_camera_preview(frames: list[np.ndarray]) -> np.ndarray:
    labels = ["head left", "head right", "wrist left", "wrist right"]
    tiles = [
        add_label(resize_camera(frame, 640, 480), text)
        for frame, text in zip(frames, labels)
    ]
    top = np.hstack(tiles[:2])
    bottom = np.hstack(tiles[2:])
    return np.vstack((top, bottom))


def make_dex1_message(frames: list[np.ndarray], quality: int) -> bytes:
    full_color = np.hstack(frames)
    color_jpeg = encode_jpeg(full_color, quality)
    # image_client_dex1.py expects color_size + ir_size + JPEG data.
    return struct.pack("II", len(color_jpeg), 0) + color_jpeg


class AsyncZmqPublisher:
    """JPEG+ZMQ off the capture loop; keep only the newest pending payload."""

    def __init__(self, socket: "zmq.Socket") -> None:
        self._socket = socket
        # None = stop; bytes = ready message; (frames, quality) = encode then send.
        self._queue: queue.Queue = queue.Queue(maxsize=1)
        self._thread = threading.Thread(
            target=self._run, name="zmq-publish", daemon=True
        )
        self._dropped = 0
        self._sent = 0
        self._lock = threading.Lock()
        self._thread.start()

    def submit_message(self, message: bytes) -> None:
        self._put(message)

    def submit_frames(self, frames: list[np.ndarray], quality: int) -> None:
        # Caller must pass owned snapshots (latest/resize/rotate already copy).
        self._put((list(frames), int(quality)))

    def _put(self, item) -> None:
        while True:
            try:
                self._queue.put_nowait(item)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                    with self._lock:
                        self._dropped += 1
                except queue.Empty:
                    pass

    def stats(self) -> tuple[int, int]:
        with self._lock:
            return self._sent, self._dropped

    def close(self) -> None:
        while True:
            try:
                self._queue.put_nowait(None)
                break
            except queue.Full:
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    pass
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while True:
            item = self._queue.get()
            if item is None:
                return
            try:
                if isinstance(item, tuple):
                    frames, quality = item
                    message = make_dex1_message(frames, quality)
                else:
                    message = item
                self._socket.send(message, copy=False)
                with self._lock:
                    self._sent += 1
            except Exception as exc:
                print(f"[zmq-publish] send failed: {exc}", file=sys.stderr)


def decode_dex1_message(message: bytes) -> tuple[list[np.ndarray], dict]:
    header_size = struct.calcsize("II")
    if len(message) < header_size:
        raise RuntimeError("Dex1 message is shorter than its header")
    color_size, _ir_size = struct.unpack("II", message[:header_size])
    color_data = message[header_size : header_size + color_size]
    full_color = decode_jpeg(color_data)
    if full_color.shape[1] % 4:
        raise RuntimeError(
            f"Dex1 color width {full_color.shape[1]} is not divisible into four cameras"
        )
    frames = list(np.hsplit(full_color, 4))
    return frames, {"timestamp": None, "wire_format": "dex1"}


def run_bridge(args: argparse.Namespace) -> None:
    context = zmq.Context.instance()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.SNDHWM, 2)
    publisher.setsockopt(zmq.LINGER, 0)
    try:
        publisher.bind(args.bind)
    except zmq.ZMQError as exc:
        raise SystemExit(f"Cannot bind ZMQ at {args.bind}: {exc}") from exc

    params_socket = None
    if args.params_bind:
        params_socket = context.socket(zmq.REP)
        params_socket.setsockopt(zmq.LINGER, 0)
        try:
            params_socket.bind(args.params_bind)
        except zmq.ZMQError as exc:
            publisher.close()
            raise SystemExit(f"Cannot bind camera parameters at {args.params_bind}: {exc}") from exc

    left_reader = RtpReader("head left", args.left_port, args)
    right_reader = RtpReader("head right", args.right_port, args)
    head_readers = (left_reader, right_reader)
    wrist_readers: tuple[OpenCvReader, OpenCvReader] | tuple[()] = ()
    running = True

    def stop(_signum, _frame) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    print("R1 four-camera bridge")
    print(
        f"ZMQ publishing at {args.bind}, wire_format={args.wire_format!r}, "
        f"topic={args.topic!r}"
    )
    print(
        f"head: PC1 .161 -> PC2 .164 UDP {args.left_port}/{args.right_port} "
        f"RTP/{args.head_codec.upper()} payload={args.payload}"
    )
    print("wrist: PC2 local V4L2 devices -> user PC .120 TCP 5555")
    try:
        left_reader.start()
        right_reader.start()
        if not args.no_wrist:
            wrist_left_reader = OpenCvReader(
                "wrist left",
                parse_camera(args.wrist_left),
                args.wrist_width,
                args.wrist_height,
                args.wrist_fps,
                args.wrist_fourcc,
                args.wrist_exposure,
                args.wrist_gain,
            )
            wrist_right_reader = OpenCvReader(
                "wrist right",
                parse_camera(args.wrist_right),
                args.wrist_width,
                args.wrist_height,
                args.wrist_fps,
                args.wrist_fourcc,
                args.wrist_exposure,
                args.wrist_gain,
            )
            wrist_readers = (wrist_left_reader, wrist_right_reader)
            wrist_left_reader.start()
            wrist_right_reader.start()
    except Exception:
        for reader in (*head_readers, *wrist_readers):
            reader.close()
        publisher.close()
        if params_socket is not None:
            params_socket.close()
        raise

    last_left = 0
    last_right = 0
    last_wrist_left = 0
    last_wrist_right = 0
    total = 0
    period_frames = 0
    last_status = time.monotonic()
    last_publish = 0.0
    waiting_logged = False
    min_period = (1.0 / float(args.max_fps)) if args.max_fps > 0 else 0.0
    async_pub = (
        AsyncZmqPublisher(publisher) if args.async_publish else None
    )
    publish_gate = args.publish_gate
    print(
        f"speed: publish_gate={publish_gate}, max_fps={args.max_fps}, "
        f"jpeg_quality={args.jpeg_quality}, async_publish={args.async_publish}, "
        f"wrist_fourcc={args.wrist_fourcc!r}"
    )
    try:
        while running:
            if params_socket is not None:
                try:
                    request = params_socket.recv_string(flags=zmq.NOBLOCK)
                    if request == "get_params":
                        params_socket.send_json(
                            {
                                "head_cameras": [
                                    {"position": "head_left", "source": f"udp:{args.left_port}"},
                                    {"position": "head_right", "source": f"udp:{args.right_port}"},
                                ],
                                "wrist_cameras": [
                                    {"position": "wrist_left", "source": args.wrist_left},
                                    {"position": "wrist_right", "source": args.wrist_right},
                                ] if wrist_readers else [],
                                "output_per_camera": {
                                    "width": args.output_width,
                                    "height": args.output_height,
                                },
                            }
                        )
                    else:
                        params_socket.send_json({"error": f"unknown request: {request}"})
                except zmq.Again:
                    pass

            for reader in head_readers:
                reader.check_alive()

            # Cheap seq peek first — avoid copying 4 frames on every idle spin.
            left_seq = left_reader.peek_sequence()
            right_seq = right_reader.peek_sequence()
            if wrist_readers:
                wrist_left_seq = wrist_readers[0].peek_sequence()
                wrist_right_seq = wrist_readers[1].peek_sequence()
            else:
                wrist_left_seq = wrist_right_seq = 0

            missing_head = left_seq == 0 or right_seq == 0
            missing_wrist = bool(wrist_readers) and (
                wrist_left_seq == 0 or wrist_right_seq == 0
            )
            if missing_head or missing_wrist:
                now = time.monotonic()
                if not waiting_logged or now - last_status >= args.print_period:
                    print(
                        "Waiting for four streams: "
                        f"head_left={left_seq}, head_right={right_seq}, "
                        f"wrist_left={wrist_left_seq}, wrist_right={wrist_right_seq}"
                    )
                    waiting_logged = True
                    last_status = now
                time.sleep(0.01)
                continue

            head_changed = left_seq != last_left or right_seq != last_right
            wrist_changed = (
                wrist_left_seq != last_wrist_left or wrist_right_seq != last_wrist_right
            )
            if publish_gate == "head":
                should_publish = head_changed
            elif publish_gate == "wrist":
                should_publish = wrist_changed if wrist_readers else head_changed
            else:  # any
                should_publish = head_changed or wrist_changed
            if not should_publish:
                time.sleep(0.001)
                continue

            now = time.monotonic()
            if min_period > 0.0 and (now - last_publish) < min_period:
                time.sleep(0.001)
                continue

            left_seq, left = left_reader.latest(copy=True)
            right_seq, right = right_reader.latest(copy=True)
            if wrist_readers:
                wrist_left_seq, wrist_left = wrist_readers[0].latest(copy=True)
                wrist_right_seq, wrist_right = wrist_readers[1].latest(copy=True)
            else:
                wrist_left = wrist_right = None

            if left is None or right is None:
                time.sleep(0.002)
                continue
            if wrist_readers and (wrist_left is None or wrist_right is None):
                time.sleep(0.002)
                continue

            last_left, last_right = left_seq, right_seq
            last_wrist_left, last_wrist_right = wrist_left_seq, wrist_right_seq
            last_publish = now

            if args.swap:
                left, right = right, left
            if wrist_left is not None and wrist_right is not None:
                wrist_left = rotate_frame(wrist_left, args.left_wrist_rotation)
                wrist_right = rotate_frame(wrist_right, args.right_wrist_rotation)
                if args.swap_wrist:
                    wrist_left, wrist_right = wrist_right, wrist_left

            output_frames = [
                resize_camera(left, args.output_width, args.output_height),
                resize_camera(right, args.output_width, args.output_height),
            ]
            if wrist_left is not None and wrist_right is not None:
                output_frames.extend(
                    [
                        resize_camera(wrist_left, args.output_width, args.output_height),
                        resize_camera(wrist_right, args.output_width, args.output_height),
                    ]
                )

            meta = {
                "timestamp": time.time(),
                "frame_id": total,
                "streams": ["head_left", "head_right", "wrist_left", "wrist_right"][:len(output_frames)],
                "shapes": [list(frame.shape) for frame in output_frames],
                "encoding": "jpeg",
                "source": f"R1 RTP/{args.head_codec.upper()} PC1",
                "wire_format": args.wire_format,
            }
            if args.wire_format == "dex1":
                if async_pub is not None:
                    async_pub.submit_frames(output_frames, args.jpeg_quality)
                else:
                    publisher.send(
                        make_dex1_message(output_frames, args.jpeg_quality),
                        copy=False,
                    )
            else:
                publisher.send_multipart(
                    [
                        args.topic.encode(),
                        json.dumps(meta).encode(),
                        *[encode_jpeg(frame, args.jpeg_quality) for frame in output_frames],
                    ]
                )
            if args.preview:
                preview = (
                    four_camera_preview(output_frames)
                    if len(output_frames) == 4
                    else stereo_preview(*output_frames)
                )
                cv2.imshow("R1 four-camera bridge", preview)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break

            total += 1
            period_frames += 1
            now = time.monotonic()
            if total == 1:
                print(f"Received {len(output_frames)} cameras. Publishing to the user PC now.")
            if now - last_status >= args.print_period:
                fps = period_frames / (now - last_status)
                extra = ""
                if async_pub is not None:
                    sent, dropped = async_pub.stats()
                    extra = f", zmq_sent={sent}, zmq_dropped={dropped}"
                print(
                    f"published={total}, fps={fps:.1f}, cameras={len(output_frames)}, "
                    f"per_camera={args.output_width}x{args.output_height}{extra}"
                )
                period_frames = 0
                last_status = now
    finally:
        if async_pub is not None:
            async_pub.close()
        for reader in (*head_readers, *wrist_readers):
            reader.close()
        publisher.close()
        if params_socket is not None:
            params_socket.close()
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        print("Bridge stopped.")


def run_view(args: argparse.Namespace) -> None:
    context = zmq.Context.instance()
    subscriber = context.socket(zmq.SUB)
    subscriber.setsockopt(zmq.RCVHWM, 2)
    subscriber.setsockopt(zmq.LINGER, 0)
    # Subscribe to everything so this viewer can auto-detect both the legacy
    # Dex1 single-message format and the multipart format.
    subscriber.setsockopt_string(zmq.SUBSCRIBE, "")
    subscriber.connect(args.connect)

    poller = zmq.Poller()
    poller.register(subscriber, zmq.POLLIN)
    print(f"Connecting to PC2 four-camera bridge: {args.connect}")
    print("Press q or ESC in the image window to exit.")
    total = 0
    last_status = time.monotonic()
    try:
        while True:
            events = dict(poller.poll(1000))
            if subscriber not in events:
                print(f"Waiting for PC2 at {args.connect} ...")
                continue
            parts = subscriber.recv_multipart()
            try:
                if len(parts) == 1:
                    frames, meta = decode_dex1_message(parts[0])
                elif len(parts) >= 4:
                    topic_raw, meta_raw, *frame_parts = parts
                    if topic_raw.decode(errors="replace") != args.topic:
                        continue
                    meta = json.loads(meta_raw)
                    frames = [decode_jpeg(data) for data in frame_parts]
                else:
                    print(f"Ignoring invalid message with {len(parts)} parts")
                    continue
            except (RuntimeError, ValueError, json.JSONDecodeError) as exc:
                print(f"Ignoring invalid camera message: {exc}")
                continue

            if len(frames) not in (2, 4):
                print(f"Ignoring message containing {len(frames)} camera frames")
                continue
            if args.swap:
                frames[0], frames[1] = frames[1], frames[0]
            if len(frames) == 4 and args.swap_wrist:
                frames[2], frames[3] = frames[3], frames[2]
            preview = (
                four_camera_preview(frames)
                if len(frames) == 4
                else stereo_preview(*frames)
            )
            cv2.imshow("R1 head and wrist cameras from PC2", preview)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
            total += 1
            now = time.monotonic()
            if now - last_status >= args.print_period:
                timestamp = meta.get("timestamp")
                latency = (
                    f", latency~{(time.time() - float(timestamp)) * 1000:.1f} ms"
                    if timestamp is not None
                    else ""
                )
                print(
                    f"received={total}, cameras={len(frames)}, "
                    f"format={meta.get('wire_format', 'multipart')}{latency}, "
                    f"view={preview.shape[1]}x{preview.shape[0]}"
                )
                last_status = now
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.close()
        cv2.destroyAllWindows()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="mode", required=True)

    bridge = subparsers.add_parser("bridge", help="Run on R1 PC2 (192.168.123.164).")
    bridge.add_argument("--bind", default="tcp://0.0.0.0:5555")
    bridge.add_argument(
        "--params-bind",
        default="tcp://0.0.0.0:5557",
        help="Camera metadata REP endpoint used by image_client_dex1.py; empty disables it.",
    )
    bridge.add_argument(
        "--wire-format",
        choices=["dex1", "multipart"],
        default="dex1",
        help="dex1 is compatible with wbc_pico_record image_client_dex1.py.",
    )
    bridge.add_argument("--topic", default=DEFAULT_TOPIC)
    bridge.add_argument(
        "--head-codec",
        choices=["jpeg", "h264"],
        default="jpeg",
        help="Head RTP codec. The upgraded official stream uses jpeg.",
    )
    bridge.add_argument("--left-port", type=int, default=5004)
    bridge.add_argument("--right-port", type=int, default=5005)
    bridge.add_argument("--payload", type=int, default=26)
    bridge.add_argument("--clock-rate", type=int, default=90000)
    bridge.add_argument(
        "--latency",
        type=int,
        default=0,
        help="RTP jitter-buffer latency in ms; 0 disables the jitter buffer.",
    )
    bridge.add_argument("--width", type=int, default=544)
    bridge.add_argument("--height", type=int, default=448)
    bridge.add_argument("--output-width", type=int, default=640, help="Published width per camera.")
    bridge.add_argument("--output-height", type=int, default=480, help="Published height per camera.")
    bridge.add_argument("--jpeg-quality", type=int, default=60)
    bridge.add_argument("--gst-bin", default="/usr/bin/gst-launch-1.0")
    bridge.add_argument("--preview", action="store_true", help="Show a window on PC2.")
    bridge.add_argument("--swap", action="store_true", help="Swap head left/right.")
    bridge.add_argument("--no-wrist", action="store_true", help="Publish only the head cameras.")
    bridge.add_argument("--wrist-left", default="/dev/cam_wrist_left")
    bridge.add_argument("--wrist-right", default="/dev/cam_wrist_right")
    bridge.add_argument("--wrist-width", type=int, default=640)
    bridge.add_argument("--wrist-height", type=int, default=480)
    bridge.add_argument("--wrist-fps", type=int, default=30)
    bridge.add_argument(
        "--wrist-fourcc",
        default="MJPG",
        help="V4L2 fourcc; MJPG is usually much faster than YUYV. Use '' to negotiate.",
    )
    bridge.add_argument("--wrist-exposure", type=float, default=None, help="Manual OpenCV exposure value; omit to keep the camera default.")
    bridge.add_argument("--wrist-gain", type=float, default=None, help="Manual OpenCV gain value; omit to keep the camera default.")
    bridge.set_defaults(swap_wrist=True, async_publish=True)
    bridge.add_argument("--swap-wrist", dest="swap_wrist", action="store_true")
    bridge.add_argument("--no-swap-wrist", dest="swap_wrist", action="store_false")
    bridge.add_argument(
        "--publish-gate",
        choices=["any", "head", "wrist"],
        default="any",
        help=(
            "When to publish a new ZMQ frame. "
            "'head' is the old behavior (wrist stuck to head RTP). "
            "'any' publishes when head OR wrist updates (recommended)."
        ),
    )
    bridge.add_argument(
        "--max-fps",
        type=float,
        default=30.0,
        help="Cap publish rate (0 = uncapped). Avoids flooding when gate=any.",
    )
    bridge.add_argument(
        "--async-publish",
        dest="async_publish",
        action="store_true",
        help="JPEG/ZMQ send on a background thread (drop oldest if busy).",
    )
    bridge.add_argument(
        "--no-async-publish",
        dest="async_publish",
        action="store_false",
        help="Encode and send on the capture thread (old behavior).",
    )
    bridge.add_argument(
        "--left-wrist-rotation",
        choices=["none", "cw", "ccw", "180"],
        default="180",
    )
    bridge.add_argument(
        "--right-wrist-rotation",
        choices=["none", "cw", "ccw", "180"],
        default="180",
    )
    bridge.add_argument("--print-period", type=float, default=2.0)

    view = subparsers.add_parser("view", help="Run on the user PC (192.168.123.120).")
    view.add_argument("--connect", default="tcp://192.168.123.164:5555")
    view.add_argument("--topic", default=DEFAULT_TOPIC)
    view.add_argument("--swap", action="store_true", help="Swap displayed head left/right.")
    view.add_argument("--swap-wrist", action="store_true", help="Swap displayed wrist left/right.")
    view.add_argument("--print-period", type=float, default=2.0)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.mode == "bridge":
        if args.wire_format == "dex1" and args.no_wrist:
            raise SystemExit("--wire-format dex1 requires both wrist cameras; use multipart with --no-wrist")
        if args.wrist_fourcc and len(args.wrist_fourcc) != 4:
            raise SystemExit("--wrist-fourcc must be exactly four characters, for example MJPG")
        if not 1 <= args.jpeg_quality <= 100:
            raise SystemExit("--jpeg-quality must be between 1 and 100")
        if args.max_fps < 0:
            raise SystemExit("--max-fps must be >= 0")
        run_bridge(args)
    else:
        run_view(args)


if __name__ == "__main__":
    main()
