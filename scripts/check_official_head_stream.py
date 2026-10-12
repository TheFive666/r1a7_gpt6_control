#!/usr/bin/env python3
"""Test the official R1 binocular RTP/JPEG streams on UDP 5004/5005.

Run this script on the computer that actually receives the official ports. In
the current R1 layout that is likely the head computer PC1 (192.168.123.161),
while PC2 receives the custom H264 relay on 5002/5003. Confirm the destination
with tcpdump when uncertain. The script invokes the system GStreamer tools, so
it does not depend on the Python OpenCV GUI build.
"""

from __future__ import annotations

import argparse
import selectors
import shlex
import shutil
import signal
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path


def probe_udp(args: argparse.Namespace) -> int:
    """Count raw UDP datagrams without requiring tcpdump or GStreamer decode."""
    duration = args.duration if args.duration > 0 else 10.0
    selector = selectors.DefaultSelector()
    sockets: dict[socket.socket, str] = {}
    counts = {"head_left": 0, "head_right": 0}
    byte_counts = {"head_left": 0, "head_right": 0}
    senders: dict[str, tuple[str, int] | None] = {"head_left": None, "head_right": None}
    rtp_packets = {"head_left": 0, "head_right": 0}
    payload_types: dict[str, set[int]] = {"head_left": set(), "head_right": set()}
    timestamps: dict[str, set[int]] = {"head_left": set(), "head_right": set()}
    marker_counts = {"head_left": 0, "head_right": 0}
    first_marker_at: dict[str, float | None] = {"head_left": None, "head_right": None}
    last_marker_at: dict[str, float | None] = {"head_left": None, "head_right": None}

    try:
        for name, port in (("head_left", args.left_port), ("head_right", args.right_port)):
            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                udp_socket.bind(("0.0.0.0", port))
            except OSError as exc:
                udp_socket.close()
                raise SystemExit(
                    f"failed to bind UDP {port}: {exc}. Check existing listeners with "
                    f"`ss -ulpn | grep :{port}`."
                ) from exc
            udp_socket.setblocking(False)
            selector.register(udp_socket, selectors.EVENT_READ, name)
            sockets[udp_socket] = name

        print(
            f"[Probe] listening on UDP {args.left_port}/{args.right_port} for {duration:.1f}s...",
            flush=True,
        )
        started = time.monotonic()
        deadline = started + duration
        while time.monotonic() < deadline:
            for key, _events in selector.select(timeout=min(0.5, deadline - time.monotonic())):
                udp_socket = key.fileobj
                name = key.data
                try:
                    payload, sender = udp_socket.recvfrom(65535)
                except BlockingIOError:
                    continue
                counts[name] += 1
                byte_counts[name] += len(payload)
                if senders[name] is None:
                    senders[name] = sender
                if len(payload) >= 12 and payload[0] >> 6 == 2:
                    rtp_packets[name] += 1
                    payload_types[name].add(payload[1] & 0x7F)
                    _sequence, timestamp = struct.unpack_from("!HI", payload, 2)
                    timestamps[name].add(timestamp)
                    if payload[1] & 0x80:
                        now = time.monotonic()
                        marker_counts[name] += 1
                        if first_marker_at[name] is None:
                            first_marker_at[name] = now
                        last_marker_at[name] = now

        elapsed = max(time.monotonic() - started, 1e-6)
        for name, port in (("head_left", args.left_port), ("head_right", args.right_port)):
            mbps = byte_counts[name] * 8.0 / elapsed / 1_000_000.0
            marker_elapsed = (
                (last_marker_at[name] - first_marker_at[name])
                if first_marker_at[name] is not None and last_marker_at[name] is not None
                else 0.0
            )
            frame_rate = (
                (marker_counts[name] - 1) / marker_elapsed
                if marker_counts[name] > 1 and marker_elapsed > 0
                else 0.0
            )
            print(
                f"[Probe] {name} UDP {port}: packets={counts[name]} "
                f"packet_rate={counts[name] / elapsed:.1f}/s bitrate={mbps:.2f} Mbps "
                f"sender={senders[name]}",
                flush=True,
            )
            print(
                f"[Probe] {name} RTP: valid_packets={rtp_packets[name]} "
                f"payload_types={sorted(payload_types[name])} "
                f"unique_timestamps={len(timestamps[name])} markers={marker_counts[name]} "
                f"estimated_fps={frame_rate:.2f}",
                flush=True,
            )
        if not any(counts.values()):
            print(
                "[Probe] no UDP packets reached this computer. The camera stream is disabled, "
                "sent to another destination IP, or only available locally on PC1."
            )
            return 2
        return 0
    finally:
        selector.close()
        for udp_socket in sockets:
            udp_socket.close()


def rtp_jpeg_source(port: int, latency_ms: int, payload_type: int) -> list[str]:
    source = [
        "udpsrc",
        "address=0.0.0.0",
        f"port={port}",
        "!",
        (
            "application/x-rtp,media=video,clock-rate=90000,"
            f"encoding-name=JPEG,payload={payload_type}"
        ),
    ]
    if latency_ms > 0:
        source += [
            "!",
            "rtpjitterbuffer",
            f"latency={latency_ms}",
            "drop-on-latency=true",
        ]
    source += [
        "!",
        "rtpjpegdepay",
        "!",
        "jpegparse",
        "!",
        "jpegdec",
    ]
    return source


def frame_counter(name: str) -> list[str]:
    return [
        "!",
        "identity",
        f"name={name}_counter",
        "silent=false",
    ]


def build_measure_pipeline(args: argparse.Namespace) -> list[str]:
    command = [args.gst_bin, "-v"]
    command += rtp_jpeg_source(args.left_port, args.latency, args.payload_type)
    command += frame_counter("head_left")
    command += ["!", "fakesink", "sync=false"]
    command += rtp_jpeg_source(args.right_port, args.latency, args.payload_type)
    command += frame_counter("head_right")
    command += ["!", "fakesink", "sync=false"]
    return command


def preview_branch(
    port: int,
    name: str,
    compositor_pad: str,
    args: argparse.Namespace,
) -> list[str]:
    branch = rtp_jpeg_source(port, args.latency, args.payload_type)
    branch += frame_counter(name)
    branch += [
        "!",
        "videoconvert",
        "!",
        "videoscale",
        "!",
        (
            "video/x-raw,format=BGRx,"
            f"width={args.preview_width},height={args.preview_height},pixel-aspect-ratio=1/1"
        ),
        "!",
        "textoverlay",
        f"text={name}",
        "valignment=top",
        "halignment=left",
        "font-desc=Sans 20",
        "!",
        "queue",
        "leaky=downstream",
        "max-size-buffers=2",
        "!",
        compositor_pad,
    ]
    return branch


def build_preview_pipeline(args: argparse.Namespace) -> list[str]:
    command = [
        args.gst_bin,
        "-v",
        "compositor",
        "name=mix",
        "background=black",
        "sink_0::xpos=0",
        f"sink_1::xpos={args.preview_width}",
        "!",
        "videoconvert",
        "!",
        "autovideosink",
        "sync=false",
    ]
    command += preview_branch(args.left_port, "head_left", "mix.sink_0", args)
    command += preview_branch(args.right_port, "head_right", "mix.sink_1", args)
    return command


class GstOutputMonitor:
    """Count decoded buffers from verbose ``identity`` element messages."""

    def __init__(self, stream) -> None:
        self.stream = stream
        self.lock = threading.Lock()
        self.counts = {"head_left": 0, "head_right": 0}
        self.first_at = {"head_left": None, "head_right": None}
        self.last_at = {"head_left": None, "head_right": None}
        self.caps_printed = {"head_left": False, "head_right": False}
        self.thread = threading.Thread(target=self._run, name="gst-output-monitor", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _run(self) -> None:
        for line in self.stream:
            stripped = line.rstrip()
            handled = False
            for name in ("head_left", "head_right"):
                marker = f"GstIdentity:{name}_counter:"
                if marker not in stripped:
                    continue
                if "last-message = chain" in stripped:
                    now = time.monotonic()
                    with self.lock:
                        self.counts[name] += 1
                        if self.first_at[name] is None:
                            self.first_at[name] = now
                        self.last_at[name] = now
                    handled = True
                elif "GstPad:src: caps =" in stripped and not self.caps_printed[name]:
                    caps = stripped.split("caps =", 1)[1].strip()
                    print(f"[{name} caps] {caps}", flush=True)
                    self.caps_printed[name] = True
                    handled = True
            if handled:
                continue
            upper = stripped.upper()
            if "ERROR" in upper or "WARNING" in upper or "错误" in stripped or "警告" in stripped:
                print(f"[GStreamer] {stripped}", flush=True)

    def snapshot(self) -> dict[str, int]:
        with self.lock:
            return dict(self.counts)

    def averages(self) -> dict[str, float]:
        result = {}
        with self.lock:
            for name in ("head_left", "head_right"):
                count = self.counts[name]
                first_at = self.first_at[name]
                last_at = self.last_at[name]
                if count > 1 and first_at is not None and last_at is not None:
                    result[name] = (count - 1) / max(last_at - first_at, 1e-6)
                else:
                    result[name] = 0.0
        return result

    def join(self) -> None:
        self.thread.join(timeout=2.0)


def stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def run(args: argparse.Namespace) -> int:
    if args.probe_only:
        return probe_udp(args)
    if not Path(args.gst_bin).is_file() and shutil.which(args.gst_bin) is None:
        raise SystemExit(f"GStreamer launcher not found: {args.gst_bin}")

    command = build_measure_pipeline(args) if args.no_show else build_preview_pipeline(args)
    print("Official R1 head stream test")
    print(f"left:  RTP/JPEG UDP {args.left_port}")
    print(f"right: RTP/JPEG UDP {args.right_port}")
    print(f"RTP payload type: {args.payload_type}")
    print("The script prints decoded left/right FPS once per second.")
    print("Command:")
    print(" ".join(shlex.quote(part) for part in command))

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    assert process.stdout is not None
    monitor = GstOutputMonitor(process.stdout)
    monitor.start()
    exit_code = 0
    started = time.monotonic()
    previous_at = started
    previous_counts = monitor.snapshot()
    try:
        deadline = started + args.duration if args.duration > 0 else None
        while process.poll() is None and (deadline is None or time.monotonic() < deadline):
            time.sleep(0.1)
            now = time.monotonic()
            if now - previous_at < 1.0:
                continue
            counts = monitor.snapshot()
            elapsed = now - previous_at
            left_fps = (counts["head_left"] - previous_counts["head_left"]) / elapsed
            right_fps = (counts["head_right"] - previous_counts["head_right"]) / elapsed
            print(
                f"[FPS] head_left={left_fps:.1f} head_right={right_fps:.1f} "
                f"frames={counts['head_left']}/{counts['head_right']}",
                flush=True,
            )
            previous_at = now
            previous_counts = counts
        exit_code = process.returncode or 0
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        stop_process(process)
        monitor.join()

    counts = monitor.snapshot()
    averages = monitor.averages()
    print(
        f"[Result] head_left: frames={counts['head_left']} average={averages['head_left']:.2f} Hz"
    )
    print(
        f"[Result] head_right: frames={counts['head_right']} average={averages['head_right']:.2f} Hz"
    )
    if counts["head_left"] == 0 or counts["head_right"] == 0:
        print(
            "[Result] one or both streams received no decoded frames. Check that the robot "
            "sends UDP 5004/5005 to this computer's IP address."
        )
        print(
            "[Result] if PC2 receives no packets, run this test on head computer PC1 "
            "(192.168.123.161), where the official streams may be local."
        )
    return exit_code


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-port", type=int, default=5004)
    parser.add_argument("--right-port", type=int, default=5005)
    parser.add_argument(
        "--payload-type",
        type=int,
        default=26,
        help="RTP payload type; the upgraded R1 stream reports static JPEG PT 26",
    )
    parser.add_argument(
        "--latency",
        type=int,
        default=0,
        help="RTP jitter-buffer latency in ms; 0 follows the official pipeline without a buffer",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 runs until Ctrl-C")
    parser.add_argument(
        "--probe-only",
        action="store_true",
        help="count raw UDP packets without GStreamer decode (uses 10s when duration is 0)",
    )
    parser.add_argument("--no-show", action="store_true", help="measure FPS without opening a window")
    parser.add_argument("--preview-width", type=int, default=640, help="display width per eye")
    parser.add_argument("--preview-height", type=int, default=544, help="display height per eye")
    parser.add_argument(
        "--gst-bin",
        default=(
            "/usr/bin/gst-launch-1.0"
            if Path("/usr/bin/gst-launch-1.0").is_file()
            else "gst-launch-1.0"
        ),
    )
    return parser


def main() -> None:
    raise SystemExit(run(build_parser().parse_args()))


if __name__ == "__main__":
    main()
