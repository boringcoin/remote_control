#!/usr/bin/env python3
"""UDP glove teleoperation for AuraOS: choose calibration or control mode."""

from __future__ import annotations

import argparse
import json
import math
import re
import socket
import time
import urllib.error
import urllib.request
from pathlib import Path


FRAME = re.compile(r"^-?\d+(?:\.\d+)?(?:,-?\d+(?:\.\d+)?){7}$")
FINGER_NAMES = ("thumb", "index", "middle", "ring", "little")

# Empirical closed-fist profile from the live glove.  The thumb value shifts a
# lot between grips, so it is intentionally not used as a required condition.
FIST_ENTER_RING = -12000.0
FIST_ENTER_INDEX = -1800.0
FIST_ENTER_MIDDLE = -2300.0
FIST_EXIT_RING = -9000.0
FIST_EXIT_INDEX = -1000.0
FIST_EXIT_MIDDLE = -1800.0


class AuraMotionApi:
    def __init__(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")

    def request(self, method: str, route: str, body: dict[str, float] | None = None) -> dict:
        data = None if body is None else json.dumps(body).encode("utf-8")
        request = urllib.request.Request(
            self.base_url + route,
            data=data,
            method=method,
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=0.5) as response:
                return json.loads(response.read().decode("utf-8") or "{}")
        except urllib.error.URLError as exc:
            raise RuntimeError(f"AuraOS {method} {route} failed: {exc}") from exc

    def status(self) -> dict:
        return self.request("GET", "/status")

    def cmd_vel(self, linear_x: float, angular_z: float) -> None:
        self.request("POST", "/cmd_vel", {"linear_x": linear_x, "angular_z": angular_z})

    def stop(self) -> None:
        self.request("POST", "/stop", {})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("menu", "calibrate", "control"), default="menu")
    parser.add_argument("--port", type=int, default=5010)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="seconds to control; 0 keeps running until Ctrl+C",
    )
    parser.add_argument("--api-base", default="http://127.0.0.1:8765/api/motion")
    parser.add_argument("--neutral-samples", type=int, default=60)
    parser.add_argument("--frame-timeout", type=float, default=0.30)
    parser.add_argument("--max-linear", type=float, default=0.15)
    # AuraOS motor backend clamps angular velocity at 0.30 rad/s.
    parser.add_argument("--max-angular", type=float, default=0.30)
    parser.add_argument("--full-scale-degrees", type=float, default=30.0)
    parser.add_argument("--deadband-degrees", type=float, default=10.0)
    parser.add_argument(
        "--telemetry-interval",
        type=float,
        default=0.0,
        help="seconds between complete telemetry lines; 0 prints every valid frame",
    )
    parser.add_argument(
        "--calibration-file",
        default=str(Path(__file__).with_name("glove_calibration_roll_pitch.json")),
    )
    return parser.parse_args()


def open_udp(port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.10)
    return sock


def read_frame(sock: socket.socket) -> tuple[float, ...] | None:
    try:
        data, _peer = sock.recvfrom(1024)
    except TimeoutError:
        return None
    line = data.decode("ascii", errors="ignore").strip()
    if not FRAME.fullmatch(line):
        return None
    values = tuple(float(item) for item in line.split(","))
    return values if all(math.isfinite(value) for value in values) else None


def map_axis(value: float, neutral: float, maximum: float, full_scale: float, deadband: float) -> float:
    delta = value - neutral
    if abs(delta) <= deadband:
        return 0.0
    return maximum * max(-1.0, min(1.0, delta / full_scale))


def format_input(frame: tuple[float, ...]) -> str:
    """Return every glove field with stable, human-readable labels."""
    fingers = ", ".join(
        f"{name}={value:.1f}" for name, value in zip(FINGER_NAMES, frame[:5])
    )
    # Live USB frames are ordered as roll, pitch, yaw.
    roll, pitch, yaw = frame[5:]
    raw_csv = ",".join(f"{value:.3f}" for value in frame)
    return (
        f"fingers[{fingers}] "
        f"pose[roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}] "
        f"csv={raw_csv}"
    )


def should_print(now: float, last_print: float, interval: float) -> bool:
    return interval <= 0.0 or now - last_print >= interval


def update_fist(fingers: tuple[float, ...], was_active: bool) -> bool:
    """Recognize a closed fist with hysteresis to avoid frame-to-frame flicker."""
    _, index, middle, ring, _ = fingers
    if was_active:
        # Only release after the ring is visibly open and both helper sensors
        # are also open. This prevents a held fist from briefly dropping out.
        return not (
            ring > FIST_EXIT_RING
            and index > FIST_EXIT_INDEX
            and middle > FIST_EXIT_MIDDLE
        )
    # Ring bends the most consistently. Either index or middle confirms it;
    # the unstable thumb/little channels are not required.
    return ring <= FIST_ENTER_RING and (
        index <= FIST_ENTER_INDEX or middle <= FIST_ENTER_MIDDLE
    )


def choose_mode(mode: str) -> str:
    if mode != "menu":
        return mode
    while True:
        print("\n请选择模式：\n  1) 校准零点（手套静止）\n  2) 控制机器人（读取已保存零点）")
        selection = input("输入 1 或 2：").strip()
        if selection == "1":
            return "calibrate"
        if selection == "2":
            return "control"
        print("请输入 1 或 2。")


def calibrate(args: argparse.Namespace) -> None:
    print(f"校准模式：请保持手套静止，正在收集 {args.neutral_samples} 帧数据…", flush=True)
    sock = open_udp(args.port)
    samples: list[tuple[float, float, float]] = []
    # Calibration must still time out; duration=0 is reserved for continuous
    # control mode below.
    deadline = time.monotonic() + (args.duration if args.duration > 0 else 60.0)
    last_print = 0.0
    frame_count = 0
    try:
        while len(samples) < args.neutral_samples and time.monotonic() < deadline:
            frame = read_frame(sock)
            if frame is not None:
                samples.append(frame[5:])  # live source format: fingers, roll, pitch, yaw
                frame_count += 1
                now = time.monotonic()
                if should_print(now, last_print, args.telemetry_interval):
                    print(
                        f"calibration frame={frame_count} samples={len(samples)}/{args.neutral_samples} "
                        f"{format_input(frame)}",
                        flush=True,
                    )
                    last_print = now
        if len(samples) < args.neutral_samples:
            raise RuntimeError("校准超时：没有收到足够的手套数据。请确认 Mac 转发程序正在运行。")
        neutral = tuple(
            sum(sample[index] for sample in samples) / len(samples)
            for index in range(3)
        )
        destination = Path(args.calibration_file)
        destination.write_text(
            json.dumps(
                {"roll": neutral[0], "pitch": neutral[1], "yaw": neutral[2]},
                ensure_ascii=False,
                indent=2,
            ) + "\n",
            encoding="utf-8",
        )
        print(f"校准完成：roll/pitch/yaw={tuple(round(v, 2) for v in neutral)}")
        print(f"已保存到：{destination}")
    finally:
        sock.close()


def load_neutral(path: str) -> tuple[float, float, float]:
    try:
        saved = json.loads(Path(path).read_text(encoding="utf-8"))
        return (float(saved["roll"]), float(saved["pitch"]), float(saved["yaw"]))
    except (FileNotFoundError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"找不到有效校准文件 {path}。请先选择模式 1 校准。") from exc


def control(args: argparse.Namespace) -> None:
    neutral = load_neutral(args.calibration_file)
    api = AuraMotionApi(args.api_base)
    status = api.status()
    if not status.get("connected") or not status.get("running"):
        raise RuntimeError(f"AuraOS motor backend is not ready: {status}")

    print(
        "控制模式：已加载零点 "
        f"roll/pitch/yaw={tuple(round(v, 2) for v in neutral)}"
    )
    print("无需手指解锁；pitch 控制前后；roll 控制左右转向（方向已反转修正）。", flush=True)
    sock = open_udp(args.port)
    deadline = None if args.duration <= 0 else time.monotonic() + args.duration
    last_frame_at = time.monotonic()
    last_command: tuple[float, float] = (0.0, 0.0)
    last_print = 0.0
    frame_count = 0
    fist_active = False
    try:
        api.stop()
        while deadline is None or time.monotonic() < deadline:
            frame = read_frame(sock)
            if frame is not None:
                fingers, angles = frame[:5], frame[5:]
                frame_count += 1
                last_frame_at = time.monotonic()
                linear_x = map_axis(
                    angles[1], neutral[1], args.max_linear,
                    args.full_scale_degrees, args.deadband_degrees,
                )
                # The previous direct mapping turned the robot the opposite way.
                angular_z = -map_axis(
                    angles[0], neutral[0], args.max_angular,
                    args.full_scale_degrees, args.deadband_degrees,
                )
                requested_command = (linear_x, angular_z)
                fist_active = update_fist(fingers, fist_active)
                command = requested_command if fist_active else (0.0, 0.0)
                # AuraOS has a motor watchdog. Refresh every valid glove frame,
                # including an unchanged tilt, so a held command remains active.
                api.cmd_vel(*command)
                last_command = command
                now = time.monotonic()
                if should_print(now, last_print, args.telemetry_interval):
                    delta_roll = angles[0] - neutral[0]
                    delta_pitch = angles[1] - neutral[1]
                    delta_yaw = angles[2] - neutral[2]
                    print(
                        f"control frame={frame_count} {format_input(frame)} "
                        f"neutral[roll={neutral[0]:.2f}, pitch={neutral[1]:.2f}, yaw={neutral[2]:.2f}] "
                        f"delta[roll={delta_roll:.2f}, pitch={delta_pitch:.2f}, yaw={delta_yaw:.2f}] "
                        f"gesture[fist={'ON' if fist_active else 'OFF'} rule=ring+(index|middle)] "
                        f"requested[linear_x={requested_command[0]:.3f}m/s, angular_z={requested_command[1]:.3f}rad/s] "
                        f"robot_sent[linear_x={command[0]:.3f}m/s, angular_z={command[1]:.3f}rad/s]",
                        flush=True,
                    )
                    last_print = now
            if time.monotonic() - last_frame_at > args.frame_timeout and last_command != (0.0, 0.0):
                api.stop()
                last_command = (0.0, 0.0)
                print("已停止：手套数据超时", flush=True)
    finally:
        try:
            api.stop()
        except RuntimeError:
            pass
        sock.close()


def main() -> None:
    args = parse_args()
    mode = choose_mode(args.mode)
    if mode == "calibrate":
        calibrate(args)
    else:
        control(args)


if __name__ == "__main__":
    main()
