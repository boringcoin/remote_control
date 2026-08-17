#!/usr/bin/env python3
"""Direct USB teleoperation for the Huaner glove Aura-stream firmware.

The firmware emits: H,thumb,index,middle,ring,little,roll,pitch
Finger values are calibrated in both open and closed positions, so each glove
uses its own threshold rather than hard-coded sensor values.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import select
import socket
import termios
import time
from pathlib import Path

from rdk_udp_glove_controller import AuraMotionApi, map_axis


FINGER_NAMES = ("thumb", "index", "middle", "ring", "little")
MIN_FINGER_SPAN = 20.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("menu", "calibrate-open", "calibrate-fist", "control"), default="menu")
    parser.add_argument("--port", default="/dev/ttyUSB_HUANER", help="direct Huaner serial port when --udp-port=0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--udp-port", type=int, default=5011, help="Mac forwarder UDP port; 0 reads --port directly")
    parser.add_argument("--samples", type=int, default=60)
    parser.add_argument("--duration", type=float, default=0.0, help="control duration; 0 runs until Ctrl+C")
    parser.add_argument("--api-base", default="http://127.0.0.1:8765/api/motion")
    parser.add_argument("--max-linear", type=float, default=0.35)
    parser.add_argument("--max-angular", type=float, default=0.60)
    parser.add_argument("--command-hz", type=float, default=5.0, help="maximum AuraOS velocity command rate")
    parser.add_argument("--full-scale-degrees", type=float, default=20.0)
    parser.add_argument("--linear-deadband-degrees", type=float, default=4.0)
    parser.add_argument("--angular-deadband-degrees", type=float, default=10.0)
    parser.add_argument("--calibration-file", default=str(Path(__file__).with_name("huaner_calibration.json")))
    return parser.parse_args()


def choose_mode(mode: str) -> str:
    if mode != "menu":
        return mode
    choices = {"1": "calibrate-open", "2": "calibrate-fist", "3": "control"}
    while True:
        print("\n1) 张手校准  2) 握拳校准  3) 控制机器人")
        selected = choices.get(input("输入 1、2 或 3：").strip())
        if selected:
            return selected


class HuanerSerial:
    def __init__(self, path: str, baud: int) -> None:
        self.fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        settings = termios.tcgetattr(self.fd)
        rate = {115200: termios.B115200, 9600: termios.B9600}[baud]
        settings[0] = termios.IGNPAR
        settings[1] = 0
        settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        settings[3] = 0
        settings[4] = rate
        settings[5] = rate
        settings[6][termios.VMIN] = 0
        settings[6][termios.VTIME] = 1
        termios.tcsetattr(self.fd, termios.TCSANOW, settings)
        self.buffer = bytearray()

    def close(self) -> None:
        os.close(self.fd)

    def read(self, timeout: float = 0.1) -> tuple[float, ...] | None:
        readable, _, _ = select.select([self.fd], [], [], timeout)
        if not readable:
            return None
        self.buffer.extend(os.read(self.fd, 512))
        while b"\n" in self.buffer:
            raw, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            frame = parse_frame(raw.decode("ascii", errors="ignore"))
            if frame is not None:
                return frame
        return None


def parse_frame(line: str) -> tuple[float, ...] | None:
    fields = line.strip().split(",")
    if len(fields) != 8 or fields[0] != "H":
        return None
    try:
        values = tuple(float(value) for value in fields[1:])
    except ValueError:
        return None
    return values if all(math.isfinite(value) for value in values) else None


class HuanerUdp:
    def __init__(self, port: int) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.1)

    def close(self) -> None:
        self.sock.close()

    def read(self, timeout: float = 0.1) -> tuple[float, ...] | None:
        self.sock.settimeout(timeout)
        try:
            raw, _ = self.sock.recvfrom(512)
        except TimeoutError:
            return None
        return parse_frame(raw.decode("ascii", errors="ignore"))


def open_reader(args: argparse.Namespace) -> HuanerSerial | HuanerUdp:
    return HuanerUdp(args.udp_port) if args.udp_port else HuanerSerial(args.port, args.baud)


def average(samples: list[tuple[float, ...]]) -> tuple[float, ...]:
    return tuple(sum(row[i] for row in samples) / len(samples) for i in range(len(samples[0])))


def capture(reader: HuanerSerial, count: int, prompt: str) -> tuple[float, ...]:
    print(prompt, flush=True)
    samples: list[tuple[float, ...]] = []
    deadline = time.monotonic() + 30.0
    while len(samples) < count and time.monotonic() < deadline:
        frame = reader.read()
        if frame is not None:
            samples.append(frame)
            print(f"calibration {len(samples)}/{count} fingers={tuple(round(v) for v in frame[:5])} roll={frame[5]:.1f} pitch={frame[6]:.1f}", flush=True)
    if len(samples) < count:
        raise RuntimeError("校准超时：未收到 Huaner 的 H,... 数据帧。请确认已刷入 huaner_aura_stream 固件。")
    return average(samples)


def load(path: str) -> dict:
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError) as exc:
        raise RuntimeError("请先依次完成张手校准和握拳校准。") from exc


def update_calibration(args: argparse.Namespace, mode: str) -> None:
    reader = open_reader(args)
    try:
        sample = capture(
            reader, args.samples,
            "保持手掌自然张开且朝向中立…" if mode == "calibrate-open" else "保持完全握拳且朝向中立…",
        )
    finally:
        reader.close()
    saved = {} if mode == "calibrate-open" else load(args.calibration_file)
    if mode == "calibrate-open":
        saved["open_fingers"] = sample[:5]
        saved["neutral_roll"] = sample[5]
        saved["neutral_pitch"] = sample[6]
    else:
        changed = [
            name
            for name, open_value, fist_value in zip(FINGER_NAMES, saved["open_fingers"], sample[:5])
            if abs(fist_value - open_value) >= MIN_FINGER_SPAN
        ]
        if len(changed) < 2:
            raise RuntimeError(
                "握拳校准无效：至少需要两个手指通道相对张开态变化 20 以上；"
                f"当前有效通道={','.join(changed) or '无'}。请确认拇指和食指已真正握紧后重试。"
            )
        saved["fist_fingers"] = sample[:5]
    Path(args.calibration_file).write_text(json.dumps(saved, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"已保存：{args.calibration_file}")


def fist_fraction(value: float, open_value: float, fist_value: float) -> float:
    span = fist_value - open_value
    if abs(span) < MIN_FINGER_SPAN:
        return 0.0
    return max(0.0, min(1.0, (value - open_value) / span))


def control(args: argparse.Namespace) -> None:
    saved = load(args.calibration_file)
    open_fingers = tuple(saved["open_fingers"])
    fist_fingers = tuple(saved["fist_fingers"])
    active_finger_indices = tuple(
        index
        for index, (open_value, fist_value) in enumerate(zip(open_fingers, fist_fingers))
        if abs(fist_value - open_value) >= MIN_FINGER_SPAN
    )
    if not active_finger_indices:
        raise RuntimeError("校准数据中没有检测到有效的手指变化；请重新进行张开和握拳校准。")
    enter_required = max(1, math.ceil(len(active_finger_indices) * 0.7))
    hold_required = max(1, math.ceil(len(active_finger_indices) * 0.4))
    active_finger_names = ",".join(FINGER_NAMES[index] for index in active_finger_indices)
    thumb_index = FINGER_NAMES.index("thumb")
    use_thumb_gate = thumb_index in active_finger_indices
    neutral_roll, neutral_pitch = float(saved["neutral_roll"]), float(saved["neutral_pitch"])
    api = AuraMotionApi(args.api_base)
    if not api.status().get("running"):
        raise RuntimeError("AuraOS 运动服务未就绪。")
    reader = open_reader(args)
    deadline = None if args.duration <= 0 else time.monotonic() + args.duration
    active = False
    frame_count = 0
    last_command_time = 0.0
    published = (0.0, 0.0)
    command_interval = 1.0 / max(args.command_hz, 0.1)
    print(
        f"控制模式：握拳启用；松拳停止。有效手指={active_finger_names}；"
        + (
            "进入条件=拇指弯曲加任一其他有效手指。"
            if use_thumb_gate
            else f"进入阈值={enter_required}/{len(active_finger_indices)}。"
        )
        + "上下倾斜前后，左右倾斜转向。",
        flush=True,
    )
    try:
        api.stop()
        while deadline is None or time.monotonic() < deadline:
            frame = reader.read()
            if frame is None:
                continue
            fingers, roll, pitch = frame[:5], frame[5], frame[6]
            fractions = tuple(fist_fraction(v, o, f) for v, o, f in zip(fingers, open_fingers, fist_fingers))
            threshold = 0.35 if active else 0.65
            close_count = sum(fractions[index] >= threshold for index in active_finger_indices)
            if use_thumb_gate:
                thumb_threshold = 0.35 if active else 0.45
                thumb_closed = fractions[thumb_index] >= thumb_threshold
                other_closed = sum(
                    fractions[index] >= threshold
                    for index in active_finger_indices
                    if index != thumb_index
                )
                active = thumb_closed and other_closed >= 1
                gesture_detail = f"thumb={'ON' if thumb_closed else 'OFF'}, other={other_closed}"
            else:
                active = close_count >= (hold_required if active else enter_required)
                gesture_detail = f"close={close_count}/{len(active_finger_indices)}"
            # The Huaner firmware's axis labels are rotated relative to the
            # wearable orientation: its roll changes on physical up/down tilt,
            # while pitch changes on left/right tilt.
            linear = map_axis(
                roll,
                neutral_roll,
                args.max_linear,
                args.full_scale_degrees,
                args.linear_deadband_degrees,
            )
            angular = -map_axis(
                pitch,
                neutral_pitch,
                args.max_angular,
                args.full_scale_degrees,
                args.angular_deadband_degrees,
            )
            sent = (linear, angular) if active else (0.0, 0.0)
            now = time.monotonic()
            if now - last_command_time >= command_interval or (sent == (0.0, 0.0) and published != sent):
                api.cmd_vel(*sent)
                published = sent
                last_command_time = now
            frame_count += 1
            labels = ", ".join(f"{n}={v:.0f}" for n, v in zip(FINGER_NAMES, fingers))
            print(
                f"control frame={frame_count} fingers[{labels}] pose[roll={roll:.1f}, pitch={pitch:.1f}] "
                f"fist_progress={tuple(round(v, 2) for v in fractions)} "
                f"gesture[fist={'ON' if active else 'OFF'}, {gesture_detail}] "
                f"requested[linear_x={linear:.3f}, angular_z={angular:.3f}] "
                f"robot_sent[linear_x={published[0]:.3f}, angular_z={published[1]:.3f}]",
                flush=True,
            )
    finally:
        api.stop()
        reader.close()


def main() -> None:
    args = parse_args()
    mode = choose_mode(args.mode)
    if mode in ("calibrate-open", "calibrate-fist"):
        update_calibration(args, mode)
    else:
        control(args)


if __name__ == "__main__":
    main()
