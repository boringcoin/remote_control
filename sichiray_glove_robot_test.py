#!/usr/bin/env python3
"""Drive the ESP robot from SichirayGlove roll (linear) and pitch (turning).

The glove is read over BLE FFE1. The robot motor controller is driven through
the binary UART protocol in history/esp_control:

    FF A3 <left float32 LE> <right float32 LE> <checksum>

Forward/backward control is intentionally the only enabled motion. Every exit,
BLE disconnect, stale-data condition, or exception sends an explicit STOP.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
import struct
import time
from collections import deque
from pathlib import Path

import serial
from bleak import BleakClient


GLOVE_ADDRESS = "AC:0B:FB:2F:7A:62"
GLOVE_NOTIFY_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
ROBOT_SERIAL_PORT = "/dev/ttyUSB0"
ROBOT_BAUDRATE = 115200

CMD_STOP = 0xA0
CMD_SET_BOTH_SPEEDS = 0xA3


def with_checksum(payload: bytes) -> bytes:
    return payload + bytes((sum(payload) & 0xFF,))


class RobotSerial:
    def __init__(self, port: str, baudrate: int) -> None:
        self._serial = serial.Serial(port, baudrate, timeout=0.05)
        self._last_speeds: tuple[float, float] | None = None
        self.stop("serial opened")

    def set_speeds(self, left_mm_s: float, right_mm_s: float, reason: str) -> None:
        speeds = (float(left_mm_s), float(right_mm_s))
        payload = struct.pack(
            "<BBff", 0xFF, CMD_SET_BOTH_SPEEDS, speeds[0], speeds[1]
        )
        self._serial.write(with_checksum(payload))
        self._serial.flush()
        if self._last_speeds != speeds:
            logging.info(
                "MOTOR command left=%.1f right=%.1f mm/s reason=%s",
                speeds[0],
                speeds[1],
                reason,
            )
            self._last_speeds = speeds

    def stop(self, reason: str) -> None:
        payload = bytes((0xFF, CMD_STOP))
        self._serial.write(with_checksum(payload))
        self._serial.flush()
        if self._last_speeds != (0.0, 0.0):
            logging.warning("MOTOR STOP reason=%s", reason)
            self._last_speeds = (0.0, 0.0)

    def close(self) -> None:
        try:
            self.stop("program closing")
        finally:
            self._serial.close()


class GloveController:
    def __init__(
        self,
        *,
        robot: RobotSerial | None,
        deadzone: float,
        speed: float,
        turn_speed: float,
        calibration_samples: int,
        data_timeout: float,
    ) -> None:
        self.robot = robot
        self.deadzone = deadzone
        self.speed = speed
        self.turn_speed = turn_speed
        self.calibration_samples = calibration_samples
        self.data_timeout = data_timeout
        self.buffer = bytearray()
        self.neutral_roll_samples: list[float] = []
        self.neutral_pitch_samples: list[float] = []
        self.neutral_roll: float | None = None
        self.neutral_pitch: float | None = None
        self.latest_roll: float | None = None
        self.latest_pitch: float | None = None
        self.last_frame_at = 0.0
        self.frame_count = 0
        self.requested_state = "STOP"
        self.active_state = "STOP"
        self.commanded_left_speed = 0.0
        self.commanded_right_speed = 0.0
        self.state_votes: deque[str] = deque(maxlen=3)
        self.last_status_log = 0.0
        self.disconnected = asyncio.Event()
        self.closing = False

    @property
    def live(self) -> bool:
        return self.robot is not None

    def on_disconnect(self, _client: BleakClient) -> None:
        if self.closing:
            logging.info("BLE disconnected during normal shutdown")
            self.disconnected.set()
            return
        logging.error("BLE disconnected unexpectedly")
        if self.robot is not None:
            self.robot.stop("BLE disconnected")
        self.disconnected.set()

    def on_notification(self, _sender: object, data: bytearray) -> None:
        self.buffer.extend(data)
        if len(self.buffer) > 4096:
            del self.buffer[:-4096]
        while b"\n" in self.buffer:
            raw_line, _, remainder = self.buffer.partition(b"\n")
            self.buffer = bytearray(remainder)
            line = raw_line.decode("ascii", errors="ignore").strip()
            self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        fields = line.split(",")
        if fields and fields[0] == "H":
            fields = fields[1:]
            roll_index = 5
            pitch_index = 6
        else:
            roll_index = 6
            pitch_index = 5
        if len(fields) not in (8, 14):
            logging.debug("FRAME ignored fields=%d raw=%r", len(fields), line)
            return
        try:
            roll = float(fields[roll_index])
            pitch = float(fields[pitch_index])
        except (ValueError, IndexError):
            logging.debug("FRAME parse failed raw=%r", line)
            return

        self.latest_roll = roll
        self.latest_pitch = pitch
        self.last_frame_at = time.monotonic()
        self.frame_count += 1

        if self.neutral_roll is None or self.neutral_pitch is None:
            self.neutral_roll_samples.append(roll)
            self.neutral_pitch_samples.append(pitch)
            logging.info(
                "CALIBRATION frame=%d/%d roll=%.2f pitch=%.2f: keep glove neutral",
                len(self.neutral_roll_samples),
                self.calibration_samples,
                roll,
                pitch,
            )
            if len(self.neutral_roll_samples) >= self.calibration_samples:
                self.neutral_roll = sum(self.neutral_roll_samples) / len(
                    self.neutral_roll_samples
                )
                self.neutral_pitch = sum(self.neutral_pitch_samples) / len(
                    self.neutral_pitch_samples
                )
                logging.info(
                    "CALIBRATION complete neutral_roll=%.2f neutral_pitch=%.2f degrees",
                    self.neutral_roll,
                    self.neutral_pitch,
                )
            return

        # This glove is mounted with roll as forward/backward and pitch as
        # left/right. On the tested hardware, forward tilt increases roll and
        # left tilt increases pitch.
        linear_delta = roll - self.neutral_roll
        turn_delta = pitch - self.neutral_pitch
        motion = "FORWARD" if linear_delta > self.deadzone else (
            "BACKWARD" if linear_delta < -self.deadzone else ""
        )
        turn = "LEFT" if turn_delta > self.deadzone else (
            "RIGHT" if turn_delta < -self.deadzone else ""
        )
        requested = "_".join(part for part in (motion, turn) if part) or "STOP"

        self.requested_state = requested
        self.state_votes.append(requested)
        if len(self.state_votes) == self.state_votes.maxlen and len(
            set(self.state_votes)
        ) == 1:
            self._apply_state(requested, linear_delta, turn_delta)

        now = time.monotonic()
        if now - self.last_status_log >= 1.0:
            logging.info(
                "STATUS frame=%d roll=%.2f neutral_roll=%.2f linear_delta=%.2f "
                "pitch=%.2f neutral_pitch=%.2f turn_delta=%.2f "
                "requested=%s active=%s left_speed=%.1f right_speed=%.1f live=%s",
                self.frame_count,
                roll,
                self.neutral_roll,
                linear_delta,
                pitch,
                self.neutral_pitch,
                turn_delta,
                requested,
                self.active_state,
                self.commanded_left_speed,
                self.commanded_right_speed,
                self.live,
            )
            self.last_status_log = now

    def _apply_state(
        self, state: str, linear_delta: float, turn_delta: float
    ) -> None:
        linear = self.speed if "FORWARD" in state else (
            -self.speed if "BACKWARD" in state else 0.0
        )
        turn = self.turn_speed if "LEFT" in state else (
            -self.turn_speed if "RIGHT" in state else 0.0
        )
        limit = max(self.speed, self.turn_speed)
        left = max(-limit, min(limit, linear - turn))
        right = max(-limit, min(limit, linear + turn))
        reason = f"roll_delta={linear_delta:.2f} pitch_delta={turn_delta:.2f}"
        previous_state = self.active_state
        self.active_state = state
        self.commanded_left_speed = left
        self.commanded_right_speed = right
        if previous_state != state:
            logging.info(
                "CONTROL state=%s left_speed=%.1f right_speed=%.1f %s",
                state,
                left,
                right,
                reason,
            )
        if self.robot is None:
            return
        if state == "STOP":
            self.robot.stop(reason)
        else:
            self.robot.set_speeds(left, right, reason)

    async def watchdog(self) -> None:
        while not self.disconnected.is_set():
            await asyncio.sleep(0.05)
            if self.last_frame_at == 0.0:
                continue
            age = time.monotonic() - self.last_frame_at
            if age > self.data_timeout:
                if self.active_state != "STOP":
                    logging.error("DATA timeout age=%.3fs", age)
                self.active_state = "STOP"
                self.commanded_left_speed = 0.0
                self.commanded_right_speed = 0.0
                self.state_votes.clear()
                if self.robot is not None:
                    self.robot.stop(f"glove data timeout {age:.3f}s")


def configure_logging(log_file: Path) -> None:
    log_file.parent.mkdir(parents=True, exist_ok=True)
    formatter = logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)s %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    stream = logging.StreamHandler()
    stream.setFormatter(formatter)
    file_handler = logging.FileHandler(log_file)
    file_handler.setFormatter(formatter)
    root.handlers[:] = [stream, file_handler]


async def run(args: argparse.Namespace) -> None:
    robot = RobotSerial(args.serial_port, args.baudrate) if args.live else None
    controller = GloveController(
        robot=robot,
        deadzone=args.deadzone,
        speed=args.speed,
        turn_speed=args.turn_speed,
        calibration_samples=args.calibration_samples,
        data_timeout=args.data_timeout,
    )
    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_event.set)

    logging.info(
        "START mode=%s glove=%s serial=%s deadzone=%.1f speed=%.1f turn_speed=%.1f",
        "LIVE" if args.live else "DRY-RUN",
        args.glove_address,
        args.serial_port,
        args.deadzone,
        args.speed,
        args.turn_speed,
    )
    watchdog_task: asyncio.Task[None] | None = None
    try:
        async with BleakClient(
            args.glove_address,
            disconnected_callback=controller.on_disconnect,
            timeout=15.0,
        ) as client:
            logging.info("BLE connected address=%s", args.glove_address)
            await client.start_notify(GLOVE_NOTIFY_UUID, controller.on_notification)
            logging.info(
                "BLE subscribed uuid=%s; keep glove neutral for %d frames",
                GLOVE_NOTIFY_UUID,
                args.calibration_samples,
            )
            watchdog_task = asyncio.create_task(controller.watchdog())
            waiters = [
                asyncio.create_task(stop_event.wait()),
                asyncio.create_task(controller.disconnected.wait()),
            ]
            if args.duration > 0:
                waiters.append(asyncio.create_task(asyncio.sleep(args.duration)))
            done, pending = await asyncio.wait(
                waiters, return_when=asyncio.FIRST_COMPLETED
            )
            for task in pending:
                task.cancel()
            for task in done:
                task.result()
            controller.closing = True
            await client.stop_notify(GLOVE_NOTIFY_UUID)
    finally:
        controller.closing = True
        if watchdog_task is not None:
            watchdog_task.cancel()
        if robot is not None:
            robot.close()
        logging.info("EXIT frames=%d", controller.frame_count)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--live", action="store_true", help="enable motor output")
    parser.add_argument("--glove-address", default=GLOVE_ADDRESS)
    parser.add_argument("--serial-port", default=ROBOT_SERIAL_PORT)
    parser.add_argument("--baudrate", type=int, default=ROBOT_BAUDRATE)
    parser.add_argument("--deadzone", type=float, default=8.0)
    parser.add_argument("--speed", type=float, default=150.0)
    parser.add_argument("--turn-speed", type=float, default=80.0)
    parser.add_argument("--calibration-samples", type=int, default=15)
    parser.add_argument("--data-timeout", type=float, default=1.0)
    parser.add_argument(
        "--duration", type=float, default=0.0, help="seconds; 0 runs indefinitely"
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        default=Path("/home/sunrise/glove_control_test.log"),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass
    except Exception:
        logging.exception("FATAL controller failed")
        raise


if __name__ == "__main__":
    main()
