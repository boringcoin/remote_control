#!/usr/bin/env python3
"""Drive the AuraOS robot from SichirayGlove roll (linear) and pitch (turning).

The glove is read over BLE FFE1. Wheel speeds are sent to the AuraOS daemon
through its motion REST API (POST /api/motion/cmd_vel and /api/motion/stop)
instead of the raw serial FF A3 protocol, so this controller coexists with the
daemon that owns the serial port.

Control logic (roll -> forward/backward, pitch -> left/right, deadzone,
3-frame vote debounce, stale-data watchdog, explicit STOP on every exit) is
preserved from the upstream tools/sichiray_glove_robot_test.py; only the
transport layer was replaced.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import signal
import time
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path

from bleak import BleakClient


GLOVE_ADDRESS = "AC:0B:FB:2F:7A:62"
GLOVE_NOTIFY_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
DEFAULT_API_BASE = "http://127.0.0.1:8765/api/motion"
DEFAULT_WHEEL_BASE = 0.3  # meters; must match config.yaml robot.wheel_base


class RobotHttp:
    """Send wheel-speed targets to the AuraOS daemon motion REST API.

    Differential inverse kinematics converts the upstream left/right wheel
    speeds (mm/s) into a body twist (m/s, rad/s) before the POST.
    """

    def __init__(
        self,
        api_base: str,
        wheel_base: float,
        *,
        invert_linear: bool = False,
        invert_angular: bool = False,
    ) -> None:
        self.api_base = api_base.rstrip("/")
        self.wheel_base = wheel_base
        self.invert_linear = invert_linear
        self.invert_angular = invert_angular
        self._last_speeds: tuple[float, float] | None = None
        self.stop("initialized")

    def set_speeds(self, left_mm_s: float, right_mm_s: float, reason: str) -> None:
        speeds = (float(left_mm_s), float(right_mm_s))
        self._post_cmd_vel(speeds, reason)

    def stop(self, reason: str) -> None:
        if self._last_speeds != (0.0, 0.0):
            logging.warning("MOTOR STOP reason=%s", reason)
        self._last_speeds = (0.0, 0.0)
        self._post("stop", {}, reason, (0.0, 0.0))

    def close(self) -> None:
        self.stop("program closing")

    def _post_cmd_vel(self, speeds: tuple[float, float], reason: str) -> None:
        linear_x = (speeds[0] + speeds[1]) / 2.0 / 1000.0  # mm/s -> m/s
        # AuraOS twist_to_wheel_speeds() negates angular_z when
        # robot.invert_angular is true (default), so send (left - right) here
        # so the final wheel speeds match the requested ones after that negation.
        angular_z = (speeds[0] - speeds[1]) / 1000.0 / self.wheel_base
        if self.invert_linear:
            linear_x = -linear_x
        if self.invert_angular:
            angular_z = -angular_z
        self._post(
            "cmd_vel",
            {"linear_x": linear_x, "angular_z": angular_z},
            reason,
            speeds,
        )

    def _post(
        self,
        path: str,
        payload: dict,
        reason: str,
        speeds: tuple[float, float],
    ) -> None:
        try:
            body = json.dumps(payload or {}).encode("utf-8")
            request = urllib.request.Request(
                f"{self.api_base}/{path}",
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(request, timeout=0.5) as response:
                status = getattr(response, "status", 200)
            if status != 200:
                logging.warning("MOTOR %s HTTP %d reason=%s", path, status, reason)
                return
            if self._last_speeds != speeds:
                logging.info(
                    "MOTOR command left=%.1f right=%.1f mm/s reason=%s",
                    speeds[0],
                    speeds[1],
                    reason,
                )
            self._last_speeds = speeds
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            # Keep the previous speeds so the next frame retries the POST;
            # the daemon watchdog will stop the robot if it keeps failing.
            logging.error(
                "MOTOR %s failed (%s) reason=%s",
                path,
                exc,
                reason,
            )


class GloveController:
    def __init__(
        self,
        *,
        robot: RobotHttp | None,
        deadzone: float,
        speed: float,
        turn_speed: float,
        calibration_samples: int,
        data_timeout: float,
        neutral_roll: float | None = None,
        neutral_pitch: float | None = None,
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
        self.neutral_roll: float | None = neutral_roll
        self.neutral_pitch: float | None = neutral_pitch
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
                self._save_calibration()
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
        ) == 1 and (requested != self.active_state or requested != "STOP"):
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


    _calibration_file: Path | None = None

    def _save_calibration(self) -> None:
        if self._calibration_file is None:
            return
        try:
            import json
            data = {"roll": self.neutral_roll, "pitch": self.neutral_pitch}
            self._calibration_file.write_text(json.dumps(data, indent=2))
            logging.info("CALIBRATION saved to %s", self._calibration_file)
        except Exception as exc:
            logging.warning("CALIBRATION save failed: %s", exc)

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
    # Load pre-saved calibration if available
    neutral_roll: float | None = None
    neutral_pitch: float | None = None
    cal_file: Path | None = args.calibration_file
    if cal_file is not None and cal_file.is_file():
        try:
            import json
            saved = json.loads(cal_file.read_text())
            neutral_roll = float(saved["roll"])
            neutral_pitch = float(saved["pitch"])
            logging.info(
                "CALIBRATION loaded from %s roll=%.2f pitch=%.2f",
                cal_file, neutral_roll, neutral_pitch,
            )
        except Exception as exc:
            logging.warning("CALIBRATION load failed (%s), will calibrate", exc)

    robot = (
        RobotHttp(
            args.api_base,
            args.wheel_base,
            invert_linear=args.invert_linear,
            invert_angular=args.invert_angular,
        )
        if args.live
        else None
    )
    controller = GloveController(
        robot=robot,
        deadzone=args.deadzone,
        speed=args.speed,
        turn_speed=args.turn_speed,
        calibration_samples=args.calibration_samples,
        data_timeout=args.data_timeout,
        neutral_roll=neutral_roll,
        neutral_pitch=neutral_pitch,
    )
    if cal_file is not None:
        controller._calibration_file = cal_file

    if neutral_roll is not None and neutral_pitch is not None:
        cal_msg = "pre-loaded, skipping calibration"
    else:
        cal_msg = f"keep glove neutral for {args.calibration_samples} frames"
    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_event.set)

    logging.info(
        "START mode=%s glove=%s api=%s deadzone=%.1f speed=%.1f turn_speed=%.1f %s",
        "LIVE" if args.live else "DRY-RUN",
        args.glove_address,
        args.api_base,
        args.deadzone,
        args.speed,
        args.turn_speed,
        cal_msg,
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
                "BLE subscribed uuid=%s", GLOVE_NOTIFY_UUID,
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
    parser.add_argument("--api-base", default=DEFAULT_API_BASE)
    parser.add_argument("--wheel-base", type=float, default=DEFAULT_WHEEL_BASE)
    parser.add_argument("--invert-linear", action="store_true")
    parser.add_argument("--invert-angular", action="store_true")
    parser.add_argument("--deadzone", type=float, default=8.0)
    parser.add_argument("--speed", type=float, default=150.0)
    parser.add_argument("--turn-speed", type=float, default=80.0)
    parser.add_argument("--calibration-samples", type=int, default=15)
    parser.add_argument(
        "--calibration-file",
        type=Path,
        default=Path("/home/sunrise/remote_control/glove_calibration_roll_pitch.json"),
        help="JSON file with pre-saved {'roll':..., 'pitch':...}; skip calibration if valid",
    )
    parser.add_argument("--data-timeout", type=float, default=1.0)
    parser.add_argument(
        "--duration", type=float, default=0.0, help="seconds; 0 runs indefinitely"
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        default=Path("/home/sunrise/remote_control/glove_http_control.log"),
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
