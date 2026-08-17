#!/usr/bin/env python3
"""Receive the JDY-18 glove stream and safely control AuraOS motion.

The glove firmware sends newline-terminated UTF-8 CSV frames:
    finger_1,finger_2,finger_3,finger_4,finger_5,angle_x,angle_y,angle_z

The final three values are JY901 orientation angles in degrees.  They are not
raw angular-rate / acceleration samples: the supplied glove firmware only
transmits JY901.stcAngle.Angle[0..2].
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
import signal
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

try:
    from bleak import BleakClient, BleakScanner
    from bleak.backends.device import BLEDevice
except ImportError:  # Gives a useful error before asyncio starts.
    BleakClient = None  # type: ignore[assignment,misc]
    BleakScanner = None  # type: ignore[assignment,misc]
    BLEDevice = object  # type: ignore[assignment,misc]


DEFAULT_API_BASE = "http://127.0.0.1:8765/api/motion"
KNOWN_JDY_NOTIFY_UUIDS = {
    # Observed on the JY-XT11-3332 glove during a successful RDK connection.
    "0000ae02-0000-1000-8000-00805f9b34fb",
    "0000ffe1-0000-1000-8000-00805f9b34fb",
    "0000ffe2-0000-1000-8000-00805f9b34fb",
}


@dataclass(frozen=True)
class GloveFrame:
    """One complete frame emitted by the Arduino glove firmware."""

    timestamp: float
    fingers: tuple[float, float, float, float, float]
    angles: tuple[float, float, float]
    raw: str

    @classmethod
    def parse(cls, line: bytes) -> "GloveFrame":
        raw = line.decode("utf-8", errors="strict").strip()
        values = [float(value.strip()) for value in raw.split(",")]
        if len(values) != 8:
            raise ValueError(f"expected 8 CSV values, got {len(values)}")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("frame contains non-finite data")
        return cls(
            timestamp=time.time(),
            fingers=(values[0], values[1], values[2], values[3], values[4]),
            angles=(values[5], values[6], values[7]),
            raw=raw,
        )


class CsvLog:
    """Durable recording of every valid glove frame and its mapped command."""

    HEADER = [
        "timestamp_unix",
        "finger_1_deg",
        "finger_2_deg",
        "finger_3_deg",
        "finger_4_deg",
        "finger_5_deg",
        "angle_x_deg",
        "angle_y_deg",
        "angle_z_deg",
        "linear_x_m_s",
        "angular_z_rad_s",
        "motion_enabled",
        "deadman_active",
        "raw_csv",
    ]

    def __init__(self, log_dir: Path) -> None:
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.path = log_dir / f"glove_{stamp}.csv"
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self._file.flush()

    def write(
        self,
        frame: GloveFrame,
        linear_x: float,
        angular_z: float,
        motion_enabled: bool,
        deadman_active: bool,
    ) -> None:
        self._writer.writerow(
            [
                f"{frame.timestamp:.6f}",
                *(f"{value:.3f}" for value in frame.fingers),
                *(f"{value:.3f}" for value in frame.angles),
                f"{linear_x:.4f}",
                f"{angular_z:.4f}",
                int(motion_enabled),
                int(deadman_active),
                frame.raw,
            ]
        )
        self._file.flush()

    def close(self) -> None:
        self._file.close()


class AuraMotionApi:
    """The daemon API is the sole path to the ESP motor serial link."""

    def __init__(self, base_url: str, dry_run: bool) -> None:
        self.base_url = base_url.rstrip("/")
        self.dry_run = dry_run

    async def status(self) -> dict[str, object]:
        return await asyncio.to_thread(self._request, "GET", "/status", None)

    async def set_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        if self.dry_run:
            return
        await asyncio.to_thread(
            self._request,
            "POST",
            "/cmd_vel",
            {"linear_x": linear_x, "angular_z": angular_z},
        )

    async def stop(self) -> None:
        if self.dry_run:
            return
        await asyncio.to_thread(self._request, "POST", "/stop", {})

    def _request(
        self,
        method: str,
        route: str,
        body: dict[str, object] | None,
    ) -> dict[str, object]:
        data = None if body is None else json.dumps(body).encode("utf-8")
        request = urllib.request.Request(
            f"{self.base_url}{route}",
            data=data,
            method=method,
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=0.6) as response:
                payload = response.read().decode("utf-8")
        except urllib.error.URLError as exc:
            raise RuntimeError(f"AuraOS API {method} {route} failed: {exc}") from exc
        return json.loads(payload) if payload else {}


class Teleop:
    """BLE transport, calibration, deadman gating, logging, and motion mapping."""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.api = AuraMotionApi(args.api_base, args.dry_run)
        self.log = CsvLog(Path(args.log_dir))
        self.latest_frame: GloveFrame | None = None
        self.latest_at = 0.0
        self.neutral_samples: list[tuple[float, float, float]] = []
        self.neutral: tuple[float, float, float] | None = None
        self.last_target: tuple[float, float] | None = None
        self._rx_buffer = bytearray()
        self._stop_requested = asyncio.Event()
        self._last_print_at = 0.0

    async def run(self) -> None:
        if BleakClient is None or BleakScanner is None:
            raise RuntimeError(
                "缺少 bleak。请先运行："
                "/home/sunrise/auraos/.venv-linux/bin/python -m pip install -r requirements.txt"
            )

        if self.args.enable_motion:
            status = await self.api.status()
            if not status.get("connected") or not status.get("running"):
                raise RuntimeError(f"AuraOS motor backend is not ready: {status}")
            print("AuraOS motor backend is ready. Motion still waits for calibration and deadman.")
        else:
            print("Monitor mode: no robot movement will be sent. Add --enable-motion to arm it.")

        self._install_signal_handlers()
        try:
            while not self._stop_requested.is_set():
                try:
                    device = await self._find_device()
                    await self._run_connection(device)
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    print(
                        f"BLE connection error ({type(exc).__name__}): {exc!r}",
                        file=sys.stderr,
                    )
                finally:
                    await self._safe_stop("BLE disconnected")
                if not self._stop_requested.is_set():
                    await asyncio.sleep(self.args.reconnect_seconds)
        finally:
            await self._safe_stop("program exit")
            self.log.close()

    def _install_signal_handlers(self) -> None:
        loop = asyncio.get_running_loop()
        for signal_name in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(signal_name, self._stop_requested.set)
            except NotImplementedError:
                pass

    async def _find_device(self) -> BLEDevice | str:
        if self.args.address:
            print(f"Connecting to glove address {self.args.address}")
            # On Bleak 0.22/Linux BLEDevice is backend-owned and cannot be
            # constructed by callers. BleakClient accepts a MAC string.
            return self.args.address

        print(f"Scanning for a BLE device whose name contains '{self.args.name}'...")
        discovered = await BleakScanner.discover(
            timeout=self.args.scan_seconds,
            return_adv=True,
        )
        needle = self.args.name.lower()
        for _address, (device, advertisement) in discovered.items():
            # BlueZ can expose a cached address as BLEDevice.name while the
            # actual advertised local name is present only in AdvertisementData.
            advertised_name = advertisement.local_name or device.name or ""
            if needle in advertised_name.lower():
                print(f"Selected {device.name} ({device.address})")
                return device
        discovered = ", ".join(
            f"{advertisement.local_name or device.name or '<unnamed>'} ({device.address})"
            for device, advertisement in discovered.values()
        ) or "none"
        raise RuntimeError(f"glove not found; discovered: {discovered}")

    async def _run_connection(self, device: BLEDevice | str) -> None:
        address = device if isinstance(device, str) else device.address
        print(f"Connecting to {address}...")
        async with BleakClient(device, timeout=15.0) as client:
            characteristic = self._select_notify_characteristic(client)
            print(f"Connected. Receiving newline CSV on {characteristic.uuid}; log: {self.log.path}")
            await client.start_notify(characteristic, self._on_notification)
            try:
                while client.is_connected and not self._stop_requested.is_set():
                    await self._control_tick()
                    await asyncio.sleep(1.0 / self.args.command_hz)
            finally:
                try:
                    await client.stop_notify(characteristic)
                except Exception:
                    pass

    def _select_notify_characteristic(self, client: BleakClient):
        services = client.services
        if self.args.characteristic:
            characteristic = services.get_characteristic(self.args.characteristic)
            if characteristic is None:
                raise RuntimeError(f"BLE characteristic not found: {self.args.characteristic}")
            return characteristic

        characteristics = [
            characteristic
            for service in services
            for characteristic in service.characteristics
        ]
        for characteristic in characteristics:
            if characteristic.uuid.lower() in KNOWN_JDY_NOTIFY_UUIDS and (
                "notify" in characteristic.properties or "indicate" in characteristic.properties
            ):
                return characteristic
        for characteristic in characteristics:
            if "notify" in characteristic.properties or "indicate" in characteristic.properties:
                return characteristic
        detail = "; ".join(
            f"{characteristic.uuid}={','.join(characteristic.properties)}"
            for characteristic in characteristics
        )
        raise RuntimeError(f"no notify characteristic found; services: {detail}")

    def _on_notification(self, _sender: object, data: bytearray) -> None:
        if self.args.debug_notifications:
            ascii_data = bytes(data).decode("utf-8", errors="replace").rstrip()
            print(f"BLE RX {bytes(data).hex()} | {ascii_data!r}")
        self._rx_buffer.extend(data)
        if len(self._rx_buffer) > 1024:
            # A missing newline must never allow an unbounded buffer.
            del self._rx_buffer[:-256]
        while b"\n" in self._rx_buffer:
            line, _, remainder = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(remainder)
            if not line.strip():
                continue
            try:
                frame = GloveFrame.parse(line)
            except (UnicodeDecodeError, ValueError) as exc:
                print(f"Discarded malformed glove frame {line!r}: {exc}", file=sys.stderr)
                continue
            self.latest_frame = frame
            self.latest_at = time.monotonic()
            self._calibrate(frame)

    def _calibrate(self, frame: GloveFrame) -> None:
        if self.neutral is not None:
            return
        self.neutral_samples.append(frame.angles)
        if len(self.neutral_samples) < self.args.neutral_samples:
            return
        self.neutral = tuple(
            sum(sample[index] for sample in self.neutral_samples)
            / len(self.neutral_samples)
            for index in range(3)
        )
        print(
            "Neutral calibration complete: "
            f"x={self.neutral[0]:.1f}, y={self.neutral[1]:.1f}, z={self.neutral[2]:.1f} deg"
        )

    async def _control_tick(self) -> None:
        frame = self.latest_frame
        fresh = frame is not None and time.monotonic() - self.latest_at <= self.args.frame_timeout
        if not fresh or frame is None:
            await self._safe_stop("glove frame timeout")
            return

        linear_x, angular_z, deadman = self._mapped_command(frame)
        self.log.write(frame, linear_x, angular_z, self.args.enable_motion, deadman)
        self._print_frame(frame, linear_x, angular_z, deadman)

        if not self.args.enable_motion or self.neutral is None or not deadman:
            await self._safe_stop("monitor/calibration/deadman")
            return

        try:
            await self.api.set_cmd_vel(linear_x, angular_z)
            self.last_target = (linear_x, angular_z)
        except RuntimeError as exc:
            print(exc, file=sys.stderr)
            await self._safe_stop("AuraOS API error")

    def _mapped_command(self, frame: GloveFrame) -> tuple[float, float, bool]:
        deadman = self._deadman_active(frame)
        if self.neutral is None:
            return 0.0, 0.0, deadman

        linear = self._map_axis(
            self._axis_value(frame.angles, self.args.linear_axis),
            self._axis_value(self.neutral, self.args.linear_axis),
            self.args.max_linear,
            self.args.invert_linear,
        )
        angular = self._map_axis(
            self._axis_value(frame.angles, self.args.angular_axis),
            self._axis_value(self.neutral, self.args.angular_axis),
            self.args.max_angular,
            self.args.invert_angular,
        )
        return linear, angular, deadman

    def _map_axis(self, value: float, neutral: float, maximum: float, invert: bool) -> float:
        delta = value - neutral
        if abs(delta) <= self.args.angle_deadband:
            return 0.0
        normalized = max(-1.0, min(1.0, delta / self.args.full_scale_degrees))
        return (-1.0 if invert else 1.0) * maximum * normalized

    def _deadman_active(self, frame: GloveFrame) -> bool:
        if self.args.deadman_finger < 0:
            return True
        return frame.fingers[self.args.deadman_finger] >= self.args.deadman_threshold

    @staticmethod
    def _axis_value(values: tuple[float, float, float], axis: str) -> float:
        return values[{"x": 0, "y": 1, "z": 2}[axis]]

    def _print_frame(
        self,
        frame: GloveFrame,
        linear_x: float,
        angular_z: float,
        deadman: bool,
    ) -> None:
        now = time.monotonic()
        if now - self._last_print_at < 0.5:
            return
        self._last_print_at = now
        calibration = "ready" if self.neutral is not None else "calibrating"
        print(
            f"fingers={[round(value, 1) for value in frame.fingers]} "
            f"angles={[round(value, 1) for value in frame.angles]} "
            f"cmd=({linear_x:.3f}m/s,{angular_z:.3f}rad/s) "
            f"deadman={deadman} {calibration}"
        )

    async def _safe_stop(self, reason: str) -> None:
        if self.last_target == (0.0, 0.0):
            return
        if self.args.enable_motion:
            try:
                await self.api.stop()
            except RuntimeError as exc:
                print(f"Could not stop robot ({reason}): {exc}", file=sys.stderr)
        self.last_target = (0.0, 0.0)


async def scan(scan_seconds: float, details: bool = False) -> None:
    if BleakScanner is None:
        raise RuntimeError("缺少 bleak，请先安装 requirements.txt")
    if details:
        discovered = await BleakScanner.discover(
            timeout=scan_seconds,
            return_adv=True,
        )
        if not discovered:
            print("No BLE devices found. Confirm the glove is powered and advertising.")
            return
        for address, (device, advertisement) in sorted(discovered.items()):
            manufacturer = ", ".join(
                f"0x{company_id:04X}:{data.hex()}"
                for company_id, data in advertisement.manufacturer_data.items()
            ) or "-"
            services = ", ".join(advertisement.service_uuids) or "-"
            name = advertisement.local_name or device.name or "<unnamed>"
            print(
                f"{address}\tname={name}\trssi={advertisement.rssi}\t"
                f"services={services}\tmfg={manufacturer}"
            )
        return

    devices = await BleakScanner.discover(timeout=scan_seconds)
    if not devices:
        print("No BLE devices found. Confirm the glove is powered and advertising.")
        return
    for device in devices:
        print(f"{device.address}\t{device.name or '<unnamed>'}")


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="JDY-18 glove → AuraOS safe teleoperation")
    parser.add_argument("--scan", action="store_true", help="list nearby BLE devices, then exit")
    parser.add_argument(
        "--scan-details",
        action="store_true",
        help="list advertisements with service UUIDs and manufacturer data, then exit",
    )
    parser.add_argument("--scan-seconds", type=float, default=8.0)
    parser.add_argument("--address", help="BLE MAC address of the glove; preferred after scanning")
    parser.add_argument("--name", default="JDY", help="case-insensitive BLE name substring when --address is omitted")
    parser.add_argument("--characteristic", help="notify characteristic UUID; auto-detects JDY FFE1 by default")
    parser.add_argument("--api-base", default=DEFAULT_API_BASE)
    parser.add_argument("--log-dir", default="./logs")
    parser.add_argument("--enable-motion", action="store_true", help="explicitly permit API motion commands")
    parser.add_argument("--dry-run", action="store_true", help="map and log commands but never call AuraOS")
    parser.add_argument(
        "--debug-notifications",
        action="store_true",
        help="print every raw BLE notification; use only while diagnosing the glove",
    )
    parser.add_argument("--command-hz", type=float, default=10.0)
    parser.add_argument("--frame-timeout", type=float, default=0.25, help="stop when glove data is older than this many seconds")
    parser.add_argument("--reconnect-seconds", type=float, default=2.0)
    parser.add_argument("--neutral-samples", type=int, default=30, help="still-hand samples captured before motion can arm")
    parser.add_argument("--linear-axis", choices=("x", "y", "z"), default="y")
    parser.add_argument("--angular-axis", choices=("x", "y", "z"), default="x")
    parser.add_argument("--max-linear", type=float, default=0.12, help="m/s, limited below AuraOS configured 0.15")
    parser.add_argument("--max-angular", type=float, default=0.16, help="rad/s, limited below AuraOS configured 0.20")
    parser.add_argument("--full-scale-degrees", type=float, default=35.0)
    parser.add_argument("--angle-deadband", type=float, default=8.0)
    parser.add_argument("--invert-linear", action="store_true")
    parser.add_argument("--invert-angular", action="store_true")
    parser.add_argument("--deadman-finger", type=int, choices=(-1, 0, 1, 2, 3, 4), default=0)
    parser.add_argument("--deadman-threshold", type=float, default=60.0)
    args = parser.parse_args(argv)
    if args.command_hz <= 0 or args.frame_timeout <= 0 or args.neutral_samples <= 0:
        parser.error("command-hz, frame-timeout, and neutral-samples must be positive")
    if args.max_linear < 0 or args.max_angular < 0 or args.full_scale_degrees <= 0:
        parser.error("speed limits must be non-negative and full-scale-degrees must be positive")
    return args


def main() -> int:
    args = parse_args()
    try:
        if args.scan or args.scan_details:
            asyncio.run(scan(args.scan_seconds, details=args.scan_details))
        else:
            asyncio.run(Teleop(args).run())
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
