#!/usr/bin/env python3
"""Monitor UDP glove frames on the RDK without controlling the robot."""

from __future__ import annotations

import argparse
import re
import socket
import time


FRAME = re.compile(r"^-?\d+(?:\.\d+)?(?:,-?\d+(?:\.\d+)?){7}$")
FINGER_NAMES = ("thumb", "index", "middle", "ring", "little")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5010)
    parser.add_argument("--duration", type=float, default=30.0)
    return parser.parse_args()


def format_frame(values: tuple[float, ...]) -> str:
    fingers = ", ".join(
        f"{name}={value:.1f}" for name, value in zip(FINGER_NAMES, values[:5])
    )
    # On the live glove firmware the final three columns are roll, pitch, yaw.
    roll, pitch, yaw = values[5:]
    return (
        f"fingers[{fingers}] "
        f"pose[roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}]"
    )


def main() -> None:
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", args.port))
    sock.settimeout(1.0)
    count = 0
    deadline = time.monotonic() + args.duration
    print(f"Listening for glove CSV on UDP {args.port}; monitor only", flush=True)
    while time.monotonic() < deadline:
        try:
            data, peer = sock.recvfrom(1024)
        except TimeoutError:
            continue
        line = data.decode("ascii", errors="ignore").strip()
        if not FRAME.fullmatch(line):
            continue
        values = tuple(float(item) for item in line.split(","))
        count += 1
        print(
            f"rx={count} from={peer[0]} {format_frame(values)} raw_csv={line}",
            flush=True,
        )
    print(f"Finished: received {count} valid frames", flush=True)


if __name__ == "__main__":
    main()
