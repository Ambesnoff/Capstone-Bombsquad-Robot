"""Bounded process recovery. Each child establishes a new inhibited session."""
from __future__ import annotations

import argparse
import logging
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


BACKOFF_SECONDS = (2, 4, 8, 16)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config.json"))
    parser.add_argument("--telemetry", type=Path, default=Path("logs/robot.csv"))
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    stop = threading.Event()
    child: subprocess.Popen | None = None

    def shutdown(*_args: object) -> None:
        stop.set()
        if child is not None and child.poll() is None:
            child.terminate()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    # Bound total restart attempts, including repeated quick failures. Never
    # reset this counter just because the radio reconnects or a child ran longer.
    for attempt in range(len(BACKOFF_SECONDS) + 1):
        if stop.is_set():
            return 0
        logging.info("Supervisor start %d/5; new session requires neutral and a fresh arm cycle", attempt + 1)
        child = subprocess.Popen([
            sys.executable, str(Path(__file__).resolve().parents[1] / "robot_main.py"),
            "run", "--config", str(args.config), "--telemetry", str(args.telemetry),
        ])
        # A signal can arrive while process creation is pending, before the
        # handler can see this child. Preserve shutdown precedence afterward.
        if stop.is_set() and child.poll() is None:
            child.terminate()
        result = child.wait()
        if stop.is_set() or result == 0:
            return 0
        logging.error("Supervisor exited %d; preserving fault in journal; motion permission discarded", result)
        if attempt == len(BACKOFF_SECONDS):
            logging.error("Recovery limit reached; inspect robot and manually restart service")
            return result if 0 < result < 256 else 1
        delay = BACKOFF_SECONDS[attempt]
        logging.warning("Recovery attempt in %ds; HAT stop and fresh arm cycle remain required", delay)
        if stop.wait(delay):
            return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
