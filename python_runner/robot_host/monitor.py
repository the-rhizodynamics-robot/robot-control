"""Watchdog loop: supervise the robot cycle-by-cycle.

Each cycle the firmware reports "home". The monitor:
  - kills the robot if no "home" arrives within (cycle_interval + margin);
  - counts newly-saved images and kills after two consecutive zero-image
    cycles (the camera-silently-failing case), alerting on other mismatches.
Image counting reuses the notebook's listdir_nohidden() approach.
"""
from __future__ import annotations

import logging
import os

from .config import Config
from .link import RobotLink


class RobotStopped(Exception):
    """Raised when the monitor has killed the robot and the run should end."""


def listdir_nohidden(path: str) -> list[str]:
    return [f for f in os.listdir(path) if not f.startswith(".")]


def count_images(image_dir: str) -> int:
    try:
        return len(listdir_nohidden(image_dir))
    except FileNotFoundError:
        return 0


class Monitor:
    def __init__(self, link: RobotLink, config: Config, notifier,
                 logger: logging.Logger | None = None,
                 heartbeat_every_cycles: int = 8):
        self.link = link
        self.cfg = config
        self.notifier = notifier
        self.log = logger or logging.getLogger("robot")
        self.heartbeat_every = heartbeat_every_cycles

    def _kill(self, reason: str) -> None:
        self.log.error("KILLING ROBOT: %s", reason)
        self.link.send_kill()
        self.notifier.alert("Robot killed", reason)
        raise RobotStopped(reason)

    def run(self) -> None:
        expected = self.cfg.expected_images_per_cycle
        timeout_s = self.cfg.cycle_timeout_s
        prev_count = count_images(self.cfg.image_dir)
        zero_streak = 0
        cycle = 0

        self.log.info(
            "Monitoring: expecting %d images/cycle, home within %.0fs",
            expected, timeout_s,
        )

        while True:
            got_home = self.link.wait_for_home(timeout_s, on_status=self.log.info)
            if not got_home:
                self._kill(
                    f"no 'home' within {timeout_s:.0f}s "
                    f"(interval {self.cfg.cycle_interval_min} + "
                    f"margin {self.cfg.kill_margin_min} min)"
                )

            cycle += 1
            now_count = count_images(self.cfg.image_dir)
            delta = now_count - prev_count
            prev_count = now_count

            if delta == expected:
                zero_streak = 0
                self.log.info("Cycle %d OK: %d new images (total %d)",
                              cycle, delta, now_count)
            elif delta == 0:
                zero_streak += 1
                msg = f"cycle {cycle}: NO images saved (streak {zero_streak})"
                self.log.warning(msg)
                self.notifier.alert("No images this cycle", msg)
                if zero_streak >= 2:
                    self._kill("two consecutive cycles with no images")
            else:
                zero_streak = 0
                msg = (f"cycle {cycle}: aberrant image count "
                       f"(expected {expected}, got {delta}) - continuing")
                self.log.warning(msg)
                self.notifier.alert("Aberrant image count", msg)

            if self.heartbeat_every and cycle % self.heartbeat_every == 0:
                self.notifier.heartbeat(
                    f"Robot alive: {cycle} cycles, "
                    f"{now_count} images so far"
                )
