"""Entry point: python -m robot_host

Flow: load defaults -> prompt for config -> confirm capture software ->
open serial + handshake -> supervise cycles. Any exit path (kill, Ctrl-C,
fatal error) sends the killcode so the robot never keeps running unattended
after the host stops.
"""
from __future__ import annotations

import logging
import sys

from .config import load_defaults, prompt_config
from .link import RobotLink
from .monitor import Monitor, RobotStopped
from .notifier import Notifier


def setup_logging() -> logging.Logger:
    logger = logging.getLogger("robot")
    logger.setLevel(logging.INFO)
    fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s", "%H:%M:%S")
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(fmt)
    logger.addHandler(sh)
    fh = logging.FileHandler("robot_host.log")
    fh.setFormatter(fmt)
    logger.addHandler(fh)
    return logger


def main() -> None:
    logger = setup_logging()

    cfg = prompt_config(load_defaults())
    logger.info(
        "Config: %d shelves x %d photos, %d-min cycle, day_hours=%d, port=%s",
        cfg.num_shelves, cfg.photos_per_shelf, cfg.cycle_interval_min,
        cfg.day_hours, cfg.com_port,
    )

    input(f"\nConfirm FlyCap is running and saving to {cfg.image_dir}, "
          "then press Enter to start... ")

    link: RobotLink | None = None
    try:
        logger.info("Opening %s ...", cfg.com_port)
        link = RobotLink(cfg.com_port)

        logger.info("Handshaking...")
        if not link.handshake(cfg.handshake_values(), on_status=logger.info):
            logger.warning("Handshake echoes did not all match - "
                           "check the firmware/wiring before trusting the run")

        notifier = Notifier(logger)
        Monitor(link, cfg, notifier, logger).run()

    except RobotStopped as exc:
        logger.error("Run ended: %s", exc)
    except KeyboardInterrupt:
        logger.warning("Interrupted by operator - sending kill")
        if link:
            link.send_kill()
    except Exception as exc:  # noqa: BLE001 - last-resort safety net
        logger.exception("Fatal error - sending kill: %s", exc)
        if link:
            link.send_kill()
    finally:
        if link:
            link.close()
        logger.info("Serial closed.")


if __name__ == "__main__":
    main()
