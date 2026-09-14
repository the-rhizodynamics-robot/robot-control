"""Entry point: python -m robot_host

Flow: load defaults -> prompt for config -> confirm capture software ->
open serial + handshake -> supervise cycles. Any exit path (kill, Ctrl-C,
fatal error) sends the killcode so the robot never keeps running unattended
after the host stops.
"""
from __future__ import annotations

import logging
import sys

from .capture import start_capture
from .config import load_defaults, make_run_dir, prompt_config
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

    # Mint the per-run output folder inside the operator's chosen path and
    # point the rest of the run at it. Done before the FlyCap/Spinnaker prompt
    # so the operator can copy this exact path into the capture software.
    run_dir = make_run_dir(cfg.image_dir, cfg.num_shelves, cfg.photos_per_shelf)
    cfg.image_dir = str(run_dir)

    logger.info(
        "Config: %d shelves x %d photos, %d-min cycle, day_hours=%d, port=%s",
        cfg.num_shelves, cfg.photos_per_shelf, cfg.cycle_interval_min,
        cfg.day_hours, cfg.com_port,
    )
    logger.info("Run output folder: %s", run_dir)

    # Try to run capture ourselves (PySpin). If that isn't available, fall back
    # to the operator pointing external capture software (SpinView/FlyCap) at
    # the run folder, exactly as before.
    capture = None
    if cfg.use_internal_capture:
        capture = start_capture(run_dir, logger, user_set=cfg.camera_user_set)

    if capture:
        input("\nIn-process camera capture is running (frames saved on each "
              "trigger). Press Enter to start the robot... ")
    else:
        input(f"\nPoint FlyCap/Spinnaker to save into {cfg.image_dir} and confirm it is "
              "running, then press Enter to start... ")

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
        if capture:
            capture.stop()
        if link:
            link.close()
        logger.info("Serial closed.")


if __name__ == "__main__":
    main()
