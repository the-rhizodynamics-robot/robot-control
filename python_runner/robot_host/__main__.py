"""Entry point: python -m robot_host

Flow: load defaults -> prompt for config -> confirm capture software ->
open serial + handshake -> supervise cycles. Any exit path (kill, Ctrl-C, closing
the console window, fatal error) sends the killcode and releases the camera, so the
robot never keeps running unattended after the host stops.
"""
from __future__ import annotations

import logging
import sys
import threading

from .capture import CameraCapture, CaptureUnavailable, start_capture
from .config import load_defaults, make_run_dir, prompt_config, write_run_config
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


class RunEnd:
    """End-of-run cleanup: release the camera, close serial. Runs once, from any thread.

    Shared by main()'s finally and the console-close handler, which runs on its own
    thread and may race it.
    """

    def __init__(self, logger: logging.Logger, capture: CameraCapture | None):
        self.log = logger
        self.capture = capture
        self.link: RobotLink | None = None
        self._lock = threading.Lock()
        self._closed = False
        self._handler = None   # the console handler's ctypes callback, kept alive here

    def close(self, send_kill: bool = False, join_timeout: float | None = None) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            if send_kill and self.link:
                try:
                    self.link.send_kill()
                except Exception as exc:  # noqa: BLE001 - still release the camera
                    self.log.error("Could not send kill: %s", exc)
            if self.capture:
                self.capture.stop(join_timeout)
            if self.link:
                self.link.close()
                self.log.info("Serial closed.")


# Windows console control events that end the process without a KeyboardInterrupt.
_CONSOLE_CLOSE_EVENTS = {2: "console window closed", 5: "user logoff",
                         6: "system shutdown"}


def install_console_close_handler(end: RunEnd) -> None:
    """Send the kill and release the camera when the console window is closed.

    For these events Windows doesn't raise KeyboardInterrupt, it terminates the process,
    so main()'s finally never runs: no killcode, and the camera is left acquiring. The
    handler has ~5 s before the process is killed, so it sends the kill first and gives
    the capture thread a short join. Logoff/shutdown are best effort (Windows doesn't
    deliver them to every console process); capture.start()'s camera reset covers
    whatever this misses. Ctrl-C/Ctrl-Break are left to Python.
    """
    if sys.platform != "win32":
        return
    import ctypes
    from ctypes import wintypes

    def handler(event):
        reason = _CONSOLE_CLOSE_EVENTS.get(event)
        if reason is None:
            return False   # not ours: pass it on to Python's handler
        end.log.warning("%s - sending kill and releasing the camera", reason)
        end.close(send_kill=True, join_timeout=1.0)
        return True

    end._handler = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.DWORD)(handler)
    if not ctypes.windll.kernel32.SetConsoleCtrlHandler(end._handler, True):
        end.log.warning("Could not install the console-close handler; closing the window "
                        "will skip the kill and leave the camera armed")


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

    # In-process capture (PySpin) is all-or-nothing: if it can't run as configured,
    # refuse to start rather than run on the wrong camera settings or fall back to
    # SpinView. use_internal_capture = false selects external capture on purpose.
    capture = None
    if cfg.use_internal_capture:
        try:
            capture = start_capture(run_dir, logger, user_set=cfg.camera_user_set)
        except CaptureUnavailable as exc:
            logger.error("REFUSING TO START: %s", exc)
            logger.error("Fix that and start again (or set use_internal_capture = false "
                         "in config.toml to capture with SpinView). The robot was not started.")
            try:
                run_dir.rmdir()   # only succeeds if still empty
            except OSError:
                pass
            sys.exit(1)

    # From here on the camera is armed, so every way out must release it: a camera left
    # acquiring refuses the next run's User Set load until it is power-cycled.
    end = RunEnd(logger, capture)
    install_console_close_handler(end)
    try:
        # Record the run (geometry, settings, host commit, camera settings) in the run
        # folder before the robot starts.
        try:
            manifest = write_run_config(run_dir, cfg, capture.settings if capture else None)
            logger.info("Run config written: %s", manifest)
        except OSError as exc:
            logger.warning("Could not write run_config.json (%s) - continuing", exc)

        if capture:
            input("\nIn-process camera capture is running (frames saved on each "
                  "trigger). Press Enter to start the robot... ")
        else:
            input(f"\nPoint FlyCap/Spinnaker to save into {cfg.image_dir} and confirm it "
                  "is running, then press Enter to start... ")

        link: RobotLink | None = None
        try:
            logger.info("Opening %s ...", cfg.com_port)
            link = end.link = RobotLink(cfg.com_port)

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
    except KeyboardInterrupt:
        logger.warning("Interrupted before the robot started")
    finally:
        end.close()


if __name__ == "__main__":
    main()
