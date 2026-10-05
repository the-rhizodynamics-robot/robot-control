"""Optional in-process camera capture via PySpin (FLIR Spinnaker).

This replaces manually pointing SpinView at the run folder. When PySpin and a
camera are available, the runner arms the camera for the SAME hardware trigger
the Arduino already sends (Line0, rising edge) and, in a background thread,
saves each triggered frame as a JPG into the run folder. Capture therefore
starts and stops WITH the run, and every save is logged -- so there is no
Record-dialog state, buffer policy, or silent self-stop to babysit.

Strict by design: if in-process capture can't run exactly as configured (SpinView
is open, PySpin or the camera is missing, the User Set won't load),
``start_capture()`` raises ``CaptureUnavailable`` and the host refuses to start
the run. A run on the wrong settings, or falling back to SpinView and its memory
leak, is worse than no run. To capture with SpinView deliberately, set
``use_internal_capture = false``.

To USE integrated capture, run the host from an environment that has BOTH
pyserial and PySpin (e.g. the pyspin-env venv with `pip install pyserial`),
with the 64-bit Spinnaker CTI env vars set (see the Spinnaker 64-bit setup).
"""
from __future__ import annotations

import csv
import logging
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional


class CaptureUnavailable(Exception):
    """In-process capture can't run as configured; the host refuses to start."""


def spinview_running() -> list[str]:
    """SpinView processes currently running, as 'name (pid N)'. [] if none, or not on Windows."""
    try:
        out = subprocess.run(["tasklist", "/FO", "CSV", "/NH"], capture_output=True,
                             text=True, timeout=15).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [f"{row[0]} (pid {row[1]})" for row in csv.reader(out.splitlines())
            if len(row) >= 2 and "spinview" in row[0].lower()]


class CameraCapture:
    """Grabs hardware-triggered frames in a background thread and saves JPGs.

    Lifecycle: construct -> start() -> (frames saved as triggers arrive) -> stop().
    All PySpin objects are created in start() and torn down in stop() in the
    strict order Spinnaker requires (release images, EndAcquisition, DeInit
    camera, clear list, release system).
    """

    def __init__(self, out_dir, logger: logging.Logger, user_set: str = "",
                 trigger_source: str = "Line0", grab_timeout_ms: int = 1000,
                 name_prefix: str = "robotcap"):
        self._out_dir = Path(out_dir)
        self._log = logger
        self._user_set = user_set
        self._trigger_source = trigger_source
        self._grab_timeout_ms = grab_timeout_ms
        self._name_prefix = name_prefix

        self._spin = None
        self._system = None
        self._cam_list = None
        self._cam = None
        self._processor = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._count = 0
        self._acquiring = False
        self._loaded_user_set: Optional[str] = None
        self.settings: dict = {}   # camera settings in use, read at start()

    @property
    def saved(self) -> int:
        return self._count

    # -- setup --------------------------------------------------------------
    def start(self) -> None:
        # Checked first, before touching the camera: with SpinView holding it, the User Set
        # load is refused and the run would go ahead on whatever settings SpinView left.
        running = spinview_running()
        if running:
            raise CaptureUnavailable(
                f"SpinView is open: {', '.join(running)}. Close it first; only one "
                "program can hold the camera")
        try:
            import PySpin  # lazy, so a missing PySpin is a clear refusal, not an ImportError
        except ImportError as exc:
            raise CaptureUnavailable(
                f"PySpin not importable ({exc}); run the host from pyspin-env") from exc
        self._spin = PySpin

        try:
            self._system = PySpin.System.GetInstance()
            self._open_camera()
            try:
                self._prepare_camera()
            except PySpin.SpinnakerException as exc:
                # Usually a camera still acquiring from a run that didn't shut down cleanly
                # (window closed, crash, PC rebooted with the USB port powered). It refuses
                # User Set loads and acquisition settings until power-cycled, so reset it in
                # software -- the equivalent of unplugging it -- and try once more.
                self._log.warning("capture: camera refused setup (%s); resetting it and "
                                  "retrying", exc)
                self._reset_camera()
                try:
                    self._prepare_camera()
                except PySpin.SpinnakerException as exc2:
                    raise CaptureUnavailable(
                        f"camera refused setup even after a reset ({exc2}). Is another "
                        "program using the camera? If not, unplug and replug its USB "
                        "cable") from exc2

            self._processor = PySpin.ImageProcessor()
            self._processor.SetColorProcessing(
                PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)
            # PySpin defaults to JPEG quality 75; SpinView's recorder saved at 100, which
            # the downstream analysis was tuned on.
            self._jpeg = PySpin.JPEGOption()
            self._jpeg.quality = 100

            self.settings = self._read_settings()
            self.settings.update(user_set_loaded=self._loaded_user_set,
                                 jpeg_quality=self._jpeg.quality,
                                 color_processing="HQ_LINEAR")
            self._log.info("capture: camera settings: %s",
                           ", ".join(f"{k}={v}" for k, v in self.settings.items()))

            self._out_dir.mkdir(parents=True, exist_ok=True)
            self._cam.BeginAcquisition()
            self._acquiring = True
        except Exception:
            self._teardown()   # release the camera so the next attempt can open it
            raise

        self._thread = threading.Thread(target=self._run, name="camera-capture",
                                        daemon=True)
        self._thread.start()
        self._log.info("capture: armed on %s (rising edge), saving JPGs to %s",
                       self._trigger_source, self._out_dir)

    def _open_camera(self) -> None:
        """Find the (single) camera, Init it, and stop any acquisition left running."""
        self._cam_list = self._system.GetCameras()
        if self._cam_list.GetSize() == 0:
            raise CaptureUnavailable("no camera detected")
        cam = self._cam_list.GetByIndex(0)
        try:
            cam.Init()
        except Exception:
            del cam   # Spinnaker won't release the system while a camera reference lives
            raise
        self._cam = cam
        self._stop_stale_acquisition()

    def _node(self, nodemap, name: str, ptr_type):
        """`name` as a `ptr_type` pointer, or None if this camera lacks it."""
        node = nodemap.GetNode(name)
        if node is None:
            return None
        ptr = ptr_type(node)
        return ptr if self._spin.IsAvailable(ptr) else None

    def _stop_stale_acquisition(self) -> None:
        """Stop acquisition a previous, uncleanly ended run may have left running.

        A fresh Init() doesn't stop it, and while it runs the camera refuses User Set loads
        and acquisition settings. Best effort and harmless on an idle camera; if it doesn't
        clear the camera, start() falls back to a reset.
        """
        PySpin = self._spin
        nodemap = self._cam.GetNodeMap()
        try:
            stop = self._node(nodemap, "AcquisitionStop", PySpin.CCommandPtr)
            if stop is not None and PySpin.IsWritable(stop):
                stop.Execute()
            locked = self._node(nodemap, "TLParamsLocked", PySpin.CIntegerPtr)
            if locked is not None and PySpin.IsWritable(locked) and locked.GetValue() != 0:
                locked.SetValue(0)
        except PySpin.SpinnakerException as exc:
            self._log.debug("capture: clearing stale acquisition: %s", exc)

    def _prepare_camera(self) -> None:
        self._apply_user_set()
        self._configure_trigger()

    def _reset_camera(self, timeout_s: float = 30.0) -> None:
        """Reboot the camera (DeviceReset) and reopen it: unplugging it, in software."""
        PySpin = self._spin
        reset = self._node(self._cam.GetNodeMap(), "DeviceReset", PySpin.CCommandPtr)
        if reset is None or not PySpin.IsWritable(reset):
            raise CaptureUnavailable(
                "camera refused setup and can't be reset in software. Is another program "
                "using the camera? If not, unplug and replug its USB cable")
        reset.Execute()
        del reset
        self._release_camera(quiet=True)   # the old handle is dead once the camera reboots
        time.sleep(5)                      # let it drop off the bus before looking again
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                self._open_camera()
                self._log.info("capture: camera back after reset")
                return
            except (CaptureUnavailable, PySpin.SpinnakerException) as exc:
                self._release_camera(quiet=True)
                if time.monotonic() > deadline:
                    raise CaptureUnavailable(
                        f"camera did not come back within {timeout_s:.0f} s of a reset "
                        f"({exc}); unplug and replug its USB cable") from exc
                time.sleep(1)

    def _apply_user_set(self) -> None:
        """Load a camera User Set (e.g. one tuned + saved in SpinView), if named.

        A refused load raises SpinnakerException, which start() answers with a reset.
        """
        if not self._user_set:
            return
        PySpin = self._spin
        nodemap = self._cam.GetNodeMap()
        sel = PySpin.CEnumerationPtr(nodemap.GetNode("UserSetSelector"))
        entry = sel.GetEntryByName(self._user_set)
        if entry is None:
            raise CaptureUnavailable(
                f"camera has no user set '{self._user_set}' (check camera_user_set "
                "in config.toml)")
        sel.SetIntValue(entry.GetValue())
        PySpin.CCommandPtr(nodemap.GetNode("UserSetLoad")).Execute()
        self._loaded_user_set = self._user_set
        self._log.info("capture: loaded user set '%s'", self._user_set)

    def _configure_trigger(self) -> None:
        """Hardware trigger: FrameStart on the trigger line, rising edge."""
        PySpin = self._spin
        cam = self._cam
        # Turn trigger off to reconfigure, then re-enable.
        cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)
        cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
        cam.TriggerSelector.SetValue(PySpin.TriggerSelector_FrameStart)
        src = getattr(PySpin, "TriggerSource_" + self._trigger_source)
        cam.TriggerSource.SetValue(src)
        cam.TriggerActivation.SetValue(PySpin.TriggerActivation_RisingEdge)
        cam.TriggerMode.SetValue(PySpin.TriggerMode_On)

    # Recorded in the log and in each run's run_config.json. The imaging settings live in
    # the camera's User Set, not in git, so this is the record of what a run actually used.
    RECORDED_NODES = (
        "DeviceModelName", "DeviceSerialNumber", "DeviceFirmwareVersion",
        "PixelFormat", "Width", "Height", "OffsetX", "OffsetY",
        "ExposureAuto", "ExposureTime", "GainAuto", "Gain",
        "BalanceWhiteAuto", "GammaEnable", "Gamma", "BlackLevel",
        "TriggerSelector", "TriggerSource", "TriggerActivation", "TriggerMode",
        "UserSetDefault",
    )

    def _read_settings(self) -> dict:
        """Current camera settings as strings; nodes this camera lacks are skipped."""
        PySpin = self._spin
        nodemap = self._cam.GetNodeMap()
        out = {}
        for name in self.RECORDED_NODES:
            node = nodemap.GetNode(name)
            try:
                if node is not None and PySpin.IsAvailable(node) and PySpin.IsReadable(node):
                    out[name] = PySpin.CValuePtr(node).ToString()
            except PySpin.SpinnakerException:
                pass
        # White balance is one ratio per colour channel, behind a selector; read both and put
        # the selector back.
        try:
            sel = PySpin.CEnumerationPtr(nodemap.GetNode("BalanceRatioSelector"))
            ratio = PySpin.CFloatPtr(nodemap.GetNode("BalanceRatio"))
            if PySpin.IsWritable(sel) and PySpin.IsReadable(ratio):
                original = sel.GetIntValue()
                for ch in ("Red", "Blue"):
                    entry = sel.GetEntryByName(ch)
                    if entry is not None and PySpin.IsReadable(entry):
                        sel.SetIntValue(entry.GetValue())
                        out[f"BalanceRatio{ch}"] = f"{ratio.GetValue():.5g}"
                sel.SetIntValue(original)
        except PySpin.SpinnakerException:
            pass
        return out

    # -- capture loop -------------------------------------------------------
    def _run(self) -> None:
        PySpin = self._spin
        while not self._stop.is_set():
            try:
                img = self._cam.GetNextImage(self._grab_timeout_ms)
            except PySpin.SpinnakerException as exc:
                # A timeout just means no trigger arrived in the window -- that
                # is the normal idle state between cycles, so keep waiting.
                if "timed out" not in str(exc).lower():
                    self._log.debug("capture: GetNextImage error: %s", exc)
                continue
            try:
                if img.IsIncomplete():
                    self._log.warning("capture: incomplete frame (status %s)",
                                      img.GetImageStatus())
                    continue
                converted = self._processor.Convert(img, PySpin.PixelFormat_BGR8)
                path = self._out_dir / f"{self._name_prefix}-{self._count:06d}.jpg"
                converted.Save(str(path), self._jpeg)
                self._count += 1
                if self._count <= 5 or self._count % 20 == 0:
                    self._log.info("capture: %d images saved (latest %s)",
                                   self._count, path.name)
            finally:
                try:
                    img.Release()
                except Exception:
                    pass

    # -- teardown -----------------------------------------------------------
    def stop(self, join_timeout: Optional[float] = None) -> None:
        """Stop capturing and release the camera.

        join_timeout bounds the wait for the capture thread (default: one grab timeout plus
        2 s). The console-close handler passes a short one: Windows kills the process ~5 s
        after the window is closed.
        """
        self._stop.set()
        if self._thread is not None:
            if join_timeout is None:
                join_timeout = self._grab_timeout_ms / 1000.0 + 2.0
            self._thread.join(timeout=join_timeout)
            if self._thread.is_alive():
                self._log.warning("capture: capture thread still busy; releasing the "
                                  "camera anyway")
        self._teardown()
        self._log.info("capture: stopped, %d images saved", self._count)

    def _release_camera(self, quiet: bool = False) -> None:
        """DeInit and drop the camera handle and list. quiet: failures are expected."""
        log = self._log.debug if quiet else self._log.warning
        if self._cam is not None:
            try:
                self._cam.DeInit()
            except Exception as exc:
                log("capture: teardown: DeInit failed: %s", exc)
            self._cam = None
        if self._cam_list is not None:
            try:
                self._cam_list.Clear()
            except Exception as exc:
                log("capture: teardown: clearing the camera list failed: %s", exc)
            self._cam_list = None

    def _teardown(self) -> None:
        """Release the camera and the Spinnaker system. Safe on a half-started capture.

        Failures are logged, not raised: a camera left acquiring is what makes the next
        start fail ("Is another program using the camera?"), so the log should say so.
        """
        PySpin = self._spin
        if self._cam is not None:
            steps = [("trigger off", lambda: self._cam.TriggerMode.SetValue(
                PySpin.TriggerMode_Off))]
            if self._acquiring:
                steps.insert(0, ("EndAcquisition", self._cam.EndAcquisition))
            for name, step in steps:
                try:
                    step()
                except Exception as exc:
                    self._log.warning("capture: teardown: %s failed: %s", name, exc)
            self._acquiring = False
        self._release_camera()
        if self._system is not None:
            try:
                self._system.ReleaseInstance()
            except Exception as exc:
                self._log.warning("capture: teardown: releasing Spinnaker failed: %s", exc)
            self._system = None


def start_capture(out_dir, logger: logging.Logger, user_set: str = "",
                  trigger_source: str = "Line0") -> CameraCapture:
    """Start in-process capture and return the handle.

    Raises CaptureUnavailable, saying why, if it can't run as configured. The caller
    refuses to start the run.
    """
    cap = CameraCapture(out_dir, logger, user_set=user_set, trigger_source=trigger_source)
    try:
        cap.start()
    except CaptureUnavailable:
        raise
    except Exception as exc:  # noqa: BLE001 - any camera setup error means no run
        raise CaptureUnavailable(f"camera setup failed ({exc})") from exc
    return cap
