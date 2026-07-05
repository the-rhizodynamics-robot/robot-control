"""Optional in-process camera capture via PySpin (FLIR Spinnaker).

This replaces manually pointing SpinView at the run folder. When PySpin and a
camera are available, the runner arms the camera for the SAME hardware trigger
the Arduino already sends (Line0, rising edge) and, in a background thread,
saves each triggered frame as a JPG into the run folder. Capture therefore
starts and stops WITH the run, and every save is logged -- so there is no
Record-dialog state, buffer policy, or silent self-stop to babysit.

Fail-soft by design: if PySpin isn't importable or no camera is found,
``start_capture()`` returns ``None`` and the caller falls back to the manual
"point your capture software here" prompt -- the SpinView workflow is unchanged
on machines without PySpin.

To USE integrated capture, run the host from an environment that has BOTH
pyserial and PySpin (e.g. the pyspin-env venv with `pip install pyserial`),
with the 64-bit Spinnaker CTI env vars set (see the Spinnaker 64-bit setup).
"""
from __future__ import annotations

import logging
import threading
from pathlib import Path
from typing import Optional


class CaptureUnavailable(Exception):
    """PySpin is missing or no camera is present; caller should fall back."""


class CameraCapture:
    """Grabs hardware-triggered frames in a background thread and saves JPGs.

    Lifecycle: construct -> start() -> (frames saved as triggers arrive) -> stop().
    All PySpin objects are created in start() and torn down in stop() in the
    strict order Spinnaker requires (release images, EndAcquisition, DeInit
    camera, clear list, release system).
    """

    def __init__(self, out_dir, logger: logging.Logger, user_set: str = "",
                 trigger_source: str = "Line0", grab_timeout_ms: int = 5000,
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

    @property
    def saved(self) -> int:
        return self._count

    # -- setup --------------------------------------------------------------
    def start(self) -> None:
        try:
            import PySpin  # lazy: absence => fall back, not a crash
        except ImportError as exc:
            raise CaptureUnavailable(f"PySpin not importable ({exc})") from exc
        self._spin = PySpin

        self._system = PySpin.System.GetInstance()
        self._cam_list = self._system.GetCameras()
        if self._cam_list.GetSize() == 0:
            self._cam_list.Clear()
            self._system.ReleaseInstance()
            self._system = self._cam_list = None
            raise CaptureUnavailable("no camera detected")

        self._cam = self._cam_list.GetByIndex(0)
        self._cam.Init()
        self._apply_user_set()
        self._configure_trigger()

        self._processor = PySpin.ImageProcessor()
        self._processor.SetColorProcessing(
            PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)

        self._out_dir.mkdir(parents=True, exist_ok=True)
        self._cam.BeginAcquisition()

        self._thread = threading.Thread(target=self._run, name="camera-capture",
                                        daemon=True)
        self._thread.start()
        self._log.info("capture: armed on %s (rising edge), saving JPGs to %s",
                       self._trigger_source, self._out_dir)

    def _apply_user_set(self) -> None:
        """Load a camera User Set (e.g. one tuned + saved in SpinView), if named."""
        if not self._user_set:
            return
        PySpin = self._spin
        try:
            nodemap = self._cam.GetNodeMap()
            sel = PySpin.CEnumerationPtr(nodemap.GetNode("UserSetSelector"))
            entry = sel.GetEntryByName(self._user_set)
            if entry is None:
                self._log.warning("capture: user set '%s' not found; leaving current settings",
                                  self._user_set)
                return
            sel.SetIntValue(entry.GetValue())
            PySpin.CCommandPtr(nodemap.GetNode("UserSetLoad")).Execute()
            self._log.info("capture: loaded user set '%s'", self._user_set)
        except PySpin.SpinnakerException as exc:
            self._log.warning("capture: could not load user set '%s' (%s)",
                              self._user_set, exc)

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
                converted.Save(str(path))
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
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=self._grab_timeout_ms / 1000.0 + 2.0)
        PySpin = self._spin
        try:
            if self._cam is not None:
                for step in (
                    lambda: self._cam.EndAcquisition(),
                    lambda: self._cam.TriggerMode.SetValue(PySpin.TriggerMode_Off),
                    lambda: self._cam.DeInit(),
                ):
                    try:
                        step()
                    except Exception:
                        pass
        finally:
            self._cam = None
            try:
                if self._cam_list is not None:
                    self._cam_list.Clear()
                if self._system is not None:
                    self._system.ReleaseInstance()
            except Exception:
                pass
            self._cam_list = self._system = None
            self._log.info("capture: stopped, %d images saved", self._count)


def start_capture(out_dir, logger: logging.Logger, user_set: str = "",
                  trigger_source: str = "Line0") -> Optional[CameraCapture]:
    """Try to start in-process capture; return the handle, or None to fall back.

    Never raises: any failure (no PySpin, no camera, config error) is logged and
    returns None so the caller can use the manual capture-software prompt.
    """
    try:
        cap = CameraCapture(out_dir, logger, user_set=user_set,
                            trigger_source=trigger_source)
        cap.start()
        return cap
    except CaptureUnavailable as exc:
        logger.warning("In-process capture unavailable (%s) - "
                       "falling back to manual capture software.", exc)
        return None
    except Exception as exc:  # noqa: BLE001 - never let capture setup kill the run
        logger.warning("In-process capture failed to start (%s) - "
                       "falling back to manual capture software.", exc)
        return None
