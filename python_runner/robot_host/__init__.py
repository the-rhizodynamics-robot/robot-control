"""robot_host: operator-facing host program for the plant-imaging robot.

Collects run configuration, hands it to the Arduino (robot_device_serial.ino)
over a serial handshake, then supervises the run: watching for the per-cycle
"home" report and killing the robot if a report is late or the camera stops
saving images.
"""

__all__ = ["config", "link", "monitor", "notifier"]
