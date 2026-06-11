"""Serial link to the Arduino running robot_device_serial.ino.

Protocol (must match the firmware):
  - handshake: host sends each config value as ASCII text; firmware echoes
    the parsed int back (newline-terminated) for verification.
  - report:    firmware prints "home" once per completed cycle.
  - kill:      host sends KILLCODE ("2048") at any time to stop the robot.
Link: 9600 baud, firmware uses Serial.setTimeout(2).
"""
from __future__ import annotations

import time
from typing import Callable, Optional

import serial

KILLCODE = "2048"
BAUD = 9600
HOME_TOKEN = "home"


class RobotLink:
    def __init__(self, port: str, baud: int = BAUD, timeout: float = 0.3,
                 reset_delay: float = 2.0):
        # Opening the port resets the Arduino; wait for it to boot into setup().
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(reset_delay)
        self.ser.reset_input_buffer()

    # -- handshake ----------------------------------------------------------
    def _send_value(self, value: int) -> str:
        """Send one handshake value and return the firmware's echoed line."""
        self.ser.write(str(value).encode())
        self.ser.flush()
        time.sleep(0.3)  # give the firmware time to echo
        return self.ser.readline().decode(errors="ignore").strip()

    def handshake(self, values: list[int],
                  on_status: Optional[Callable[[str], None]] = None) -> bool:
        """Send config values in order; verify each echo. Returns True if all matched."""
        ok = True
        for v in values:
            echo = self._send_value(v)
            matched = echo == str(v)
            ok = ok and matched
            if on_status:
                flag = "OK" if matched else f"echo='{echo}' (mismatch)"
                on_status(f"handshake: sent {v} -> {flag}")
        return ok

    # -- runtime ------------------------------------------------------------
    def read_line(self) -> str:
        return self.ser.readline().decode(errors="ignore").strip()

    def wait_for_home(self, timeout_s: float,
                      on_status: Optional[Callable[[str], None]] = None) -> bool:
        """Block until a 'home' line arrives or timeout elapses.

        Non-home status lines (calibration/progress chatter) are passed to
        on_status as they arrive and do not reset the timeout.
        """
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            line = self.read_line()
            if not line:
                continue
            # Match the heartbeat line EXACTLY. The firmware also prints
            # status lines that contain the word "home" ("Returning to home
            # position...", "Photography sequence complete! Returning home...");
            # a substring test would treat each of those as a finished cycle.
            if line.strip().lower() == HOME_TOKEN:
                return True
            if on_status:
                on_status(f"robot: {line}")
        return False

    def send_kill(self) -> None:
        self.ser.write(KILLCODE.encode())
        self.ser.flush()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass
