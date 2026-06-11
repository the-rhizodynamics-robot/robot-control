"""Alert notifier.

Email is the final phase; for now alerts and heartbeats are log-only, so the
rest of the system can be built and tested without SMTP credentials.
"""
from __future__ import annotations

import logging


class Notifier:
    """Default notifier: writes alerts/heartbeats to the log only."""

    def __init__(self, logger: logging.Logger | None = None):
        self.log = logger or logging.getLogger("robot")

    def alert(self, subject: str, body: str = "") -> None:
        self.log.warning("ALERT: %s | %s", subject, body)

    def heartbeat(self, body: str = "") -> None:
        self.log.info("HEARTBEAT: %s", body)


class EmailNotifier(Notifier):
    """SMTP email alerts. TODO: implement in the final phase.

    Plan: port send_email() / password_return() from robot_runner.ipynb.
    Credentials come from env vars (ROBOT_EMAIL_USER / ROBOT_EMAIL_APP_PASSWORD)
    or getpass at startup -- never hardcoded. On any send failure, fall back to
    the log-only behaviour above so a flaky mail server never crashes a run.
    """

    def __init__(self, to_addr: str, from_addr: str | None = None,
                 app_password: str | None = None,
                 logger: logging.Logger | None = None):
        super().__init__(logger)
        raise NotImplementedError("EmailNotifier is implemented in the final phase")
