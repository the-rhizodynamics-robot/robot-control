"""Robot run configuration: defaults, optional TOML file, interactive prompts.

The operator is prompted for each value with a sensible default shown in
brackets; pressing Enter accepts the default. Numeric values are bounds-checked
(ported from the notebook's get_user_input()).
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

# Built-in fallback defaults. config.toml (if present) overrides these, and
# the interactive prompts override that.
DEFAULTS = {
    "com_port": "COM4",
    "num_shelves": 3,
    "photos_per_shelf": 8,
    "cycle_interval_min": 15,
    "day_hours": 24,          # 24 = constant light
    "start_hour": 0,
    "image_dir": r"D:\images\robot4",
    "kill_margin_min": 2,     # grace added to the cycle interval before a kill
}

# (low, high) inclusive bounds for numeric prompts.
BOUNDS = {
    "num_shelves": (1, 6),
    "photos_per_shelf": (1, 20),
    "cycle_interval_min": (1, 240),
    "day_hours": (0, 24),
    "start_hour": (0, 24),
    "kill_margin_min": (0, 60),
}


@dataclass
class Config:
    com_port: str
    num_shelves: int
    photos_per_shelf: int
    cycle_interval_min: int
    day_hours: int
    start_hour: int
    image_dir: str
    kill_margin_min: int

    @property
    def expected_images_per_cycle(self) -> int:
        return self.num_shelves * self.photos_per_shelf

    @property
    def cycle_timeout_s(self) -> float:
        """How long the watchdog waits for 'home' before killing."""
        return (self.cycle_interval_min + self.kill_margin_min) * 60

    def handshake_values(self) -> list[int]:
        """Values sent to the firmware, in the protocol-contract order.

        Order MUST match robot_device_serial.ino's setup() handshake.
        The trailing 1 is the 'start' signal.
        """
        return [
            self.num_shelves,
            self.photos_per_shelf,
            self.cycle_interval_min,
            self.day_hours,
            self.start_hour,
            1,
        ]


def load_defaults(toml_path: str | Path = "config.toml") -> dict:
    """Merge config.toml (if present and parseable) over the built-in DEFAULTS."""
    merged = dict(DEFAULTS)
    path = Path(toml_path)
    if not path.exists():
        return merged
    try:
        import tomllib  # Python 3.11+
        with path.open("rb") as fh:
            data = tomllib.load(fh)
        merged.update({k: v for k, v in data.items() if k in DEFAULTS})
    except Exception as exc:  # missing tomllib or malformed file -> just use defaults
        print(f"(could not read {path}: {exc}; using built-in defaults)")
    return merged


def make_run_dir(base: str | Path, num_shelves: int, boxes_per_shelf: int) -> Path:
    """Create and return a fresh, timestamped run folder inside `base`.

    The folder is named '<YYYYMMDDHHMMSS>_<num_shelves>_<boxes_per_shelf>'
    (timestamp to the second; the timestamp itself contains no underscores).
    The trailing '_<shelves>_<boxes>' is load-bearing: the downstream
    file-sorting tool reads the imaging geometry from the run-folder name --
    the number after the *final* underscore is boxes-per-shelf and the number
    before it is the shelf count -- so both must stay at the end of the name,
    in that order. (`boxes_per_shelf` is the robot's photos-per-shelf: one
    photo per box position per shelf per cycle.)
    """
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    run_dir = Path(base) / f"{timestamp}_{num_shelves}_{boxes_per_shelf}"
    run_dir.mkdir(parents=True, exist_ok=True)
    return run_dir


def _prompt_int(name: str, default: int, lo: int, hi: int) -> int:
    while True:
        raw = input(f"{name} [{default}] (between {lo} and {hi}): ").strip()
        if raw == "":
            return default
        try:
            val = int(raw)
        except ValueError:
            print("  please enter a whole number")
            continue
        if lo <= val <= hi:
            return val
        print(f"  must be between {lo} and {hi}")


def _prompt_str(name: str, default: str) -> str:
    raw = input(f"{name} [{default}]: ").strip()
    return raw if raw else default


def prompt_config(defaults: dict | None = None) -> Config:
    """Interactively build a Config, starting from `defaults`."""
    d = dict(DEFAULTS)
    if defaults:
        d.update(defaults)

    print("\nRobot run configuration (press Enter to accept the [default]):\n")
    com_port = _prompt_str("COM port", d["com_port"])
    num_shelves = _prompt_int("Shelves to image", d["num_shelves"], *BOUNDS["num_shelves"])
    photos_per_shelf = _prompt_int("Photos per shelf", d["photos_per_shelf"], *BOUNDS["photos_per_shelf"])
    cycle_interval_min = _prompt_int("Cycle interval (min)", d["cycle_interval_min"], *BOUNDS["cycle_interval_min"])
    day_hours = _prompt_int("Daylight hours per 24h (24 = constant light)", d["day_hours"], *BOUNDS["day_hours"])
    start_hour = _prompt_int("Current hour into the day cycle", d["start_hour"], *BOUNDS["start_hour"])
    image_dir = _prompt_str(
        "Output directory (a timestamped run folder is created inside)",
        d["image_dir"],
    )
    kill_margin_min = _prompt_int("Kill margin past interval (min)", d["kill_margin_min"], *BOUNDS["kill_margin_min"])

    return Config(
        com_port=com_port,
        num_shelves=num_shelves,
        photos_per_shelf=photos_per_shelf,
        cycle_interval_min=cycle_interval_min,
        day_hours=day_hours,
        start_hour=start_hour,
        image_dir=image_dir,
        kill_margin_min=kill_margin_min,
    )
