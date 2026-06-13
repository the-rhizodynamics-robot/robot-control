# robot-control

Control system for an automated plant time-lapse imaging robot. A 2-axis stepper
gantry moves a camera across shelves of plants, triggers a photo at each position,
and repeats on a fixed schedule. The system has two halves:

- **Arduino firmware** that drives the motors, lights, and camera trigger.
- **A Python host program** (`robot_host`) that configures a run, hands the
  configuration to the Arduino over serial, and supervises it — killing the robot
  if it stops reporting in or stops saving images.

> **For both humans and AI assistants:** this README is the single source of truth
> for how the project is wired and how it runs. Read it before changing firmware
> pins/constants or the serial protocol — the two halves share a contract that
> breaks silently if only one side changes.

---

## Hardware

- **Board:** Arduino Mega 2560 (confirmed on the rig; pin map below assumes Mega).
- **Motion:** two stepper axes driven by step/dir/enable drivers.
  - *Horizontal (X):* moves the camera left/right across a shelf.
  - *Vertical (Y):* moves between shelves.
- **Homing:** two photointerrupters (one per axis) establish a repeatable home position.
- **Camera:** a FLIR/Point Grey camera captured by **FlyCap** on the host PC, fired by
  a hardware trigger from the Arduino. **The capture software must be running and saving
  to the configured image directory** for a run to produce images.
- **Lighting:** one relay per shelf (active-LOW) for grow lights.
- **Link:** USB serial between the Arduino and the host PC.

### Pin map (Mega 2560)

| Function | Pin(s) | Notes |
|---|---|---|
| Horizontal motor (X) step / dir / enable | 5 / 7 / 6 | enable is **active-LOW** |
| Vertical motor (Y) step / dir / enable | 8 / 10 / 9 | enable is **active-LOW** |
| Horizontal photointerrupter | D3 | `INPUT_PULLUP`; reads **LOW** when triggered |
| Vertical photointerrupter | D2 | `INPUT_PULLUP`; reads **LOW** when triggered |
| Camera trigger | D12 | ~**50 ms** HIGH pulse (a shorter pulse is ignored by the camera) |
| Shelf light relays | 23, 25, 27, 29, 31, 33 | index 0 = shelf 1; **active-LOW** |

**Directions:** right = `LOW`, left = `HIGH`, up = `LOW`, down = `HIGH`.

---

## Repository layout

```
arduino/
  robot_device_serial/robot_device_serial.ino  # host-driven firmware (serial handshake + watchdog) — the production sketch
  test_code/
    robot_device/...                       # standalone firmware (hardcoded config, no host) — run the gantry without Python
    photointerrupter_testing/...           # sensor bring-up sketch
    camera_trigger_test/...                # camera-trigger bring-up sketch
python_runner/
  robot_host/                              # the host program (run as: python -m robot_host)
    __main__.py                            # entry: prompt config -> handshake -> supervise
    config.py                              # defaults, bounds, interactive prompts, derived values
    link.py                                # serial open, handshake, home heartbeat, kill
    monitor.py                             # watchdog loop + image-count validation + kill decision
    notifier.py                            # alerts (log-only now; email is the final phase)
  config.toml                              # editable persistent defaults for the prompts
  requirements.txt                         # pyserial
  robot_runner.ipynb                       # legacy Jupyter controller (superseded by robot_host)
```

### Firmware sketches

- **`robot_device_serial.ino`** is the **production firmware**: the proven
  motion/camera/light logic with the serial protocol layered on. **This is the sketch
  the Python host drives.** Flash this for supervised operation.
- **`test_code/robot_device/robot_device.ino`** is a **standalone** version with the
  configuration compiled in — it never reads serial and never reports back. Handy for
  exercising the gantry/camera/lights without the host (or as a no-host fallback).

---

## Serial protocol (host ⇄ firmware contract)

Both sides must agree on this exactly; changing one side without the other fails silently.

- **Link:** 9600 baud, firmware uses `Serial.setTimeout(2)`.
- **Startup handshake** — host sends each value as ASCII text; firmware parses it,
  **echoes the parsed integer back** for verification, in this fixed order:

  | # | value | meaning |
  |---|---|---|
  | 1 | `num_shelves` | shelves to image |
  | 2 | `photos_per_shelf` | photos per shelf |
  | 3 | `cycle_interval_min` | minutes between cycle **starts** |
  | 4 | `day_hours` | daylight hours per 24 h (`24` = constant light) |
  | 5 | `start_hour` | current hour into the day cycle |
  | 6 | start signal | any non-kill value (e.g. `1`) |

- **Heartbeat:** the firmware prints a line that is exactly `home` once per completed
  cycle. (It also prints chatty status lines containing the word "home" — the host
  matches the heartbeat **exactly**, not as a substring.)
- **Kill:** the host sends `2048` (`KILLCODE`) at any time; the firmware stops.

### Timing model

`cycle_interval_min` is a **fixed cadence, not a gap**: the next cycle starts
`cycle_interval_min` after the previous one *started*. Each cycle the firmware stamps
the start time, does the (short, ~30 s) photography sweep, returns home, prints `home`,
then idles out the remainder of the interval before the next cycle. So `home` arrives
early in each interval, right after the photos for that cycle are taken.

---

## Running the host program

### Prerequisites

- **Python 3.11+** (developed on 3.12). Install `pyserial`:
  ```bash
  pip install -r python_runner/requirements.txt
  ```
- The `robot_device_serial.ino` sketch flashed to the Arduino (Arduino IDE).
- **FlyCap running** and saving images to the directory you'll configure.
- **Close the Arduino IDE Serial Monitor** before running the host — only one program
  can hold the COM port at a time.

### Run

```bash
cd python_runner
python -m robot_host
```

You'll be prompted for the run configuration (press **Enter** to accept each
`[default]`); defaults come from `config.toml`. After confirming FlyCap is running,
the host opens the port, performs the handshake (each value should log `... -> OK`),
and begins supervising. Output goes to stdout and to `robot_host.log`.

### Configuration

Edit `python_runner/config.toml` to change the defaults the prompts start from:

| key | meaning |
|---|---|
| `com_port` | serial port, e.g. `COM4` |
| `num_shelves`, `photos_per_shelf` | run size (also sets expected images/cycle) |
| `cycle_interval_min` | minutes between cycle starts |
| `day_hours` | daylight hours per 24 h (`24` = constant light) |
| `start_hour` | hour into the day cycle at startup |
| `image_dir` | where FlyCap saves images (the watchdog counts files here) |
| `kill_margin_min` | grace added to the interval before a late-`home` kill |

> TOML note: Windows paths use single-quoted **literal** strings so backslashes are
> taken verbatim, e.g. `image_dir = 'D:\images\robot4'`.

---

## Supervision & safety

The host is a **watchdog, not a driver** — the Arduino runs the whole cycle on its own
and only reports in. Each cycle the host:

1. **Waits for `home`** within `(cycle_interval_min + kill_margin_min) × 60` seconds.
   No `home` in time → **late report → kill**.
2. On `home`, **counts new image files** in `image_dir` since the last cycle:
   - delta == `num_shelves × photos_per_shelf` → OK.
   - delta == 0 → alert; **two consecutive zero-image cycles → kill** (the
     camera-silently-failing case).
   - other nonzero → aberrant count → alert, keep running.

A periodic heartbeat ("Robot alive: N cycles…") is logged every few cycles.

### Stopping the robot

- **Graceful stop:** press **Ctrl-C** in the host. It sends the killcode (`2048`)
  before closing the port, and the firmware halts (lights off, motors disabled).
- **Watchdog stop:** the host sends the same killcode automatically on a late `home`
  or two zero-image cycles.
- **Emergency stop:** the Arduino **reset button** (or pulling power) — instant, and
  independent of serial.
- ⚠️ **Do not** close the terminal window with the X or hard-kill the process. That
  skips the killcode, and the robot keeps cycling on its own (it doesn't need the host
  to continue). Note also that the firmware only checks for the killcode at safe points
  (between boxes, between shelves, during the inter-cycle wait), so a kill lands at the
  next checkpoint, not mid-move.

---

## Development workflow

### Firmware
1. Compile/upload with the Arduino IDE (or bundled `arduino-cli`). Target: Arduino Mega 2560.
2. Bring up components with the `arduino/test_code/` sketches before a full run.
3. Keep firmware serial settings in sync with the host (9600 baud, `Serial.setTimeout(2)`).
4. To run the gantry without the host (or as a fallback), flash the standalone
   `test_code/robot_device/robot_device.ino`.

### Host
1. `pip install -r python_runner/requirements.txt`.
2. Sanity-check without hardware: `python -m compileall robot_host` and step through the
   config prompts (Ctrl-C before it opens the port).
3. Any change to the handshake order/values **must** be mirrored in
   `robot_device_serial.ino`'s `setup()`.

### Roadmap / not yet done
- **Email alerts** (`notifier.EmailNotifier`) are the final phase — currently a stub;
  alerts are log-only. Credentials will come from environment variables / `getpass`,
  never hardcoded.
- Optional **non-interactive/headless mode** (read `config.toml`, skip prompts) so the
  host can be launched unattended (e.g. via Windows Task Scheduler).
