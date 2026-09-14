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
- **Camera:** a FLIR/Point Grey camera captured by **FlyCap/Spinnaker** on the host PC, fired by
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
- **Camera capture:** by default the host captures images itself with PySpin, so run it
  from the `pyspin-env` venv and keep **SpinView closed**. Only one program can hold the
  camera. See [Camera capture](#camera-capture). With `use_internal_capture = false`,
  run FlyCap/Spinnaker yourself and save into the run folder instead.
- **Close the Arduino IDE Serial Monitor** before running the host — only one program
  can hold the COM port at a time.

> ⚠️ **The host machine must stay awake and not reboot during a run.** The host is a
> watchdog that has to be alive every cycle; if it sleeps or restarts, it stops feeding
> the watchdog and the run is lost (and on resume it may even false-kill the robot).
> On the dedicated control PC, disable all of the following:
> - **Sleep / display-off** — Power Options → set "Put the computer to sleep" to **Never**
>   (also under the active power plan's advanced settings).
> - **USB selective suspend** — Power Options → Advanced → USB settings → **Disabled**
>   (keeps the Arduino's COM port from dropping).
> - **Automatic update reboots** — Windows Update will install patches and reboot itself
>   (this is what a monthly "Patch Tuesday" restart is). Either **Pause updates** for the
>   duration of an experiment, or set the policy
>   `HKLM\SOFTWARE\Policies\Microsoft\Windows\WindowsUpdate\AU\NoAutoRebootWithLoggedOnUsers = 1`
>   so updates install but never auto-reboot — then reboot manually between runs.

### Run

```bat
cd python_runner
C:\Users\zhu.lab\pyspin-env\Scripts\python.exe -m robot_host
```

(`pyspin-env` is the Python 3.10 venv with PySpin, pyserial and tomli. Plain `python`
works only for external capture.)

You'll be prompted for the run configuration (press **Enter** to accept each
`[default]`). Defaults come from `config.toml`. With in-process capture the host loads the
camera User Set, logs the camera settings, arms the camera (`capture: armed on Line0`),
writes `run_config.json` into the run folder, and asks you to press Enter to start the robot.
It then opens the port, performs the handshake (each value should log `... -> OK`), and
begins supervising. Output goes to stdout and to `robot_host.log`.

### Configuration

Edit `python_runner/config.toml` to change the defaults the prompts start from:

| key | meaning |
|---|---|
| `com_port` | serial port, e.g. `COM4` |
| `num_shelves`, `photos_per_shelf` | run size (also sets expected images/cycle) |
| `cycle_interval_min` | minutes between cycle starts |
| `day_hours` | daylight hours per 24 h (`24` = constant light) |
| `start_hour` | hour into the day cycle at startup |
| `image_dir` | **base** output directory; the host creates a per-run folder inside it (see below) |
| `kill_margin_min` | grace added to the interval before a late-`home` kill |
| `use_internal_capture` | `true`: the host captures via PySpin. `false`: run SpinView/FlyCap yourself (not prompted) |
| `camera_user_set` | camera User Set loaded at capture start, e.g. `"UserSet1"` (not prompted) |

> TOML note: Windows paths use single-quoted **literal** strings so backslashes are
> taken verbatim, e.g. `image_dir = 'D:\images\robot4'`.

### Output folder naming

`image_dir` is a **base** directory, not the final image folder. At run start the host
creates a fresh per-run folder inside it named:

```
<YYYYMMDDHHMMSS>_<num_shelves>_<photos_per_shelf>     e.g. 20260613120000_3_2
```

(timestamp to the second; the timestamp itself has no underscores). The host prints this
path and points the watchdog at it. With in-process capture the images and
`run_config.json` go there automatically. With external capture, **repoint FlyCap/Spinnaker
to save into this new folder each run** (it's printed at the "Point FlyCap/Spinnaker…" prompt).

> ⚠️ **The trailing `_<shelves>_<boxes>` is a contract with downstream processing.** The
> [file-sorting](https://github.com/the-rhizodynamics-robot/file-sorting) pipeline reads the
> imaging geometry straight from the folder name (`<timestamp>_<shelves>_<boxes>`, boxes =
> `photos_per_shelf`). Keep both counts as the final two underscore-segments, as plain
> integers, or sorting downstream breaks. Implemented in
> `robot_host/config.py::make_run_dir()`.

### Camera capture

**In-process (default).** The host opens the camera with PySpin, loads the User Set named
by `camera_user_set`, arms it on the Arduino's hardware trigger (Line0, rising edge), and
saves each frame as `robotcap-NNNNNN.jpg` (JPEG quality 100, like SpinView's recorder).
Capture starts and stops with the run. This replaces SpinView for capture: SpinView's
recorder leaks ~34 MB per frame and ended every long run at ~3,400 images when Windows ran
out of memory. The in-process path was bench-tested flat over 300 frames.

**Imaging settings live in the camera, not in git.** Exposure, gain, gamma, white balance
and crop are stored in a camera User Set (`UserSet1`), which survives power cycles. To change them:

1. Set `use_internal_capture = false` in `config.toml` and start a run. The firmware only
   turns the lights on during a run, so tune with the run going.
2. Open SpinView and adjust the settings on the live image.
3. Save them to the camera: **User Set Control → User Set Selector = UserSet1 → User Set
   Save**. Keep **User Set Default = UserSet1** so the camera also powers up with them.
4. Ctrl-C the run, close SpinView, set `use_internal_capture = true`, and start the real run.

**`run_config.json`.** Every run folder gets a visible `run_config.json`, written before the
robot starts. It holds the run geometry and settings, the host code's git commit and, with
in-process capture, the camera settings actually in use (model, serial, firmware, crop,
exposure, gain, gamma, white-balance ratios, trigger, User Set, JPEG quality). The same
settings are logged to `robot_host.log`. This is the record of what a run used, and the
first step of the run-config manifest on the roadmap. file-sorting ignores non-image files,
so it can sit beside the images.

---

## Supervision & safety

The host is a **watchdog, not a driver** — the Arduino runs the whole cycle on its own
and only reports in. Each cycle the host:

1. **Waits for `home`** within `(cycle_interval_min + kill_margin_min) × 60` seconds.
   No `home` in time → **late report → kill**.
2. On `home`, **counts new image files** in `image_dir` since the last cycle (image
   extensions only, so `run_config.json` doesn't count):
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
