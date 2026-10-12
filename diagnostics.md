# Diagnostics: connecting to the Emio robot

Date: 2026-10-12

This document records what went wrong while connecting the Emio robot to `cabledriven.py`, how each problem was diagnosed and fixed, and what changed in the repository and in the Emio Labs install.

**Status:** the connection code has been fixed, but it has **not been tested end to end on the real robot**. After the last restart the robot was no longer visible to the computer (see [Open items](#open-items)).

## Summary of problems

| # | Problem | Cause | Status |
|---|---|---|---|
| 1 | The script never connected to the robot and silently ran in mock mode | `from emioapi import emio` does not exist in any published `emioapi` release | Fixed in `cabledriven.py` |
| 2 | Port `/dev/ttyUSB0` could not be opened | The user was not in the `dialout` group, and group changes only apply after a new login | Fixed (user added to group, session restarted) |
| 3 | Connecting would have moved the motors unexpectedly | `connect()` called `home()`, which sends all motors to 0 rad, but the motors were resting at other angles | Fixed in `cabledriven.py` |
| 4 | Robot not visible after the restart | Not diagnosed; USB board and camera are absent from `lsusb` | **Open** |

## 1. Wrong `emioapi` import (silent mock mode)

### Symptom
With the robot plugged in, `cabledriven.py` would never talk to it. Because of the `try/except ImportError`, it set `HAS_EMIO_API = False` and fell back to mock mode with no error about the real cause.

### Diagnosis
- The robot was visible to the system: an FTDI FT232H on `/dev/ttyUSB0` (serial `FT9BTLU1`) and a RealSense camera (serial `218622274394`). `EmioAPI.listEmioDevices()` returned `['/dev/ttyUSB0']`.
- In the venv, `from emioapi import emio` failed with `ImportError: cannot import name 'emio'`.
- The installed `emioapi` 0.2.0 exports `EmioAPI`, `EmioMotors`, `EmioCamera`, `MultiprocessEmioCamera` and others, but no module-level `emio` instance.
- Downloading `emioapi` 0.1.0 and 0.1.1 from PyPI showed the same: none of the three published versions has a module-level `emio`.

### Probable origin
The upstream `Emio.API` GitHub README shows this usage example, which does not match the released code:

```python
from emioapi import emio
emio.connectToEmioDevice()
emio.motors.angles = [0] * 4
emio.disconnect()
```

The original `cabledriven.py` matched this pattern line for line. The library's own docstring (`emioapi/emioapi.py`) correctly uses `emio = EmioAPI(...)`. This is an inference from the evidence, not something confirmed with the maintainers.

### Fix
`cabledriven.py` now uses the real API:

- `from emioapi import EmioAPI`, with the import wrapped in `except Exception` so a broken dependency does not crash the script.
- Connection uses `EmioAPI.listEmioDevices()`, then `EmioAPI().motors.open(port)`.
- The camera is deliberately skipped. `EmioAPI.connectToEmioDevice()` also opens the RealSense and returns `False` if the camera fails, which would block motor control even when the motors connected fine.
- `send_angles()` sets `self.emio.motors.angles`.
- `disconnect()` calls `self.emio.motors.close()` (the old code called `emio.disconnect()`).
- After connecting, `current_angles` is read from the motors, so `move_smooth()` starts from the real position instead of assuming 0.

The library clamps angles to 0 to pi rad. The script's own limit of 0 to pi/2 is narrower, so it stays within that.

## 2. No permission to open the serial port

### Symptom
```
[WARN] Could not connect to physical EMIO device (no read/write access to /dev/ttyUSB0. Add your user to the 'dialout' group and log in again.).
```

### Diagnosis
- `/dev/ttyUSB0` is `crw-rw---- root:dialout`.
- The user was not in `dialout`, so the port could not be opened.
- After running `sudo usermod -aG dialout $USER`, `getent group dialout` listed the user, but `id` in the running session did not. Linux reads group membership at login, so existing sessions and any terminal opened from them keep the old list.
- `sg dialout -c ...` confirmed access worked once the group was active.

### Fix
- The user was added to `dialout` and the machine was restarted.
- `id` now shows `20(dialout)`.
- `connect()` now checks `os.access(port, R_OK | W_OK)` before opening the port and reports the missing group clearly, instead of failing quietly.

## 3. Automatic homing on connect

### Symptom (potential, caught before it happened)
`connect()` called `home()` immediately, sending all four motors to 0 rad over one second.

### Diagnosis
A read-only probe (ping plus register reads, no writes) found all four motors responding, model 1030 (XM430-W210), torque on, position mode, and not at zero. Positions in the library's convention (0 deg = pulse 2048):

| Motor ID | Pulses | Angle |
|---|---|---|
| 0 | 2261 | -18.7 deg |
| 1 | 1764 | +25.0 deg |
| 2 | 2727 | -59.7 deg |
| 3 | 2035 | +1.1 deg |

The library treats 0 rad as the motor centre. Whether 0 is a slack cable on this build is unknown. Moving every motor there could have moved motor 2 by about 60 degrees and tensioned or released cables unexpectedly. Three of the four motors were also outside the script's 0 to 90 degree range.

### Fix
- `connect()` no longer homes on real hardware. It prints the current angles and states that the motors were not moved. Mock mode still homes, which is harmless.
- `disconnect()` no longer homes on real hardware. It says it is disabling torque and closing the port. The library's `close()` turns torque off, so motors may go limp.
- `connect()` returns early if already connected, to avoid opening the port twice.
- Added `python cabledriven.py --check` (`check_hardware()`), a read-only report of the board, port access, each motor's model, torque state and angle. It never writes to the motors.

## 4. Robot not visible after restart (open)

After the restart, `python cabledriven.py --check` reported:

```
[FAIL] No EMIO motor board found. Is it plugged in?
```

`lsusb` no longer listed the FTDI board or the RealSense camera, and `/dev/ttyUSB0` did not exist. The cause has not been diagnosed. The likely candidates are a loose USB cable, no power to the robot, or an unpowered hub, but none of these has been checked.

## What changed

### `cabledriven.py` (repo root)
- Import: `EmioAPI` instead of the non-existent `emio`.
- `__init__`: new `self.emio = None` attribute.
- `connect()`: device listing, port access check, motors-only open, reads real starting angles, no auto-home on hardware, reports current angles, early return if already connected.
- `send_angles()` and `disconnect()`: use `self.emio.motors`.
- `disconnect()`: no homing on real hardware.
- New `check_hardware()` and the `--check` command-line flag.
- Extra tip in the mock-mode message when `emioapi` fails to import.

`cabledriven_gui.py` needed no changes. It imports the controller from `cabledriven.py`.

### Emio Labs integration
These changes were made so the project can be opened in the Emio Labs app. They are separate from the connection fixes.
- Renamed `lab_empty.md` and `lab_empty.py` to `lab_me4242.md` and `lab_me4242.py`.
- `lab.json`: name `lab_me4242`, file `lab_me4242.md`, title "ME4242 Soft Robotics", new description.
- Updated every `assets/labs/lab_empty/...` path to `assets/labs/lab_me4242/...` in `lab_me4242.md`, `sections/section1.md`, `sections/authors.md` and `modules/site-packages/README.md`.
- `lab_me4242.md`: new title, overview, a warning about driving real motors, and buttons to install `emioapi` and launch `cabledriven_gui.py`.
- Copied the lab to `~/emio-labs/v26.06.01/assets/labs/lab_me4242/` and added `{"name":"lab_me4242"}` to `labsConfig.json` there. A backup of the original is `labsConfig.json.bak`.
- The copy in the Emio Labs install is not a link. Re-copy changed files after editing in this repo. `cabledriven.py` was last synced after the `--check` change.
- Not changed: `README.md` and `setLabName.sh` still describe the template.

## Open items

1. **Robot not detected.** Check the USB cable, power and hub, then run `venv/bin/python cabledriven.py --check`.
2. **The new connect path is untested on hardware.** It was only compiled and run in mock mode and against a missing device. Do a first live test with the cables clear.
3. **0 is not known to be safe.** Menu option 5 ("Home"), menu options 1 to 4 (the motion tests, which home first), and every "return to neutral" step still command 0 rad. Compare the angles reported by `--check` against the physical cable state to learn where slack actually is. A per-motor home offset could be added if 0 is not slack.
4. **Torque-off on disconnect.** The library's `close()` disables torque, so the structure may sag or cables may go slack when the script exits. This has not been observed.
5. **GUI threading.** From the earlier review (unchanged): `cabledriven_gui.py` updates Tkinter widgets from worker threads, and `run_async_routine` calls `connect_hardware()` after every routine. With the new `connect()` that call only re-reads angles on hardware, but it is worth cleaning up.
6. **`tkinter` in Emio Labs.** Whether the app's bundled Python includes `tkinter` for the GUI button has not been tested.
7. **README error upstream.** The `Emio.API` README example uses an `emio` object that does not exist in released versions. This could be reported to Compliance Robotics.

## Useful commands

```sh
# Is the robot visible to the OS?
lsusb | grep -i -E "ftdi|realsense"
ls -l /dev/ttyUSB*

# Do I have access to the port? (should list dialout)
id

# Read-only hardware report (does not move anything)
venv/bin/python cabledriven.py --check
```
