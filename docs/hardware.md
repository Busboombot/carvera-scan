# Hardware

The machine, the rig and the services that carvera-scan depends on. What has been done to each host is in busboom-home-network `hosts/<name>/README.md`.

## Machine and rig

| | |
|---|---|
| Machine | Makera Carvera C1 `carvera.home` 192.168.6.32:2222, Community firmware 2.2.0c-RC3. Host record: `busboom-home-network/hosts/carvera/README.md` |
| Cameras | Two Arducam OV9281 USB (mono, global shutter, 1280×800) on **kinsey** (`ssh kinsey`), picked by USB port: **1.2 = A** (reference frame), **1.1 = B**. They share one USB 2 link, so they record one at a time. Camera A's optical centre is ~162 mm above the bed at Z −2. Mount geometry: `design/camera-mount/README.md` |
| Scan laser | Cross laser, arms at about +48° and −41°, on the head beside the cameras. It has its own switch, so it is on in every photo |
| Built-in laser module | `laser_module_test_power 0.01` (1%); offset from spindle X −36.6, Y +4.2, Z −45 mm. Used only by the early dot-laser tests |
| Calibration | `config/calib/*.json` (per camera: port, centre, height, axis matrix, laser model) |
| Travel | Machine Z 0 (top) to −135 (soft endstop); clearance position X −75, Y −3, Z −3 |

## Commands we use

| G-code | What it does | Moves? |
|---|---|---|
| `?` | status | no |
| `M821` / `M822` | light on / off | no |
| `M321` / `M322` | laser mode on / off. On a C1, `M321` runs `M6T-1` (drops the tool into the rack) and `G92.5Z0` | **yes, ATC** |
| `M321.2` / `M322.2` | laser mode on / off **without** the tool drop or offset change | no |
| `M323` / `M324` | laser test fire on / off at `laser_module_test_power` × scale. Does nothing outside laser mode | no |
| `M325 S<n>` | laser power scale % (`S50` → 0.5% with test power 1%) | no |
| `$H` | home | **yes** |
| `M6T-1` | put the loaded tool back in the rack | **yes, ATC** |
| `G38.2` | probe toward the work. **Never used directly**: probe only with `M6 T0`, `M466`, `M6 T-1` (`carvera_scan.acquire.probe_z`) | **yes** |

Source: `src/modules/tools/laser/Laser.cpp` in Carvera_Community_Firmware.
Laser spot recipe, no motion: `M321.2`, `M325 S50`, `M323` … `M324`, `M322.2`.

Nothing in this repo moves the machine without Eric's go-ahead.

## Services on kinsey

`src/hosts/kinsey/deploy.sh` copies the package to `kinsey:~/carvera-scan/carvera_scan/` and cmux to `~/carvera-scan/cmux.py`, then (re)starts:

| Service | Port | What |
|---|---|---|
| `cmux` ([src/hosts/kinsey/cmux.py](../src/hosts/kinsey/cmux.py)) | TCP 2222 | Holds the Carvera's single connection; any number of controllers and scripts connect here. Forwards whole frames only, broadcasts replies, locks the machine to one client during file transfers, keeps the link alive, announces itself on UDP 3333 as `Carvera_03628-mux`. Commands are logged: `journalctl -u cmux -f` |
| `x11vnc` | TCP 5900 | VNC of kinsey's desktop (`open vnc://kinsey.home`); password is dotconfig secret `KINSEY_VNC_PASSWORD` in busboom-home-network |

On kinsey's desktop (user `carvera`, autologin) the Community Controller is
set to `127.0.0.1:2222` (through cmux). The capture scripts open the Arducams
directly. The old webcam server (`camserver`) was disabled on kinsey and deleted on
2026-11-05.

Capture scripts run from `~/carvera-scan` as `.venv/bin/python -m carvera_scan.acquire.<script>`
(venv: Python 3.12, numpy, `opencv-contrib-python-headless`) and write to `cap/<name>/`.

