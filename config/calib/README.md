# Calibration sets

One JSON file per calibration, built by `python -m carvera_scan.calib build` (see its
docstring). A set holds everything rig-specific that processing needs: per camera role
(A, B), the USB port, principal point, height, axis matrix and per-arm laser model, plus
the B→A shift guess and the map→machine offset. A new rig or a recalibration is a new
file, not a code change.

| Set | Z sweep | Axis test | Used for |
|---|---|---|---|
| `2026-11-05-cross45-deep.json` | `data/2026-11-05/zcal45deep` (to 78 mm) | `data/2026-11-05/axis45` | the blocks (tall objects) |
| `2026-11-05-cross45-shallow.json` | `data/2026-11-05/zcal45` | `data/2026-11-05/axis45` | the sheet |

Each file's `sources` field names the exact capture files it came from.
