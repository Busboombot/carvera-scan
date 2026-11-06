# carvera-scan as a package: design, analysis and plan

> **Paths (2026-11-05, after the reorganisation):** this document was written before the
> move, so it says `analysis/x.py`, `kinsey/x.py` and `calib/`. They are now
> `src/carvera_scan/{process,measure,view,acquire}/x.py` (run as `python -m carvera_scan.…`),
> `config/calib/`, and the bench runs in `scratch/bench/`. See `docs/structure.md`.

Status: plan, decisions recorded 2026-11-05 (section 9). Covers what the scanner does
today, what is wrong with it (measured), how the package should be laid out, which
point-cloud library to build on, and the order of work.

## 1. Goal

One installable package that, given the rig on the Carvera's head, will:

1. **Check the rig**: find both cameras, check exposure and the laser cross.
2. **Calibrate**: image axes and scale, a Z sweep for laser height, alignment between
   the two cameras, and the offset from camera to spindle.
3. **Plan and run a scan safely**: survey photos, a quick map at the top of Z, then
   passes along X and Y.
4. **Process the frames**: turn them into points, a cleaned cloud, a height map, a
   mesh with the photos draped on it, and measurements.
5. **Export and report**: PLY/LAS files, the browser viewer, and an HTML report.

Each step is a command with recorded inputs and outputs, so bugs can be reproduced
from saved data without the machine.

## 2. What exists (the 2026-11-05 pipeline)

All of this ran today as loose scripts. It works end to end: blocks to ±0.1–0.5 mm in
height, a sheet to 0.06 mm against the probe.

| Stage | Today | Where it runs |
|---|---|---|
| Machine link | `kinsey/machine.py` (framed protocol, limit-checked G53 moves, resend lost moves); `cmux.py` service shares the one connection | kinsey |
| Cameras | `arducam.py` (V4L2 MJPG, by USB port); the two can't stream at once on the shared USB 2 link | kinsey |
| Axis test | `axis_test.py` moves; matrix from SIFT with the laser masked (done by hand in scratch scripts) | kinsey + Mac |
| Z sweep | `z_sweep.py`; `analysis/cross.py zcal` (per-arm model) | kinsey + Mac |
| Survey / photos | `survey.py` (grid of lit stills, both cameras) | kinsey |
| Sweeps | `line_scan.py` (one G1 per pass, 120 fps, status polled ~14 ms; `--along x\|y`) | kinsey |
| Points | `cross.py sweep` (arms found, height, XY = head + image offset) | Mac |
| Merge | `blocks_merge.py` (camera 1.1 → 1.2 shift by grid matching) | Mac |
| Grid / measure | `grid.py`, `flatness.py`, `heights_report.py`, `probe_compare.py`, `edge_lengths.py` (not run yet) | Mac |
| Probe | `probe_z.py` (`M6 T0`, firmware `M466` only) | kinsey |
| Visuals | `view3d.py` (photo draping, cloud, PLY, relief), three.js `index.html` | Mac |

Legacy, moved to `attic/` on 2026-11-05 (still in git history, see `attic/README.md`):
the webcam and dot laser (`camserver`, `camctl`, `camview`, `laser_capture`,
`laser_scan`, `laser.py`, `laser_height.py`, `heightmap.py`, `block_calibrate.py`),
ChArUco (`calibrate.py`, `charuco.py`, `detect.py`, `grid_capture.py`), motion and
two-camera stereo (`stereo*.py`, `plot3d.py`, `compare_methods.py`, `magnify.py`). The
one-off plate CAD and Cutebot scripts went to `examples/2026-11-04-plate-cutebot/`.

- **Moved first:** the two helpers the current scripts used from legacy files
  (`light_on`, `axis`) now live in `kinsey/machine.py`.
- **Kept for the line option:** the straight-line pipeline (`line_height.py`,
  `sweep_height.py`, `zcal.py`) stays in `analysis/`. It is the "line" laser option,
  and its helpers (`ridge`/`pick`/`runs`/`load`) are shared with the cross code.
- **Deploy:** `deploy.sh` no longer installs camserver, and the service was disabled on
  kinsey on 2026-11-05.

Hardware facts the design depends on:

- **kinsey** runs Ubuntu 24.04 with Python 3.12, but has only 4 cores and 3 GB RAM. It
  captures; it doesn't process.
- **Buzzkill** processes: a GMKtec K16 with a Ryzen 7 7735HS (16 threads), 25 GB RAM
  and 349 GB free, running Ubuntu 26.04 with Python 3.14. It is on both the house LAN
  (192.168.1.23) and the Robot Garage (192.168.5.23), and reaches kinsey directly.
  Open3D 0.20 has cp314 Linux wheels. It needs uv, a project venv, and a host
  change-log entry in busboom-home-network.
- **The Mac** is where code is edited and results are looked at (Python 3.13, uv).
  Small jobs can still run here.
- **The cameras** share one USB 2 link, so each pass is recorded once per camera, which
  doubles scan time. The fix is hardware: separate root ports, or a USB 3 path.
- **The laser** has its own switch, so it is on in every photo; colouring masks it out.
  The work light is under G-code control (`M821`/`M822`).
- **Safety rules carried over:** stay 2 mm from travel ends (Y 0 trips HALT 21); probe
  only with firmware cycles (`M6 T0`, `M466`), never hand-driven G38; only move Z after
  a clearance check from the map.

## 3. Measured problems (evidence from the block scan)

### 3.1 Spikes and fins

The 23 M points from `blocks_all.npz` (8 sources: 2 scan directions × 2 cameras × 2 arms)
were analysed to find where the spikes come from.

| Finding | Number |
|---|---|
| Points above 80 mm (nothing on the bed is taller than 76.5) | 44,005 (0.19%), median 88 mm |
| Where those are | narrow gaps between polished block faces (1/8" slot between the 2" and 1" blocks, around the 3" block) |
| Cause | laser reflected off a block side; the arm finder takes the strongest ridge in a search band that reaches ~120 mm, beyond the 78 mm calibrated |
| Cells flagged as spikes (> 3 mm above their 7×7 median) | 1,321; 95% within 10 mm of a > 15 mm step |
| Sources behind a flagged cell | one source 61%, three or more 31% |
| XY placement of tops, by source | edges agree to ~1 mm across all 8 sources, so the camera model doesn't smear tops |

So there are two kinds of spike:

- **Reflections:** a single source, often above the calibrated range.
- **Wall cells:** a 0.5 mm cell on a block edge takes the median of real points from the
  whole wall, and a median of a wall is a mid-wall height (the "fins" in the viewer). A
  2.5D grid can't represent a wall; the cell must choose top or bed.

Fixes tried on the same data:

| Grid method | Filled | Flagged cells | Clusters ≥ 10 cells | Cells > 80 mm | Time |
|---|---|---|---|---|---|
| median per cell (current) | 68.4% | 1,378 | 25 | 266 | – |
| range gate −3…80 mm | 67.2% | 1,386 | 21 | 0 | – |
| gate + consensus (≥ 2 of 8 sources agree within 1 mm) | 63.1% | 1,240 | 10 | 0 | 98 s |
| **gate + densest 1 mm band per cell** | **67.3%** | 1,234 | **11** | 0 | **9 s** |
| same, band holds ≥ 40% of the cell's points | 66.6% | 1,276 | 9 | 0 | 9 s |

Block heights from the densest-band grid: 76.34 / 50.76 / 25.38 mm, the same as before.
What still gets flagged is mostly inside the block holes (real depth plus reflections off
the hole walls) and a thin outline on top edges. The median-filter despeckle in
`view3d.py` was a cosmetic patch and gets replaced.

### 3.2 Height disagreement between arms and cameras

- **Per-arm calibration:** arm +48° fits the Z sweep within ±0.17 mm at every step;
  arm −41° is up to +0.3 mm off.
- **Combinations disagree:** on the same top, the four camera/arm combinations spread up
  to 0.78 mm. This shows as a diagonal hatch on flat tops.
- **Bars read high:** 5/32" and 3/32" read +0.47 and +0.29 mm in all four combinations,
  and are tilted ~1°. They may be really lifted; calipers will tell.

### 3.3 Geometry and calibration gaps

- **XY scale** comes from a 20 mm axis test (SIFT, 5–10 inliers), good to about ±1%.
  The 1" top measures 49.6 × 75.2 mm (nominal 50.8 × 76.2), part chamfer, part scale.
- **The principal points and camera heights** are from the straight-line Z sweep
  (`zcal5`). No lens distortion model is used.
- **Camera → spindle offset** (−71.7, −68.1) comes from the anchor-corner assumption.
  The cameras moved ~1 mm when the laser was turned. There is no routine to measure it.
- **Merging cameras** uses a single XY shift found by grid matching. It should be a
  rigid fit (ICP), checked on raised features.

### 3.4 Process gaps

- **By hand:** clearance checks, choice of the Z-sweep spot, object boxes (`objects.json`),
  label positions. Today's clearance misread happened in a hand step.
- **Hard-coded:** USB ports, camera roles, rotation conventions and principal points in
  several files (`CAMERA` in `cross.py`).
- **No record** of which calibration produced which result, and no tests.

## 4. Library choice for point clouds

| Library | Licence | Wheels | Gives us | Verdict |
|---|---|---|---|---|
| **Open3D 0.20** | MIT | cp310–cp314, macOS arm64 + Linux x86_64 | voxel downsampling, statistical/radius outlier removal, KD-tree radius queries (multi-view consistency), normals, RANSAC planes, DBSCAN clustering, ICP (camera↔camera, scan↔scan), Poisson/ball-pivot meshing, PLY/PCD/XYZ I/O, offscreen rendering | **Core dependency (processing extra)** |
| laspy 2.7 | BSD | pure Python | LAS/LAZ export (Potree, CloudCompare, GIS tools) | Export extra |
| VTK / pyvista | BSD/MIT | yes | meshing, rendering | Not needed while Open3D + three.js cover it |
| PDAL | BSD | no pip wheels (conda) | pipelines, many formats | Skip |
| CloudComPy | GPL | conda only | CloudCompare in Python | Skip; use the CloudCompare app on our PLY/LAS |
| point-cloud-utils | MIT | yes | fast KD-tree, sampling | Backup if Open3D gets in the way |
| Potree | BSD | converter binary | web viewer for 100 M+ points | Later, if clouds outgrow three.js |

Open3D is the core library. The viewer stays a static three.js page written by the
package; it already works and needs no server logic. Open3D is only needed where
processing runs (Buzzkill, optionally the Mac): kinsey's acquisition side stays
numpy + OpenCV.

## 5. Package design

### 5.1 Layout (src layout, one distribution, two extras)

```
carvera-scan/
  pyproject.toml          # name carvera-scan, import carvera_scan, CLI `cscan`
                          # extras: [acquire] opencv, numpy        (kinsey)
                          #         [process] open3d, scipy, matplotlib, laspy  (Buzzkill)
  src/carvera_scan/
    config.py             # rig.yaml + defaults, validated (dataclasses)
    machine/
      protocol.py         # framed protocol, crc, status parsing   (from machine.py)
      client.py           # Carvera: send, status, move (limits, lost-move resend), light
      safety.py           # travel limits, edge margin, clearance planner, Z-move guard
      probe.py            # M6 T0 / M466 / M6 T-1 only                 (from probe_z.py)
      cmux.py             # the connection multiplexer service
      sim.py              # fake Carvera for tests (status, moves, alarms, lost moves)
    camera/
      arducam.py          # V4L2 by USB port, exposure, raw MJPG reader
      rig.py              # roles (A/B) <-> USB ports, one-at-a-time capture
    acquire/              # runs on kinsey; each writes a capture dir + manifest.json
      axis.py  zsweep.py  photos.py  sweep.py  probe_touch.py
    calib/
      axis.py             # image<->machine matrix (SIFT, laser masked, longer moves)
      laser.py            # arm angles, per-arm height model, residual report
      camera.py           # principal point + height (hole zoom), later distortion
      register.py         # camera B -> A (ICP on raised features), camera -> spindle
      selfcal.py          # per-arm/camera height offsets from overlaps
    process/
      arms.py             # arm finding (turn, ridge, pick, runs, ambiguity test)
      points.py           # frames -> (x, y, h, source, frame, col) via head timing
      clean.py            # range gate, ambiguity, multi-view consistency, SOR (Open3D)
      grid.py             # densest-band height map, coverage, support
      mesh.py             # height-map mesh with walls; optional Poisson (Open3D)
      colour.py           # photo draping (visibility, laser mask, gain)
      measure.py          # tops/segments, heights vs nominal, flatness, edge lengths, probe compare
    export/
      ply.py  las.py  viewer/ (index.html template + writer)  report.py
    cli.py                # cscan ... (below)
  tests/                  # unit + regression on trimmed recorded data; machine sim
  attic/                  # legacy scripts
  examples/               # cutebot, cad_model
```

### 5.2 Data model

- **Capture directory**: one per acquisition step, e.g. `data/2026-11-05/blocks1_A/`.
  Holds the raw frames (`frames.mjpg` or PNGs) plus `manifest.json`: kind, rig config
  hash, camera port and role, exposure, machine positions and status samples, firmware
  version, times, operator notes.
- **Calibration set**: `calib/<name>.json` holding the axis matrices, per-arm laser
  models, principal points and heights, camera B→A transform, camera→spindle offset.
  It links to the captures it came from and records its residuals.
- **Product directory** per processing run: points (npz), cleaned cloud (PLY/LAS),
  grid, mesh, measurements (JSON), viewer, report. Its manifest names the captures and
  the calibration set used.
- `data/` stays out of git. Small trimmed fixtures go in `tests/data/`.

### 5.3 Configuration (`rig.yaml`)

```yaml
machine: {host: 127.0.0.1, port: 2222, edge_margin_mm: 2, z_top: -1.3}
cameras:
  A: {usb_port: "1.2", rotate: none}
  B: {usb_port: "1.1", rotate: none}
laser: {type: cross, arms: 2}          # or: line
scan: {pitch_mm: 0.25, fps: 120, exposure_100us: 15, light_off: true}
safety:
  rig_below_spindle_mm: null           # unknown (above the dust shoe, about the built-in
                                       # laser's height; Eric to measure) -> the planner
                                       # refuses Z moves below z_top until it is set
  max_object_height_mm: 80
calibration: calib/2026-11-05-cross45.json
```

### 5.4 CLI and workflow

```
cscan check                    # machine status, cameras found, exposure test, laser seen
cscan calibrate axis           # move ±40 mm in X and Y, both cameras
cscan calibrate zsweep [--at X Y] [--depth 78]   # spot chosen and cleared by the planner
cscan calibrate fit            # -> calib/<name>.json, residual report
cscan survey                   # lit photos over the bed
cscan scan --area X0 X1 Y0 Y1 [--along x,y]      # top-Z quick map, then full passes
cscan process <captures...>    # points -> clean -> grid -> mesh -> colour
cscan measure <product> [--objects auto|file]    # segment tops, heights, flatness, lengths
cscan export <product> --ply --las --viewer --report
cscan run --area ...           # check -> survey -> scan -> process -> export
cscan probe touch --spots ...  # firmware M466 only; requires probe confirmation
```

Execution, three machines:

- **kinsey (acquire):** commands that need the machine or cameras run here
  (`cscan acquire …`). Captures land in `~/carvera-scan/data/<date>/…`.
- **Buzzkill (process):** `cscan process|measure|export` run here, on
  `~/carvera-scan/data/`. Captures are pulled from kinsey over the Robot Garage link by
  rsync (`cscan pull`), so the frames never cross the Mac.
- **Mac (control and view):** `cscan` here is a thin front end. It runs the steps above
  over SSH (`ssh kinsey cscan acquire …`, then `ssh buzzkill cscan process …`) and
  rsyncs back only the products: viewer, report, PLY.

`cscan run` chains it all. This is today's SSH + rsync pattern, made explicit. A
service on kinsey or Buzzkill can come later if SSH gets in the way.

The laser stays a 45° cross on its own switch. Its type and arm count are config
(`laser:` in `rig.yaml`). A change of rig means a new calibration set, not code
changes, as long as it is a line or a cross. The arm finder already handles any angle.

### 5.5 Safety design (enforced in code, not by convention)

- **All moves go through `safety.py`:** soft limits minus the edge margin, and Z only
  above `z_top` unless the clearance planner has approved that spot.
- **Clearance planner:** from the latest top-Z height map, the spindle footprint, the
  rig footprint and `rig_below_spindle_mm`, it computes the lowest safe Z at a given XY.
  It also picks Z-sweep spots automatically, removing today's hand step.
- **Probing:** only firmware cycles (`M6 T0`, `M466 Z-…` with a distance cap from the
  map, `M6 T-1`), with an explicit confirmation flag. Never G38 directly.
- **The simulator** (`machine/sim.py`) lets tests prove that unsafe moves are refused.

## 6. Improvements backlog (prioritised)

| # | Improvement | Evidence / expected effect | Effort |
|---|---|---|---|
| 1 | Range gate at extraction: search band limited to the calibrated depth + 3 mm; drop h outside −3…max | removes all 0.19% > 80 mm points (the tall fins) | S |
| 2 | Densest-band grid instead of median | flagged clusters 25 → 11, coverage kept, 10× faster than consensus; walls snap to top or bed | S |
| 3 | Ambiguity test in the arm finder: second-strongest ridge within ~70% of the best → drop that column (or keep the one continuous with neighbouring frames) | attacks reflections at the source, in gaps between shiny faces | M |
| 4 | Multi-view consistency for the 3D cloud: keep a point only if another source has points within r (Open3D KD-tree); then statistical outlier removal | removes single-source reflections from the cloud (61% of flagged cells were single-source), keeps real walls | M |
| 5 | Height self-calibration: per camera/arm offset (and scale) solved by least squares where sources overlap, anchored to the Z sweep | removes the diagonal hatch; should bring the four combinations from 0.78 mm spread to ~0.2 | M |
| 6 | Longer axis test (±40 mm, many features) plus a principal point and distortion fit from the Z-sweep hole tracks | XY scale from ±1% to ~0.2%; sizes within ~0.2 mm | M |
| 7 | Camera B → A registration by ICP on raised features (Open3D) instead of a 2-D grid shift | tops of both cameras land together; fewer edge outlines | S |
| 8 | Run and validate `edge_lengths.py` (lengths from machine motion) against calipers | caliper-grade lengths, independent of image scale | M |
| 9 | Automatic segmentation of tops (planes by RANSAC/DBSCAN in Open3D) replacing hand-made `objects.json`; nominal sizes from OCR of labels (optional) | removes a hand step | M |
| 10 | Mesh: height-map mesh with explicit vertical walls at steps (from the densest-band grid and the step mask); Poisson meshing as an option | cleaner viewer and PLY | M |
| 11 | Photo draping: flat-field and exposure-normalise each photo; blend overlaps by distance from the camera axis; lock exposure during the survey | removes seams | S |
| 12 | Camera → spindle offset routine (probe an edge with `M466 X/Y` cycles, or a fiducial seen by the camera) | absolute XY from ±2 mm to ~0.2 mm | M |
| 13 | Both cameras in one pass (separate USB root ports or a USB 3 path) | halves scan time, gives simultaneous frames | hardware |
| 14 | Adaptive exposure per pass (bright metal vs MDF) | fewer gaps on shiny tops | S |

## 6a. Phase 1 results (2026-11-05, on Buzzkill)

`analysis/bench.py` reruns the whole cross-laser chain from the raw frames and scores
the blocks scan and the sheet scan. The image pass (`cross.extract`) is cached in
`bench/cache/`, so changes after it rerun in minutes. Run r0 reproduces the old
pipeline exactly.

**New finding.** The −41° arm of both cameras read the bed 3–5 mm low toward one end of
the arm, in 9–16% of its points. The deep Z sweep had blocks in part of its view, so
that arm's bed row was extrapolated. Fix: `bedref scan`, which takes each column's bed
row from the scan itself as the **lowest** strongly populated row band (a sheet or
block top can outnumber the bed in a column). The Z sweep now only supplies the height
curve. Residual offsets from the Z sweep's bed rows: −0.6 to +0.6 px.

**Fixes now in the code:**

- `cross.py`:
  - `--bedref scan`
  - `--gate` (search band and kept heights limited to the calibrated depth + 3 mm)
  - `--ambig 0.6` (drop columns with two comparable ridges)
- `grid.py`: `grid_mode` (densest 1 mm band, refined by mean shift)
- `clean.py`:
  - `consistency` (a point must be seen by ≥ 2 of the 8 scan×arm views within 1 mm)
  - `sor`
  - `register_icp`
- `bench.py`: `--selfcal` (per camera/arm offset and scale from overlaps)

| | baseline | recommended (s4) |
|---|---|---|
| options | – | `--bedref scan --gate --grid mode --consistency 2 --ambig 0.6` |
| points above 80 mm | 44,005 | 2,452 (all ≤ 81, inside the gate) |
| grid cells above 80 mm | 266 | 6 |
| flagged clusters (≥ 10 cells) | 17 | **0** |
| block/bar height error, mean / max | 0.195 / 0.504 mm | **0.130 / 0.295 mm** |
| spread between camera/arm combinations, mean / max | 0.47 / 0.78 mm | **0.29 / 0.56 mm** |
| roughness: bed / 3/16" bar / 1/2" bar (MAD) | 0.185 / 0.166 / 0.180 | 0.096 / 0.107 / 0.116 |
| sheet vs probe: mean / sd | −0.027 / 0.032 | −0.051 / 0.031 |
| sheet flatness residual rms | 0.188 | 0.184 |
| coverage of the block area | 99.4% | 98.8% |

Tried and not adopted as defaults:

- **Self-calibration:** mean spread 0.29 → 0.26 mm, but max 0.56 → 0.62 mm.
- **ICP registration:** no gain over the grid-matched shift.

Heights with the recommended options:

- **Blocks:** 3" 76.14, 2" 50.71, 1" 25.26 mm.
- **Bars:** 1/2" 12.83, 1/4" 6.36, 3/16" 4.80, 5/32" 4.23, 1/8" 3.29, 3/32" 2.54; the
  1/16" bar reads 1.88 mm on top of its block.

All are within 0.30 mm of their labels.

The cleaned viewer is `data/2026-11-05/view3d_v2/`, built from
`bench/s4_saved/blocks_points.npz` without the cosmetic despeckle.

**Second round (after Eric saw the needles in view3d_v2).**

- **The metric was blind.** It counted clusters of ≥ 10 flagged cells. A single-cell
  spike counts for nothing there, yet renders as a full-height needle.
- **New metrics in `bench.py`:**
  - `needle_cells`: more than 3 mm above the median of the 8 neighbours.
  - `cells_above_block_tops`.
- **Causes found:**
  - **Consistency was keyed by scan×arm.** A reflection depends on the camera and laser
    geometry, not the head's direction, so it appeared in both the X and Y passes of the
    same camera and arm and "confirmed" itself. Now keyed by camera×arm
    (`--consistency-key src`, default).
  - **Isolated cells** at block edges and hole rims. `grid.support_filter`: a cell needs
    ≥ 3 of 8 neighbours within 1.5 mm, else it takes the neighbours' median.
  - **Ghost "posts" at the edge of coverage.** 49 scattered points became a 422-cell
    block because `fill_pits` filled large empty areas from them.
    `grid.remove_specks` drops islands < 20 cells (5 mm²), and `fill_pits` now fills
    only enclosed holes ≤ 15 mm across.
  - **Floating specks in the cloud:** Open3D radius outlier removal in `view3d.py`
    (`--cloud-ror 6`).
- **Results (s7):**
  - Cells above 80 mm 6 → 0, cells above their block's top 117 → 30, needle cells
    564 → 338. What's left is mostly real corners and hole rims.
  - Heights, spread and sheet-vs-probe unchanged; coverage 98.8 → 98.2%.
  - Viewer `data/2026-11-05/view3d_v4/`: no needles above the tops, no ghost posts.
- **Recommended options now:**
  - `bench.py --bedref scan --gate --grid mode --consistency 2 --ambig 0.6 --specks 20 --support`
  - `view3d.py --no-despeckle --specks 20 --support --cloud-ror 6`
- **Lesson:** judge cleaning by the rendered result as well as the numbers.
  `bench.py` should grow a needle-free check at render scale.

Still open from Phase 1:

- ~~Viewer artifacts (spike on the 2" block, posts at the back)~~: fixed in the second
  round. A few floating specks remain in the cloud near the edge of the 3" top.
- **Spread:** the remaining 0.3–0.56 mm between camera/arm combinations is not a simple
  offset or scale.
- **Calibration:** redo the deep Z sweep at a spot that is clear for the whole frame.

## 6b. Keeping improvements general (2026-11-05)

Fixes must reach every scan, not just the data they were found on. The rule:

1. **One chain:** `analysis/pipeline.py` runs scan → points → merge → clean → grid
   (+ surface for the viewer), with every setting in one `Params` dataclass whose
   defaults are the recommended values. The benchmark, the viewer builder and, later,
   `cscan process` all call it. Nothing else re-implements a step. `blocks_merge.py`'s
   command line is legacy.
2. **Rig constants live in a calibration set**, not in code (`analysis/calib.py`,
   `calib/*.json`). It holds, per camera role:
   - USB port
   - principal point and height
   - axis matrix
   - per-arm laser model

   Plus the B→A shift guess and map→machine offset. A new rig or a recalibration is a
   new file. (`cross.CAMERA` remains only for the legacy `place()` / `sweep()`.)
3. **Defaults change in one place.** `cross.py`'s command line now defaults to
   `--bedref scan`, the gate on and ambiguity 0.6. `view3d.py` takes its cleaning
   settings from `Params` and writes the viewer page from `analysis/viewer/`.
4. **The benchmark is the regression test.** Run `analysis/bench.py` with no options
   before and after a change. It scores the pipeline defaults on the 2026-11-05 blocks
   and sheet. Its scan names, object boxes, probe spots and sheet mask are fixtures of
   that data; the processing isn't.
5. **Tuned to this data so far:**
   - `fill_max_hole_mm = 15` (fits 1-2-3 block holes) and `specks_min_cells = 20`.
   - The gate margin of 3 mm and the ambiguity ratio of 0.6.
   - Physically motivated but checked on one rig only: the support filter (3 of 8
     within 1.5 mm) and consistency (2 camera×arm sources within ~1 mm).

   More datasets (bare metal, a different rig) should be added to the benchmark as they
   come, so these get re-checked rather than assumed.

## 7. Plan

**Phase 0: this document.** Review, then decide the open questions (section 9).

**Phase 1: improvements on recorded data (no machine).** In progress; results in
section 6a.

- **Buzzkill:** set it up first. Install uv for `eric`, create a project venv with
  Python 3.13 or 3.14, numpy, scipy, opencv-headless, open3d and matplotlib. Copy
  `data/2026-11-05/`, 2.9 GB of captures, from the Mac or kinsey. Record it all in
  `hosts/buzzkill/README.md`.
- **Baseline:** put today's results into a regression harness: blocks heights, sheet vs
  probe, coverage, flagged-cell counts, points > range.
- **Changes:** do items 1–5 and 7, each scored against that baseline.
- **Exit:** no points above the range, flagged clusters ≤ 5, camera/arm spread ≤ 0.3 mm
  on the block tops, block and bar heights unchanged or better, probe agreement
  unchanged or better.

**Phase 2: package skeleton.** Create the layout above and move the kept code in, with
small cleanups only. Add `rig.yaml`, `cscan` with `check`, `process`, `export`, and
`measure`. Add tests on trimmed fixtures (a few hundred frames from `zcal45deep` and
`blocks1`) and the machine simulator.

- **Exit:** `cscan process` reproduces today's `blocks_all` products from the raw captures.

**Phase 3: acquisition and calibration commands.** Port `acquire/*` and `calibrate/*`;
add the clearance planner and the Z-move guard; add probe commands behind confirmation.

- **Exit:** `cscan run` does check → survey → scan → process → export on the bed with
  Eric watching, with no hand-edited files.

**Phase 4: measurement quality and outputs.** Items 6, 8–12; LAS export; HTML report;
the viewer reads products directly.

- **Exit:** heights and lengths of the blocks and bars checked against calipers.

## 8. Testing

- **Unit:** protocol framing and CRC, status parsing, limit checks, arm finder on
  synthetic frames (known angle and shift), height model round trip, grid modes.
- **Regression:** trimmed real captures → points → heights. Thresholds come from
  today's numbers.
- **Simulator:** move sequences, lost-move resend, alarms, clearance refusals, probe
  command gating.
- **On the machine:** `cscan check` and a short known-object scan (the 1-2-3 blocks)
  after any rig change.

## 9. Decisions (Eric, 2026-11-05)

1. **Name:** `carvera-scan` (import `carvera_scan`, command `cscan`), in this repo.
2. **Processing runs on Buzzkill.** kinsey captures; the Mac controls and views.
3. **Rig:** the 45° cross is the standard. Other lasers must stay possible (config plus
   a new calibration), not necessarily easy.
4. **Legacy:** moved to `attic/` (one-off work to `examples/`). Done 2026-11-05.
5. **Laser:** no switchable laser for now, maybe later. Photo colouring keeps masking
   the laser lines.
6. **Rig height:** the rig's lowest point is above the dust shoe, at about the height of
   the built-in laser module. Eric will measure it. Until then the clearance planner
   allows no Z moves below `z_top` without an explicit override.

Still open:

- **Both cameras streaming at once** (separate USB root ports or a USB 3 path). No
  answer yet; scans keep recording each camera in turn.
- ~~The idle camserver service on kinsey~~: disabled 2026-11-05.
