# Lab notebook

Dated record of what was tried and measured, newest at the bottom. Paths are as they were at the time: before 2026-11-05 the code lived in `kinsey/`, `analysis/` and `examples/`, and results in `data/<date>/` (see docs/structure.md for where things are now).

## Log

- 2026-11-01: camera works; markers 2–11 decode in the first frame (board
  appears rotated 180°). Heavy barrel distortion: the ChArUco interpolation
  finds only 12 corners and a single-frame calibration is degenerate, so
  step 1 is needed. The machine was asleep (`Alarm lock`); waking it needs
  `reset`. kinsey can ping the Carvera but its TCP connections time out (see
  below), so the machine is driven from the Mac for now.
- 2026-11-01: kinsey moved to Busboom Garage. Wrote cmux and camserver
  instead of carve-control (that one speaks the old plain-text protocol
  and stalls on this firmware). Tested: the Community Controller and
  `machine.py` connected through cmux at the same time, both answered.

- 2026-11-01, first machine run (Eric's go-ahead): reset (it homed itself on
  boot), lights `M821`, tool 1 to the rack (`M6T-1`). Camera is on the head.
  Survey at Z −3 found the board; calibration grids of 36 stops at Z −3 and
  at Z −50 (kept to X ≤ −60 at Z −50, clear of the rack at X ≈ −4), plus a
  Z step −3…−33 at the board centre. `analysis/calibrate.py` results
  (`data/2026-11-01/calib-free.json`):
  - f = 1393 px, centre (964, 544), k1 −0.38; camera looks 4.0° off vertical.
  - The camera alone measures the board height to ±0.8 mm per frame, and
    the same value at Z −3 and Z −50 (−186.9 vs −186.7 mm below the optical
    centre at head Z 0). Optical centre to board: 184 mm at Z −3.
  - Solving each marker corner in 3D: board flat to 0.64 mm RMS (max 1.4),
    tilted 0.25°; residual 2.3 px (the 8-term lens model doesn't improve it).
  - Bug found and fixed: the machine reports Idle before a queued move
    starts, so `machine.move` now waits until it is Idle *at the target*.

- 2026-11-01, laser (cover had to be reseated: the camera cable runs under
  the lid and the cover sensor read open, halt reason 11 `COVER_OPEN`, until
  Eric fixed it). `kinsey/laser_capture.py`: `M321.2`, `M325 S50`, `M323`
  (0.5%), off/on frame pairs at Z −3…−24 over X −140 Y −125. The blue dot
  is near the bottom edge of the frame (row 1039 at Z −3) and moves down
  ~1.7 px/mm as the head descends; it leaves the frame below about Z −25.
  `analysis/laser.py` (beam through the lens model): 1.1 mm RMS, with a false
  10° beam tilt from lens-model error at the image edge. Mapping the dot row
  directly to distance (quadratic): 0.13 mm RMS fit, **0.32 mm
  leave-one-out**. Camera ~65 mm from the beam.

- 2026-11-01, 1-2-3 blocks (blue tape) on the board. With the camera aimed
  as it is, the laser dot on a block top falls off the bottom of the frame
  (checked: no dot visible on the big block), so the laser can only map the
  board. Motion stereo instead (`analysis/stereo.py`, 20 frames at Z −3,
  SIFT + epipolar check against the known moves): board points within
  ±2 mm; height clusters at 25.09 mm (44 points), 49.60 (13), 76.76 (5) for
  the nominal 25.4 / 50.8 / 76.2. The biggest block top: 64 points, median
  25.31 mm. Map: `analysis/plot3d.py` → `data/2026-11-01/blocks.html`.
  Tape tops have little texture, so the 2" and 3" counts are thin.

- 2026-11-01, "the laser doesn't turn on": it does, but the configured test
  power (`laser_module_test_power 0.01`, 1%) is too dim to see by eye. At
  `M325 S500` (5% = 0.01 × 500%) the spot is bright blue. The controller's
  laser test uses the same 1%. Eric moved the camera so its centre is near
  the tool point; the dot now lands near the image centre (960, 413 at
  X −94 Y −197 Z −2). **Camera calibration from earlier is void.**
  Kinsey's controller now drives the machine over USB (FTDI FT232R); my
  scripts still reach it over Wi-Fi (`--host 192.168.6.32`) at the same time.

- Power sweep at the new camera angle (dot on a blue-taped block): 0.25%
  nothing (below the diode's threshold); **0.5% (`M325 S50`) a compact spot
  with little scatter: use this for scanning**; 0.75% and up flood the
  scene blue; 1% already saturates ~3,700 px. By eye, 5% is what looks
  "really blue".
- Don't stream dozens of queued moves: a full planner queue stops the
  machine reading the link, so `?` gets no reply and the client loses
  contact (laser was still on until a second connection sent `M324`). Use
  `G2`/`G3` arcs or wait for each `ok`.

- 2026-11-01, **first laser height map** (`data/2026-11-01/scan1.html`).
  Camera re-aimed by Eric; ChArUco board removed (black squares hide the
  0.5% dot). Calibrated on bare spoilboard: Z sweeps at two spots,
  `analysis/laser_height.py calibrate` (projective row -> head Z), 0.12 mm
  leave-one-out over 45 mm. **Exposure must be a whole number of the
  laser's 1 ms PWM period** (rolling shutter): 0.3 ms gave ±18.6 px row
  jitter, 2.0 ms ±0.2 px. Scan: 441 stops, 10 mm grid, Z −3, 8.5 min.
  Real spots = saturated and < 15,000 px (lit tape gave dim false spots);
  the dot's column wanders ±15 px on taped tops, so the track check is 20 px.
  Results (above the calibration spots; floor median −0.68, sd 0.81):
  1" 25.36 mm (−0.04), 2" 50.04 (−0.76), 3" 74.50 (−1.70; at the scan's
  edge, 4 cells, extrapolated past the 45 mm calibration).

- 2026-11-02 (overnight, unattended; head kept at Z −3 and X ≥ −240):
  **finer pass and final map** `data/2026-11-01/final-map.html` (+ `.png`).
  Three fixes along the way: (1) the dot is located by its **saturated
  core**: the half-max blob takes in uneven glow from the tape and pulled
  the 3" reading 1.2 mm low; (2) **work light off** during scans (`M822`,
  confirmed from `diagnose` G): the lit 3" top otherwise saturates around
  the dot; (3) the laser-off path reconnects until the machine confirms
  `L` test = 0. 3 mm grid, 1,580 stops. **Independent result** (spoilboard
  Z-sweep calibration only, blocks not used): 1" 26.08 mm (+0.68),
  2" 50.73 (−0.07), 3" 75.70 (−0.50); top sd ~0.6 mm. Footprints match the
  blocks (2"×3", 1"×3", 1"×2" tops). Using the blocks as calibration
  (`analysis/block_calibrate.py`) fits all to 0.25 mm but leave-one-out
  predictions are +0.45 / −0.80 / +2.41, so it isn't used.

- 2026-11-04, **line laser + Arducam OV9281** on the head (Eric's rig: mono,
  global shutter, 1280×800 MJPG at up to 120 fps, `0c45:6366`; the line laser
  has its own switch). camserver doesn't serve it (it looks for the Microdia);
  `kinsey/arducam.py` opens it by `/dev/v4l/by-id`. cmux had been stopped
  since 11-02 and kinsey's controller pointed straight at the machine; with
  Eric's OK the controller now goes through cmux (`config.ini` backed up
  next to it; cmux started, still not enabled at boot).
  - `axis_test.py` (20 mm X and Y moves at Z −2): image +u ≈ machine −X,
    +v ≈ +Y, **5.73 px/mm at the bed**, camera rolled **~2.8°** (holes and
    a PCB ruler give 2.6°). The laser line is square to the hole grid to
    ~0.4°.
  - Exposure: room light is strong even with the work light off; 1.5 ms
    (`--exposure 15`). Polished aluminum tops reflect the room so brightly
    that the line adds only 10–30 counts there, so line points are kept by
    continuity (runs of ≥ 12 columns), not brightness (threshold 10).
  - Sweep `line_scan.py --x -110 --y -230 -40 0.5`: 381 frames, 289 s. The
    fan covers columns 193–1014 (~147 mm of bed). Bed line fit 0.20 px rms.
  - Height = A d / (1 + B d) on the line's shift d above the bed line.
    **Calibrated on the blocks**, so their heights are not an independent
    check: 2" top d 93.6, 3" top d 180.0 → camera 166 mm above the bed.
    Cross-check without the 3" block: the 2" top's magnification (8.10 vs
    5.73 px/mm when the head steps) predicted the 3" top at 77 mm.
  - Map `data/2026-11-04/scan1.html` (+ `scan1-top.png`): 2" 50.89 mm
    (1,171 cells, sd 0.45), 3" 76.61 (261 cells, angled, partly outside the
    fan at that height), aluminum channel walls 9.5–10.1 mm, floor sd
    0.81 mm. X/Y are head position plus the image offset (the camera's
    offset from the spindle isn't known yet).

- 2026-11-04 (afternoon), **continuous sweeps, Z-sweep calibration, fitted
  CAD model.** Blocks re-taped (blue tape). `line_scan.py` (rewritten by
  another session) moves each pass as one G1 at F1800 while the camera
  records 120 fps; frames are placed by timestamp against status polled
  every ~14 ms. Camera latency **4 ms** (`sweep_height.py --latency-search`
  on the there-and-back `sweep1`).
  - **Z sweep** (`z_sweep.py`, X −110 Y −34, Z −1 → −79 in 3 mm, Eric: safe
    to −100 there): `analysis/zcal.py`: h = (0.807 + 0.0074 uc) d /
    (1 + 0.00498 d), **0.38 mm rms per column over 78 mm**, no blocks used.
    Camera **162.6 ± 0.4 mm** above the bed at Z −1, zoom centre (603, 417),
    from four bed holes tracked by template matching (blob detection and
    whole-image registration both failed on these dark frames).
  - **Hard limit:** the first plate run sent a pass to Y 0.000 (the home
    switch): HALT 21. Eric cleared it and the machine re-homed.
    `line_scan.py` now keeps 2 mm from every travel end.
  - **Plate sweep `plate2`**: 6 passes X −40…−340, Y −245…−5, Z −2:
    **78 s for the whole plate** (6,001 frames, 2.76 M points), vs 289 s for
    one 150 mm strip stop-and-go. Bed cells sd 0.31 mm after levelling.
  - `fit_shapes.py`: tops from the 1 mm grid, rectangles refined on the
    camera's top-down image at each top's height. Blocks (heights from the
    Z sweep only, so an independent check): 1" 26.16 mm, 51.48 × 75.75;
    2" 51.20, 27.08 × 76.00; 3" 76.86, 24.68 × 48.85. Heights read
    +0.4…+0.8 mm (tape, or laser scatter in the MDF vs tape); sizes within
    ~1.7 mm. Also: channel bar 10.3 mm, tool box 15.8 mm, spool (outline).
  - **Open:** X/Y of raised points. Side faces: only a few dozen line points
    land on faces the camera sees; the 2" block's east face is 0.5 mm off
    vertical, but bed-height points continue ~5 mm "under" that block in
    one pass. Not resolved; tops are placed to ~1–2 mm.
  - `cad_model.py` → `data/2026-11-04/model/`: `plate.scad`, per-part
    STLs, three.js viewer `index.html` (serve `data/` over HTTP).

- 2026-11-04 (evening), **two cameras: laser vs stereo** (old frame, both
  Arducams rotated 90°, on hub ports 2.2 and 2.3; same USB serial, so
  `arducam.py --cam`/`port=` picks by port). They can't stream together on
  one USB 2 hub ("Not enough bandwidth for altsetting 6"), so each camera
  swept the plate on the same path (`line_scan.py --cam`, ~75 s each) and
  frames are paired by head position.
  - The cameras straddle the laser sheet but face away from it: above
    ~60 mm the line leaves both frames (no laser 3" top on this frame).
  - Per camera: `z_sweep.py --cam 2.2 2.3`, `zcal.py --rot cw|ccw` (frames
    turned so the line runs across): 0.40 / 0.38 mm rms; camera 162.6 /
    158.2 mm above the bed. Port 2.3's motion matrix is derived from port
    2.2's through the bed homography (`axis3/2.3/matrix.json`; its own
    test was mostly off-bed).
  - `stereo_sweep.py`: triangulates each match from the two camera models;
    camera offset (4.87, −119.86) mm from bed matches, vs (3.97, −120.6)
    from aligning the two laser maps. Raised points shift *left* in
    warped A (−65 px at 15 mm, −311 px at 51 mm), so B is SGBM's left image.
  - `compare_methods.py` → `data/2026-11-04/compare/`: laser A vs B
    median +0.01, MAD 0.24 mm; stereo vs laser median −0.25, MAD 0.71 mm.
    Stereo covers 94% of the plate (lasers ~50%) but blurs edges and fine
    steps and misses the 3" top. Laser A and B place raised edges ~4 mm
    apart: the raised-point X/Y issue, still open.

- 2026-11-04 (late afternoon), **Cutebot Pro** (Elecfreaks robot, facing −Y)
  scanned for a 3D-printed frame (Eric's go-ahead). Same plate path per
  camera (`robot1_2.2`, `robot1_2.3`, 78 s each), same calibrations.
  - Camera B → A offset from bed holes: (4.79, −118.63), 5 holes, 0.70 mm
    rms (`robot_laser.py`). `stereo_sweep.py` got (5.5, −94.6) from 32 bed
    matches here (the robot dominates the SIFT matches): not used.
  - Holes and bosses come from top-down images at each deck's laser height
    (`robot_ortho.py`, `fit_shapes.Mosaic`), not the height grid: lower
    deck 20.2 mm, 4 bosses (A and B agree to ~1 mm, a constant shift);
    upper deck 36.1 mm, two rows of 7 at **8.0 mm pitch** (screw, hole, boss,
    hole, boss, hole, screw), 32.2 mm apart, row fit 0.15–0.19 mm rms.
  - Wheels: tread top 51.0 mm (circle fit to the tread profile), 17–18 mm
    wide. Battery box (taped) top 58.8 mm, 17.8 × 71 mm.
  - micro:bit: the laser reads up to 79 mm around it, but those are
    reflections off the board (camera A's points slide 1 mm in Y per mm of
    height in front of it). Top-down images focus on its USB/JST connectors
    at **67–68 mm** in both cameras, which fits a 42 mm board in an edge
    connector on the 20 mm deck.
  - **All 18 bosses and holes are on one 8 mm LEGO grid** (Eric): fit with
    the pitch fixed at 8, 0.19 mm rms, 0.36 max; pitch left free 8.009. The
    robot frame sits on that grid (origin at the centreline grid point in
    line with the middle upper holes, ~0.4 mm from the axle; +x right,
    +y forward), and positions are reported snapped: bosses (±8, 80) and
    (±32, 64), upper rows x ±16, y −24…+24.
  - `robot_features.py` → `data/2026-11-04/robot1/`: `features.json`,
    `cutebot.scad`, `features.png`, `index.html` (3D view).
    `cutebot_schematic.py` → `docs/cutebot-pro-schematic.png` and
    `-outline.json`; handoff for the shell design:
    `docs/cutebot-pro-mounting.md`.

- 2026-11-05, **new rig: two Arducams + cross laser**, and a sheet in the
  clamps (Eric's go-ahead for moves and the probe). Data in `data/2026-11-05/`.
  - Cameras on hub ports 2.2 / 2.3 (now behind a VIA USB 2 hub, bus 1 only);
    both work, both upright (long axis along machine Y, 275 px apart), but
    still **can't stream together** in any mode, 640×400 included ("Not
    enough bandwidth"): grab one at a time.
  - Axes (`axis5/`, SIFT with the laser masked; phase correlation locks onto
    the cross): head −X → image +v, head −Y → image +u, 5.56–5.72 px/mm, roll
    < 0.2°. `axis5/rot2.x/matrix.json` are in the rotated frames used below.
  - Cross laser: the arm along X (image v) is the height line, seen from
    opposite sides by the two cameras (`zcal5/`, Z −1.3…−31.3, 0.44 / 0.48 mm
    rms per column, cameras 161.3 / 162.6 mm above the bed, `--rot ccw` for
    2.2, `cw` for 2.3). The arm along Y stays on row ~436 at every Z: its sheet
    contains both cameras, so it carries **no height signal**; on bright
    metal its crossing blanks ~10 mm of the height line per pass.
  - Laser map frame → spindle: **(−71.7, −68.1) mm**, from the anchor L's
    inner corner (bars 15 mm, `coordinate.anchor1` −359.208, −235.638).
    Camera 2.3's map lands on 2.2's after (−3.0, +52.5).
  - First look (`survey1`, `plate_2.x`): a ~1.5 mm metal sheet with three
    windows on a ~4.7 mm backing; the stack read ~6.7 mm and rocked 1.7 mm
    over 88 mm. Eric then removed the backing and sprayed the sheet.
  - Sprayed sheet (`plate2_2.x`, 4 passes 45 mm apart per camera so the
    cross gaps are covered; `analysis/flatness.py` → `flat2.*`): flat to
    ±0.19 mm (p1–p99, 5 mm average), tilted 0.27 mm/100 mm in X vs the bed.
    Plate-top cell noise 0.55 → 0.21 mm rms (spray plus twice the passes;
    bed cells went 0.34 → 0.25 from the passes alone).
  - **Probe check** (`kinsey/probe_z.py`: `M6 T0`, then the firmware's
    `M466 Z-<dist>` double-tap cycle only, each descent capped ~2.5 mm past
    the expected surface; probe TLO 41.565): 4 bed + 5 sheet touches,
    repeat 0.005 mm. `analysis/probe_compare.py`: **laser − probe on the
    sheet +0.008 mm mean, sd 0.057**; bed spots within ±0.06. Sheet height
    above the bed 1.55–1.92 mm (probe). Bed under the sheet at machine Z
    −111.05 (probe tip); G54 Z0 (−144.55) is 8.1 mm above the bed, left from
    the old setup. Probe back in the rack after.
  - `sheet-plan.png`: windows, clamps, cut zones and the probe spots in
    machine coordinates.
  - **Cross turned 45°** (Eric; spray gone, sheet unchanged). The hub came
    back on **ports 1.2 (= old 2.2) and 1.1 (= old 2.3)**; frames moved ~1 mm.
    Arms at +48.5° / −41.5° in the image; both carry height, 0.57–0.72 px/mm
    each (straight line: 1.04–1.15). `analysis/cross.py`: turn the frame so an
    arm runs along the rows, ridge per column, h = (a0 + a1 s) d / (1 + B d)
    per arm (`zcal45/`, 0.45–0.73 mm rms per point); `sweep` places both arms.
    Same 4-pass scan (`plate3_1.x`): 5.0–5.3 M points per camera vs 2.0–2.2 M.
    On the sheet core, usable points (within 1 mm of the sheet) **99–113/mm²
    vs 52–56 for the line on the sprayed sheet**; 75–79% of cross points
    usable vs 91–93%; cell noise 0.17–0.21 vs 0.20 mm; with 2 passes 87–90%
    of cells filled vs 77–82%. Probe check: both arms −0.059 mm mean, **sd
    0.033** (line +0.008, sd 0.057). Arm "1" (−41.5°) is the weaker arm
    (streaks on the anchor, 94–96% fill).

- 2026-11-05 (evening), **1-2-3 blocks + EANOSIC setup bars** (labels read
  from the photos: 1/16, 3/32, 1/8, 5/32, 3/16, 1/4, 1/2"). Same Y-pass scan
  (`blocks1_1.x`, 5 passes) plus new **X passes** (`line_scan.py --along x`,
  `blocksX_1.x`). **Deep Z sweep** for the cross (`zcal45deep/`, Z −1.3 →
  −79.3 at head X −60 Y −145; clearances read from the scan map first) and a
  per-arm model h = (a0 + a1 s + a2 s²) d / (1 + (b0 + b1 s) d): arm +48°
  within ±0.17 mm per Z step, arm −41° up to +0.3. Camera 1.1 → 1.2 map shift
  (−3.5, 53.5); `analysis/blocks_merge.py` → `blocks_all.npz` (23 M points).
  - Heights (`analysis/heights_report.py`, each camera/arm against its own bed
    plane, median of four; `heights.json`): 3" 76.46 (+0.26), 2" 50.91
    (+0.11), 1" 25.36 (−0.04), 1/2" 12.79 (+0.09), 1/4" 6.39 (+0.04), 3/16"
    4.87 (+0.11), 1/8" 3.21 (+0.04), 5/32" 4.43 (+0.47), 3/32" 2.67 (+0.29),
    1/16" on the 1" block 2.09 (+0.50). The 5/32" and 3/32" read high in every
    camera/arm and both are tilted (~1°): possibly not flat on the bed.
    Spread between camera/arm combinations up to 0.78 mm; camera 1.2 arm +48°
    alone is within ±0.16 on 7 of 10.
  - `analysis/view3d.py` → `view3d/`: photo-draped surface (0.5 mm) and
    voxel cloud (1.8 M points), three.js viewer `index.html`
    (http://localhost:8766/2026-11-05/view3d/ with `data/` served on 8766),
    `surface.ply` / `cloud.ply` in machine coordinates for CloudCompare or
    MeshLab, `relief.png` (labelled heights), `ortho.png` (photo mosaic; laser
    lines masked). Photos are the lit grids `photos1/` + `survey3/`.
  - `analysis/edge_lengths.py` (lengths from head travel between edge
    crossings) is written but **not run or checked yet**.

- 2026-11-05 (night), **package sprint, phase 1** (`docs/package-plan.md`).
  Legacy scripts moved to `attic/`, one-offs to `examples/`; camserver
  disabled on kinsey. Processing moved to **Buzzkill** (`~/carvera-scan`, uv
  venv with Open3D 0.20, Python 3.14; rsync to `eric@192.168.1.23`).
  `analysis/bench.py` reruns the cross chain from raw frames and scores the
  blocks and the sheet. Found: the −41° arm's bed row from the deep Z sweep
  was extrapolated past objects in its view (bed read 3–5 mm low at one end of
  the arm); `cross.py --bedref scan` takes the bed row from the scan (lowest
  strong row band). With `--bedref scan --gate --grid mode --consistency 2
  --ambig 0.6`: spike clusters 17 → 0, points above 80 mm 44,005 → 2,452
  (≤ 81), height error mean/max 0.195/0.504 → 0.130/0.295 mm, camera/arm
  spread 0.47/0.78 → 0.29/0.56 mm, sheet vs probe unchanged (sd 0.031).
  Viewer from the cleaned points: `data/2026-11-05/view3d_v2/`.
  Second round (Eric: "you'll see a bunch of spikes" in v2; the metric only
  counted clusters >= 10 cells, single-cell needles slipped through):
  consistency keyed by camera x arm, `grid.support_filter`,
  `grid.remove_specks`, bounded hole filling, Open3D radius outlier removal on
  the cloud. Needle cells 564 -> 338 (rest are corners/hole rims), cells above
  block tops 117 -> 30, above 80 mm -> 0. Viewer `view3d_v4/`.


## Older notes

Kept from the README of the time.

### Plan

1. **Camera intrinsics, from known motion.** The head moves over a grid of
   XY positions at three or four Z heights while kinsey takes frames. The
   machine's own coordinates give each camera translation, so a single
   bundle adjustment solves the lens (focal length, distortion), how the
   camera is mounted on the head, and where the board sits in machine
   coordinates, including its height. Translations alone don't constrain
   focal length; the known Z steps do.
2. **Laser line.** In laser mode with the test fire on, the dot moves
   along one image line as the surface distance changes. Dot positions at
   several Z heights over the flat board fix the laser ray in camera
   coordinates. After that, one image gives the surface height under the dot.
3. **Probe check.** The probe measures 9–12 points across the board and the
   results are compared with the camera heights. Probing gets its own
   review before it runs.

### How to scan with the line laser (kinsey, through cmux)

1. Laser on (its own switch). `axis_test.py cap/axis` once per camera
   mounting; `z_sweep.py cap/zcal --z -1 -79 3 --zmin -80` over bare bed
   (check the clearance with Eric), then `analysis/zcal.py data/…/zcal
   --holes x y … --out zcal.json` with bed holes picked in the first frame.
2. `line_scan.py cap/scan --x -40 -100 -160 -220 -280 -340 --y -240 -10`
   (continuous passes, 0.25 mm/frame, Z unchanged, light off).
3. `analysis/sweep_height.py data/…/scan --zcal zcal.json --axis data/…/axis
   --latency 0.004 --out scan.npz`; `analysis/grid.py scan.npz --cell 1
   --out grid1.npz`; `analysis/fit_shapes.py data/…/scan --grid grid1.npz …
   --out shapes.json`; `analysis/cad_model.py shapes.json grid1.npz --out model`.

### How to scan (kinsey, machine over Wi-Fi at 192.168.6.32, controller closed)

1. Calibrate on bare spoilboard (no objects within ~50 mm):
   `laser_scan.py cap/lcal --host 192.168.6.32 --at X Y --z -3 -6 … -48 --zmin -48`
   then `uv run python analysis/laser_height.py calibrate data/…/lcal`.
2. Scan at Z −3: `laser_scan.py cap/scan --host 192.168.6.32 --z -3 --x -40 -240 3 --y -20 -220 3`
   (laser 0.5%, exposure 2 ms, light off, all restored at the end).
3. `analysis/laser_height.py map lasercal.json data/…/scan` then
   `analysis/heightmap.py scan.npz --cell 3 --out map.html`.

### Next

- The 3" block was only partly scanned (it sits past X −240). Move it in,
  or confirm how far left the head can go since the camera-mount fix.
- Calibrate with a Z sweep deeper than 45 mm once probing tells us the real
  clearance under the head; the 2"/3" readings are extrapolations.
- Line laser: resolve the X/Y placement of raised points (the side-face
  puzzle above); a ChArUco intrinsic calibration of the Arducam would pin
  the camera model. Try the 3D scanning spray on the tops (tape gives
  little texture for the camera, and may explain the +0.5 mm).
- Probe: tie camera heights to machine work Z with a probe touch. Review
  the probe G-code with Eric first.

### Known issues

- **Fixed 2026-11-01:** kinsey moved to Busboom Garage, **192.168.6.62**
  (`kinsey.home`), and connects to 192.168.6.32:2222. Kept for history:
  kinsey (192.168.4.62) couldn't open TCP to the Carvera. The Carvera's own
  AP uses 192.168.4.x, so (likely) its replies to 192.168.4.x go into its
  AP. Ping works; TCP from Home (192.168.1.x) works. Possible fixes: USB
  cable from kinsey to the Carvera; give kinsey a source address outside
  192.168.4.0/24 for this route; or masquerade robot → carvera on sajak.
