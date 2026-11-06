# Cutebot duo: holes and bosses on a Cutebot Pro and a Cutebot

Two Elecfreaks micro:bit robots side by side on the bed, sprayed with scanning
spray, scanned on 2026-11-05: the **Cutebot Pro** (left, map X −262…−148) and the
round **Cutebot** (right, map X −109…−24). The goal is the positions and sizes of their
mounting holes and bosses.

The project has no scripts of its own: every step is a package command (below). It is
also the first test of three new pieces: `measure.features` (coarse holes and bosses
from the height map), `process.spot` (the laser cross centred in each hole) and
`measure.lattice` (grid fit as an accuracy check).

## Files

| File | What |
|---|---|
| `results/holes.json` | **The data.** 25 hole centres and diameters (camera A map frame), per-camera centres, and for the Cutebot Pro the 8 mm grid index of each hole and its distance from the grid |
| `out/` | Ignored. `scan/` and `fine/` (pipeline points and grids), `features.json`/`.png` (coarse pass), `spots1/2.json` (shot plans), `holes1/2.json` (spot fits), `refined_A/B.json` (fine-scan outlines), `view3d/` (viewer) |

## How it was done

Captures in `data/2026-11-05/` (kinsey `cap/`, copied to Buzzkill):

1. `duo_survey0`: lit photos of the whole bed, to find the robots.
2. `duo1_1.2`, `duo1_1.1`: 7 passes along Y (X −300 … −30, 45 mm apart), Y −4 … −215,
   one run per camera (the two cameras share one USB 2 link and can't record together).
   `duoX_1.2`, `duoX_1.1`: 3 passes along X (Y −100, −60, −20), the other direction for
   shadows. Z −1.3 (the calibration's Z), pitch 0.25 mm.
3. `duo_photos1`: lit photos over both robots, for the viewer.
4. `duo_spots1`, `duo_spots2`: still laser frames with the cross centred on each hole
   (round 1: one shot per hole; round 2: centred on round 1's result plus four shots
   0.4 × radius off centre along each arm).
5. `duofine_1.2`, `duofine_1.1`: one slow pass (0.05 mm/frame) along Y over each robot.

Commands (Buzzkill unless noted; `C=config/calib/2026-11-05-cross45-deep.json`,
`D=data/2026-11-05`, `O=projects/cutebot-duo/out`):

```sh
# kinsey: scans and photos
python -m carvera_scan.acquire.line_scan cap/duo1_1.2 --cam 1.2 --x -300 -255 -210 -165 -120 -75 -30 --y -4 -215
python -m carvera_scan.acquire.line_scan cap/duoX_1.2 --cam 1.2 --along x --y -100 -60 -20 --x -330 -5
python -m carvera_scan.acquire.survey cap/duo_photos1 --x -280 -220 -160 -100 -40 --y -130 -70 -10   # (and --cam 1.1 runs)

# points, coarse features, viewer
python -m carvera_scan.process.pipeline $D/duo1_1.2:1.2 $D/duo1_1.1:1.1 $D/duoX_1.2:1.2 $D/duoX_1.1:1.1 --calib $C --out $O/scan
python -m carvera_scan.measure.features $O/scan/points.npz --crop -280 0 -160 40 --min-z 3 --out $O/features.json --png $O/features.png
python -m carvera_scan.view.view3d $O/scan/points.npz --calib $C --photos $D/duo_photos1 --crop -280 0 -160 40 --out $O/view3d

# cross-in-the-hole: plan (Mac or Buzzkill), shoot (kinsey), fit
python -m carvera_scan.process.spot plan $O/features.json --calib $C --info $O/scan/info.json --out $O/spots1.json
python -m carvera_scan.acquire.survey cap/duo_spots1 --at spots1-at.json --light 0 --exposure 15 --settle 0.5
python -m carvera_scan.process.spot measure $D/duo_spots1 --plan $O/spots1.json --calib $C --info $O/scan/info.json --out $O/holes1.json
python -m carvera_scan.process.spot plan $O/holes1.json --offsets 0.4 ... --out $O/spots2.json      # then shoot and measure again -> holes2.json
python -m carvera_scan.measure.lattice $O/holes2.json --ids H03 H04 H05 H06 H07 H08 H09 H10 H11 H14 H15 H17
```

To look at it: `python3 -m http.server 8771 -d projects/cutebot-duo/out/view3d`, then
<http://localhost:8771/>.

### The cross-in-the-hole method (`process.spot`)

The calibration's laser model, run backwards, says where each arm of the cross falls
on a surface at height h, so it gives the head position that puts the cross centre on
a target at that height. Over a hole, each arm's line on the top surface has a gap; the
two ends of the gap are points on the rim, so each arm gives a chord. The arms cross at
about 90°, so one frame gives two chords (four rim points), and the perpendicular
bisectors of the chords meet at the centre. Off-centre shots give more chords; all rim
points from all shots and both cameras go into one circle fit. Each shot takes about
2 s; round 2 was 125 shots × 2 cameras for 25 holes (about 10 minutes).

## Results

### Accuracy: the Cutebot Pro's 8 mm grid

Every boss and hole on the Cutebot Pro lies on one 8 mm (LEGO) grid
(`projects/cutebot-pro/README.md`), so the distance of each measured centre from the
best-fit grid is a direct accuracy check. Over its 12 holes and inserts (the four domed
screw heads at the row ends excluded):

| Method | rms off the grid | worst | pitch, left free |
|---|---|---|---|
| Coarse height map (`measure.features`, 0.25 mm cells) | 0.26 mm | 0.49 mm | 7.967 |
| Cross in the hole, 1 shot × 2 cameras | 0.24 mm | 0.59 mm | 7.974 |
| **Cross in the hole, 5 shots × 2 cameras** | **0.11 mm** | **0.22 mm** | **7.991** |
| Fine scan (0.05 mm/frame) + outline fit, camera A | 0.48 mm | 0.74 mm | 7.966 |
| Fine scan + outline fit, camera B | 0.30 mm | 0.45 mm | 7.999 |
| (2026-11-04 line-laser scan, 18 features) | 0.19 mm | 0.36 mm | 8.009 |

With the screw heads included (16 features) the 5-shot result is 0.225 mm rms. Cameras A
and B agree on each centre to 0.08–0.46 mm (median about 0.2 mm). The fine scan with
outline fitting is worse than the coarse map: the outlines leak into nearby shadows and
the hole interiors. It is not used for the results.

### Holes

Diameters from the circle fits (rim at the top surface): leg-connector holes 4.64–4.79 mm
(LEGO pin hole 4.8), threaded holes in the upper-deck bosses 2.39–2.47 mm, lower-deck
inserts 2.36–2.85 mm (bore), screw-head recesses 1.7–2.0 mm. Full list:
`results/holes.json`.

- **Cutebot Pro**: 13 of the 14 upper-row positions (both rows of 7; the left row's
  boss at index 5 was not found by the coarse pass) and 3 of the 4 lower-deck inserts
  (the fourth, near grid (1, −13), was found only in an early 0.5 mm pass). Same layout
  as the 2026-11-04 scan.
- **Cutebot**: 9 holes: 7 on the top deck (4 corner screws, 1.6–2.3 mm; 2 larger
  holes, 4.4–4.5 mm; one 2.6 mm hole) and 2 lower-deck inserts (2.5 mm bore). Several smaller holes in the middle of the top
  deck's front row are visible in the relief but were not found.

### Bosses

Not measured yet. The coarse pass finds only 2 (on the Cutebot's lower deck); the
inserts are 1 mm proud, too little for the opening test at this noise. Outline fits on
the fine scan give the inserts' flanges as about 7.9 mm across where they work.

## What is open

- Seeds: the coarse pass misses some small holes. Lattice completion (a grid fit
  proposes the missing positions) would fill the Cutebot Pro's gaps.
- Bosses: a raised-chord variant of `process.spot` (kind "boss") is written but not run.
- Robot frames: results are in the map frame; the Cutebot Pro's grid fit gives its frame
  (origin, angle) directly.
