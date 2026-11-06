# Camera + line-laser mount: geometry requirements

For whoever designs the bracket that holds two cameras and a line laser on the
Carvera's head. Numbers are measured on the current rig (2026-11-04) unless
marked as derived.

## What the rig is for

1. **Laser scanning.** A line laser draws a line on the bed. The head sweeps
   across the bed and a camera watches the line. The height of the surface
   moves the line sideways in the image. This works now, at 0.38 mm rms per
   point over 0–78 mm of height.
2. **Stereo.** Two cameras see the same area from slightly different
   positions, and depth comes from the sideways shift (disparity) between the
   two images. This does **not** work with the current mount: the cameras'
   views overlap in only a ~17 mm strip of bed.

The new bracket should make both work, from one rigid part.

## Hardware

- **Cameras (2×):** Arducam OV9281 USB, mono, global shutter, 1280 × 800 px
  at 3 µm. Lens: focal length 926 px ≈ 2.8 mm (derived), field of view about
  69° along the long side (1280 px) and 47° along the short side (800 px).
- **Line laser:** one. It draws a straight line, and its light sheet is
  currently vertical (within 1°).
- **Machine axes:** X and Y are horizontal, Z vertical. The head moves in all
  three. Scans run at the top of Z travel.

## Current geometry (measured)

- **Camera A height:** optical centre 162 mm above the bed with the head at
  Z −2. Image footprint on the bed: 223 × 140 mm, with the long side along
  machine X.
- **Camera A pointing:** straight down to within ~2.5°. Rolled 2.8° about
  the vertical relative to the machine axes.
- **Laser:** line along machine X. The sheet is vertical and 35 mm from
  camera A's optical axis, offset in machine Y. Height sensitivity: 1.24 px
  per mm at the bed, 4.4 px/mm at 76 mm.
- **Laser fan:** covers only ~142 mm of the 223 mm the camera sees along the
  line, so a wider fan would scan more per pass.
- **Camera B:** about 80 mm from camera A in machine Y. Its view on the bed
  is ~120 mm from A's view, so the two optical axes diverge by about 14°.
  Overlap: ~17 mm of bed.
- **Clearance:** with the head at Z −100 the lowest point of the rig is
  26.5 mm above the bed (it is ~124 mm above the bed at Z −2). The tallest
  part scanned is 76 mm, so the new rig must not hang lower than the
  current one.

## Rule 1: the laser line must be at right angles to the laser–camera offset

A camera measures height from how far the line moves **across** itself.
That only happens if the camera is offset from the laser sheet in the
direction perpendicular to the line. Offset along the line gives no signal at
all; the first mount made this mistake.

Sensitivity for a vertical sheet, in px per mm of height, where f = 926 px,
b = horizontal distance from the camera's optical axis to the sheet
(measured across the line), and z = distance from the camera down to the
surface:

    sensitivity ≈ f · b / z²

| b (mm) | at bed, z = 162 | at 3" top, z = 86 |
|---|---|---|
| 20 | 0.71 px/mm | 2.5 px/mm |
| 25 | 0.88 | 3.1 |
| 30 | 1.06 | 3.8 |
| 35 (now) | 1.24 | 4.4 |

The line is located to ~0.1–0.2 px, so b ≥ 20 mm is usable and ≥ 30 mm is
better. Tilting the sheet away from the camera raises b at depth, if a
bracket needs the laser closer.

## Rule 2: put the stereo baseline along the cameras' long axis

The baseline B is the distance between the two cameras' optical centres. For
parallel cameras, the overlap along the baseline is the view width at that
depth minus B. Depth resolution, for 0.25 px matching precision:

    Δz ≈ z² · 0.25 / (f · B)

Overlap, as a fraction of each image, and depth resolution, at the bed and at
a 3" (76 mm) top:

| B (mm) | baseline along long axis | baseline along short axis | Δz at bed / at 3" top |
|---|---|---|---|
| 20 | 91% / 83% | 86% / 73% | 0.35 / 0.10 mm |
| 30 | 87% / 75% | 79% / 60% | 0.24 / 0.07 mm |
| 40 | 82% / 66% | 71% / 46% | 0.18 / 0.05 mm |
| 50 | 78% / 58% | 64% / 33% | 0.14 / 0.04 mm |
| 80 (now) | 64% / 33% | 43% / 0% | 0.09 / 0.02 mm |

The current mount has B ≈ 80 mm along the **short** axis, and the axes
diverge. That's the worst case.

## Tilt (toe-in) or move the cameras closer?

**Prefer moving the cameras closer, with the optical axes parallel.**

- **Parallel cameras:** depth is a simple function of disparity, matching is
  along one image direction, and overlap shrinks smoothly as surfaces rise.
- **Toe-in** works, after calibration and rectification (resampling both
  images so their rows line up), but it costs four things:
  - The overlap is best at one depth (where the axes cross) and falls off on
    both sides of it.
  - The two cameras see surfaces at different slants, which makes matching
    harder on tilted faces.
  - The calibration is more sensitive to small mount changes.
  - Rectification throws away image corners.
- **When to tilt:** only if B must exceed ~50 mm. Keep each camera within
  ~10° of vertical, aimed so the axes cross about 40 mm above the bed:
  tilt = atan(B / (2 × 120 mm)).

## Recommended layout: cameras on either side of the laser

```
   machine Y →
   [cam 1]   |laser|   [cam 2]        (viewed from above; line runs along X)
     ←b→       sheet      ←b→
     ←———————— B = 2b ————————→
```

- **Laser:** in the middle, sheet vertical, line along machine X (as now).
- **Cameras:** one on each side of the sheet, along machine Y, each
  b = 22–25 mm from it. That gives B = 45–50 mm. Both point straight down,
  axes parallel within ±2°.
- **Camera orientation:** rotate both cameras 90° from today, so the sensor's
  **long axis (1280 px) runs along machine Y**, along the baseline. The laser
  line then runs along the image's short axis.
- **Height:** same as now (optical centres ~160 mm above the bed at Z −2),
  both at the same height within ±2 mm, and no part lower than the current
  rig.
- **What this gives:**
  - stereo overlap of ~80% at the bed and ~60% at a 3" top
  - Δz about 0.15 mm at the bed
  - laser sensitivity of ~0.8 px/mm per camera at the bed
  - **both cameras see the laser line from opposite sides**, so a block face
    hidden from one camera is seen by the other. Missing data at block sides
    is the main weakness of the scans today.
- **If the laser must sit closer than ~20 mm to a camera**, tilt the laser
  sheet 5–10° away from that camera rather than tilting the cameras.

Fallback if only small changes are possible: keep today's camera
orientation, bring the cameras to B = 20–30 mm along machine X, side by side,
parallel, with the laser 30–35 mm away in Y as now. Stereo overlap is then
87–91%, and laser scanning stays as it is from either camera.

## Mechanical requirements

- **One rigid part** holds both cameras and the laser. Their positions
  relative to each other are calibrated once, and any flex or play beyond
  ~0.05 mm or ~0.1° shows up as measurement error.
- **Repeatable mounting** to the head (dowels, or a keyed face), so the
  bracket comes off and goes back on without a full recalibration.
- **Adjustment:** none needed beyond focus. Calibration absorbs small angle
  and position errors. Don't add slots that can slip.
- **Lens focus:** set for ~100–160 mm from the lens, which covers bed to
  3" top.
- **Cables:** each camera has its own USB cable. The two cameras report the
  same USB serial number, so the software tells them apart by which hub port
  they're plugged into. Label the cables (e.g. "cam 1 → port 2", "cam 2 →
  port 3") and keep them on the same ports.
