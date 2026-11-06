# Cutebot Pro: mounting points and clearances for a printed shell

Measurements of an Elecfreaks Cutebot Pro (micro:bit robot), from a line-laser
scan on the Carvera on 2026-11-04. Use them to design a 3D-printed shell that
screws into the robot's bosses and clears everything else.

The schematic (top, side and front views) is generated into
`out/cutebot-pro-schematic.png`; see "Rebuild".

## Files

| File | What it holds |
|---|---|
| `results/features.json` | **The data.** Bosses, holes, deck heights, wheels, battery, micro:bit, all in the robot frame |
| `results/outline.json` | Outer outline polygons (robot frame): `footprint` (everything over 3 mm, wheels included) and `lower_deck` |
| `results/laser.json` | Laser heights of the decks and parts (`robot_laser.py`) |
| `robot_laser.py`, `robot_ortho.py`, `robot_features.py`, `cutebot_schematic.py` | The scripts, in pipeline order |
| `out/` | Ignored. The schematic, `cutebot.scad` (OpenSCAD reference: `lower_bosses()`, `upper_row(kind)`, `envelope()` keep-out solids) and the 3D viewer |

The JSON and the SCAD come from the same numbers; the tables below copy them.
If they ever disagree, `features.json` wins.

## Rebuild

The scripts read and write the capture folder `data/2026-11-04/robot1/` (inputs from
`robot1_2.2`, `robot1_2.3`); see each script's docstring for the full command line.

```sh
python projects/cutebot-pro/robot_features.py data/2026-11-04/robot1     # features.json, cutebot.scad
python projects/cutebot-pro/cutebot_schematic.py data/2026-11-04/robot1 --out projects/cutebot-pro/out/cutebot-pro
```

Copy a new `features.json` or outline into `results/` when you mean to keep it.

## Coordinate frame

- **+x** toward the robot's right, **+y** forward (toward the front, where the sonar
  socket is), **+z** up. Millimetres.
- **Every boss and hole is on one 8 mm LEGO grid**, and the frame sits on that
  grid. The origin is on the table at the grid point on the centreline, in
  line with the middle upper holes (L4 and R4). The wheel axle is on that same
  grid row to within about 0.4 mm (axle z ≈ 25.5).
- So every mounting position is exactly 8 × (i, j) mm, with i and j whole
  numbers. All 18 scanned bosses and holes fit that grid to 0.19 mm rms
  (0.36 mm worst); a fit with the pitch left free gives 8.009 mm. **Use the
  grid values, not the raw measurements**: they are more accurate than any
  single measurement.
- The front of the lower deck is at y ≈ +93 and the rear of the robot at
  y ≈ −37.

## Mounting points

### Lower deck: 4 bosses at z = 20.2

Metal threaded inserts in the lower (front) deck. Their tops stand about 1 mm
above the deck surface (≈ 21.2).

| Label | Where | Grid (i, j) | x | y |
|---|---|---|---|---|
| B1 | left side | (−4, 8) | −32 | 64 |
| B2 | front, left of centre | (−1, 10) | −8 | 80 |
| B3 | front, right of centre | (1, 10) | +8 | 80 |
| B4 | right side | (4, 8) | +32 | 64 |

### Upper deck: 2 rows of 7, z = 36.1

The upper deck lies either side of the battery. The left row is at x = −16
(i = −2) and the right row at x = +16 (i = 2), 32 mm apart. Both run rear to
front on the 8 mm pitch, at the same y values:

| Index | Grid j | y | Kind |
|---|---|---|---|
| 1 | −3 | −24 | screw head (battery holder): clear it |
| 2 | −2 | −16 | leg-connector hole |
| 3 | −1 | −8 | **boss** |
| 4 | 0 | 0 | leg-connector hole |
| 5 | 1 | +8 | **boss** |
| 6 | 2 | +16 | leg-connector hole |
| 7 | 3 | +24 | screw head (battery holder): clear it |

So the upper bosses are L3 (−16, −8), L5 (−16, 8), R3 (16, −8) and R5 (16, 8).
The leg-connector holes are plain holes for LEGO-style pins. In
`features.json` each entry has `grid`, `xy` (snapped to the grid) and
`xy_measured` (as scanned).

## What the shell must clear

| Part | Footprint (x; y) | Top (z) |
|---|---|---|
| Wheels (2) | x −55.3…−38.8 and +36.7…+54.0; axle at y ≈ 0, 51 mm across, so y ≈ −25.5…+25.5 | 51.0 |
| Battery box (on the upper deck, centre) | x −7.8…+10.5; y −33.5…+37.3 | 58.8 |
| micro:bit (stands vertically, LEDs face forward) | x −26.9…+24.7 (51.6 wide); board at y ≈ +51.4, 1.6 thick | 68.0 (bottom edge probably ≈ 26, in an edge connector on the lower deck) |
| Lower deck surface | see `lower_deck` outline | 20.2 |
| Upper deck surface | between the wheels, beside the battery | 36.1 |
| Whole robot | x −58…+58; y −37…+96 | 68 (micro:bit) |

The wheels turn, so give them running clearance around their full 51 mm
circle, not just the top.

## Accuracy and what is not known

- **Thread size and hole diameters were not measured.** The rings in the
  images are about 6.5 mm across, but that is the insert's flange, not the
  thread. Confirm the screw size (M3 is common, but nobody has checked) with
  Eric before you size clearance holes. The leg-connector holes match the
  standard LEGO pin hole (4.8 mm) by pitch, but their diameter wasn't measured
  either.
- **Boss and hole centres:** exact grid values, assuming the board is made
  to the LEGO grid; Eric says it is, and the scan agrees to 0.19 mm rms. The
  raw measurements (`xy_measured`) are good to about ±0.5 mm.
- **Clearances** (wheels, battery, micro:bit, outlines) are measured, not
  snapped, so treat them as ±1 mm or as stated below.
- **Heights:** decks ±0.4 mm. Wheel tops ±0.5 mm. Battery top ±0.5 mm. The
  micro:bit top is 68 ± 1 mm and its y position is good to ±1.5 mm.
- **Battery box:** it measures 1.4 mm right of the centreline. That may be
  real, or it may be a known placement error for high points (it grows with
  height). Leave 2 mm extra around it.
- **Outlines:** good to about 1–2 mm. The footprint is gap-filled; the
  `lower_deck` outline is ragged where the scan had shadows.
- **Not scanned:** the underside, anything under the decks, and the sonar
  module (it isn't fitted; its socket is on the front deck, between and just
  behind B2 and B3, under the "SONAR" label). Details smaller
  than about 1 mm (headers, LEDs) aren't in the outlines.
