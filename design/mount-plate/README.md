# Mount plate and dongle bracket

OpenSCAD sources, from Eric's hand measurements (2026-11-04).

| File | What |
|---|---|
| `plate.scad` | The aluminium mount plate. Origin at the plate centre on the front face |
| `bracket.scad` | Angle bracket for the USB-to-vacuum dongle, glued to the plate; the 4-pin 1/2" panel connector goes through its upright face |
| `print.scad` | Print layout: the plate on its back face, the bracket on its holed face |
| `print_plate.scad` | Print layout: the plate alone |

The print layouts `import()` the STLs, so export `plate.stl` and `bracket.stl` from
OpenSCAD first. STL files and renders are generated and not kept in git.

The scanned version of the plate is `projects/plate-model/`.
