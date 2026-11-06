# Plate model: scan to OpenSCAD

The aluminium mount plate (the one in `design/mount-plate/`) with objects on it, scanned
on 2026-11-04 with the straight-line laser. The tops were fitted as shapes, and
`cad_model.py` turns them into an OpenSCAD model of the plate: the spoilboard with the
holes the scan found, and the objects as terraces of their measured contours.

## Files

| File | What |
|---|---|
| `cad_model.py` | Shapes + grid → `OUT/plate.scad`, `OUT/parts/*.scad`/`.stl`, `OUT/index.html` |
| `results/plate2-shapes.json`, `plate3_2.2-shapes.json` | Fitted shapes (`carvera_scan.measure.shapes`) |
| `out/` | Ignored. The generated model |

## Rebuild

```sh
python -m carvera_scan.process.grid data/2026-11-04/plate2.npz --cell 1 --out data/2026-11-04/plate2-grid1.npz
python projects/plate-model/cad_model.py projects/plate-model/results/plate2-shapes.json \
    data/2026-11-04/plate2-grid1.npz --out projects/plate-model/out
```

Shape fitting: `python -m carvera_scan.measure.shapes data/2026-11-04/plate2 --grid … --out …`
(see its docstring).
