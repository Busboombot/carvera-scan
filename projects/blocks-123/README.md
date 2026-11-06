# 1-2-3 blocks: accuracy test

The 1-2-3 blocks (1", 2", 3" tops) and the setup bars (1/16" to 1/2"), scanned on
2026-11-05 with the 45° cross laser, both cameras, passes along X and along Y. The
known heights make this the accuracy test for the processing chain. It is also the
blocks case of the benchmark.

## Files

| File | What |
|---|---|
| `objects.json` | Hand-made: each top's name, nominal size (inches), box in the camera-A map frame, height band, and what it sits on |
| `results/heights.json` | Measured heights vs nominal, from the current pipeline defaults |
| `results/metrics.json` | The benchmark's scores for the same run (blocks and sheet) |
| `out/` | Ignored. `bench/` (points, grids) and `view3d/` (the viewer build) |

## Rebuild (on Buzzkill)

Captures: `data/2026-11-05/blocks1_1.{1,2}` (Y passes), `blocksX_1.{1,2}` (X passes),
`photos1`, `survey3`. Calibration: `config/calib/2026-11-05-cross45-deep.json`.

```sh
.venv/bin/python -m carvera_scan.process.bench --out projects/blocks-123/out/bench --save
.venv/bin/python -m carvera_scan.view.view3d projects/blocks-123/out/bench/blocks_points.npz \
    --calib config/calib/2026-11-05-cross45-deep.json \
    --photos data/2026-11-05/photos1 data/2026-11-05/survey3 \
    --heights projects/blocks-123/out/bench/heights.json --objects projects/blocks-123/objects.json \
    --crop -295 25 -183 70 --out projects/blocks-123/out/view3d
```

To look at it: `python3 -m http.server 8000 -d projects/blocks-123/out/view3d`, then
open <http://localhost:8000/>. The page is `projects/viewer/index.html`, which `view3d`
copies into each build.

## Results (pipeline defaults, 2026-11-05)

From `docs/package-plan.md` §6a: blocks 3" 76.14, 2" 50.71, 1" 25.26 mm; every bar
within 0.30 mm of its label; mean height error 0.13 mm, max 0.30 mm.
