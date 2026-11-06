# Aluminium sheet: flatness and probe check

A 1.5 mm aluminium sheet lying bare on the bed, scanned on 2026-11-05, and nine M466
probe touches on and around it. The probe touches check the laser heights; it is the
sheet case of the benchmark.

## Files

| File | What |
|---|---|
| `results/flat2.json` | Flatness of the sheet from the two-camera map (straight-line rig): bed and sheet planes, thickness, residuals |
| `results/flat3_*.json` | The same from the cross-laser scan, per laser arm and both |
| `results/probe1-compare.json` | Each probe spot: probe height, laser height, difference |

## Rebuild

Captures: `data/2026-11-05/plate2_2.{2,3}`, `plate3_1.{1,2}`; probe log
`data/2026-11-05/probe1/probe1.jsonl` (from `carvera_scan.acquire.probe_z touch`).

```sh
python -m carvera_scan.measure.flatness data/2026-11-05/plate2_merged.npz --out projects/sheet-flatness/out/flat2
python -m carvera_scan.measure.probe_compare data/2026-11-05/probe1/probe1.jsonl data/2026-11-05/flat2.npz \
    --offset -71.7 -68.1
```

The benchmark scores the cross-laser version: `python -m carvera_scan.process.bench --case sheet`.

## Results

Sheet vs probe: mean −0.05 mm, sd 0.03 mm (package-plan §6a).
