# Working in carvera-scan

Read `docs/structure.md` for the layout. The short version:

| Put it in | When |
|---|---|
| `scratch/<date>-<topic>/` | Anything you make while working: test scripts, plots, images, trial runs, notes. The default. Ignored by git; don't ask before writing here. |
| `src/carvera_scan/` | Reusable code: the package. |
| `src/hosts/<machine>/` | Code or setup that belongs to one machine (kinsey's cmux and services). |
| `projects/<name>/` | One thing we scan or show: README, its own scripts, hand-made inputs, small text `results/`. Generated output goes in its `out/` (ignored). |
| `design/` | OpenSCAD sources and requirements for parts on the machine. |
| `config/` | Calibration sets (`config/calib/`), dotconfig. |
| `docs/` | `hardware.md` (machine facts, G-codes), `log.md` (lab notebook), `package-plan.md`. |
| `data/<date>/<capture>/` | Raw captures only. Ignored. Never write products here. |

Rules:

1. No images in git. Plots and renders go in `scratch/` or a project's `out/`.
2. Don't add a file to a kept directory unless it is meant to be kept. Edit what is there.
3. Nothing in `src/` or `projects/` reads from `scratch/`.
4. `git status` should be short. If it lists many files, something is in the wrong place.
5. Benchmark before and after any processing change:
   `python -m carvera_scan.process.bench --out scratch/bench/<name>` (on Buzzkill).

## Machines

- **kinsey** captures: it drives the Carvera through cmux and owns the cameras.
  `src/hosts/kinsey/deploy.sh` installs the package and services there.
- **Buzzkill** processes: `~/carvera-scan` (eric@192.168.1.23), venv with Open3D.
  Push code with `src/hosts/buzzkill/push.sh`.
- **The Mac** edits and views.

## Machine safety

- Nothing moves the machine without Eric's go-ahead.
- Probe only with the firmware probe commands (`M6 T0`, `M466`, `M6 T-1`, via
  `carvera_scan.acquire.probe_z`). Never hand-drive `G38`.
- Stay 2 mm inside the travel limits (Y 0 trips HALT 21). Move Z below the scan height
  only after a clearance check against the latest height map.
- `M321` on a C1 drops the tool into the rack; use `M321.2` / `M322.2` for laser mode
  without motion. More in `docs/hardware.md`.
