# carvera-scan

3D-scan the Carvera's bed with a 45° cross laser and two Arducam cameras on the head.
kinsey drives the machine and captures, Buzzkill processes, the Mac edits and views.
Blocks come out within 0.3 mm of their labelled heights, and a sheet within 0.05 mm of
the probe.

GitHub: [Busboombot/carvera-scan](https://github.com/Busboombot/carvera-scan) (private).

Nothing in this repo moves the machine without Eric's go-ahead. Agents: read
[AGENTS.md](AGENTS.md) first.

## Layout

| Directory | What |
|---|---|
| [src/carvera_scan/](src/carvera_scan/) | The Python package. `machine`, `arducam`, `calib`; `acquire/` (runs on kinsey), `process/` (frames → points → grid; `pipeline` is the standard chain, `bench` its regression test), `measure/`, `view/` |
| [src/hosts/](src/hosts/) | Per-machine code: `kinsey/` (cmux, services, `deploy.sh`), `buzzkill/` (`push.sh`) |
| [projects/](projects/) | One folder per thing we scan or show: `blocks-123`, `sheet-flatness`, `cutebot-pro`, `plate-model`, and `viewer` (the three.js page) |
| [design/](design/) | Parts for the machine: `camera-mount/` (requirements), `mount-plate/` (OpenSCAD) |
| [config/](config/) | `calib/` calibration sets; dotconfig env layers |
| [docs/](docs/) | [hardware.md](docs/hardware.md) (machine, cameras, G-codes, services), [log.md](docs/log.md) (lab notebook), [package-plan.md](docs/package-plan.md), [structure.md](docs/structure.md) |
| [tests/](tests/) | `uv run pytest` |
| `data/` | Ignored. Raw captures, `data/<date>/<capture>/` |
| `scratch/` | Ignored. Working files |

## Setup

- **Mac:** `uv sync`. Add `--extra process` for Open3D if you want to run the full
  pipeline here.
- **Buzzkill:** `src/hosts/buzzkill/push.sh` copies the code; see
  [src/hosts/buzzkill/README.md](src/hosts/buzzkill/README.md).
- **kinsey:** `src/hosts/kinsey/deploy.sh` (`-n` to preview). It installs the package
  in `~/carvera-scan/carvera_scan/`, cmux, and the services. It restarts cmux.

## How to scan (cross laser)

On kinsey, in `~/carvera-scan`, with `.venv/bin/python -m`:

1. **Axis test**, once per camera mounting: `carvera_scan.acquire.axis_test cap/axis --step 20`.
2. **Z sweep** over bare bed (check the clearance with Eric):
   `carvera_scan.acquire.z_sweep cap/zcal --z -2 -80 3 --zmin -80 --cam 1.2 1.1`.
3. **Photos** for colour: `carvera_scan.acquire.survey cap/photos --x … --y … --cam 1.2 1.1`.
4. **Sweeps:** `carvera_scan.acquire.line_scan cap/scan --cam 1.2 --x … --y …`, then the
   same with `--cam 1.1`, and again with `--along x` for the other direction.

On Buzzkill, after copying the captures into `data/<date>/`:

5. **Calibration:** `carvera_scan.process.cross zcal data/…/zcal/<port>` per camera, then
   `carvera_scan.calib build … --out config/calib/<name>.json`.
6. **Process and view:** `carvera_scan.process.pipeline.run()` does points → merge →
   clean → grid. The benchmark and `carvera_scan.view.view3d` call it; see
   [projects/blocks-123/README.md](projects/blocks-123/README.md) for a full example.
