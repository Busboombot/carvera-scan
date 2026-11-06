# Buzzkill: processing

Buzzkill (eric@192.168.1.23, Robot Garage 192.168.5.23) runs the processing:
`~/carvera-scan`, a uv venv on Python 3.14 with Open3D 0.20. Its host record is
`busboom-home-network/hosts/buzzkill/README.md`.

- **Code:** pushed from the Mac with `src/hosts/buzzkill/push.sh` (`-n` to preview).
  Buzzkill's `data/`, `scratch/`, `projects/*/out/` and `.venv` are not touched.
- **Install** (once, and after changing `pyproject.toml`):
  `cd ~/carvera-scan && uv pip install --python .venv/bin/python --no-deps -e .`
  This makes `carvera_scan` importable from the checkout without changing the venv's
  pinned packages.
- **Captures:** `data/<date>/<capture>/`, copied from kinsey (`~/carvera-scan/cap/`) or
  the Mac with rsync.
- **Benchmark:** `.venv/bin/python -m carvera_scan.process.bench --out scratch/bench/<name>`.
  The extraction cache is `scratch/bench/cache/`.
