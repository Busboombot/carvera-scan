# kinsey: capture

kinsey drives the Carvera (through cmux) and owns the two cameras. Host record:
`busboom-home-network/hosts/kinsey/README.md`.

| File | What |
|---|---|
| `cmux.py` | Shares the Carvera's single TCP connection on port 2222. Runs on the system `python3`, no dependencies |
| `systemd/` | `cmux.service`, `x11vnc.service` |
| `udev/` | WHB04B pendant receiver rule |
| `deploy.sh` | Copies the package and cmux to `~/carvera-scan`, installs the units and the rule, restarts cmux. `-n` previews the copy |

Services and the capture commands: `docs/hardware.md`.
