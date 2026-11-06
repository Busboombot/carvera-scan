#!/bin/sh
# Install on kinsey (~/carvera-scan): the carvera_scan package (the capture scripts run
# from it), cmux, and the services.
#   src/hosts/kinsey/deploy.sh        copy + install + restart cmux
#   src/hosts/kinsey/deploy.sh -n     show what would be copied; change nothing
# On kinsey: cd ~/carvera-scan && .venv/bin/python -m carvera_scan.acquire.line_scan ...
set -e
HOST=${HOST:-kinsey}
HERE=$(cd "$(dirname "$0")" && pwd)
PKG="$HERE/../../carvera_scan"
if [ "$1" = "-n" ]; then
    rsync -ain --delete --exclude __pycache__ "$PKG/" "$HOST":carvera-scan/carvera_scan/
    rsync -ain "$HERE/cmux.py" "$HOST":carvera-scan/
    exit 0
fi
rsync -a --delete --exclude __pycache__ "$PKG/" "$HOST":carvera-scan/carvera_scan/
rsync -a "$HERE/cmux.py" "$HOST":carvera-scan/
scp -q "$HERE"/systemd/*.service "$HERE"/udev/*.rules "$HOST":/tmp/
ssh "$HOST" 'set -e
sudo install -m 644 /tmp/cmux.service /tmp/x11vnc.service /etc/systemd/system/
dpkg -s libhidapi-hidraw0 >/dev/null 2>&1 || sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -q libhidapi-hidraw0
sudo install -m 644 /tmp/99-carvera-pendant.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger --attr-match=idVendor=10ce
sudo systemctl daemon-reload
sudo systemctl enable -q cmux x11vnc
sudo systemctl restart cmux
sleep 2; systemctl is-active cmux'
# start x11vnc if it is down; never restart it here (that would drop VNC viewers)
ssh "$HOST" 'systemctl is-active -q x11vnc || sudo systemctl restart x11vnc; systemctl is-active x11vnc'
