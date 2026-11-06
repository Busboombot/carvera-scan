#!/bin/sh
# Copy the working tree (code, config, projects; not data/, scratch/ or out/) from the Mac
# to Buzzkill ~/carvera-scan. Buzzkill's data/, scratch/, projects/*/out/ and .venv are
# left alone.
#   src/hosts/buzzkill/push.sh        copy
#   src/hosts/buzzkill/push.sh -n     show what would change
set -e
HOST=${HOST:-eric@192.168.1.23}
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
N=""; [ "$1" = "-n" ] && N="-n -i"
rsync -a $N --delete --exclude .git --exclude .venv --exclude /data --exclude /scratch \
    --exclude 'projects/*/out' --exclude __pycache__ --exclude .DS_Store \
    "$ROOT/" "$HOST":carvera-scan/
