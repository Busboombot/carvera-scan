#!/bin/sh
# Publish the project site (run on the Mac, from anywhere in the repo):
#   1. stage the viewers and pictures named in site.json (from data/, out/ and Buzzkill)
#   2. put them on the site-data branch as one commit (force-pushed: the branch keeps no history)
#   3. pin that commit in data.lock, commit and push main -> GitHub Actions builds and deploys
#   projects/site/publish.sh        all three
#   projects/site/publish.sh -n     stage only (scratch/site-data); build locally to look:
#       python3 projects/site/build.py build --data scratch/site-data --out scratch/site && python3 -m http.server -d scratch/site
set -e
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
cd "$ROOT"
STAGE=scratch/site-data
.venv/bin/python projects/site/build.py stage --out "$STAGE"
[ "$1" = "-n" ] && exit 0

# one orphan commit of the staged tree, made with a private index (the working tree is untouched)
IDX=$(mktemp -u)
GIT_INDEX_FILE=$IDX git --work-tree="$STAGE" add -A -f .
TREE=$(GIT_INDEX_FILE=$IDX git write-tree)
rm -f "$IDX"
COMMIT=$(git commit-tree "$TREE" -m "site data $(date '+%Y-%m-%d %H:%M')")
git push -f origin "$COMMIT:refs/heads/site-data"

echo "$COMMIT" > projects/site/data.lock
git add projects/site/data.lock
git commit -q -m "site: publish data $(echo "$COMMIT" | cut -c1-7)" -- projects/site/data.lock
git push origin HEAD:main
echo "published $COMMIT; the pages workflow deploys it"
