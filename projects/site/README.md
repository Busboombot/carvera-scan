# Project site (GitHub Pages)

One page with a card per project: a picture, the headline result, and links to its 3D
viewer, its pictures and its README. Built by GitHub Actions
(`.github/workflows/pages.yml`) and served by GitHub Pages.

| File | What |
|---|---|
| `site.json` | The projects: title, date, summary, result, and where the viewer and pictures come from |
| `index.html` | The page template (`{{CARDS}}` etc. filled in by `build.py`) |
| `build.py` | `stage` gathers viewers and pictures (Mac, needs OpenCV); `build` assembles the site (stdlib only, runs in Actions) |
| `publish.sh` | Stage, push the data branch, pin it, push main |
| `data.lock` | The `site-data` commit the site is built from |

## Where the data lives

Viewer data and pictures are generated from captures that are not in git, so Actions
can't rebuild them. `publish.sh` puts them on the **`site-data` branch** as a single
commit, replaced (force-pushed) on every publish, so the branch never accumulates
history. `main` holds no images; it pins the data commit in `data.lock`, and pushing
that change triggers the deploy. About 45 MB in all.

## Publish

```sh
projects/site/publish.sh -n     # stage into scratch/site-data only; then look:
python3 projects/site/build.py build --data scratch/site-data --out scratch/site
python3 -m http.server 8781 -d scratch/site
projects/site/publish.sh        # stage, push site-data, commit data.lock, push main
```

To add a project: give it an entry in `site.json` (a viewer folder and/or pictures, by
repo path or `buzzkill:<path>` under `~/carvera-scan`), then publish. A viewer is any
self-contained folder with an `index.html` that loads its data by relative path, as
`carvera_scan.view.view3d` builds them.

One-time setup on GitHub: **Settings → Pages → Build and deployment → Source: GitHub
Actions**.
