# Viewer

The three.js page that shows a scan: the height-map surface with the photos draped on
it, and the point cloud. It loads `meta.json`, `surface.bin`, `surface_gray.bin`,
`cloud.bin` and `cloud_gray.bin` from its own folder.

`carvera_scan.view.view3d` writes those files and copies this `index.html` next to
them, so every build is a self-contained folder. Serve it over HTTP (the page fetches
its data):

```sh
python3 -m http.server 8000 -d projects/blocks-123/out/view3d   # then http://localhost:8000/
```

Edit the page here, not in a build. Rebuild, or copy it into an existing build, to
see the change.
