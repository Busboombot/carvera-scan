"""The project site: a page linking each project's 3D viewer, pictures and README.

    python projects/site/build.py stage --out scratch/site-data      # Mac: gather viewers + pictures (needs OpenCV)
    python projects/site/build.py build --data site-data --out _site  # anywhere: assemble the site (stdlib only)

`site.json` lists the projects. Each viewer and picture names its source: a path in this
repo (data/ or a project's out/), or `buzzkill:<path>` under Buzzkill's ~/carvera-scan.
`stage` copies the viewers, turns the pictures into JPEGs (full size, up to 2400 px, and a
720 px thumbnail) and writes STAGE/<project>/... . `build` copies a staged data tree into
the site and writes index.html from `index.html` here. publish.sh runs stage, puts the
data on the site-data branch and pins it in data.lock; GitHub Actions runs build.
"""
import argparse
import fnmatch
import html
import json
import os
import shutil
import subprocess
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
BUZZKILL = os.environ.get("BUZZKILL", "eric@192.168.1.23")


def fetch(src, dst):
    """Copy a source (file or directory, local or buzzkill:) to dst."""
    if src.startswith("buzzkill:"):
        path = src.split(":", 1)[1]
        subprocess.run(["rsync", "-a", "%s:carvera-scan/%s%s" % (BUZZKILL, path, "/" if dst.endswith("/") else ""), dst],
                       check=True)
    else:
        s = os.path.join(ROOT, src)
        if os.path.isdir(s):
            shutil.copytree(s, dst, dirs_exist_ok=True)
        else:
            shutil.copy(s, dst)


def keep(name, v):
    if "only" in v and name not in v["only"]:
        return False
    return not any(fnmatch.fnmatch(name, pat) for pat in v.get("exclude", []))


def stage(a):
    import cv2
    site = json.load(open(os.path.join(HERE, "site.json")))
    for p in site["projects"]:
        out = os.path.join(a.out, p["name"])
        shutil.rmtree(out, ignore_errors=True)
        os.makedirs(out)
        if "viewer" in p:
            v = p["viewer"]
            tmp = tempfile.mkdtemp()
            fetch(v["from"], tmp + "/")
            os.makedirs(os.path.join(out, "viewer"))
            for name in os.listdir(tmp):
                if os.path.isfile(os.path.join(tmp, name)) and keep(name, v):
                    shutil.copy(os.path.join(tmp, name), os.path.join(out, "viewer", name))
                elif os.path.isdir(os.path.join(tmp, name)) and keep(name, v):
                    shutil.copytree(os.path.join(tmp, name), os.path.join(out, "viewer", name))
            shutil.rmtree(tmp)
        for i, im in enumerate(p.get("images", [])):
            tmp = tempfile.mkdtemp()
            src = os.path.join(tmp, os.path.basename(im["from"].split(":", 1)[-1]))
            fetch(im["from"], src)
            img = cv2.imread(src)
            shutil.rmtree(tmp)
            if img is None:
                raise SystemExit("could not read " + im["from"])
            for suffix, size in (("", 2400), ("-thumb", 720)):
                k = min(1.0, size / max(img.shape[:2]))
                small = cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_AREA) if k < 1 else img
                cv2.imwrite(os.path.join(out, "img%d%s.jpg" % (i, suffix)), small, [cv2.IMWRITE_JPEG_QUALITY, 85])
        n = sum(len(fs) for _, _, fs in os.walk(out))
        size = sum(os.path.getsize(os.path.join(d, f)) for d, _, fs in os.walk(out) for f in fs)
        print("%-16s %3d files %6.1f MB" % (p["name"], n, size / 1e6))


def card(p, repo, have):
    e = html.escape
    base = "p/%s/" % p["name"]
    imgs = [(i, im) for i, im in enumerate(p.get("images", [])) if os.path.exists(os.path.join(have, "img%d.jpg" % i))]
    viewer = "viewer" in p and os.path.isdir(os.path.join(have, "viewer"))
    thumb = ""
    if imgs:
        href = base + ("viewer/" if viewer else "img%d.jpg" % imgs[0][0])
        thumb = '<a class="thumb" href="%s"><img src="%simg%d-thumb.jpg" alt="%s" loading="lazy"></a>' % (
            href, base, imgs[0][0], e(imgs[0][1]["caption"]))
    links = []
    if viewer:
        links.append('<a class="btn primary" href="%sviewer/">Open 3D viewer</a>' % base)
    links += ['<a class="btn" href="%simg%d.jpg" title="%s">%s</a>' % (base, i, e(im["caption"]), e(im.get("label", im["caption"].split(":")[0].split(",")[0])))
              for i, im in imgs]
    links.append('<a class="btn" href="%s/tree/main/projects/%s">README</a>' % (repo, p["name"]))
    return """<article class="card">%s
  <div class="body">
    <div class="meta"><time>%s</time></div>
    <h2>%s</h2>
    <p>%s</p>
    <p class="result">%s</p>
    <nav>%s</nav>
  </div>
</article>""" % (thumb, e(p["date"]), e(p["title"]), e(p["summary"]), e(p.get("result", "")), "".join(links))


def build(a):
    site = json.load(open(os.path.join(HERE, "site.json")))
    shutil.rmtree(a.out, ignore_errors=True)
    os.makedirs(a.out)
    cards = []
    for p in site["projects"]:
        have = os.path.join(a.data, p["name"])
        if os.path.isdir(have):
            shutil.copytree(have, os.path.join(a.out, "p", p["name"]))
        cards.append(card(p, site["repo"], os.path.join(a.out, "p", p["name"])))
    page = open(os.path.join(HERE, "index.html")).read()
    for k, v in {"TITLE": html.escape(site["title"]), "TAGLINE": html.escape(site["tagline"]),
                 "REPO": site["repo"], "CARDS": "\n".join(cards)}.items():
        page = page.replace("{{%s}}" % k, v)
    open(os.path.join(a.out, "index.html"), "w").write(page)
    open(os.path.join(a.out, ".nojekyll"), "w").close()
    print("site: %d projects -> %s" % (len(cards), a.out))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("stage")
    s.add_argument("--out", required=True)
    b = sub.add_parser("build")
    b.add_argument("--data", required=True, help="staged data (the site-data branch)")
    b.add_argument("--out", required=True)
    a = ap.parse_args()
    {"stage": stage, "build": build}[a.cmd](a)


if __name__ == "__main__":
    main()
