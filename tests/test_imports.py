"""Every module in the package imports (catches broken imports after a move)."""
import importlib
import pkgutil

import pytest

import carvera_scan

MODULES = [m.name for m in pkgutil.walk_packages(carvera_scan.__path__, "carvera_scan.")]


def test_found_modules():
    assert "carvera_scan.process.pipeline" in MODULES
    assert "carvera_scan.acquire.line_scan" in MODULES


@pytest.mark.parametrize("name", MODULES)
def test_import(name):
    importlib.import_module(name)


def test_acquire_does_not_need_processing():
    """kinsey has only numpy + OpenCV: the capture side must not pull in process/measure/view."""
    import sys
    import subprocess
    code = ("import sys, carvera_scan.machine, carvera_scan.arducam\n"
            "import " + ", ".join(n for n in MODULES if n.startswith("carvera_scan.acquire.")) + "\n"
            "bad = [m for m in sys.modules if m.startswith(('carvera_scan.process', 'carvera_scan.measure', "
            "'carvera_scan.view', 'scipy', 'matplotlib', 'open3d'))]\n"
            "assert not bad, bad\n")
    subprocess.run([sys.executable, "-c", code], check=True)
