"""Arducam OV9281 (mono, global shutter) on the head.

    cam = Arducam()            # 1280x800 MJPG, 120 fps; the camera found by its by-id name
    cam = Arducam(port="2.3")  # the camera on that USB port (two cameras share one serial)
    cam.exposure(50)           # manual, 100 us units; None = auto
    img = cam.grab()           # grayscale frame taken after the call

    cam = Arducam(raw=True)    # undecoded frames, for streaming to disk
    jpeg, t = cam.read_raw()   # JPEG bytes, buffer timestamp (time.monotonic() clock)
"""
import glob
import subprocess
import time

import cv2
import numpy as np

BY_ID = "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_*-video-index0"
BY_PORT = "/dev/v4l/by-path/pci-*-usb-0:%s:1.0-video-index0"


def device(port=None):
    """The camera's video device. Both Arducams report serial UC762, so by-id
    names only one of them: with two connected, pick by USB port (e.g. "2.3",
    as in /dev/v4l/by-path)."""
    found = glob.glob(BY_PORT % port if port else BY_ID)
    if not found:
        raise RuntimeError("Arducam OV9281 not connected" + (" on port " + port if port else ""))
    return found[0]


class Arducam:
    def __init__(self, width=1280, height=800, fps=120, raw=False, port=None):
        self.dev, self.raw = device(port), raw
        self.cap = cv2.VideoCapture(self.dev, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        if raw:
            self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        else:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError("could not open " + self.dev)
        self.grab(settle=0.5)

    def exposure(self, units):
        """Manual exposure in 100 us units, or None for auto (aperture priority)."""
        if units is None:
            ctl = ["auto_exposure=3"]
        else:
            ctl = ["auto_exposure=1", "exposure_time_absolute=%d" % units]
        subprocess.run(["v4l2-ctl", "-d", self.dev, "-c", ",".join(ctl)], check=True)

    def grab(self, settle=0.15):
        """A frame whose exposure started after this call: drain for `settle` s first."""
        end = time.time() + settle
        while time.time() < end:
            self.cap.grab()
        ok, img = self.cap.read()
        if not ok:
            raise RuntimeError("no frame from " + self.dev)
        if self.raw:
            return cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img

    def read_raw(self):
        """Next frame as (JPEG bytes, V4L2 buffer timestamp in s). Needs raw=True.

        The timestamp is on CLOCK_MONOTONIC, the same clock as time.monotonic().
        It is taken when the frame reaches the host, a fixed delay after the
        exposure; the analysis calibrates that delay.
        """
        ok, buf = self.cap.read()
        if not ok:
            raise RuntimeError("no frame from " + self.dev)
        return np.asarray(buf).tobytes(), self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0

    def close(self):
        self.cap.release()
