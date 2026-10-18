"""Port of src/ArucoDetector.cpp and src/MotionDetector.cpp (CakeAI-V1)."""
import math

import cv2
import numpy as np


class ArucoDetector:
    def __init__(self, fast_mode=True):
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters()
        self.set_parameters(fast_mode)
        self.marker_ids = []
        self.marker_corners = []

    def set_parameters(self, fast_mode=True):
        p = self.params
        if fast_mode:
            p.adaptiveThreshWinSizeMin = 5
            p.adaptiveThreshWinSizeMax = 21
            p.adaptiveThreshConstant = 7
            p.minMarkerPerimeterRate = 0.05
            p.maxMarkerPerimeterRate = 0.8
            p.polygonalApproxAccuracyRate = 0.05
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
            p.errorCorrectionRate = 0.3
            p.minDistanceToBorder = 1
            p.minOtsuStdDev = 3.0
            p.perspectiveRemovePixelPerCell = 6
        else:
            p.adaptiveThreshWinSizeMin = 3
            p.adaptiveThreshWinSizeMax = 23
            p.adaptiveThreshConstant = 7
            p.minMarkerPerimeterRate = 0.03
            p.maxMarkerPerimeterRate = 4.0
            p.polygonalApproxAccuracyRate = 0.03
            p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            p.perspectiveRemovePixelPerCell = 4
        self._rebuild()

    def set_perimeter_rates(self, vmin, vmax):
        self.params.minMarkerPerimeterRate = vmin
        self.params.maxMarkerPerimeterRate = vmax
        self._rebuild()

    def _rebuild(self):
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.params)

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        self.marker_ids, self.marker_corners = [], []
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                if i in (0, 1):                      # only IDs 0 and 1
                    self.marker_ids.append(int(i))
                    self.marker_corners.append(c.reshape(4, 2))

    def has_markers(self):
        return bool(self.marker_ids)

    def marker_center(self, idx=0):
        return self.marker_corners[idx].mean(axis=0)

    def marker_rotation(self, idx=0):
        tl, tr = self.marker_corners[idx][0], self.marker_corners[idx][1]
        a = math.degrees(math.atan2(tr[1] - tl[1], tr[0] - tl[0])) + 90.0
        a = math.fmod(a, 360.0)
        return a + 360.0 if a < 0 else a

    def draw(self, frame):
        if self.marker_ids:
            cv2.aruco.drawDetectedMarkers(
                frame, [c.reshape(1, 4, 2) for c in self.marker_corners],
                np.array(self.marker_ids).reshape(-1, 1))


class MotionDetector:
    def __init__(self):
        self.prev_gray = None
        self.motion_threshold = 25
        self.min_w = self.min_h = 20
        self.morph_w = self.morph_h = 5
        self.dilation_iter = 2
        self.erosion_iter = 1
        self.blur_w = self.blur_h = 5
        self.blur_sigma = 1.5
        self.last_mask = None   # for visualisation

    def apply_config(self, cfg):
        self.motion_threshold = int(cfg.get("MotionThreshold", 25))
        self.morph_w = int(cfg.get("MorphKernelWidth", 5))
        self.morph_h = int(cfg.get("MorphKernelHeight", 5))
        self.dilation_iter = int(cfg.get("DilationIterations", 2))
        self.erosion_iter = int(cfg.get("ErosionIterations", 1))
        self.blur_w = int(cfg.get("BlurKernelWidth", 5))
        self.blur_h = int(cfg.get("BlurKernelHeight", 5))
        self.blur_sigma = float(cfg.get("BlurSigma", 1.5))

    def reset(self):
        self.prev_gray = None

    def detect(self, frame, excl_center, excl_radius=150):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (self.blur_w, self.blur_h), self.blur_sigma)
        if self.prev_gray is None or self.prev_gray.shape != gray.shape:
            self.prev_gray = gray
            return (-1, -1)
        diff = cv2.absdiff(self.prev_gray, gray)
        _, th = cv2.threshold(diff, self.motion_threshold, 255, cv2.THRESH_BINARY)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (self.morph_w, self.morph_h))
        th = cv2.dilate(th, k, iterations=self.dilation_iter)
        th = cv2.erode(th, k, iterations=self.erosion_iter)
        if excl_center[0] >= 0 and excl_center[1] >= 0:
            cv2.circle(th, (int(excl_center[0]), int(excl_center[1])), int(excl_radius), 0, -1)
        contours, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = np.zeros_like(th)
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            if w >= self.min_w and h >= self.min_h:
                cv2.drawContours(big, [c], -1, 255, -1)
        self.last_mask = big
        self.prev_gray = gray
        ys, xs = np.nonzero(big)
        if len(xs) == 0:
            return (-1, -1)
        return (int(xs.mean()), int(ys.mean()))

    @staticmethod
    def offset_exclusion_center(marker_center, rotation_deg, offset_px=20):
        r = math.radians(rotation_deg)
        return (marker_center[0] + int(offset_px * math.sin(r)),
                marker_center[1] + int(-offset_px * math.cos(r)))
