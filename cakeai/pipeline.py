"""Port of CakeAI-V1's main loop (src/main.cpp) — without the GUI and the COM port.

Every overhead frame: arena crop -> ArUco (ID 0/1) -> 3 s marker memory ->
offset exclusion zone -> motion detector (5 s target memory) ->
RobotController.process(...) -> Arduino commands.
"""
import configparser
import math

import cv2

from .robot_controller import RobotController
from .vision import ArucoDetector, MotionDetector

MARKER_PERSISTENCE_MS = 3000
MOTION_PERSISTENCE_MS = 5000


def load_config(path):
    cp = configparser.ConfigParser()
    cp.optionxform = str
    cp.read(path, encoding="utf-8")
    return dict(cp["PanelValues"]) if "PanelValues" in cp else {}


class ArduinoLink:
    """Emulator of the src/ArduinoProgram.ino serial protocol.

    Accepts the same lines ArduinoController sends ("1:P:73") and returns
    normalised commands: left / right drive and weapon.
    DAC: 0..100 % -> 170..4000 (stick centre 50 % ~ 2085).
    """
    DAC_MIN, DAC_MAX = 170, 4000

    def __init__(self):
        self.percent = [50, 50, 0]
        self.dac = [self._dac(50), self._dac(50), self._dac(0)]
        self.log = []

    @classmethod
    def _dac(cls, p):
        p = max(0, min(100, p))
        return cls.DAC_MIN + (p * (cls.DAC_MAX - cls.DAC_MIN)) // 100   # Arduino map()

    def send_command(self, cmd):
        cmd = cmd.strip().upper()
        if len(cmd) < 4 or cmd[1] != ":":
            return
        sel, mode, val = cmd[0], cmd[2], int(cmd.split(":")[2] or 0)
        if mode != "P":
            return
        chans = [int(sel)] if sel in "123" else ([1, 2, 3] if sel == "A" else [])
        for c in chans:
            self.percent[c - 1] = max(0, min(100, val))
            self.dac[c - 1] = self._dac(val)

    def set_motors(self, p1, p2, p3):
        for ch, p in ((1, p1), (2, p2), (3, p3)):
            p = max(0, min(100, p))
            self.send_command(f"{ch}:P:{p}")

    def normalized(self):
        """(left, right) in -1..1 from the DAC values, weapon 0..1."""
        mid = self._dac(50)
        half = self.DAC_MAX - mid
        l = (self.dac[0] - mid) / half
        r = (self.dac[1] - mid) / half
        w = (self.dac[2] - self.DAC_MIN) / (self.DAC_MAX - self.DAC_MIN)
        return max(-1, min(1, l)), max(-1, min(1, r)), w


class CakeAIPipeline:
    def __init__(self, clock, config, verbose=False):
        self.clock = clock
        self.cfg = config
        self.link = ArduinoLink()
        self.rc = RobotController(clock, self.link, 0.1, 0.1, 0.1, verbose=verbose)
        self.rc.apply_config(config)
        self.aruco = ArucoDetector(True)
        self.aruco.set_perimeter_rates(float(config.get("MinMarkerPerimeterRate", 0.05)),
                                       float(config.get("MaxMarkerPerimeterRate", 8.0)))
        self.motion = MotionDetector()
        self.motion.apply_config(config)
        self.excl_radius = int(config.get("Robot1Exclusion", 150))
        self.offset_px = int(config.get("Robot1Offset", 20))
        self.arena_center = (int(config.get("ArenaX", 640)), int(config.get("ArenaY", 512)))
        self.crop = tuple(int(config.get(k, -1)) for k in
                          ("ArenaSizeX1", "ArenaSizeY1", "ArenaSizeX2", "ArenaSizeY2"))
        self.active = False
        self.saved_marker = None          # (x, y, rot, t)
        self.saved_motion = (-1, -1)
        self.last_motion_time = clock()
        self.debug = {}

    # START / STOP buttons of the control panel
    def start(self):
        self.rc.reset()
        self.rc.startCh3RampUp(100.0, 3000)
        self.active = True

    def stop(self):
        self.rc.reset()
        self.active = False
        self.rc.setChannels(50, 50, 0)
        self.rc.sendToArduino()
        self.saved_marker = None

    def crop_frame(self, frame):
        x1, y1, x2, y2 = self.crop
        if -1 not in self.crop and x2 <= frame.shape[1] and y2 <= frame.shape[0]:
            return frame[y1:y2, x1:x2].copy()
        return frame.copy()

    def step(self, frame):
        now = self.clock()
        f = self.crop_frame(frame)

        self.aruco.process_frame(f)
        has_marker, mc, mrot, mid = False, (-1, -1), 0.0, -1
        if self.aruco.has_markers():
            c = self.aruco.marker_center(0)
            mrot = self.aruco.marker_rotation(0)
            mid = self.aruco.marker_ids[0]
            mc = (int(c[0]), int(c[1]))
            has_marker = True
            self.saved_marker = (mc[0], mc[1], mrot, now)
        elif self.saved_marker is not None:
            if (now - self.saved_marker[3]) * 1000 < MARKER_PERSISTENCE_MS:
                mc, mrot, has_marker, mid = self.saved_marker[:2], self.saved_marker[2], True, -1
            else:
                self.saved_marker = None
        self.rc.markerDetected = has_marker

        excl = (-1, -1)
        if mc[0] >= 0:
            excl = MotionDetector.offset_exclusion_center(mc, mrot, self.offset_px)

        motion = (-1, -1)
        p = self.motion.detect(f, excl, self.excl_radius)
        if p[0] >= 0 and p[1] >= 0:
            motion = p
            self.saved_motion = p
            self.last_motion_time = now
        elif (now - self.last_motion_time) * 1000 < MOTION_PERSISTENCE_MS and self.saved_motion[0] >= 0:
            motion = self.saved_motion
        else:
            self.saved_motion = (-1, -1)

        if self.active:
            x1, y1 = self.crop[0], self.crop[1]
            cx, cy = self.arena_center
            if -1 not in self.crop:
                cx, cy = cx - x1, cy - y1
            self.rc.process(mrot, cx, cy, mc[0], mc[1], motion[0], motion[1], mid)

        self.debug = dict(frame=f, marker=mc, rot=mrot, marker_id=mid, live_marker=self.aruco.has_markers(),
                          excl=excl, motion=motion, live_motion=p[0] >= 0)
        return self.link.normalized()

    def draw_overlay(self, img):
        """Overlay in the style of CakeAI's 'Motion Detection with Exclusion' window."""
        d = self.debug
        self.aruco.draw(img)
        if d.get("excl", (-1, -1))[0] >= 0:
            cv2.circle(img, d["excl"], self.excl_radius, (0, 0, 255), 2)
            cv2.putText(img, "Exclusion Zone", (d["excl"][0] - 40, d["excl"][1] - self.excl_radius - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        if d.get("motion", (-1, -1))[0] >= 0:
            m = d["motion"]
            col = (0, 255, 0) if d["live_motion"] else (0, 160, 0)
            cv2.line(img, (m[0] - 20, m[1]), (m[0] + 20, m[1]), col, 3)
            cv2.line(img, (m[0], m[1] - 20), (m[0], m[1] + 20), col, 3)
            cv2.circle(img, m, 15, col, 2)
        mc = d.get("marker", (-1, -1))
        if mc[0] >= 0:
            r = math.radians(d["rot"] - 90)
            tip = (int(mc[0] + 60 * math.cos(r)), int(mc[1] + 60 * math.sin(r)))
            cv2.arrowedLine(img, mc, tip, (255, 255, 0), 2, tipLength=0.3)
        cx, cy = self.arena_center
        if -1 not in self.crop:
            cx, cy = cx - self.crop[0], cy - self.crop[1]
        cv2.drawMarker(img, (cx, cy), (0, 255, 255), cv2.MARKER_TILTED_CROSS, 16, 2)
        cv2.circle(img, (cx, cy), self.rc.centerTriggerThreshold, (0, 200, 255), 1)
        st = {0: "0 CENTER", 1: "1 ATTACK", 2: "2 SEARCH"}[self.rc.getState()]
        lines = [f"State: {st}{'  ROTATE' if self.rc.rotatingInPlace else ''}{'  REV' if self.rc.reverse else ''}",
                 f"CH1: {int(self.rc.ch1)} | CH2: {int(self.rc.ch2)} | CH3: {int(self.rc.ch3)}",
                 f"Offset: {int(self.rc.currentOffset)} deg  attacks: {self.rc.attack_count}"]
        for i, t in enumerate(lines):
            cv2.putText(img, t, (10, 25 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(img, t, (10, 25 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        return img
