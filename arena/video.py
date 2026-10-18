"""Broadcast-style video: CakeAI's own window on the left, TV camera + HUD on the right."""
import math
import os
import platform
import shutil
import subprocess

import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT_CANDIDATES = {
    "Windows": ["C:/Windows/Fonts/segoeuib.ttf", "C:/Windows/Fonts/arialbd.ttf", "C:/Windows/Fonts/arial.ttf"],
    "Darwin": ["/System/Library/Fonts/Supplemental/Arial Bold.ttf", "/System/Library/Fonts/Helvetica.ttc",
               "/Library/Fonts/Arial.ttf"],
    "Linux": ["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"],
}
CAKE_RGB = (240, 200, 140)
OPP_RGB = (120, 180, 255)


def font(size):
    """A bold sans font available on this OS; Pillow's built-in font as a last resort."""
    for p in FONT_CANDIDATES.get(platform.system(), []) + sum(FONT_CANDIDATES.values(), []):
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except OSError:
                pass
    try:
        return ImageFont.load_default(size=size)
    except TypeError:                                # Pillow < 10.1
        return ImageFont.load_default()


def ffmpeg_exe():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return shutil.which("ffmpeg")


class VideoSink:
    """H.264 via ffmpeg (imageio-ffmpeg ships a binary) with an OpenCV fallback."""

    def __init__(self, path, w, h, fps):
        self.path, self.proc, self.cv = path, None, None
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        exe = ffmpeg_exe()
        if exe:
            self.proc = subprocess.Popen(
                [exe, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{w}x{h}",
                 "-r", str(fps), "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "21",
                 "-pix_fmt", "yuv420p", "-movflags", "+faststart", path], stdin=subprocess.PIPE)
        else:
            self.cv = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))

    def write(self, frame):
        if self.proc:
            self.proc.stdin.write(frame.tobytes())
        else:
            self.cv.write(frame)

    def close(self):
        if self.proc:
            self.proc.stdin.close()
            self.proc.wait()
        else:
            self.cv.release()


class Broadcast:
    """Composes frames for a Match. Use .frame() for live display, Recorder for files."""

    def __init__(self, match, h=720, w_ai=720, w_tv=960):
        self.mt, self.H, self.W_AI, self.W_TV = match, h, w_ai, w_tv
        self.W = w_ai + w_tv
        self.tv = mujoco.Renderer(match.m, h, w_tv)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.distance, self.cam.elevation, self.cam.azimuth = 2.3, -38.0, 90.0
        self.cam.lookat[:] = (0, 0, 0)
        s = h / 720
        self.f_big, self.f_mid, self.f_small, self.f_tiny = font(int(34 * s)), font(int(23 * s)), font(int(18 * s)), font(int(15 * s))

    def _tv(self):
        m = self.mt
        p0, p1 = m.pose("r0"), m.pose("r1")
        mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
        sep = math.hypot(p0[0] - p1[0], p0[1] - p1[1])
        a = 0.08
        self.cam.lookat[0] += a * (mx * 0.7 - self.cam.lookat[0])
        self.cam.lookat[1] += a * (my * 0.7 - self.cam.lookat[1])
        self.cam.distance += a * (max(1.6, min(2.8, 1.3 + sep)) - self.cam.distance)
        self.cam.azimuth = 90.0 + 12.0 * math.sin(m.t * 0.15)
        self.tv.update_scene(m.d, camera=self.cam)
        return self.tv.render()                      # RGB

    def _hud(self, rgb):
        m = self.mt
        img = Image.fromarray(rgb)
        dr = ImageDraw.Draw(img, "RGBA")
        W, H = self.W_TV, self.H
        dr.rectangle([0, 0, W, int(H * 0.13)], fill=(0, 0, 0, 160))
        for i, (s, col) in enumerate((("r0", CAKE_RGB), ("r1", OPP_RGB))):
            S = m.sides[s]
            x0 = 14 if i == 0 else W // 2 + 52
            bw = W // 2 - 80
            dr.text((x0, 6), S.name + ("" if i == 0 else f"  ·  {m.opp.blurb}"), font=self.f_mid if i == 0 else self.f_small, fill=col)
            y = int(H * 0.05)
            dr.rectangle([x0, y, x0 + bw, y + 16], outline=(255, 255, 255, 200))
            hpc = (80, 220, 90) if S.hp > 50 else (240, 200, 60) if S.hp > 25 else (240, 70, 60)
            dr.rectangle([x0 + 2, y + 2, x0 + 2 + int((bw - 4) * S.hp / 100), y + 14], fill=hpc)
            wk = m.wkind[s]
            wtxt = (f"{m.weapon_rpm(s):5.0f} rpm" if wk in ("drum", "vertical", "horizontal")
                    else "flipper" if wk == "flipper" else "no weapon")
            dr.text((x0, y + 21), f"HP {S.hp:3.0f}   hits {S.weapon_hits}   rams {S.rams}   flips {S.flips}   {wtxt}",
                    font=self.f_tiny, fill=(230, 230, 230))
        tl = max(0.0, m.secs - m.t)
        txt = f"{int(tl // 60)}:{int(tl % 60):02d}"
        dr.text(((W - dr.textlength(txt, font=self.f_big)) / 2, int(H * 0.035)), txt, font=self.f_big, fill=(255, 255, 255))
        recent = [e for e in m.events if m.t - e["t"] < 2.5][-3:]
        for k, e in enumerate(reversed(recent)):
            col = (255, 90, 70) if e["type"] in ("ko", "flip") else (255, 230, 80)
            dr.text((14, H - 38 - 28 * k), e["text"], font=self.f_mid, fill=col)
        dr.text((W - 14 - dr.textlength("MuJoCo · 1.5 kg class · 2x2 m arena", font=self.f_tiny), H - 24),
                "MuJoCo · 1.5 kg class · 2x2 m arena", font=self.f_tiny, fill=(200, 200, 200))
        return np.asarray(img)

    def _ai(self):
        m = self.mt
        f = m.ai.debug.get("frame")
        panel = np.zeros((self.H, self.W_AI, 3), np.uint8)
        if f is not None:
            view = m.ai.draw_overlay(f.copy())
            s = self.H / view.shape[0]
            view = cv2.resize(view, (int(view.shape[1] * s), self.H), interpolation=cv2.INTER_AREA)
            w = min(self.W_AI, view.shape[1])
            panel[:, :w] = view[:, :w]
        img = Image.fromarray(cv2.cvtColor(panel, cv2.COLOR_BGR2RGB))
        dr = ImageDraw.Draw(img, "RGBA")
        dr.rectangle([0, self.H - 30, self.W_AI, self.H], fill=(0, 0, 0, 170))
        dbg = m.ai.debug
        mk = (f"marker ID {dbg['marker_id']}" if dbg.get("live_marker") else
              "marker: 3 s memory" if dbg.get("marker", (-1, -1))[0] >= 0 else "marker lost")
        dr.text((8, self.H - 26), f"CakeAI vision · overhead cam 1280x1024 @ {m.fps} fps · {mk}", font=self.f_tiny, fill=(255, 255, 255))
        return np.asarray(img)

    def frame(self):
        """BGR composite frame."""
        rgb = np.hstack([self._ai(), self._hud(self._tv())])
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def card(self, lines, base=None):
        bg = base if base is not None else np.zeros((self.H, self.W, 3), np.uint8)
        img = Image.fromarray(cv2.cvtColor((bg * 0.3).astype(np.uint8), cv2.COLOR_BGR2RGB))
        dr = ImageDraw.Draw(img)
        y = self.H // 2 - 28 * len(lines)
        for i, ln in enumerate(lines):
            f = self.f_big if i == 0 else self.f_mid
            dr.text(((self.W - dr.textlength(ln, font=f)) / 2, y), ln, font=f, fill=(255, 220, 120) if i == 0 else (255, 255, 255))
            y += 54 if i == 0 else 38
        return cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)

    def close(self):
        self.tv.close()
