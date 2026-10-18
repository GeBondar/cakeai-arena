"""Build the README media: hero shot, robot cards, vision-pipeline figure,
GIF highlights and the highlight reel.

    python tools/make_media.py                  # stills only
    python tools/make_media.py --fights out/tournament   # + GIFs / videos from a tournament run
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys

import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from arena import scene  # noqa: E402
from arena.match import Match  # noqa: E402
from arena.pilots import ROSTER  # noqa: E402
from arena.video import ffmpeg_exe, font  # noqa: E402

MEDIA = os.path.join(ROOT, "media")
ROBOT_INFO = {
    "cake": ("CakeAI", "invertible drum spinner · the autonomous one"),
    "vspin": ("RAZOR", "vertical disc spinner · 10 000 rpm"),
    "hspin": ("TORNADO", "horizontal bar spinner · 6 000 rpm"),
    "wedge": ("DOORSTOP", "wedge pusher"),
    "flipper": ("CATAPULT", "electric flipper · 9 N·m"),
}


def _render(entries, cam_kw, w, h, settle=0.4, flipper_up=None):
    m = mujoco.MjModel.from_xml_string(scene.build_mjcf(entries, ai_slot="r0"))
    d = mujoco.MjData(m)
    if flipper_up is not None:
        a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{flipper_up}_weapon")
    for _ in range(int(settle / m.opt.timestep)):           # let the robots settle on their wheels
        if flipper_up is not None:
            d.ctrl[a] = 0.9
        mujoco.mj_step(m, d)
    r = mujoco.Renderer(m, h, w)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    for k, v in cam_kw.items():
        if k == "lookat":
            cam.lookat[:] = v
        else:
            setattr(cam, k, v)
    r.update_scene(d, camera=cam)
    img = r.render()
    r.close()
    return img


def hero():
    entries = [("r0", "cake", (0.0, 0.08, math.radians(-90), False), None),
               ("r1", "vspin", (-0.42, 0.12, math.radians(-60), False), None),
               ("r2", "hspin", (0.45, 0.16, math.radians(-120), False), None),
               ("r3", "wedge", (-0.24, -0.22, math.radians(-80), False), None),
               ("r4", "flipper", (0.25, -0.22, math.radians(-100), False), None)]
    img = _render(entries, dict(lookat=(0, -0.02, 0.02), distance=1.2, elevation=-21, azimuth=90), 1600, 800, flipper_up="r4")
    pil = Image.fromarray(img)
    dr = ImageDraw.Draw(pil, "RGBA")
    dr.rectangle([0, 0, 1600, 120], fill=(0, 0, 0, 140))
    dr.text((40, 20), "CakeAI Arena", font=font(54), fill=(255, 220, 120))
    dr.text((44, 84), "autonomous 1.5 kg combat robots · one overhead camera · MuJoCo", font=font(24), fill=(235, 235, 235))
    pil.save(os.path.join(MEDIA, "hero.png"))


def robot_cards():
    os.makedirs(os.path.join(MEDIA, "robots"), exist_ok=True)
    tiles = []
    for k, (name, blurb) in ROBOT_INFO.items():
        img = _render([("r0", k, (0, 0, math.radians(-50), False), None)],
                      dict(lookat=(0.015, 0.0, 0.025), distance=0.42, elevation=-22, azimuth=100), 800, 600,
                      flipper_up="r0" if k == "flipper" else None)
        pil = Image.fromarray(img)
        dr = ImageDraw.Draw(pil, "RGBA")
        dr.rectangle([0, 520, 800, 600], fill=(0, 0, 0, 150))
        dr.text((24, 528), name, font=font(34), fill=(255, 220, 120))
        dr.text((26, 568), blurb + " · 1.500 kg", font=font(20), fill=(235, 235, 235))
        pil.save(os.path.join(MEDIA, "robots", f"{k}.png"))
        tiles.append(np.asarray(pil.resize((400, 300))))
    # 6th tile: the arena as CakeAI's single overhead camera sees it
    m = mujoco.MjModel.from_xml_string(scene.build_mjcf([("r0", "cake", (-0.4, -0.3, 0.6, False), None),
                                                         ("r1", "hspin", (0.45, 0.35, -2.4, False), None)]))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, scene.CAM_HPX, scene.CAM_W)
    r.update_scene(d, camera="overhead")
    top = Image.fromarray(r.render()[70:945, 193:1085]).resize((400, 300))
    r.close()
    dr = ImageDraw.Draw(top, "RGBA")
    dr.rectangle([0, 260, 400, 300], fill=(0, 0, 0, 160))
    dr.text((12, 264), "THE ARENA", font=font(17), fill=(255, 220, 120))
    dr.text((13, 284), "2x2 m · one overhead camera", font=font(10), fill=(235, 235, 235))
    row1 = np.hstack(tiles[:3])
    row2 = np.hstack(tiles[3:] + [np.asarray(top)])
    Image.fromarray(np.vstack([row1, row2])).save(os.path.join(MEDIA, "robots", "gallery.png"))


def vision_figure():
    """Raw overhead frame | CakeAI's view with its overlay — what the robot actually 'sees'."""
    mt = Match("wedge", secs=4, seed=7)
    while mt.t < 0.4 or not (mt.ai.debug.get("live_motion") and
                             math.dist(mt.pose("r0")[:2], mt.pose("r1")[:2]) > 0.55):   # approaching, not yet touching
        mt.step()
    raw = mt.ai.debug["frame"].copy()
    ann = mt.ai.draw_overlay(raw.copy())
    mt.close()
    h = 520
    a = cv2.resize(raw, (int(raw.shape[1] * h / raw.shape[0]), h))
    b = cv2.resize(ann, (int(ann.shape[1] * h / ann.shape[0]), h))
    gap = np.full((h, 16, 3), 18, np.uint8)
    img = Image.fromarray(cv2.cvtColor(np.hstack([a, gap, b]), cv2.COLOR_BGR2RGB))
    dr = ImageDraw.Draw(img, "RGBA")
    for x, t in ((0, "1 · overhead camera (cropped to the arena)"), (a.shape[1] + 16, "2 · CakeAI: pose · target · exclusion zone")):
        dr.rectangle([x, h - 34, x + a.shape[1], h], fill=(0, 0, 0, 170))
        dr.text((x + 10, h - 29), t, font=font(18), fill=(255, 255, 255))
    img.save(os.path.join(MEDIA, "vision_pipeline.png"))


def key_moment(res):
    ev = res.get("events", [])
    for typ in ("ko", "flip"):
        hits = [e for e in ev if e["type"] == typ]
        if hits:
            return hits[0]["t"] if typ == "flip" else max(0.0, hits[0]["t"] - 1.5)
    hits = [e for e in ev if e["type"] == "hit"]
    return max(hits, key=lambda e: e["value"])["t"] if hits else res["t"] / 2


def gifs_and_videos(fdir):
    fdir = os.path.abspath(fdir)
    exe = ffmpeg_exe()
    if not exe:
        print("ffmpeg not found — skipping GIFs/videos")
        return
    with open(os.path.join(fdir, "results.json"), encoding="utf-8") as f:
        results = json.load(f)
    gdir, vdir = os.path.join(MEDIA, "gifs"), os.path.join(MEDIA, "videos")
    os.makedirs(gdir, exist_ok=True)
    os.makedirs(vdir, exist_ok=True)
    clips = []
    for r in results:
        src = os.path.join(fdir, r["video"])
        tag = os.path.splitext(os.path.basename(src))[0].replace("fight_", "")
        shutil.copy(src, os.path.join(vdir, os.path.basename(src)))
        t0 = max(0.0, r["video_offset_s"] + key_moment(r) - 3.0)
        dur = 6.5
        gif = os.path.join(gdir, f"{tag}.gif")
        vf = "fps=12,scale=840:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=4"
        subprocess.run([exe, "-y", "-loglevel", "error", "-ss", f"{t0:.2f}", "-t", str(dur), "-i", src, "-vf", vf, gif], check=True)
        clip = os.path.join(fdir, f"clip_{tag}.mp4")
        subprocess.run([exe, "-y", "-loglevel", "error", "-ss", f"{t0:.2f}", "-t", str(dur), "-i", src, "-c:v", "libx264",
                        "-crf", "22", "-pix_fmt", "yuv420p", "-an", clip], check=True)
        clips.append(clip)
        print("gif", gif, f"{os.path.getsize(gif) / 1e6:.1f} MB")
    lst = os.path.join(fdir, "reel.txt")
    with open(lst, "w", encoding="utf-8") as f:
        f.writelines(f"file '{c.replace(os.sep, '/')}'\n" for c in clips)
    reel = os.path.join(vdir, "highlights.mp4")
    subprocess.run([exe, "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", lst, "-c", "copy",
                    "-movflags", "+faststart", reel], check=True)
    os.remove(lst)
    hero_src = max(results, key=lambda r: (r["opp"]["flips"] + r["cake"]["flips"], r["cake"]["energy_J"]))
    tag = os.path.splitext(os.path.basename(hero_src["video"]))[0].replace("fight_", "")
    shutil.copy(os.path.join(gdir, f"{tag}.gif"), os.path.join(MEDIA, "hero.gif"))
    shutil.copy(os.path.join(fdir, "results.md"), os.path.join(MEDIA, "results.md"))
    shutil.copy(os.path.join(fdir, "results.json"), os.path.join(MEDIA, "results.json"))
    print("reel", reel)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fights", help="tournament output folder with results.json + videos")
    ap.add_argument("--skip-stills", action="store_true")
    a = ap.parse_args()
    os.makedirs(MEDIA, exist_ok=True)
    if not a.skip_stills:
        hero(); robot_cards(); vision_figure()
        print("stills ->", MEDIA)
    if a.fights:
        gifs_and_videos(a.fights)


if __name__ == "__main__":
    main()
