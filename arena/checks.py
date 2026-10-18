"""Sanity check: CakeAI's detector finds the rendered markers and the heading matches,
upright (ID 0) and upside down (ID 1)."""
import math

import cv2
import mujoco

from cakeai.pipeline import load_config
from cakeai.vision import ArucoDetector
from . import scene
from .match import CONFIG_PATH


def check_markers(verbose=True):
    cfg = load_config(CONFIG_PATH)
    crop = [int(cfg[k]) for k in ("ArenaSizeX1", "ArenaSizeY1", "ArenaSizeX2", "ArenaSizeY2")]
    det = ArucoDetector(True)
    det.set_perimeter_rates(float(cfg["MinMarkerPerimeterRate"]), float(cfg["MaxMarkerPerimeterRate"]))
    worst, ok = 0.0, True
    for flipped in (False, True):
        for (x, y, yaw) in [(0.0, 0.0, 0), (0.6, -0.5, 37), (-0.75, 0.7, -120), (0.8, 0.8, 200)]:
            m = mujoco.MjModel.from_xml_string(scene.build_mjcf([("r0", "cake", (x, y, math.radians(yaw), flipped), None)]))
            d = mujoco.MjData(m)
            mujoco.mj_forward(m, d)
            r = mujoco.Renderer(m, scene.CAM_HPX, scene.CAM_W)
            r.update_scene(d, camera="overhead")
            img = cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR)
            r.close()
            det.process_frame(img[crop[1]:crop[3], crop[0]:crop[2]])
            if not det.has_markers():
                ok = False
                if verbose:
                    print(f"  {'flipped' if flipped else 'upright'} ({x:+.2f},{y:+.2f},{yaw:+4d}°): marker NOT found")
                continue
            err = (det.marker_rotation(0) - 90.0 + yaw + 180) % 360 - 180   # image y points down
            worst = max(worst, abs(err))
            good = abs(err) < 3 and det.marker_ids[0] == (1 if flipped else 0)
            ok &= good
            if verbose:
                print(f"  {'flipped' if flipped else 'upright'} ({x:+.2f},{y:+.2f},{yaw:+4d}°): id={det.marker_ids[0]} "
                      f"heading error {err:+.1f}° {'OK' if good else 'FAIL'}")
    if verbose:
        print(f"markers: {'OK' if ok else 'FAIL'} (worst heading error {worst:.1f}°)")
    return ok
