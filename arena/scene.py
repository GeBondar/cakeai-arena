"""MJCF builder: a 1.5 kg arena with robots generated from the CADAM models.

Approach follows the GeBondar/battlebots_docs simulator (hand-written MJCF,
free-joint chassis, hinge wheels/weapons, collision groups, contact-based
hit counting), re-scaled for the 1.5 kg class:

* arena 2.0 x 2.0 m, 5 cm steel rail + 40 cm polycarbonate walls;
* robots = CAD meshes from cad/robots_1p5kg, masses/inertia from robots.json;
* drive = brushed DC motor  tau = T_stall * (u - w / w_free),  u in [-1, 1];
* spinners = physical rotors (drum / vertical disc / horizontal bar),
  flipper = hinged plate on a torque-limited position servo;
* ArUco DICT_4X4_50 textures (ID 0 on top, ID 1 underneath) — only on the
  robot that runs the CakeAI brain, exactly like the real robot;
* one overhead camera 1280x1024 above the centre: the arena lands inside the
  (193,70)-(1085,945) crop from CakeAI-V1's config.ini.

World axes: X = image right, Y = image up, Z = up. Metres.
"""
import json
import math
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CAD_DIR = os.path.join(ROOT, "cad", "robots_1p5kg")
ASSETS = os.path.join(HERE, "assets")

ARENA = 2.0                    # inner arena size, m
RAIL_H, WALL_H = 0.05, 0.40
CAM_H = 2.5                    # overhead camera height, m
CAM_W, CAM_HPX = 1280, 1024
CAM_FOVY = 51.5                # ~425 px/m on the floor: 2 m arena ~ 850 px
CAM_SHIFT = (-0.5 / 425.0, -4.5 / 425.0)   # centre the arena in CakeAI's crop (639, 507.5)
WHEEL_R = 0.032
FLIPPER_KP, FLIPPER_KV = 60.0, 0.6


def load_catalogue():
    with open(os.path.join(CAD_DIR, "robots.json"), encoding="utf-8") as f:
        return json.load(f)


def make_marker_png(marker_id, px=360, border_frac=0.1):
    """DICT_4X4_50 marker with a white quiet zone (the sticker on the robot)."""
    os.makedirs(ASSETS, exist_ok=True)
    path = os.path.join(ASSETS, f"aruco_{marker_id}.png")
    if os.path.exists(path):
        return path
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    inner = int(px * (1 - 2 * border_frac))
    img = np.full((px, px), 255, np.uint8)
    o = (px - inner) // 2
    img[o:o + inner, o:o + inner] = cv2.aruco.generateImageMarker(d, marker_id, inner)
    cv2.imwrite(path, cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
    return path


def plate_mesh_xml(name, size_m, up):
    """1 mm plate with UVs so that the marker's TL->TR edge points along robot +X.

    up=+1 faces +Z; up=-1 faces -Z and reads correctly once the robot is rolled
    180 deg about X (verified by `python -m arena check`)."""
    h, t = size_m / 2, 0.0005
    quad = [(-h, -h), (h, -h), (h, h), (-h, h)] if up > 0 else [(-h, h), (h, h), (h, -h), (-h, -h)]
    uv = [(0, 1), (1, 1), (1, 0), (0, 0)]          # MuJoCo: v = 0 is the top row of the PNG
    zf = t * up
    verts = [(x, y, zf) for x, y in quad] + [(x, y, -zf) for x, y in quad]
    tex = uv + uv
    faces = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6)] if up > 0 else [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7)]
    faces += [(0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2), (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0)]
    vs = " ".join(f"{a:.5f} {b:.5f} {c:.5f}" for a, b, c in verts)
    ts = " ".join(f"{u} {v}" for u, v in tex)
    fs = " ".join(f"{a} {b} {c}" for a, b, c in faces)
    return f'<mesh name="{name}" vertex="{vs}" texcoord="{ts}" face="{fs}"/>'


def _fi(I):
    return f"{I[0][0]:.4e} {I[1][1]:.4e} {I[2][2]:.4e} {I[0][1]:.4e} {I[0][2]:.4e} {I[1][2]:.4e}"


def _v(p_mm):
    return " ".join(f"{c / 1000:.5f}" for c in p_mm)


def _rgba(c, a=1.0):
    return f"{c[0]} {c[1]} {c[2]} {a}"


def robot_xml(slot, kind, spec, pose, colbits, color=None, markers=False):
    """MJCF body tree for one robot. slot = unique prefix (r0, r1, ...)."""
    x, y, yaw, flipped = pose
    p = spec["parts"]
    ch = p["chassis"]
    ct, ca = colbits
    col = f'contype="{ct}" conaffinity="{ca}"'
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    quat = f"0 {cy:.6f} {sy:.6f} 0" if flipped else f"{cy:.6f} 0 0 {sy:.6f}"   # flipped = roll 180 about X
    out = [f'<body name="{slot}_base" pos="{x:.4f} {y:.4f} {WHEEL_R + 0.002:.4f}" quat="{quat}">',
           f'  <freejoint name="{slot}_free"/>',
           f'  <inertial pos="{_v(ch["com_mm"])}" mass="{ch["mass_kg"]}" fullinertia="{_fi(ch["inertia_kgm2"])}"/>',
           f'  <geom name="{slot}_chassis" type="mesh" mesh="{kind}_chassis" rgba="{_rgba(color or ch["rgb"])}" '
           f'{col} friction="0.25 0.005 0.0001" solref="0.003 1"/>']
    if kind == "cake":         # cherries on the cake (visual only)
        for a_deg in (130, 155, 180, 205, 230):
            a = math.radians(a_deg)
            for zs in (1, -1):
                out.append(f'  <geom type="sphere" size="0.006" pos="{0.084 * math.cos(a):.4f} {0.084 * math.sin(a):.4f} '
                           f'{zs * 0.024}" contype="0" conaffinity="0" mass="0" rgba="0.80 0.05 0.10 1"/>')
    if markers:
        for m in spec.get("markers", []):
            out.append(f'  <geom name="{slot}_marker{m["id"]}" type="mesh" mesh="plate_{m["id"]}_{"up" if m["up"] > 0 else "dn"}" '
                       f'material="aruco{m["id"]}" pos="{_v(m["pos"])}" contype="0" conaffinity="0" mass="0"/>')
    for side in ("wheel_l", "wheel_r"):
        w = p[side]
        out += [f'  <body name="{slot}_{side}" pos="{_v(w["pos_mm"])}">',
                f'    <joint name="{slot}_{side}_j" type="hinge" axis="0 1 0" damping="0.0005"/>',
                f'    <inertial pos="0 0 0" mass="{w["mass_kg"]}" fullinertia="{_fi(w["inertia_kgm2"])}"/>',
                f'    <geom type="cylinder" size="{WHEEL_R} 0.009" euler="1.5708 0 0" {col} '
                f'friction="0.95 0.01 0.0002" solref="0.004 1" rgba="0.1 0.1 0.1 1" group="3"/>',
                f'    <geom type="mesh" mesh="{kind}_wheel" contype="0" conaffinity="0" mass="0" rgba="0.13 0.13 0.13 1"/>',
                '  </body>']
    wpn = spec.get("weapon")
    if wpn:
        wp = p[wpn["part"]]
        axis = {"y": (0, 1, 0), "z": (0, 0, 1)}[wpn["axis"]]
        axis = " ".join(str(a * wpn["spin_dir"]) for a in axis)
        if wpn["kind"] == "flipper":
            jnt = f'<joint name="{slot}_weapon_j" type="hinge" axis="{axis}" range="0 {math.radians(wpn["range_deg"]):.4f}" damping="0.02" armature="0.0005"/>'
        else:
            jnt = f'<joint name="{slot}_weapon_j" type="hinge" axis="{axis}" damping="0.00002"/>'
        out += [f'  <body name="{slot}_weapon" pos="{_v(wp["pos_mm"])}">',
                f'    {jnt}',
                f'    <inertial pos="{_v(wp["com_mm"])}" mass="{wp["mass_kg"]}" fullinertia="{_fi(wp["inertia_kgm2"])}"/>',
                f'    <geom name="{slot}_weapon_g" type="mesh" mesh="{kind}_{wpn["part"]}" {col} '
                f'friction="0.6 0.005 0.0001" solref="0.002 1" rgba="{_rgba(wp["rgb"])}"/>',
                '  </body>']
    out.append('</body>')
    return "\n".join(out)


def actuators_xml(slot, spec):
    dr = spec["drive"]
    w_free = dr["v_max"] / WHEEL_R
    g, b = dr["t_stall"], -dr["t_stall"] / w_free
    out = [f'<general name="{slot}_{s}" joint="{slot}_wheel_{s[0]}_j" ctrlrange="-1 1" gainprm="{g}" '
           f'biastype="affine" biasprm="0 0 {b:.5f}"/>' for s in ("left", "right")]
    wpn = spec.get("weapon")
    if wpn and wpn["kind"] == "flipper":
        out.append(f'<position name="{slot}_weapon" joint="{slot}_weapon_j" kp="{FLIPPER_KP}" kv="{FLIPPER_KV}" '
                   f'ctrlrange="0 {math.radians(wpn["range_deg"]):.4f}" forcerange="-{wpn["torque"]} {wpn["torque"]}"/>')
    elif wpn:
        wf = wpn["max_rpm"] * 2 * math.pi / 60
        tg = wpn["t_stall"]
        out.append(f'<general name="{slot}_weapon" joint="{slot}_weapon_j" ctrlrange="0 1" gainprm="{tg}" '
                   f'biastype="affine" biasprm="0 0 {-tg / wf:.7f}"/>')
    return "\n".join(out)


def build_mjcf(entries, timestep=0.0005, ai_slot="r0"):
    """entries: [(slot, kind, (x, y, yaw, flipped), rgb_or_None), ...]"""
    cat = load_catalogue()
    kinds = sorted({e[1] for e in entries})
    asset, seen = [], set()
    for k in kinds:
        for pname, pp in cat[k]["parts"].items():
            mname = f"{k}_wheel" if pname.startswith("wheel") else f"{k}_{pname}"
            if mname in seen:
                continue
            seen.add(mname)
            path = os.path.join(CAD_DIR, pp["stl"]).replace("\\", "/")
            asset.append(f'<mesh name="{mname}" file="{path}" scale="0.001 0.001 0.001"/>')
    msz = cat["cake"]["marker_size_mm"] / 1000 + 0.01
    asset += [plate_mesh_xml("plate_0_up", msz, +1), plate_mesh_xml("plate_1_dn", msz, -1)]
    for mid in (0, 1):
        png = make_marker_png(mid).replace("\\", "/")
        asset.append(f'<texture name="aruco{mid}_tex" type="2d" file="{png}"/>')
        asset.append(f'<material name="aruco{mid}" texture="aruco{mid}_tex" specular="0" shininess="0" reflectance="0"/>')

    allbits = sum(1 << (i + 1) for i in range(len(entries)))
    bodies, acts = [], []
    for i, (slot, kind, pose, color) in enumerate(entries):
        me = 1 << (i + 1)
        bodies.append(robot_xml(slot, kind, cat[kind], pose, (me, 1 | (allbits & ~me)), color,
                                markers=(slot == ai_slot)))
        acts.append(actuators_xml(slot, cat[kind]))

    h, wt = ARENA / 2, 0.03
    walls = []
    for nm, (px, py, sx, sy) in dict(n=(0, h + wt, h + 2 * wt, wt), s=(0, -h - wt, h + 2 * wt, wt),
                                     e=(h + wt, 0, wt, h), w=(-h - wt, 0, wt, h)).items():
        walls.append(f'<geom name="rail_{nm}" type="box" pos="{px} {py} {RAIL_H / 2}" size="{sx} {sy} {RAIL_H / 2}" '
                     f'material="steel" contype="1" conaffinity="{allbits}"/>')
        walls.append(f'<geom name="poly_{nm}" type="box" pos="{px} {py} {RAIL_H + WALL_H / 2}" size="{sx} {sy} {WALL_H / 2}" '
                     f'rgba="0.75 0.85 0.95 0.10" contype="1" conaffinity="{allbits}"/>')
    nl = "\n"
    return f"""<mujoco model="arena_1p5kg">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{timestep}" integrator="implicitfast" cone="elliptic"/>
  <size memory="64M"/>
  <visual>
    <global offwidth="1920" offheight="1088"/>
    <quality offsamples="4" shadowsize="2048"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.45 0.45 0.45" specular="0.05 0.05 0.05"/>
    <map znear="0.01"/>
  </visual>
  <default>
    <geom friction="0.25 0.005 0.0001"/>
  </default>
  <asset>
{nl.join("    " + a for a in asset)}
    <texture name="floor_tex" type="2d" builtin="flat" rgb1="0.27 0.28 0.30" width="64" height="64" mark="edge" markrgb="0.32 0.33 0.35"/>
    <material name="floor_mat" texture="floor_tex" texrepeat="8 8" reflectance="0.02"/>
    <texture name="out_tex" type="2d" builtin="checker" rgb1="0.12 0.12 0.13" rgb2="0.16 0.16 0.17" width="256" height="256"/>
    <material name="out_mat" texture="out_tex" texrepeat="20 20"/>
    <material name="center_mat" rgba="0.22 0.23 0.25 1" specular="0" shininess="0" reflectance="0"/>
    <material name="steel" rgba="0.55 0.56 0.58 1" specular="0.3" shininess="0.5"/>
  </asset>
  <worldbody>
    <light name="key" pos="0 0 3.2" dir="0 0 -1" diffuse="0.55 0.55 0.55" specular="0.05 0.05 0.05" castshadow="false"/>
    <light name="k1" pos="1.2 1.2 2.6" dir="-0.3 -0.3 -1" diffuse="0.25 0.25 0.25" castshadow="false"/>
    <light name="k2" pos="-1.2 -1.2 2.6" dir="0.3 0.3 -1" diffuse="0.25 0.25 0.25" castshadow="true"/>
    <geom name="outside" type="plane" size="6 6 0.1" pos="0 0 -0.001" material="out_mat" contype="0" conaffinity="0"/>
    <geom name="floor" type="box" size="{h + 0.06} {h + 0.06} 0.005" pos="0 0 -0.005" material="floor_mat" contype="1" conaffinity="{allbits}"/>
    <geom name="center_mark" type="cylinder" size="0.12 0.0003" pos="0 0 0.0003" material="center_mat" contype="0" conaffinity="0"/>
{nl.join("    " + w for w in walls)}
    <camera name="overhead" pos="{CAM_SHIFT[0]:.5f} {CAM_SHIFT[1]:.5f} {CAM_H}" xyaxes="1 0 0 0 1 0" fovy="{CAM_FOVY}"/>
{nl.join(bodies)}
  </worldbody>
  <actuator>
{nl.join(acts)}
  </actuator>
</mujoco>
"""
