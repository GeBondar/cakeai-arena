# -*- coding: utf-8 -*-
"""
Five 1.5 kg combat robots for the single-overhead-camera arena simulator.

Method: CADAM (code-first CAD: CadQuery / OpenCascade B-rep -> STEP + STL).

Robots
    cake     CakeAI ("Tort II") — round invertible drum spinner, ArUco
             DICT_4X4_50 markers: ID 0 on top, ID 1 on the bottom (as in CakeAI-V1).
    vspin    vertical disc spinner (classic beetle layout).
    hspin    horizontal bar spinner / undercutter.
    wedge    wedge pusher, no active weapon.
    flipper  electric flipper: the front wedge plate is a hinged arm.

Robot frame: X forward, Y left, Z up, origin = wheel axle centre at axle
height (floor at z = -WHEEL_D/2). Units: mm. Every part is exported in its
OWN local frame (joint / part centre); its placement in the robot frame is
written to robots.json (pos_mm). Wheel/drum/disc spin axis = local Y,
bar spin axis = local Z, flipper hinge = local Y at the plate's rear edge.

Run:   python make_step.py
Out:   <robot>/<part>.step|.stl, <robot>/assembly.step|.stl,
       robots.json (masses, COM, inertia tensors, placements), check_report.txt
"""

import json
import math
import os
import sys

import cadquery as cq

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

HERE = os.path.dirname(os.path.abspath(__file__))

ROBOT_MASS = 1.5            # kg — weight class
WHEEL_D, WHEEL_W = 64.0, 18.0
FLOOR = -WHEEL_D / 2
MARKER = 90.0               # ArUco side incl. black border, mm (on a 100 mm white plate) ~38 px on camera
WHEEL_RGB = (0.12, 0.12, 0.12)

# ------------------------------------------------------------------ helpers


def cyl_y(r, w):
    """Cylinder along Y, centred at the origin."""
    return cq.Workplane("XZ").circle(r).extrude(w / 2, both=True)


def cyl_z(r, h):
    return cq.Workplane("XY").circle(r).extrude(h / 2, both=True)


def make_wheel():
    tire = cyl_y(WHEEL_D / 2, WHEEL_W).cut(cyl_y(WHEEL_D / 2 - 7, WHEEL_W + 2))
    hub = cyl_y(WHEEL_D / 2 - 7, WHEEL_W - 4).cut(cyl_y(4, WHEEL_W))
    for i in range(5):
        a = math.radians(i * 72)
        hub = hub.cut(cq.Workplane("XZ").center(15 * math.cos(a), 15 * math.sin(a))
                      .circle(4.5).extrude(WHEEL_W, both=True))
    return tire.union(hub)


def wells(body, x, track, h=200):
    for s in (1, -1):
        body = body.cut(cq.Workplane("XY").box(WHEEL_D + 8, WHEEL_W + 8, h).translate((x, s * track / 2, 0)))
    return body


# ---------------------------------------------------------------- CAKE (CakeAI)
CAKE_R, CAKE_H = 95.0, 50.0    # wheel axle at mid-height -> drives both ways up
CAKE_TRACK = 144.0
DRUM_D, DRUM_L = 40.0, 104.0   # teeth reach R=26 -> 6 mm floor clearance
DRUM_X = 82.0


def make_cake_chassis():
    body = cq.Workplane("XY").circle(CAKE_R).extrude(CAKE_H / 2, both=True)
    for zs in (1, -1):         # "cream" rim at the top and bottom edges
        ring = (cq.Workplane("XY").circle(CAKE_R + 3).circle(CAKE_R - 6)
                .extrude(5).translate((0, 0, zs * CAKE_H / 2 - (5 if zs > 0 else 0))))
        body = body.union(ring)
    body = body.cut(cq.Workplane("XY").box(88, DRUM_L + 8, CAKE_H + 20).translate((DRUM_X + 7, 0, 0)))
    body = wells(body, 0, CAKE_TRACK, CAKE_H + 20)
    body = body.cut(cq.Workplane("XY").circle(CAKE_R - 10).extrude(CAKE_H / 2 - 6, both=True)
                    .cut(cq.Workplane("XY").box(WHEEL_D + 20, 400, CAKE_H))
                    .cut(cq.Workplane("XY").box(90, DRUM_L + 30, CAKE_H).translate((DRUM_X + 5, 0, 0))))
    for s in (1, -1):          # drum bearing ears
        ear = (cq.Workplane("XY").box(44, 8, 34).translate((DRUM_X - 6, s * (DRUM_L / 2 + 8), 0))
               .union(cyl_y(15, 8).translate((DRUM_X, s * (DRUM_L / 2 + 8), 0))))
        body = body.union(ear)
        body = body.cut(cyl_y(6.2, 20).translate((DRUM_X, s * (DRUM_L / 2 + 8), 0)))
    return body


def make_drum():
    d = cyl_y(DRUM_D / 2, DRUM_L)
    for a in (0, 180):
        d = d.union(cq.Workplane("XY").box(10, DRUM_L * 0.8, 10)
                    .translate((DRUM_D / 2 + 1, 0, 0)).rotate((0, 0, 0), (0, 1, 0), a))
    return d.union(cyl_y(6, DRUM_L + 16))


# -------------------------------------------------------------------- WEDGE
WEDGE_L, WEDGE_W, WEDGE_H = 230.0, 180.0, 52.0
WEDGE_TRACK = 150.0


def make_wedge_chassis():
    zb = FLOOR + 3
    zt = zb + WEDGE_H
    xb, xf = -WEDGE_L * 0.45, WEDGE_L * 0.55
    prof = (cq.Workplane("XZ").polyline([(xb, zb), (xf, zb - 1), (xf, zb + 2), (xb + 70, zt), (xb, zt)])
            .close().extrude(WEDGE_W / 2, both=True))
    prof = wells(prof, 0, WEDGE_TRACK)
    return prof.cut(cq.Workplane("XY").box(120, WEDGE_W - 30, WEDGE_H - 10).translate((-30, 0, zb + WEDGE_H / 2)))


# -------------------------------------------------------------- VERTICAL SPINNER
VS_BODY = (150.0, 150.0, 56.0)
VS_TRACK = 120.0
DISK_D, DISK_T = 100.0, 8.0
DISK_X, DISK_Z = 105.0, 34.0   # teeth (R=58) pass 8 mm above the floor


def make_vspin_chassis():
    L, W, H = VS_BODY
    zb = FLOOR + 8
    b = cq.Workplane("XY").box(L, W, H).translate((-10, 0, zb + H / 2))
    b = wells(b, -10, VS_TRACK)
    b = b.cut(cq.Workplane("XY").box(L - 16, W - 50, H - 10).translate((-10, 0, zb + H / 2 + 5)))
    # front skid 2 mm above the floor: the nose-heavy robot rests on it, never on the disc
    b = b.union(cq.Workplane("XY").box(14, W - 20, 8).translate((L / 2 - 17, 0, FLOOR + 2 + 4)))
    for s in (1, -1):
        fork =(cq.Workplane("XY").box(70, 10, 30).translate((DISK_X - 30, s * (DISK_T / 2 + 9), DISK_Z - 10))
                .union(cyl_y(15, 10).translate((DISK_X, s * (DISK_T / 2 + 9), DISK_Z))))
        b = b.union(fork).cut(cyl_y(5.2, 12).translate((DISK_X, s * (DISK_T / 2 + 9), DISK_Z)))
    return b.cut(cq.Workplane("XY").box(40, DISK_T + 6, 200).translate((DISK_X - 45, 0, 0)))


def make_disk():
    d = cyl_y(DISK_D / 2, DISK_T)
    for a in (0, 180):
        d = d.union(cq.Workplane("XY").box(16, DISK_T + 4, 14)
                    .translate((DISK_D / 2, 0, 0)).rotate((0, 0, 0), (0, 1, 0), a))
    return d.union(cyl_y(10, DISK_T + 8)).union(cyl_y(5, DISK_T + 2 * 9 + 10))


# ------------------------------------------------------------ HORIZONTAL SPINNER
HS_TRACK = 140.0
HS_WHEEL_X = -10.0             # wheels forward, heavy rear -> rests on the rear skid, bar tilts up
BAR_L, BAR_W, BAR_T = 170.0, 28.0, 8.0
BAR_X, BAR_Z = 92.0, -14.0     # bar plane 14 mm above the floor


def make_hspin_chassis():
    zb = FLOOR + 7
    body = cq.Workplane("XY").box(130, 160, 46).translate((-65, 0, zb + 23))
    body = wells(body, HS_WHEEL_X, HS_TRACK)
    body = body.cut(cq.Workplane("XY").box(70, 100, 30).translate((-40, 0, zb + 28)))   # electronics bay
    # top beam carrying the bar's motor/axle above the bar
    beam = cq.Workplane("XY").box(BAR_X + 22, 44, 12).translate(((BAR_X + 22) / 2 - 8, 0, BAR_Z + BAR_T / 2 + 10))
    boss = cyl_z(16, 12).translate((BAR_X, 0, BAR_Z + BAR_T / 2 + 10))
    body = body.union(beam).union(boss)
    body = body.cut(cyl_z(6.2, 40).translate((BAR_X, 0, BAR_Z + 10)))
    # front face chamfer so the bar clears the body: body ends at x = 0
    return body


def make_bar():
    bar = cq.Workplane("XY").box(BAR_L, BAR_W, BAR_T).edges("|Z").fillet(6)
    for s in (1, -1):          # impact teeth
        bar = bar.union(cq.Workplane("XY").box(14, 10, BAR_T + 4).translate((s * (BAR_L / 2 - 7), s * (BAR_W / 2 + 3), 0)))
    return bar.union(cyl_z(14, BAR_T + 2)).union(cyl_z(6, BAR_T + 16))


# --------------------------------------------------------------------- FLIPPER
FL_TRACK = 150.0
FL_WHEEL_X = -45.0
HINGE_X, HINGE_Z = 12.0, 18.0
PLATE_L, PLATE_W, PLATE_T = 120.0, 170.0, 5.0


def _plate_angle(drop=HINGE_Z - FLOOR - 2.0):
    """Solve L·sinθ + t·cosθ = drop: the plate's front-bottom edge sits 2 mm above the floor."""
    th = 0.3
    for _ in range(50):
        th -= (PLATE_L * math.sin(th) + PLATE_T * math.cos(th) - drop) / (PLATE_L * math.cos(th) - PLATE_T * math.sin(th))
    return math.degrees(th)


PLATE_ANGLE = _plate_angle()


def make_flipper_chassis():
    zb = FLOOR + 5
    front = HINGE_X - 8        # leave room for the hinge knuckle
    body = cq.Workplane("XY").box(122, 180, HINGE_Z - zb + 6).translate((front - 61, 0, (HINGE_Z + 6 + zb) / 2))
    body = wells(body, FL_WHEEL_X, FL_TRACK)
    body = body.cut(cq.Workplane("XY").box(60, 110, 30).translate((-70, 0, zb + 25)))
    # low front sub-frame under the plate (actuator housing)
    sub = cq.Workplane("XY").box(60, 120, 8).translate((front + 30, 0, zb + 4))
    for s in (1, -1):          # hinge cheeks
        body = body.union(cq.Workplane("XY").box(24, 10, 26).translate((HINGE_X - 6, s * (PLATE_W / 2 + 6), HINGE_Z - 6)))
    return body.union(sub)


def make_flipper_plate():
    """Local frame: hinge axis = local Y through the origin; plate slopes forward/down."""
    p = (cq.Workplane("XY").box(PLATE_L, PLATE_W, PLATE_T).translate((PLATE_L / 2, 0, -PLATE_T / 2))
         .rotate((0, 0, 0), (0, 1, 0), PLATE_ANGLE))
    knuckle = cyl_y(6, PLATE_W)
    ribs = None
    for y in (-50, 0, 50):
        r = (cq.Workplane("XY").box(PLATE_L * 0.6, 6, 8).translate((PLATE_L * 0.3, y, -PLATE_T - 3.5))
             .rotate((0, 0, 0), (0, 1, 0), PLATE_ANGLE))
        ribs = r if ribs is None else ribs.union(r)
    return p.union(knuckle).union(ribs)


# ------------------------------------------------------------------ catalogue
# Part masses follow a 1.5 kg budget; inertia comes from the CAD solid at uniform density.
W = lambda track, x=0.0: [("wheel_l", make_wheel, (x, track / 2, 0), 0.11, WHEEL_RGB),
                          ("wheel_r", make_wheel, (x, -track / 2, 0), 0.11, WHEEL_RGB)]
ROBOTS = {
    "cake": dict(
        title="CakeAI (Tort II) — invertible drum spinner",
        parts=[("chassis", make_cake_chassis, (0, 0, 0), 1.03, (0.93, 0.80, 0.62)),
               ("drum", make_drum, (DRUM_X, 0, 0), 0.25, (0.55, 0.57, 0.60))] + W(CAKE_TRACK),
        markers=[dict(id=0, pos=(-25, 0, CAKE_H / 2 + 0.6), up=+1),
                 dict(id=1, pos=(-25, 0, -CAKE_H / 2 - 0.6), up=-1)],
        weapon=dict(part="drum", kind="drum", axis="y", spin_dir=-1, max_rpm=9000, t_stall=0.20),
        drive=dict(track=CAKE_TRACK, wheel_d=WHEEL_D, v_max=2.6, t_stall=0.45, invertible=True),
    ),
    "vspin": dict(
        title="Vertical disc spinner",
        parts=[("chassis", make_vspin_chassis, (0, 0, 0), 1.0, (0.80, 0.25, 0.20)),
               ("disk", make_disk, (DISK_X, 0, DISK_Z), 0.28, (0.70, 0.72, 0.75))] + W(VS_TRACK, -10),
        markers=[],
        weapon=dict(part="disk", kind="vertical", axis="y", spin_dir=-1, max_rpm=10000, t_stall=0.22),
        drive=dict(track=VS_TRACK, wheel_d=WHEEL_D, v_max=2.2, t_stall=0.45, invertible=False),
    ),
    "hspin": dict(
        title="Horizontal bar spinner (undercutter)",
        parts=[("chassis", make_hspin_chassis, (0, 0, 0), 0.99, (0.95, 0.55, 0.10)),
               ("bar", make_bar, (BAR_X, 0, BAR_Z), 0.29, (0.62, 0.64, 0.68))] + W(HS_TRACK, HS_WHEEL_X),
        markers=[],
        weapon=dict(part="bar", kind="horizontal", axis="z", spin_dir=1, max_rpm=6000, t_stall=0.35),
        drive=dict(track=HS_TRACK, wheel_d=WHEEL_D, v_max=2.0, t_stall=0.45, invertible=False),
    ),
    "wedge": dict(
        title="Wedge pusher",
        parts=[("chassis", make_wedge_chassis, (0, 0, 0), 1.28, (0.20, 0.45, 0.80))] + W(WEDGE_TRACK),
        markers=[], weapon=None,
        drive=dict(track=WEDGE_TRACK, wheel_d=WHEEL_D, v_max=2.4, t_stall=0.50, invertible=False),
    ),
    "flipper": dict(
        title="Electric flipper",
        parts=[("chassis", make_flipper_chassis, (0, 0, 0), 1.08, (0.25, 0.70, 0.35)),
               ("plate", make_flipper_plate, (HINGE_X, 0, HINGE_Z), 0.20, (0.75, 0.77, 0.80))] + W(FL_TRACK, FL_WHEEL_X),
        markers=[],
        weapon=dict(part="plate", kind="flipper", axis="y", spin_dir=-1, range_deg=95.0, torque=9.0),
        drive=dict(track=FL_TRACK, wheel_d=WHEEL_D, v_max=2.3, t_stall=0.50, invertible=False),
    ),
}


def mass_props(solid, mass_kg):
    """COM (mm) and inertia tensor about the COM (kg·m²) at uniform density."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    vol = solid.Volume()
    com = solid.Center()
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(solid.wrapped, props)
    m = props.MatrixOfInertia()
    rho = mass_kg / vol
    I = [[m.Value(i + 1, j + 1) * rho * 1e-6 for j in range(3)] for i in range(3)]
    return vol, (com.x, com.y, com.z), I


def main():
    only = sys.argv[1:]
    report = ["Self-check: 1.5 kg robots (CADAM / CadQuery)", ""]
    path_json = os.path.join(HERE, "robots.json")
    catalogue = {}
    if only and os.path.exists(path_json):
        with open(path_json, encoding="utf-8") as f:
            catalogue = json.load(f)
    for rname, spec in ROBOTS.items():
        if only and rname not in only:
            continue
        out = os.path.join(HERE, rname)
        os.makedirs(out, exist_ok=True)
        asm = cq.Assembly(name=f"robot_{rname}")
        placed = []
        rj = dict(title=spec["title"], parts={}, markers=spec["markers"], weapon=spec["weapon"],
                  drive=spec["drive"], marker_size_mm=MARKER)
        total = 0.0
        report.append(f"== {rname}: {spec['title']}")
        for pname, fn, pos, mass, col in spec["parts"]:
            solid = fn().val()
            cq.exporters.export(solid, os.path.join(out, f"{pname}.step"))
            cq.exporters.export(solid, os.path.join(out, f"{pname}.stl"), tolerance=0.15, angularTolerance=0.2)
            vol, com, I = mass_props(solid, mass)
            bb = solid.BoundingBox()
            placed.append((pname, solid.located(cq.Location(cq.Vector(*pos)))))
            asm.add(solid, name=pname, color=cq.Color(*col), loc=cq.Location(cq.Vector(*pos)))
            rj["parts"][pname] = dict(stl=f"{rname}/{pname}.stl", pos_mm=list(pos), mass_kg=mass,
                                      com_mm=[round(c, 3) for c in com],
                                      inertia_kgm2=[[float(f"{v:.6e}") for v in row] for row in I],
                                      bbox_mm=[bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax],
                                      volume_mm3=round(vol, 1), rgb=list(col))
            total += mass
            report.append(f"  {pname:8s} m={mass:.3f} kg  V={vol / 1000:8.1f} cm3  bbox={bb.xlen:.0f}x{bb.ylen:.0f}x{bb.zlen:.0f}")
        asm.export(os.path.join(out, "assembly.step"))
        comp = cq.Compound.makeCompound([p for _, p in placed])
        cq.exporters.export(comp, os.path.join(out, "assembly.stl"), tolerance=0.2)
        bb = comp.BoundingBox()
        rj["bbox_mm"] = [bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax]
        rj["mass_kg"] = round(total, 4)
        report.append(f"  TOTAL {total:.3f} kg (limit {ROBOT_MASS} kg) -> {'OK' if total <= ROBOT_MASS + 1e-9 else 'OVERWEIGHT'}")
        zlow = min(p.BoundingBox().zmin for n, p in placed if not n.startswith("wheel"))
        report.append(f"  envelope {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm; lowest non-wheel point "
                      f"{zlow - FLOOR:.1f} mm above the floor -> {'OK' if zlow >= FLOOR + 0.5 else 'TOUCHES FLOOR'}")
        for i in range(len(placed)):
            for j in range(i + 1, len(placed)):
                v = placed[i][1].intersect(placed[j][1]).Volume()
                report.append(f"  {placed[i][0]} ∩ {placed[j][0]} = {v:.1f} mm3  {'OK' if v < 5 else 'INTERSECTION'}")
        report.append("")
        catalogue[rname] = rj
    with open(path_json, "w", encoding="utf-8") as f:
        json.dump(catalogue, f, ensure_ascii=False, indent=1)
    with open(os.path.join(HERE, "check_report.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(report))
    print("\n".join(report))


if __name__ == "__main__":
    main()
