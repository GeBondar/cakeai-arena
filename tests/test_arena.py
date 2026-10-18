"""Arena tests: CAD catalogue, MJCF compile, physics of every weapon, camera-in-the-loop smoke match.

    pytest -q                 # everything
    pytest -q -m "not render" # no OpenGL needed
"""
import math
import os
import sys

import mujoco
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arena import scene  # noqa: E402
from arena.pilots import ROSTER  # noqa: E402

CAT = scene.load_catalogue()


def _model(kind, pose=(0, 0, 0, False)):
    m = mujoco.MjModel.from_xml_string(scene.build_mjcf([("r0", kind, pose, None)]))
    return m, mujoco.MjData(m)


def _run(m, d, secs, ctrl=None):
    for _ in range(int(secs / m.opt.timestep)):
        if ctrl:
            for name, v in ctrl.items():
                d.ctrl[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)] = v
        mujoco.mj_step(m, d)
    assert np.all(np.isfinite(d.qpos)), "simulation blew up"


def test_every_robot_is_exactly_1p5_kg():
    for k, r in CAT.items():
        assert abs(r["mass_kg"] - 1.5) < 1e-6, k
        assert abs(sum(p["mass_kg"] for p in r["parts"].values()) - 1.5) < 1e-6, k


def test_roster_uses_known_robots():
    assert {o.robot for o in ROSTER.values()} <= set(CAT)
    assert {"vspin", "hspin", "wedge", "flipper"} <= set(CAT)


@pytest.mark.parametrize("kind", sorted(CAT))
def test_robot_settles_and_drives_forward(kind):
    m, d = _model(kind)
    _run(m, d, 0.4)
    base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "r0_base")
    assert d.xmat[base][8] > 0.9, "robot should rest upright"
    x0 = d.xpos[base][0]
    _run(m, d, 0.6, {"r0_left": 0.8, "r0_right": 0.8})
    assert d.xpos[base][0] - x0 > 0.3, "full throttle should move it forward"


@pytest.mark.parametrize("kind", [k for k, r in CAT.items() if r["weapon"] and r["weapon"]["kind"] != "flipper"])
def test_spinner_spins_up(kind):
    m, d = _model(kind)
    _run(m, d, 2.0, {"r0_weapon": 1.0})
    dof = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "r0_weapon_j")]
    rpm = abs(d.qvel[dof]) * 60 / (2 * math.pi)
    assert rpm > 0.6 * CAT[kind]["weapon"]["max_rpm"], f"{kind}: {rpm:.0f} rpm"


def test_flipper_fires_fast():
    m, d = _model("flipper")
    _run(m, d, 0.3)
    target = math.radians(CAT["flipper"]["weapon"]["range_deg"])
    _run(m, d, 0.2, {"r0_weapon": target})
    q = d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "r0_weapon_j")]]
    assert q > 0.7 * target, f"flipper only reached {math.degrees(q):.0f} deg in 0.2 s"


@pytest.mark.render
def test_markers_detected_upright_and_flipped():
    from arena.checks import check_markers
    assert check_markers(verbose=False)


@pytest.mark.render
@pytest.mark.parametrize("opp", ["hspin", "flipper"])
def test_camera_in_the_loop_match(opp):
    from arena.match import Match
    mt = Match(opp, secs=2.0, fps=30, seed=1)
    while not mt.step():
        pass
    res = mt.finish()
    mt.close()
    assert res["winner"] in ("CakeAI", ROSTER[opp].name)
    assert mt.ai.rc.ch3 > 0, "CakeAI should be spinning its weapon up"
