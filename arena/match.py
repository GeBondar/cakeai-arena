"""A 1.5 kg match: CakeAI (camera-in-the-loop) vs a scripted opponent.

CakeAI control loop — the same as on the real arena:

    MuJoCo -> overhead render 1280x1024 (+ sensor noise) -> CakeAIPipeline
    (arena crop, ArUco, motion detector, RobotController) -> "1:P:xx" lines to
    the Arduino emulator -> N-frame latency -> motor duty.

CakeAI never sees ground truth, only pixels. The opponent pilot drives from
ground truth (it plays the human with the transmitter).

Scoring mirrors bench_core.DuelSim (contact episodes weapon->opponent and
body->opponent) plus impact energy = drop of the rotor's kinetic energy
during the episode -> damage / HP. Flips are credited to the other robot.
"""
import math
import os
from collections import deque
from dataclasses import dataclass, field

import cv2
import mujoco
import numpy as np

from cakeai.pipeline import CakeAIPipeline, load_config
from . import scene
from .pilots import ROSTER, Opponent, Pilot

CONFIG_PATH = os.path.join(scene.ROOT, "configs", "cakeai_crashilovo.ini")
HP_PER_J = 1.6              # damage model: 1 J of rotor energy ~ 1.6 % HP
MAX_HIT_DMG = 30.0          # a single hit never removes more than 30 % HP
EP_GAP = 0.25               # contacts closer than 0.25 s merge into one episode
KO_FLIP_S = 5.0             # upside down and not self-righted for 5 s -> KO


@dataclass
class Side:
    name: str
    robot: str
    hp: float = 100.0
    weapon_hits: int = 0
    rams: int = 0
    flips: int = 0           # flips this robot inflicted
    energy: float = 0.0
    biggest: float = 0.0
    flipped_t: float = 0.0
    upright_prev: float = 1.0
    ko: bool = False
    hits: list = field(default_factory=list)


class Match:
    SLOTS = ("r0", "r1")

    def __init__(self, opponent="vspin", tactic=None, secs=60.0, fps=60, latency_frames=2, seed=0,
                 cake_pose=(-0.65, -0.65, math.radians(45), False), opp_pose=None, noise_sigma=2.0,
                 config_path=CONFIG_PATH, render_camera=True):
        opp = opponent if isinstance(opponent, Opponent) else ROSTER[opponent]
        if tactic:
            opp = Opponent(opp.key, opp.robot, opp.name, tactic, opp.duty, opp.blurb)
        self.opp = opp
        self.rng = np.random.default_rng(seed)
        self.secs, self.fps, self.seed = secs, fps, seed
        opp_pose = opp_pose or (0.65, 0.65, math.radians(-135), False)
        dt = 0.0005
        self.spc = max(1, round(1.0 / (fps * dt)))
        self.dt_frame = self.spc * dt
        self.cat = scene.load_catalogue()
        xml = scene.build_mjcf([("r0", "cake", cake_pose, None), ("r1", opp.robot, opp_pose, None)], timestep=dt)
        self.m = mujoco.MjModel.from_xml_string(xml)
        self.d = mujoco.MjData(self.m)
        mujoco.mj_forward(self.m, self.d)
        self.t = 0.0
        self.noise = noise_sigma
        self.renderer = mujoco.Renderer(self.m, scene.CAM_HPX, scene.CAM_W) if render_camera else None
        self.cfg = load_config(config_path)
        self.ai = CakeAIPipeline(lambda: self.t, self.cfg)
        self.cmd_queue = deque([(0.0, 0.0, 0.0)] * latency_frames)

        name = lambda o, i: mujoco.mj_id2name(self.m, o, i) or ""
        bid = lambda n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, n)
        aid = lambda n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        self.base = {s: bid(f"{s}_base") for s in self.SLOTS}
        self.wbody = {s: bid(f"{s}_weapon") for s in self.SLOTS}
        self.act = {s: (aid(f"{s}_left"), aid(f"{s}_right"), aid(f"{s}_weapon")) for s in self.SLOTS}
        self.owner = np.full(self.m.nbody, -1, np.int8)
        self.isweap = np.zeros(self.m.nbody, bool)
        for i in range(self.m.nbody):
            n = name(mujoco.mjtObj.mjOBJ_BODY, i)
            for k, s in enumerate(self.SLOTS):
                if n.startswith(s + "_"):
                    self.owner[i] = k
        self.wkind, self.wdof = {}, {}
        M = np.zeros((self.m.nv, self.m.nv))
        mujoco.mj_fullM(self.m, M, self.d.qM)
        for s, robot in (("r0", "cake"), ("r1", opp.robot)):
            w = self.cat[robot]["weapon"]
            self.wkind[s] = w["kind"] if w else None
            if w:
                self.isweap[self.wbody[s]] = True
                dof = self.m.jnt_dofadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, f"{s}_weapon_j")]
                self.wdof[s] = (dof, M[dof, dof])
        self.flip_angle = math.radians(self.cat[opp.robot]["weapon"]["range_deg"]) if self.wkind["r1"] == "flipper" else 0.0
        self.sides = {"r0": Side("CakeAI", "cake"), "r1": Side(opp.name, opp.robot)}
        self.in_contact = {(s, k): False for s in self.SLOTS for k in "wb"}
        self.ep = {s: dict(active=False, energy=0.0, ke0=0.0, kemin=0.0, last=-9.0) for s in self.SLOTS}
        self.ram_last = {s: -9.0 for s in self.SLOTS}
        self.frame_contact = False
        self.plate_touch = False
        self.pilot = Pilot(opp, self.wkind["r1"], self.rng, fps)
        self.opp_hist = deque([(0.0, 0.0, 0.0)] * max(1, int(0.15 * fps)))   # 150 ms reaction
        self.winner, self.reason = None, ""
        self.events = []         # dicts: t, type (hit|flip|ko), side, value, text
        self.human = None        # play mode: callable -> (left, right, weapon) overrides the pilot
        self.ai.start()          # the START button on CakeAI's panel

    # ------------------------------------------------------------ helpers
    def pose(self, s):
        b = self.base[s]
        R = self.d.xmat[b].reshape(3, 3)
        return float(self.d.xpos[b][0]), float(self.d.xpos[b][1]), math.atan2(R[1, 0], R[0, 0]), float(R[2, 2])

    def weapon_ke(self, s):
        if s not in self.wdof or self.wkind[s] == "flipper":
            return 0.0
        dof, I = self.wdof[s]
        return 0.5 * I * self.d.qvel[dof] ** 2

    def weapon_rpm(self, s):
        if s not in self.wdof or self.wkind[s] == "flipper":
            return 0.0
        return abs(self.d.qvel[self.wdof[s][0]]) * 60 / (2 * math.pi)

    def render_overhead(self):
        self.renderer.update_scene(self.d, camera="overhead")
        img = cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR)
        if self.noise > 0:
            if not hasattr(self, "_noise_bank"):        # sensor noise bank: fast, never the same twice in a row
                bank = []
                for _ in range(12):
                    n = self.rng.normal(0, self.noise, img.shape)
                    bank.append((np.clip(n, 0, 255).astype(np.uint8), np.clip(-n, 0, 255).astype(np.uint8)))
                self._noise_bank = bank
            pos, neg = self._noise_bank[self.rng.integers(len(self._noise_bank))]
            img = cv2.subtract(cv2.add(img, pos), neg)
        return img

    # ----------------------------------------------------------------- step
    def step(self):
        """One camera frame: CakeAI vision+control, opponent pilot, physics. -> done"""
        l, r, w = self.ai.step(self.render_overhead())
        self.cmd_queue.append((l, r, w))
        a = self.act["r0"]
        self.d.ctrl[a[0]], self.d.ctrl[a[1]], self.d.ctrl[a[2]] = self.cmd_queue.popleft()

        me, tgt = self.pose("r1"), self.pose("r0")
        if self.human is not None:                        # a real human brings their own reaction time
            ol, orr, ow = self.human(self)
        else:
            dl, dr = self.pilot.drive(self.t, me, tgt, self.frame_contact)
            ow = self.pilot.weapon(self.t, me, tgt, self.plate_touch, self.flip_angle)
            self.opp_hist.append((dl, dr, ow))
            ol, orr, _ = self.opp_hist.popleft()
        a = self.act["r1"]
        self.d.ctrl[a[0]], self.d.ctrl[a[1]] = ol, orr
        if a[2] >= 0:
            self.d.ctrl[a[2]] = ow                        # weapon is not delayed (switch on the transmitter)

        self.frame_contact = self.plate_touch = False
        for i in range(self.spc):
            mujoco.mj_step(self.m, self.d)
            self._contacts(self.t + (i + 1) * self.m.opt.timestep)
        self.t += self.dt_frame
        self._flips_and_ko()
        return self.t >= self.secs or self.winner is not None

    def _contacts(self, ts):
        now = {k: False for k in self.in_contact}
        n = self.d.ncon
        if n:
            gb = self.m.geom_bodyid
            b1, b2 = gb[self.d.contact.geom1[:n]], gb[self.d.contact.geom2[:n]]
            o1, o2 = self.owner[b1], self.owner[b2]
            cross = (o1 >= 0) & (o2 >= 0) & (o1 != o2)
            if cross.any():
                for oo, bb in ((o1[cross], b1[cross]), (o2[cross], b2[cross])):
                    ww = self.isweap[bb]
                    for k, s in enumerate(self.SLOTS):
                        sel = oo == k
                        if sel.any():
                            now[(s, "w")] |= bool(ww[sel].any())
                            now[(s, "b")] |= bool((~ww[sel]).any())
        if any(now.values()):
            self.frame_contact = True
        if now[("r1", "w")] and self.wkind["r1"] == "flipper":
            self.plate_touch = True
        for s, o in (("r0", "r1"), ("r1", "r0")):
            if self.wkind[s] in ("drum", "vertical", "horizontal"):
                ep, ke = self.ep[s], self.weapon_ke(s)
                if now[(s, "w")]:
                    if not self.in_contact[(s, "w")]:
                        ep["ke0"] = ep["kemin"] = ke
                        if not ep["active"]:
                            ep.update(active=True, energy=0.0)
                            self.sides[s].weapon_hits += 1
                    ep["kemin"] = min(ep["kemin"], ke)
                    ep["last"] = ts
                else:
                    if self.in_contact[(s, "w")]:
                        ep["energy"] += max(0.0, ep["ke0"] - ep["kemin"])
                    if ep["active"] and ts - ep["last"] > EP_GAP:
                        ep["active"] = False
                        self._hit(s, o, ep["energy"])
            elif now[(s, "w")] and ts - self.ram_last[s] > EP_GAP:     # flipper plate touches
                self.sides[s].weapon_hits += 1
            if now[(s, "w")] and self.wkind[s] not in ("drum", "vertical", "horizontal"):
                self.ram_last[s] = ts
            if now[(s, "b")] and not now[(s, "w")] and not now[(o, "w")]:
                if ts - self.ram_last[s] > EP_GAP:
                    self.sides[s].rams += 1
                self.ram_last[s] = ts
        self.in_contact = now

    def _hit(self, s, o, e):
        S, O = self.sides[s], self.sides[o]
        e = float(e)
        S.energy += e
        S.biggest = max(S.biggest, e)
        O.hp = max(0.0, O.hp - min(MAX_HIT_DMG, HP_PER_J * e))
        if e > 1.0:
            S.hits.append((round(self.t, 2), round(e, 1)))
            self.events.append(dict(t=self.t, type="hit", side=s, value=e, text=f"{S.name}: {e:.0f} J hit"))

    def _flips_and_ko(self):
        for s, o in (("r0", "r1"), ("r1", "r0")):
            S = self.sides[s]
            up = self.pose(s)[3]
            if S.upright_prev > 0.5 and up < -0.3:
                self.sides[o].flips += 1
                self.events.append(dict(t=self.t, type="flip", side=o, value=1, text=f"{S.name} is FLIPPED!"))
            if up > 0.5 or up < -0.3:
                S.upright_prev = up
            invertible = self.cat[S.robot]["drive"]["invertible"]
            S.flipped_t = S.flipped_t + self.dt_frame if (up < 0.3 and not invertible) else 0.0
            if S.flipped_t > KO_FLIP_S or S.hp <= 0:
                S.ko = True
                self.winner = o
                self.reason = "KO — flipped, could not self-right" if S.flipped_t > KO_FLIP_S else "KO — HP 0"
                self.events.append(dict(t=self.t, type="ko", side=o, value=1, text=f"KO! {S.name} is out"))
                return

    def finish(self):
        if self.winner is None:
            a, b = self.sides["r0"], self.sides["r1"]
            if abs(a.hp - b.hp) > 1e-6:
                self.winner = "r0" if a.hp > b.hp else "r1"
                self.reason = "judges — damage"
            else:
                agg = lambda S: S.weapon_hits + S.rams + 3 * S.flips
                self.winner = "r0" if agg(a) >= agg(b) else "r1"
                self.reason = "judges — aggression"
        return self.result()

    def result(self):
        def side(S):
            return dict(name=S.name, robot=S.robot, hp=round(S.hp, 1), weapon_hits=S.weapon_hits, rams=S.rams,
                        flips=S.flips, energy_J=round(S.energy, 1), biggest_J=round(S.biggest, 1), hits=S.hits)
        return dict(opponent=self.opp.key, tactic=self.opp.tactic, seed=self.seed, t=round(self.t, 2),
                    winner=self.sides[self.winner].name if self.winner else None, winner_slot=self.winner,
                    reason=self.reason, cake=side(self.sides["r0"]), opp=side(self.sides["r1"]),
                    cake_attack_cycles=self.ai.rc.attack_count,
                    events=[dict(t=round(e["t"], 2), type=e["type"], side=e["side"], value=round(e["value"], 1))
                            for e in self.events])

    def close(self):
        if self.renderer:
            self.renderer.close()
