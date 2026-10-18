"""Opponent roster and scripted "RC pilots".

Tactics are the bench_core.opp_desired behaviours from GeBondar/battlebots_docs
re-scaled from a 12 m to a 2 m arena, driven like a human: 150 ms reaction
time, hand noise (Ornstein-Uhlenbeck), back-off after a long clinch.
"""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Opponent:
    key: str
    robot: str           # CAD model in cad/robots_1p5kg
    name: str            # HUD name
    tactic: str
    duty: float          # fraction of full throttle the pilot uses
    blurb: str


ROSTER = {o.key: o for o in [
    Opponent("vspin", "vspin", "RAZOR", "charge", 0.75, "vertical disc spinner, 10 000 rpm"),
    Opponent("hspin", "hspin", "TORNADO", "charge", 0.70, "horizontal bar spinner, 6 000 rpm"),
    Opponent("wedge", "wedge", "DOORSTOP", "charge", 0.85, "wedge pusher"),
    Opponent("flipper", "flipper", "CATAPULT", "charge", 0.80, "electric flipper, 9 N·m"),
    Opponent("wedge-juke", "wedge", "DOORSTOP-J", "juke", 0.80, "wedge, side-steps on approach"),
    Opponent("vspin-orbit", "vspin", "RAZOR-O", "circle", 0.70, "vertical spinner, circles the target"),
    Opponent("wedge-zigzag", "wedge", "DOORSTOP-Z", "zigzag", 0.90, "wedge, zig-zag approach"),
]}
TACTICS = ("charge", "juke", "circle", "zigzag", "flee", "idle")


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def desired_heading(tactic, ox, oy, x, y, t):
    """Heading the pilot wants (opponent at ox,oy; CakeAI at x,y)."""
    dist = math.hypot(ox - x, oy - y)
    to_target = math.atan2(y - oy, x - ox)
    flee = to_target + math.pi
    if tactic == "charge":
        return to_target
    if tactic == "juke":
        return to_target + (math.pi / 2 if dist < 0.45 else 0.0) * (1 if math.sin(t) > 0 else -1)
    if tactic == "circle":
        des = flee + math.pi / 2 if dist > 0.35 else to_target
    elif tactic == "zigzag":
        side = 1.0 if math.sin(2 * math.pi * t / 1.4) >= 0 else -1.0
        des = to_target + side * math.radians(55) if dist > 0.3 else to_target
    else:                                            # flee
        des = flee
    if abs(ox) > 0.82 or abs(oy) > 0.82:             # keep off the walls
        des = math.atan2(-oy, -ox)
    return des


class Pilot:
    """Turns a tactic into left/right duty + weapon command for one robot."""

    def __init__(self, opp: Opponent, weapon_kind, rng, fps):
        self.o, self.kind, self.rng, self.dt = opp, weapon_kind, rng, 1.0 / fps
        self.noise = 0.0
        self.clinch = 0.0
        self.backoff_until = -1.0
        self.backoff_turn = 0.0
        self.spin = 0.0
        self.fire_until = -1.0
        self.cooldown_until = -1.0

    def drive(self, t, me, target, in_contact):
        """me/target = (x, y, yaw, upright). Returns (left, right)."""
        ox, oy, oyaw, up = me
        self.clinch = self.clinch + self.dt if in_contact else max(0.0, self.clinch - 2 * self.dt)
        if up < 0.3 or self.o.tactic == "idle":
            return 0.0, 0.0
        if self.clinch > 1.2:                        # a human backs off and re-attacks
            self.backoff_until = t + 0.6
            self.backoff_turn = 0.35 * (1 if self.rng.random() > 0.5 else -1)
            self.clinch = 0.0
        if t < self.backoff_until:
            return -0.75 - self.backoff_turn, -0.75 + self.backoff_turn
        des = desired_heading(self.o.tactic, ox, oy, target[0], target[1], t)
        self.noise += -self.noise * 2.0 * self.dt + 0.9 * math.sqrt(self.dt) * self.rng.normal()
        err = wrap(des + 0.35 * self.noise - oyaw)
        turn = max(-1.0, min(1.0, 1.6 * err))
        fwd = self.o.duty * max(0.0, math.cos(err)) ** 2
        l, r = fwd - 0.55 * turn, fwd + 0.55 * turn
        k = max(1.0, abs(l), abs(r))
        return l / k, r / k

    def weapon(self, t, me, target, plate_touch, flip_angle):
        """Spinners: spin-up ramp. Flipper: fire on touch / target in the scoop, self-right when upside down."""
        up = me[3]
        if self.kind != "flipper":
            self.spin = min(1.0, self.spin + self.dt / 1.5)
            return self.spin if up > 0.3 else 0.0
        if t < self.fire_until:
            return flip_angle
        if t >= self.cooldown_until:
            dx, dy = target[0] - me[0], target[1] - me[1]
            dist = math.hypot(dx, dy)
            bearing = abs(wrap(math.atan2(dy, dx) - me[2]))
            if up < -0.2 or plate_touch or (dist < 0.20 and bearing < math.radians(30)):
                self.fire_until = t + 0.25
                self.cooldown_until = t + 0.25 + 0.7
                return flip_angle
        return 0.0
