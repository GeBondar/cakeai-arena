"""CakeAI-V1 -> Python port checks (behaviour must match the C++ original).

    python -m pytest tests -q
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cakeai.pipeline import ArduinoLink  # noqa: E402
from cakeai.robot_controller import RobotController  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


CFG = dict(Kp=0.25, Ki=0.0, Kd=0.05, KrlCoefficient=1.55, KfbCoefficient=0.4,
           CenterAfterAttack=1, CenterTriggerThreshold=150, EnemyTriggerThreshold=150,
           EnemyTriggerTimeout=2000, ReverseAngleThreshold=180, UseReverse=1,
           AngleMinExtraSpeed=90.0, AngleMaxExtraSpeed=0.0, InPlaceSpeedRotation=0.5, InPlaceSpeedWeapon=1.0)


def make():
    clk = Clock()
    link = ArduinoLink()
    rc = RobotController(clk, link)
    rc.apply_config(CFG)
    rc.reset()
    rc.setState(1)
    return clk, link, rc


def test_arduino_protocol():
    a = ArduinoLink()
    a.send_command("1:P:100")
    a.send_command("2:P:0")
    a.send_command("3:P:50")
    assert a.dac == [4000, 170, 170 + 50 * 3830 // 100]
    l, r, w = a.normalized()
    assert l == 1.0 and r == -1.0 and abs(w - 0.5) < 0.01
    a.send_command("A:P:50")
    assert a.percent == [50, 50, 50]


def test_marker_heading_convention():
    # marker facing image-right (TL->TR along +x): rotation = 90 -> heading 0 deg
    from cakeai.vision import ArucoDetector
    import numpy as np
    det = ArucoDetector()
    det.marker_ids = [0]
    det.marker_corners = [np.array([[0, 0], [10, 0], [10, 10], [0, 10]], float)]
    assert abs(det.marker_rotation(0) - 90.0) < 1e-9


def test_turns_toward_target_on_the_right():
    # robot at (400,400) faces image-right, target is lower in the image (y down) = on its right
    # -> left drive (CH1) must be faster than right (CH2)
    clk, link, rc = make()
    for _ in range(3):
        clk.t += 0.016
        rc.process(90.0, 100, 100, 400, 400, 400, 700, 0)
    assert rc.ch1 > rc.ch2


def test_straight_ahead_drives_forward():
    clk, link, rc = make()
    clk.t += 0.016
    rc.process(90.0, 100, 100, 400, 400, 700, 400, 0)
    assert rc.ch1 > 50 and rc.ch2 > 50 and abs(rc.ch1 - rc.ch2) < 1e-6
    assert link.percent[0] == int(rc.ch1)


def test_upside_down_marker_swaps_and_inverts():
    clk, _, a = make()
    _, _, b = make()
    clk.t += 0.016
    b.clock = a.clock = clk
    a.process(90.0, 100, 100, 400, 400, 400, 700, 0)
    b.process(90.0, 100, 100, 400, 400, 400, 700, 1)   # ID 1 -> upside down
    assert abs(b.ch1 - (100 - a.ch2)) < 1e-6 and abs(b.ch2 - (100 - a.ch1)) < 1e-6


def test_lost_marker_rotates_in_place_with_weapon():
    clk, _, rc = make()
    clk.t += 0.016
    rc.process(0.0, 100, 100, -1, -1, 300, 300, -1)
    assert rc.rotatingInPlace and rc.ch1 == 25 and rc.ch2 == 75 and rc.ch3 == 100
