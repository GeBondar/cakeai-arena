# Contributing

Thanks for wanting to make the arena better! Issues and pull requests are welcome.

## Setup

```bash
git clone https://github.com/GeBondar/cakeai-arena && cd cakeai-arena
pip install -e ".[dev,play]"
pytest -q
```

## Add a robot

1. **Model it** in `cad/robots_1p5kg/make_step.py` (CADAM: one CadQuery function per part).
   Keep the robot frame (X forward, Y left, Z up, origin on the wheel axle) and keep the
   part masses summing to **1.500 kg**. Weapons spin about local Y (drums, discs) or Z (bars);
   flippers are hinged about local Y.
2. Add the robot to `ROBOTS` there (parts, `weapon`, `drive`), then rebuild only it:
   `pip install cadquery && python cad/robots_1p5kg/make_step.py <name>`.
   Check `check_report.txt`: no part intersections, nothing below the floor.
3. Add an `Opponent(...)` entry to `arena/pilots.py` (`ROSTER`).
4. Run `pytest -q`. The physics tests run on every robot automatically (it settles, drives,
   and its weapon spins up or fires). Then `python -m arena fight <name>`.

## Add a pilot tactic

Add a branch to `desired_heading()` in `arena/pilots.py` and list it in `TACTICS`.

## Changing the CakeAI port

`cakeai/` is a line-by-line port of [CakeAI-V1](https://github.com/Vlad326/CakeAI-V1).
Behaviour changes belong upstream. Here, keep it faithful, and back every fix with a test in
`tests/test_cakeai_port.py` that compares against the C++ logic.
