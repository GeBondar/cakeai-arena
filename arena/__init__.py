"""CakeAI Arena — MuJoCo simulator for 1.5 kg autonomous combat robots."""
import os
import sys

# Headless Linux (CI, servers): pick an offscreen GL backend before mujoco is imported.
if sys.platform.startswith("linux") and not os.environ.get("DISPLAY") and "MUJOCO_GL" not in os.environ:
    os.environ["MUJOCO_GL"] = "egl"
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

__version__ = "1.0.0"
