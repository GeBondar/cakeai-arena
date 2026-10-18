"""Python port of CakeAI-V1 by Vlad Kolmogorov (github.com/Vlad326/CakeAI-V1, Apache-2.0).

The C++17/OpenCV logic is ported line by line; time comes from the simulator
and the Arduino is replaced by an emulator of its serial protocol.
"""
from .pipeline import CakeAIPipeline, ArduinoLink, load_config  # noqa: F401
