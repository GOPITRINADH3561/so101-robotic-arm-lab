"""
SO-101 Robotic Arm Lab
======================

A from-scratch robotics library for the SO-101 arm,
built layer by layer for learning and experimentation.

Layers:
    1. servo.py    — Direct servo communication
    2. arm.py      — Calibration, safety & auto-detection
    3. teleop.py   — Teleoperation (coming soon)
    4. recorder.py — Record & playback (coming soon)
"""

from .servo import ServoController, ArmSnapshot, ServoStatus
from .arm import (
    ArmController,
    CalibrationData,
    JointLimits,
    find_port,
    find_all_ports,
    get_port,
    auto_connect,
    auto_connect_pair,
)

__all__ = [
    "ServoController",
    "ArmSnapshot",
    "ServoStatus",
    "ArmController",
    "CalibrationData",
    "JointLimits",
    "find_port",
    "find_all_ports",
    "get_port",
    "auto_connect",
    "auto_connect_pair",
]
