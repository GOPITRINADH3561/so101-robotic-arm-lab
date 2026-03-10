"""
ArmController — Layer 2 of the SO-101 Arm Lab
===============================================

Layer 1 (servo.py) gave us raw communication: read bytes, write bytes.
Layer 2 makes the arm AWARE OF ITSELF:
  - Auto-detects which ports the arms are on (no more typing COM6!)
  - Knows its joint limits (won't crash into itself)
  - Calibrates itself (learns its range of motion)
  - Safety system (temperature watchdog, position limits, emergency stop)
  - Homing (returns to a known starting position every time)

Think of Layer 1 as the nervous system — raw signals.
Layer 2 is proprioception — the arm KNOWS where its body is.

Why Calibration Matters
-----------------------
Every servo is slightly different. When you tell servo 3 "go to position
2048," it doesn't mean exactly the same physical angle on every arm.
Calibration creates a mapping:

    raw position (0-4095) → actual angle (degrees) → real-world position

Without calibration, "position 2048" might be slightly different between
two arms. With calibration, "90 degrees" means exactly 90 degrees on
every arm. This is critical when you want to:
  - Record a demonstration on one arm and replay on another
  - Train a neural network that generalizes across arms
  - Describe positions in meaningful units (degrees, not raw ticks)

Auto-Detection
--------------
On Windows, USB-serial adapters get assigned COM3, COM5, COM6, etc.
On Linux/Jetson, they show up as /dev/ttyUSB0, /dev/ttyUSB1, etc.
These assignments can change every time you plug/unplug!

Auto-detection scans all available serial ports, tries to talk to
STS3215 servos on each one, and figures out which port has your
leader arm and which has your follower arm — automatically.

Author: Gopi Trinadh
Project: SO-101 Robotic Arm Lab
"""

import json
import os
import time
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import scservo_sdk as scs

try:
    from .servo import (
        ServoController, JOINT_NAMES, JOINT_TO_ID, ID_TO_JOINT,
        ADDR_PRESENT_POSITION, ADDR_PRESENT_TEMPERATURE,
        ADDR_MIN_POSITION, ADDR_MAX_POSITION,
        POSITION_MIN, POSITION_MAX, POSITION_CENTER,
        DEFAULT_BAUDRATE, SPEED_SLOW, SPEED_MEDIUM, ACCEL_GENTLE, ACCEL_MEDIUM,
    )
except ImportError:
    from servo import (
        ServoController, JOINT_NAMES, JOINT_TO_ID, ID_TO_JOINT,
        ADDR_PRESENT_POSITION, ADDR_PRESENT_TEMPERATURE,
        ADDR_MIN_POSITION, ADDR_MAX_POSITION,
        POSITION_MIN, POSITION_MAX, POSITION_CENTER,
        DEFAULT_BAUDRATE, SPEED_SLOW, SPEED_MEDIUM, ACCEL_GENTLE, ACCEL_MEDIUM,
    )


# ─── Port Detection (Unplug Method) ─────────────────────────────────
#
# The best way to identify which port is which:
#   1. List all ports with cables plugged in
#   2. Ask user to UNPLUG one specific cable
#   3. List ports again
#   4. The port that DISAPPEARED = that's the one!
#
# This is foolproof. No guessing, no pinging. The same method
# LeRobot uses. Works on Windows, Linux, Mac, Jetson — everywhere.
#

def _list_serial_ports() -> set[str]:
    """Get the set of currently available serial port names."""
    import serial.tools.list_ports
    return {p.device for p in serial.tools.list_ports.comports()}


def _list_serial_ports_detailed() -> dict[str, str]:
    """Get port names and descriptions."""
    import serial.tools.list_ports
    return {p.device: (p.description or "Unknown") for p in serial.tools.list_ports.comports()}


def find_port(device_name: str = "arm") -> str:
    """
    Identify a serial port by the unplug method.

    How it works:
      1. Shows all current serial ports
      2. Asks you to UNPLUG the USB cable for the device
      3. Detects which port disappeared
      4. Asks you to plug it back in
      5. Returns the port name

    Args:
        device_name: Human-readable name (e.g. "follower arm", "leader arm")

    Returns:
        Port name string (e.g. "COM6" or "/dev/ttyUSB0")

    Example:
        >>> port = find_port("follower arm")
        Found: COM6
        >>> port = find_port("leader arm")
        Found: COM5
    """
    print()
    print("=" * 50)
    print(f"  PORT DETECTION — {device_name}")
    print("=" * 50)

    # Step 1: List all ports with everything plugged in
    print("\nMake sure ALL cables are plugged in.")
    input("Press ENTER when ready...")

    ports_before = _list_serial_ports()
    details = _list_serial_ports_detailed()

    if not ports_before:
        print("No serial ports found at all! Check your USB connections.")
        return ""

    print(f"\nCurrently connected ports:")
    for port in sorted(ports_before):
        desc = details.get(port, "")
        print(f"  {port} — {desc}")

    # Step 2: Unplug the specific device
    print(f"\nNow UNPLUG the USB cable for the {device_name}.")
    input("Press ENTER after unplugging...")

    ports_after = _list_serial_ports()

    # Step 3: Find what disappeared
    disappeared = ports_before - ports_after

    if not disappeared:
        print("No port disappeared! Did you unplug the right cable?")
        print("Let's try again...")
        return find_port(device_name)

    if len(disappeared) > 1:
        print(f"Multiple ports disappeared: {disappeared}")
        print("Please only unplug ONE cable at a time.")
        return find_port(device_name)

    found_port = disappeared.pop()
    print(f"\n  Detected: {device_name} → {found_port}")

    # Step 4: Plug it back in
    print(f"\nNow PLUG the {device_name} cable back in.")
    input("Press ENTER after plugging back in...")

    # Verify it came back
    time.sleep(1)  # Give Windows a moment to re-enumerate
    ports_final = _list_serial_ports()
    if found_port in ports_final:
        print(f"  Confirmed: {found_port} is back online")
    else:
        print(f"  WARNING: {found_port} didn't come back. Check the cable.")

    print(f"\n  {device_name} = {found_port}")
    return found_port


def find_all_ports() -> dict[str, str]:
    """
    Identify both leader and follower ports using the unplug method.

    Guides you through unplugging each arm one at a time.

    Returns:
        Dict like {"leader": "COM5", "follower": "COM6"}

    Example:
        >>> ports = find_all_ports()
        >>> print(ports)
        {"leader": "COM5", "follower": "COM6"}
    """
    print()
    print("=" * 50)
    print("  SO-101 PORT SETUP")
    print("  We'll identify each arm by unplugging cables")
    print("=" * 50)

    # Find follower first
    follower_port = find_port("FOLLOWER arm")

    # Then leader
    leader_port = find_port("LEADER arm")

    # Summary
    print()
    print("=" * 50)
    print("  PORT ASSIGNMENT COMPLETE")
    print(f"  Leader:   {leader_port}")
    print(f"  Follower: {follower_port}")
    print("=" * 50)

    result = {
        "leader": leader_port,
        "follower": follower_port,
    }

    # Save for next time
    _save_port_config(result)

    return result


def _save_port_config(ports: dict[str, str], filepath: str = ".ports.json") -> None:
    """Save detected ports so you don't have to unplug every time."""
    with open(filepath, "w") as f:
        json.dump(ports, f, indent=2)
    print(f"\nPort config saved to {filepath}")
    print("Next time, these ports will be loaded automatically.")
    print("Run with --find-ports to re-detect.")


def _load_port_config(filepath: str = ".ports.json") -> dict[str, str] | None:
    """Load previously saved port config."""
    path = Path(filepath)
    if not path.exists():
        return None
    with open(path) as f:
        config = json.load(f)
    return config


def get_port(role: str = "follower", force_detect: bool = False) -> str:
    """
    Get port for a specific role — loads saved config or runs detection.

    This is the main function you'll use. It:
      1. Checks for a saved .ports.json file
      2. If found, uses the saved port
      3. If not found (or force_detect), runs the unplug detection

    Args:
        role: "leader" or "follower"
        force_detect: If True, always run detection even if saved

    Returns:
        Port name string

    Example:
        >>> port = get_port("follower")         # uses saved or detects
        >>> port = get_port("leader", force_detect=True)  # always detects
    """
    if not force_detect:
        config = _load_port_config()
        if config and role in config:
            port = config[role]
            print(f"Using saved port for {role}: {port}")
            # Verify it still exists
            current_ports = _list_serial_ports()
            if port in current_ports:
                return port
            else:
                print(f"  Saved port {port} not found! Running detection...")

    return find_port(f"{role.upper()} arm")


def auto_connect(role: str = "follower", force_detect: bool = False) -> ServoController:
    """
    Get port and connect to an SO-101 arm.

    First time: runs unplug detection and saves the result.
    Next times: loads saved port config automatically.

    Args:
        role: "leader" or "follower"
        force_detect: Force re-detection

    Returns:
        Connected ServoController instance

    Example:
        >>> arm = auto_connect()                    # auto
        >>> arm = auto_connect("leader")            # leader
        >>> arm = auto_connect(force_detect=True)   # re-detect
    """
    port = get_port(role, force_detect)
    ctrl = ServoController(port)
    ctrl.connect()
    return ctrl


def auto_connect_pair(force_detect: bool = False) -> tuple[ServoController, ServoController]:
    """
    Get ports and connect both leader and follower arms.

    First time: runs unplug detection for both arms.
    Next times: loads saved port config automatically.

    Returns:
        Tuple of (leader, follower) ServoControllers

    Example:
        >>> leader, follower = auto_connect_pair()
    """
    if not force_detect:
        config = _load_port_config()
        if config and "leader" in config and "follower" in config:
            current_ports = _list_serial_ports()
            if config["leader"] in current_ports and config["follower"] in current_ports:
                print(f"Using saved ports — Leader: {config['leader']}, Follower: {config['follower']}")
                leader = ServoController(config["leader"])
                follower = ServoController(config["follower"])
                leader.connect()
                follower.connect()
                return leader, follower
            else:
                print("Saved ports not all available. Running detection...")

    ports = find_all_ports()
    leader = ServoController(ports["leader"])
    follower = ServoController(ports["follower"])
    leader.connect()
    follower.connect()
    return leader, follower


# ─── Joint Limits & Safety ──────────────────────────────────────────
#
# The SO-101 arm can physically destroy itself if you send it to
# certain positions — the arm crashes into its own base, cables
# get yanked, gears strip. Joint limits prevent this.
#
# There are TWO kinds of limits:
#   1. Hardware limits — set inside the servo's memory (enforced by servo)
#   2. Software limits — checked by our code before sending commands
#
# We use BOTH for defense in depth. Even if our code has a bug,
# the hardware limits act as a safety net.
#

@dataclass
class JointLimits:
    """Position limits for a single joint."""
    min_position: int     # Minimum safe position (0-4095)
    max_position: int     # Maximum safe position (0-4095)
    home_position: int    # Default/home position

    def clamp(self, position: int) -> int:
        """Clamp a position to within safe limits."""
        return max(self.min_position, min(self.max_position, position))

    def is_safe(self, position: int) -> bool:
        """Check if a position is within safe limits."""
        return self.min_position <= position <= self.max_position

    @property
    def range(self) -> int:
        """Total range of motion in raw units."""
        return self.max_position - self.min_position

    @property
    def center(self) -> int:
        """Center of the range of motion."""
        return (self.min_position + self.max_position) // 2


# Default safe limits for the SO-101
# These are conservative — you can expand them after calibration
DEFAULT_LIMITS = {
    "shoulder_pan":  JointLimits(min_position=800,  max_position=3200, home_position=2048),
    "shoulder_lift": JointLimits(min_position=800,  max_position=3100, home_position=2048),
    "elbow_flex":    JointLimits(min_position=800,  max_position=3100, home_position=2048),
    "wrist_flex":    JointLimits(min_position=800,  max_position=3000, home_position=2048),
    "wrist_roll":    JointLimits(min_position=0,    max_position=4095, home_position=2048),
    "gripper":       JointLimits(min_position=2000, max_position=3400, home_position=2048),
}


# ─── Calibration Data ───────────────────────────────────────────────

@dataclass
class CalibrationData:
    """
    Stores calibration results for one arm.

    Calibration maps raw servo positions to meaningful angles.
    It also records the exact range of motion for each joint,
    which varies slightly between individual arms.
    """
    arm_id: str                           # Unique name for this arm
    timestamp: float = 0.0                # When calibration was done
    limits: dict[str, JointLimits] = field(default_factory=dict)
    homing_offsets: dict[str, int] = field(default_factory=dict)
    notes: str = ""

    def save(self, directory: str = ".calibration") -> str:
        """
        Save calibration to a JSON file.

        Args:
            directory: Folder to save in (created if needed)

        Returns:
            Path to the saved file
        """
        path = Path(directory)
        path.mkdir(parents=True, exist_ok=True)

        filepath = path / f"{self.arm_id}.json"
        data = {
            "arm_id": self.arm_id,
            "timestamp": self.timestamp,
            "notes": self.notes,
            "limits": {
                name: {
                    "min_position": lim.min_position,
                    "max_position": lim.max_position,
                    "home_position": lim.home_position,
                }
                for name, lim in self.limits.items()
            },
            "homing_offsets": self.homing_offsets,
        }

        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)

        print(f"Calibration saved to {filepath}")
        return str(filepath)

    @classmethod
    def load(cls, filepath: str) -> "CalibrationData":
        """
        Load calibration from a JSON file.

        Args:
            filepath: Path to the calibration JSON file

        Returns:
            CalibrationData instance
        """
        with open(filepath) as f:
            data = json.load(f)

        cal = cls(
            arm_id=data["arm_id"],
            timestamp=data.get("timestamp", 0),
            notes=data.get("notes", ""),
            homing_offsets=data.get("homing_offsets", {}),
        )

        for name, lim_data in data.get("limits", {}).items():
            cal.limits[name] = JointLimits(
                min_position=lim_data["min_position"],
                max_position=lim_data["max_position"],
                home_position=lim_data["home_position"],
            )

        print(f"Calibration loaded from {filepath}")
        return cal


# ─── ArmController ──────────────────────────────────────────────────

class ArmController:
    """
    High-level arm controller with safety, calibration, and intelligence.

    Builds on top of ServoController (Layer 1) to add:
      - Joint limit enforcement (won't let you crash the arm)
      - Calibration (knows its exact range of motion)
      - Temperature monitoring (auto-stops if overheating)
      - Homing (repeatable starting position)
      - Safe movement (checks limits before every move)

    Example:
        >>> arm = ArmController.create("follower")  # auto-detect!
        >>> arm.home()
        >>> arm.safe_move("shoulder_pan", 1500)
        >>> arm.safe_move_all([1500, 2048, 2048, 2048, 2048, 2400])
        >>> arm.disconnect()
    """

    def __init__(
        self,
        servo: ServoController,
        arm_id: str = "so101",
        calibration: CalibrationData | None = None,
    ):
        """
        Initialize with an already-connected ServoController.

        Args:
            servo: Connected ServoController from Layer 1
            arm_id: Unique name for this arm (for calibration files)
            calibration: Pre-loaded calibration, or None to use defaults
        """
        self.servo = servo
        self.arm_id = arm_id
        self._limits = dict(DEFAULT_LIMITS)  # Start with defaults
        self._temp_limit = 55  # °C — shutdown if exceeded
        self._enabled = False

        # Load calibration if provided
        if calibration:
            self._limits = calibration.limits
            print(f"Loaded calibration for '{calibration.arm_id}'")

        # Try to load saved calibration
        elif self._has_saved_calibration():
            self.load_calibration()

    # ── Factory Methods (Easy Creation) ──────────────────────────────

    @classmethod
    def create(cls, role: str = "follower", arm_id: str | None = None) -> "ArmController":
        """
        Auto-detect port and create an ArmController.

        This is the easiest way to get started:
            arm = ArmController.create()  # That's it!

        Args:
            role: "leader" or "follower"
            arm_id: Name for this arm (default: role name)

        Returns:
            Ready-to-use ArmController
        """
        servo = auto_connect(role)
        arm_id = arm_id or role
        return cls(servo, arm_id)

    @classmethod
    def create_pair(cls) -> tuple["ArmController", "ArmController"]:
        """
        Auto-detect and create both leader and follower.

        Returns:
            Tuple of (leader, follower) ArmControllers

        Example:
            >>> leader, follower = ArmController.create_pair()
        """
        leader_servo, follower_servo = auto_connect_pair()
        leader = cls(leader_servo, "leader")
        follower = cls(follower_servo, "follower")
        return leader, follower

    @classmethod
    def from_port(cls, port: str, arm_id: str = "so101") -> "ArmController":
        """
        Create from a specific port (when you know the port).

        Example:
            >>> arm = ArmController.from_port("COM6", "follower")
        """
        servo = ServoController(port)
        servo.connect()
        return cls(servo, arm_id)

    # ── Safe Movement ────────────────────────────────────────────────
    #
    # These methods CHECK LIMITS before moving. If you ask it to go
    # somewhere dangerous, it clamps to the safe range and warns you.
    #

    def safe_move(self, joint: str | int, position: int) -> int:
        """
        Move a joint to a position, enforcing safety limits.

        If the requested position is outside limits, it gets clamped
        to the nearest safe value and a warning is printed.

        Args:
            joint: Joint name or servo ID
            position: Desired position (0-4095)

        Returns:
            The actual position sent (may differ from requested)
        """
        name = self._resolve_name(joint)
        limits = self._limits[name]

        if not limits.is_safe(position):
            clamped = limits.clamp(position)
            print(f"WARNING: {name} position {position} outside limits "
                  f"[{limits.min_position}, {limits.max_position}] → clamped to {clamped}")
            position = clamped

        self.servo.move(name, position)
        return position

    def safe_move_all(self, positions: list[int] | dict[str, int]) -> list[int]:
        """
        Move all joints with safety limit enforcement.

        Args:
            positions: List of 6 positions or dict of {joint: position}

        Returns:
            List of actual positions sent (after clamping)
        """
        if isinstance(positions, dict):
            actual = {}
            for name, pos in positions.items():
                actual[name] = self.safe_move(name, pos)
            return [actual.get(n, self.servo.read_position(n)) for n in JOINT_NAMES]
        else:
            actual = []
            for i, pos in enumerate(positions):
                name = JOINT_NAMES[i]
                actual.append(self.safe_move(name, pos))
            return actual

    def safe_smooth_move(
        self, target: list[int], duration: float = 2.0, steps: int = 40
    ) -> None:
        """Smooth move with safety limit enforcement."""
        # Clamp target first
        clamped = []
        for i, pos in enumerate(target):
            name = JOINT_NAMES[i]
            clamped.append(self._limits[name].clamp(pos))
        self.servo.smooth_move(clamped, duration, steps)

    # ── Homing ───────────────────────────────────────────────────────

    def home(self, duration: float = 3.0) -> None:
        """
        Move all joints to their home positions.

        Home positions are defined in the calibration/limits.
        This gives you a known, repeatable starting position.
        """
        home_positions = [
            self._limits[name].home_position for name in JOINT_NAMES
        ]
        print("Homing...")
        self.servo.enable_torque()
        self.servo.smooth_move(home_positions, duration=duration)
        self._enabled = True
        print("Home position reached")

    def get_home_positions(self) -> dict[str, int]:
        """Get the home position for each joint."""
        return {
            name: self._limits[name].home_position
            for name in JOINT_NAMES
        }

    # ── Calibration ──────────────────────────────────────────────────
    #
    # Calibration is an interactive process:
    #   1. Disable torque so you can move the arm by hand
    #   2. Move each joint to its minimum position → record
    #   3. Move each joint to its maximum position → record
    #   4. Move to home position → record
    #   5. Save the calibration file
    #
    # This only needs to be done ONCE per arm, unless you change
    # the physical setup.
    #

    def calibrate(self) -> CalibrationData:
        """
        Interactive calibration — guides you through the process.

        Disable torque, manually move each joint to its limits,
        and the system records the positions.

        Returns:
            CalibrationData with the measured limits
        """
        print()
        print("=" * 55)
        print("  SO-101 ARM CALIBRATION")
        print(f"  Arm ID: {self.arm_id}")
        print("=" * 55)
        print()
        print("This will guide you through calibrating each joint.")
        print("Torque will be disabled so you can move the arm freely.")
        print()

        self.servo.disable_torque()
        time.sleep(0.5)

        calibration = CalibrationData(
            arm_id=self.arm_id,
            timestamp=time.time(),
        )

        for name in JOINT_NAMES:
            print(f"\n--- Calibrating: {name.upper()} ---")

            # Step 1: Move to MINIMUM
            input(f"  Move {name} to its MINIMUM position, then press ENTER...")
            min_pos = self.servo.read_position(name)
            print(f"  Recorded MIN: {min_pos}")

            # Step 2: Move to MAXIMUM
            input(f"  Move {name} to its MAXIMUM position, then press ENTER...")
            max_pos = self.servo.read_position(name)
            print(f"  Recorded MAX: {max_pos}")

            # Ensure min < max
            if min_pos > max_pos:
                min_pos, max_pos = max_pos, min_pos
                print(f"  (Swapped min/max → MIN={min_pos}, MAX={max_pos})")

            # Step 3: Move to HOME
            input(f"  Move {name} to its HOME/center position, then press ENTER...")
            home_pos = self.servo.read_position(name)
            print(f"  Recorded HOME: {home_pos}")

            # Add a safety margin (5% on each side)
            margin = int((max_pos - min_pos) * 0.05)
            safe_min = min_pos + margin
            safe_max = max_pos - margin

            calibration.limits[name] = JointLimits(
                min_position=safe_min,
                max_position=safe_max,
                home_position=home_pos,
            )

            print(f"  Safe range: [{safe_min}, {safe_max}] "
                  f"(with {margin} unit safety margin)")

        # Summary
        print()
        print("=" * 55)
        print("  CALIBRATION SUMMARY")
        print("=" * 55)
        print(f"  {'Joint':<20} {'Min':>6} {'Home':>6} {'Max':>6} {'Range':>6}")
        print("  " + "-" * 50)
        for name in JOINT_NAMES:
            lim = calibration.limits[name]
            print(f"  {name:<20} {lim.min_position:>6} {lim.home_position:>6} "
                  f"{lim.max_position:>6} {lim.range:>6}")

        # Save
        save = input("\nSave calibration? (y/n): ")
        if save.lower() == "y":
            filepath = calibration.save()
            self._limits = calibration.limits
            print(f"Calibration active and saved to {filepath}")
        else:
            apply_it = input("Apply without saving? (y/n): ")
            if apply_it.lower() == "y":
                self._limits = calibration.limits
                print("Calibration applied (not saved)")

        return calibration

    def quick_calibrate(self) -> CalibrationData:
        """
        Quick calibration — move all joints through their full range.

        Instead of calibrating one joint at a time, you move ALL joints
        through their entire range while the system records min/max.
        Faster but less precise than full calibration.

        Returns:
            CalibrationData with the measured limits
        """
        print()
        print("=" * 55)
        print("  QUICK CALIBRATION")
        print("=" * 55)
        print()
        print("Torque disabled. Move ALL joints through their")
        print("entire range of motion. The system will track")
        print("the minimum and maximum for each joint.")
        print()

        self.servo.disable_torque()
        time.sleep(0.5)

        # Set home position first
        input("Move arm to HOME position and press ENTER...")
        home_positions = self.servo.read_all_positions()

        # Track min/max
        min_positions = dict(home_positions)
        max_positions = dict(home_positions)

        input("Now move ALL joints through their full range. Press ENTER when done...")
        print("Recording... (press ENTER to stop)")
        print()

        recording = True
        samples = 0
        try:
            while recording:
                positions = self.servo.sync_read_positions()
                for name, pos in positions.items():
                    min_positions[name] = min(min_positions[name], pos)
                    max_positions[name] = max(max_positions[name], pos)
                samples += 1

                # Print live
                sys.stdout.write(f"\r  Samples: {samples}  |  ")
                for name in JOINT_NAMES:
                    sys.stdout.write(
                        f"{name[:3]}:[{min_positions[name]}-{max_positions[name]}] "
                    )
                sys.stdout.flush()
                time.sleep(0.05)

                # Check for Enter key (non-blocking)
                if sys.stdin in _select_stdin():
                    sys.stdin.readline()
                    recording = False
        except KeyboardInterrupt:
            pass

        print(f"\n\nRecorded {samples} samples")

        # Build calibration with safety margins
        calibration = CalibrationData(
            arm_id=self.arm_id,
            timestamp=time.time(),
            notes=f"Quick calibration — {samples} samples",
        )

        for name in JOINT_NAMES:
            total_range = max_positions[name] - min_positions[name]
            margin = int(total_range * 0.05)
            calibration.limits[name] = JointLimits(
                min_position=min_positions[name] + margin,
                max_position=max_positions[name] - margin,
                home_position=home_positions[name],
            )

        # Summary
        print(f"\n  {'Joint':<20} {'Min':>6} {'Home':>6} {'Max':>6} {'Range':>6}")
        print("  " + "-" * 50)
        for name in JOINT_NAMES:
            lim = calibration.limits[name]
            print(f"  {name:<20} {lim.min_position:>6} {lim.home_position:>6} "
                  f"{lim.max_position:>6} {lim.range:>6}")

        save = input("\nSave calibration? (y/n): ")
        if save.lower() == "y":
            calibration.save()
            self._limits = calibration.limits

        return calibration

    # ── Temperature Safety ───────────────────────────────────────────

    def check_temperatures(self) -> dict[str, int]:
        """
        Read temperature of all servos and warn if hot.

        Returns:
            Dict of {joint_name: temperature_celsius}
        """
        temps = {}
        for name in JOINT_NAMES:
            temp = self.servo.read_temperature(name)
            temps[name] = temp

            if temp >= self._temp_limit:
                print(f"DANGER: {name} at {temp}°C — DISABLING TORQUE!")
                self.emergency_stop()
                return temps
            elif temp >= self._temp_limit - 10:
                print(f"WARNING: {name} at {temp}°C — getting warm")

        return temps

    def print_temperatures(self) -> None:
        """Display a nice temperature readout."""
        temps = self.check_temperatures()
        print("\n  Temperature Report:")
        for name in JOINT_NAMES:
            temp = temps[name]
            bar_len = min(temp, 60)
            if temp >= self._temp_limit:
                symbol = "🔴"
            elif temp >= self._temp_limit - 10:
                symbol = "🟡"
            else:
                symbol = "🟢"
            bar = "█" * (bar_len // 3) + "░" * ((60 - bar_len) // 3)
            print(f"  {symbol} {name:>15}: {bar} {temp}°C")

    # ── Emergency Stop ───────────────────────────────────────────────

    def emergency_stop(self) -> None:
        """
        IMMEDIATELY disable all torque.

        Use this if something goes wrong — the arm goes limp instantly.
        """
        print("!!! EMERGENCY STOP !!!")
        for sid in range(1, 7):
            try:
                self.servo._packet_handler.write1ByteTxRx(
                    self.servo._port_handler, sid, 40, 0  # Torque off
                )
            except Exception:
                pass  # Best effort — don't let one failure stop the rest
        self._enabled = False
        print("All torque disabled")

    # ── Calibration File Management ──────────────────────────────────

    def _has_saved_calibration(self) -> bool:
        """Check if a calibration file exists for this arm."""
        filepath = Path(".calibration") / f"{self.arm_id}.json"
        return filepath.exists()

    def load_calibration(self, filepath: str | None = None) -> None:
        """Load calibration from file."""
        if filepath is None:
            filepath = str(Path(".calibration") / f"{self.arm_id}.json")

        if not Path(filepath).exists():
            print(f"No calibration file found at {filepath}")
            return

        cal = CalibrationData.load(filepath)
        self._limits = cal.limits
        print(f"Calibration loaded for '{cal.arm_id}'")

    @property
    def limits(self) -> dict[str, JointLimits]:
        """Current joint limits."""
        return dict(self._limits)

    def print_limits(self) -> None:
        """Display current joint limits."""
        print(f"\n  Joint Limits ({self.arm_id}):")
        print(f"  {'Joint':<20} {'Min':>6} {'Home':>6} {'Max':>6} {'Range':>6}")
        print("  " + "-" * 50)
        for name in JOINT_NAMES:
            lim = self._limits[name]
            print(f"  {name:<20} {lim.min_position:>6} {lim.home_position:>6} "
                  f"{lim.max_position:>6} {lim.range:>6}")

    # ── Status ───────────────────────────────────────────────────────

    def print_status(self) -> None:
        """Print a comprehensive status report."""
        print()
        print("=" * 55)
        print(f"  SO-101 Arm Status — {self.arm_id}")
        print(f"  Port: {self.servo.port}")
        print("=" * 55)

        # Positions
        print("\n  Positions:")
        positions = self.servo.sync_read_positions()
        for name in JOINT_NAMES:
            pos = positions[name]
            lim = self._limits[name]
            angle = (pos / 4095) * 360
            pct = ((pos - lim.min_position) / max(1, lim.range)) * 100
            pct = max(0, min(100, pct))
            bar = "█" * int(pct / 5) + "░" * (20 - int(pct / 5))
            print(f"  {name:>15}: {bar} {pos:>4} ({angle:.1f}°) [{pct:.0f}%]")

        # Temperatures
        self.print_temperatures()

        # Limits
        self.print_limits()

    # ── Connection ───────────────────────────────────────────────────

    def disconnect(self) -> None:
        """Safely disconnect — disable torque first."""
        if self._enabled:
            self.servo.disable_torque()
        self.servo.disconnect()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.disconnect()

    def __repr__(self):
        return (
            f"ArmController(id='{self.arm_id}', "
            f"port='{self.servo.port}')"
        )


# ─── Helper ─────────────────────────────────────────────────────────

def _select_stdin():
    """Non-blocking check if stdin has data (cross-platform best effort)."""
    try:
        import select
        ready, _, _ = select.select([sys.stdin], [], [], 0)
        return ready
    except Exception:
        return []


def _resolve_name(self, joint: str | int) -> str:
    """Convert joint name or ID to name string."""
    if isinstance(joint, int):
        return ID_TO_JOINT[joint]
    return joint

# Monkey-patch onto ArmController
ArmController._resolve_name = _resolve_name


# ─── Quick Test ──────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="SO-101 Arm Controller")
    parser.add_argument("--port", type=str, default=None,
                       help="Serial port (uses saved config or detects if not specified)")
    parser.add_argument(
        "--action",
        type=str,
        default="status",
        choices=["find-ports", "status", "calibrate", "quick-cal", "home", "temps"],
        help="What to do",
    )
    parser.add_argument("--id", type=str, default="follower", help="Arm ID / role")
    args = parser.parse_args()

    # ── Find ports (unplug method) ───────────────────────────────────
    if args.action == "find-ports":
        response = input("Detect [1] single arm or [2] both arms? (1/2): ")
        if response.strip() == "2":
            ports = find_all_ports()
        else:
            port = find_port(f"{args.id.upper()} arm")
            # Save it
            config = _load_port_config() or {}
            config[args.id] = port
            _save_port_config(config)
        sys.exit(0)

    # ── Create arm ───────────────────────────────────────────────────
    if args.port:
        arm = ArmController.from_port(args.port, args.id)
    else:
        arm = ArmController.create(role=args.id, arm_id=args.id)

    try:
        if args.action == "status":
            arm.print_status()

        elif args.action == "calibrate":
            arm.calibrate()

        elif args.action == "quick-cal":
            arm.quick_calibrate()

        elif args.action == "home":
            arm.home()
            time.sleep(2)

        elif args.action == "temps":
            arm.print_temperatures()

    finally:
        arm.disconnect()
