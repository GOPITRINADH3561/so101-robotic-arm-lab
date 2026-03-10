"""
ServoController — Layer 1 of the SO-101 Arm Lab
=================================================

This module provides direct control over Feetech STS3215 servos.
It is the foundation that everything else builds on: teleoperation,
recording, playback, and eventually imitation learning.

Understanding this file means understanding how YOUR robot thinks.

Hardware Background
-------------------
The SO-101 arm uses 6 Feetech STS3215 servos, daisy-chained on a
single half-duplex UART serial bus. Each servo has:
  - A unique ID (1-6)
  - A "control table" — a block of memory addresses you read/write
  - Two 3-pin ports for daisy-chain (VCC, GND, Signal)

Communication works like this:
  1. Your PC sends a packet: "Servo 3, what's your position?"
  2. Servo 3 responds: "I'm at position 2048"
  3. All other servos ignore the packet (not their ID)

The control table is the key to everything. It's a set of memory
addresses inside each servo. Writing to address 42 moves the motor.
Reading address 56 tells you where it is. That's the essence of
robotics at the hardware level.

Control Table (STS3215 — key addresses)
---------------------------------------
Address | Name                | R/W | Description
--------|---------------------|-----|---------------------------
   5    | ID                  | R/W | Servo ID (1-253)
   6    | Baud_Rate           | R/W | Communication speed
  21    | Min_Position_Limit  | R/W | Minimum allowed position
  23    | Max_Position_Limit  | R/W | Maximum allowed position
  33    | Maximum_Acceleration| R/W | Acceleration limit
  40    | Torque_Enable       | R/W | 0=free, 1=holding
  41    | Acceleration        | R/W | Target acceleration
  42    | Goal_Position       | R/W | Where to move (0-4095)
  46    | Max_Torque_Limit    | R/W | Maximum torque output
  55    | Lock                | R/W | 0=unlocked, 1=locked
  56    | Present_Position    |  R  | Current position (0-4095)
  58    | Present_Speed       |  R  | Current speed
  60    | Present_Load        |  R  | Current load/torque
  62    | Present_Voltage     |  R  | Current voltage (in 0.1V)
  63    | Present_Temperature |  R  | Current temperature (°C)

Position values: 0-4095 representing 0°-360° (12-bit resolution)
  - 2048 = center (180°)
  - 0    = 0°
  - 4095 = ~360°

Author: Gopi Trinadh
Project: SO-101 Robotic Arm Lab
"""

import time
import math
from dataclasses import dataclass, field
from typing import Optional

import scservo_sdk as scs


# ─── Servo Configuration ────────────────────────────────────────────

# The 6 joints of the SO-101 arm, in order from base to tip
JOINT_NAMES = [
    "shoulder_pan",   # ID 1 — rotates the entire arm left/right
    "shoulder_lift",  # ID 2 — raises/lowers the upper arm
    "elbow_flex",     # ID 3 — bends the forearm
    "wrist_flex",     # ID 4 — tilts the wrist up/down
    "wrist_roll",     # ID 5 — rotates the wrist
    "gripper",        # ID 6 — opens and closes the fingers
]

# Map joint names to servo IDs
JOINT_TO_ID = {name: i + 1 for i, name in enumerate(JOINT_NAMES)}
ID_TO_JOINT = {i + 1: name for i, name in enumerate(JOINT_NAMES)}

# Key addresses in the STS3215 control table
#
# Think of this like a dictionary inside each servo's brain.
# Each address stores a different piece of information.
# Some addresses you can only READ (sensors), others you can READ + WRITE (controls).
#
ADDR_RETURN_DELAY = 5        # How long servo waits before responding (µs)
ADDR_MIN_POSITION = 21       # Software limit — minimum position
ADDR_MAX_POSITION = 23       # Software limit — maximum position
ADDR_MAX_ACCELERATION = 33   # Maximum acceleration (0-254)
ADDR_TORQUE_ENABLE = 40      # Master switch: 0=free, 1=motor active
ADDR_ACCELERATION = 41       # Current acceleration setting (0-254)
ADDR_GOAL_POSITION = 42      # Where to move (0-4095)
ADDR_GOAL_SPEED = 44         # How fast to move (0-4095, 0=max speed)
ADDR_MAX_TORQUE = 46         # Maximum torque output (0-1000)
ADDR_LOCK = 55               # Write protection: 0=unlocked, 1=locked
ADDR_PRESENT_POSITION = 56   # Where the servo IS right now (0-4095)
ADDR_PRESENT_SPEED = 58      # How fast it's currently moving
ADDR_PRESENT_LOAD = 60       # How much force it's applying
ADDR_PRESENT_VOLTAGE = 62    # Power supply voltage (in 0.1V units)
ADDR_PRESENT_TEMPERATURE = 63  # Internal temperature (°C)

# Constants
DEFAULT_BAUDRATE = 1_000_000
POSITION_MIN = 0
POSITION_MAX = 4095
POSITION_CENTER = 2048

# Speed constants
# STS3215 speed is in units of ~0.0115 RPM
# Speed = 0 means "go as fast as possible" (no speed limit)
# Speed = 1-4095 sets a specific speed, higher = faster
SPEED_MAX = 0       # No limit — full speed
SPEED_SLOW = 200    # Good for careful movements
SPEED_MEDIUM = 600  # Normal operating speed
SPEED_FAST = 1500   # Quick but controlled

# Acceleration constants
# 0 = instant (no ramping, jerky)
# 254 = maximum acceleration (fastest ramp)
# Lower values = gentler starts and stops
ACCEL_GENTLE = 20    # Slow, smooth — good for demos
ACCEL_MEDIUM = 80    # Balanced
ACCEL_FAST = 200     # Snappy response
ACCEL_INSTANT = 254  # No ramping


# ─── Data Classes ────────────────────────────────────────────────────

@dataclass
class ServoStatus:
    """Snapshot of a single servo's state."""
    id: int
    name: str
    position: int           # 0-4095
    speed: int              # current speed
    load: int               # current load
    voltage: float          # in volts
    temperature: int        # in °C
    torque_enabled: bool

    @property
    def angle_degrees(self) -> float:
        """Convert raw position (0-4095) to degrees (0-360)."""
        return (self.position / 4095) * 360.0

    def __repr__(self):
        return (
            f"{self.name:>15} (ID {self.id}) | "
            f"pos={self.position:>4} ({self.angle_degrees:>6.1f}°) | "
            f"load={self.load:>4} | "
            f"temp={self.temperature}°C | "
            f"torque={'ON' if self.torque_enabled else 'OFF'}"
        )


@dataclass
class ArmSnapshot:
    """Snapshot of the entire arm's state at one moment in time."""
    timestamp: float
    servos: dict[str, ServoStatus] = field(default_factory=dict)

    @property
    def positions(self) -> dict[str, int]:
        """Get all positions as {joint_name: position}."""
        return {name: s.position for name, s in self.servos.items()}

    @property
    def position_list(self) -> list[int]:
        """Get positions as ordered list [id1, id2, ..., id6]."""
        return [self.servos[name].position for name in JOINT_NAMES]

    def __repr__(self):
        lines = [f"ArmSnapshot @ {self.timestamp:.3f}s"]
        for name in JOINT_NAMES:
            if name in self.servos:
                lines.append(f"  {self.servos[name]}")
        return "\n".join(lines)


# ─── ServoController ─────────────────────────────────────────────────

class ServoController:
    """
    Direct interface to the SO-101 arm's 6 Feetech STS3215 servos.

    This is the lowest level of control — everything else (teleoperation,
    recording, playback, learning) builds on top of this class.

    Example usage:
        >>> ctrl = ServoController("COM6")
        >>> ctrl.connect()
        >>> ctrl.enable_torque()
        >>> ctrl.move("shoulder_pan", 2048)      # move to center
        >>> ctrl.move_all([2048]*6)               # all joints to center
        >>> status = ctrl.read_all()              # read everything
        >>> print(status)
        >>> ctrl.disable_torque()
        >>> ctrl.disconnect()
    """

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE):
        """
        Initialize the controller (does NOT open the port yet).

        Args:
            port: Serial port name, e.g. "COM6" on Windows, "/dev/ttyUSB0" on Linux
            baudrate: Communication speed. STS3215 default is 1,000,000
        """
        self.port = port
        self.baudrate = baudrate
        self._port_handler: Optional[scs.PortHandler] = None
        self._packet_handler: Optional[scs.PacketHandler] = None
        self._connected = False

    # ── Connection ───────────────────────────────────────────────────

    def connect(self) -> None:
        """
        Open the serial port and establish communication.

        This is separate from __init__ because you might want to create
        the controller object first and connect later — a common pattern
        in robotics code.
        """
        if self._connected:
            print(f"Already connected to {self.port}")
            return

        self._port_handler = scs.PortHandler(self.port)
        self._packet_handler = scs.PacketHandler(0)  # Protocol version 0

        # Sync reader/writer — for fast bulk communication
        # These are reusable objects that batch-read/write all servos at once
        self._sync_reader = None  # Initialized on first use
        self._sync_writer = None  # Initialized on first use

        if not self._port_handler.openPort():
            raise ConnectionError(f"Failed to open port {self.port}")

        if not self._port_handler.setBaudRate(self.baudrate):
            raise ConnectionError(f"Failed to set baudrate to {self.baudrate}")

        self._connected = True
        print(f"Connected to SO-101 arm on {self.port} @ {self.baudrate} baud")

        # Verify all servos respond
        self._ping_all()

    def disconnect(self) -> None:
        """Disable torque and close the serial port."""
        if not self._connected:
            return

        self.disable_torque()
        self._port_handler.closePort()
        self._connected = False
        print(f"Disconnected from {self.port}")

    def _check_connected(self) -> None:
        """Raise error if not connected."""
        if not self._connected:
            raise ConnectionError(
                f"Not connected. Call connect() first."
            )

    def _ping_all(self) -> None:
        """Verify all 6 servos respond to ping."""
        print("Pinging servos...")
        for sid in range(1, 7):
            model, result, error = self._packet_handler.ping(
                self._port_handler, sid
            )
            if result != 0:
                print(f"  WARNING: Servo {sid} ({ID_TO_JOINT[sid]}) not responding!")
            else:
                print(f"  Servo {sid} ({ID_TO_JOINT[sid]}) OK")

    # ── Reading (Single Servo) ───────────────────────────────────────

    def read_position(self, joint: str | int) -> int:
        """
        Read the current position of a single servo.

        Args:
            joint: Joint name ("shoulder_pan") or servo ID (1)

        Returns:
            Position value 0-4095
        """
        self._check_connected()
        sid = self._resolve_id(joint)
        pos, result, error = self._packet_handler.read2ByteTxRx(
            self._port_handler, sid, ADDR_PRESENT_POSITION
        )
        if result != 0:
            raise ConnectionError(
                f"Failed to read position from servo {sid} ({ID_TO_JOINT[sid]})"
            )
        return pos

    def read_speed(self, joint: str | int) -> int:
        """Read the current speed of a servo."""
        self._check_connected()
        sid = self._resolve_id(joint)
        speed, _, _ = self._packet_handler.read2ByteTxRx(
            self._port_handler, sid, ADDR_PRESENT_SPEED
        )
        return speed

    def read_load(self, joint: str | int) -> int:
        """Read the current load (torque) on a servo."""
        self._check_connected()
        sid = self._resolve_id(joint)
        load, _, _ = self._packet_handler.read2ByteTxRx(
            self._port_handler, sid, ADDR_PRESENT_LOAD
        )
        return load

    def read_temperature(self, joint: str | int) -> int:
        """Read the temperature of a servo in °C."""
        self._check_connected()
        sid = self._resolve_id(joint)
        temp, _, _ = self._packet_handler.read1ByteTxRx(
            self._port_handler, sid, ADDR_PRESENT_TEMPERATURE
        )
        return temp

    def read_voltage(self, joint: str | int) -> float:
        """Read the voltage of a servo in volts."""
        self._check_connected()
        sid = self._resolve_id(joint)
        raw, _, _ = self._packet_handler.read1ByteTxRx(
            self._port_handler, sid, ADDR_PRESENT_VOLTAGE
        )
        return raw / 10.0

    # ── Reading (Full Arm) ───────────────────────────────────────────

    def read_all_positions(self) -> dict[str, int]:
        """
        Read positions of all 6 servos.

        Returns:
            Dict mapping joint names to positions, e.g.
            {"shoulder_pan": 2048, "shoulder_lift": 1500, ...}
        """
        self._check_connected()
        positions = {}
        for name in JOINT_NAMES:
            positions[name] = self.read_position(name)
        return positions

    def read_all_positions_list(self) -> list[int]:
        """Read all positions as an ordered list [id1, id2, ..., id6]."""
        self._check_connected()
        return [self.read_position(sid) for sid in range(1, 7)]

    def read_status(self, joint: str | int) -> ServoStatus:
        """Read full status of a single servo."""
        self._check_connected()
        sid = self._resolve_id(joint)
        name = ID_TO_JOINT[sid]
        return ServoStatus(
            id=sid,
            name=name,
            position=self.read_position(sid),
            speed=self.read_speed(sid),
            load=self.read_load(sid),
            voltage=self.read_voltage(sid),
            temperature=self.read_temperature(sid),
            torque_enabled=self._read_torque_enabled(sid),
        )

    def read_all(self) -> ArmSnapshot:
        """
        Read full status of all 6 servos.

        Returns:
            ArmSnapshot with all servo states and timestamp
        """
        self._check_connected()
        snapshot = ArmSnapshot(timestamp=time.time())
        for name in JOINT_NAMES:
            snapshot.servos[name] = self.read_status(name)
        return snapshot

    # ── Sync Read (Fast Bulk Reading) ────────────────────────────────
    #
    # INDIVIDUAL READ (what we used above):
    #   PC → "Servo 1, position?" → Servo 1 responds
    #   PC → "Servo 2, position?" → Servo 2 responds
    #   PC → "Servo 3, position?" → Servo 3 responds
    #   ... (6 round trips = ~12ms)
    #
    # SYNC READ (what we're adding now):
    #   PC → "ALL servos, positions!" → All respond in sequence
    #   (1 round trip = ~2ms)
    #
    # This is 6x faster. At 30Hz teleop, that's the difference between
    # "barely keeping up" and "plenty of headroom."
    #

    def sync_read_positions(self, num_retry: int = 1) -> dict[str, int]:
        """
        Read all 6 servo positions in a single bus transaction.

        This is ~6x faster than read_all_positions() because it sends
        one packet and gets all responses back, instead of 6 separate
        round trips.

        Args:
            num_retry: Number of retries on failure (Windows needs this)

        Returns:
            Dict mapping joint names to positions
        """
        self._check_connected()

        # Create sync reader if first time
        if self._sync_reader is None:
            self._sync_reader = scs.GroupSyncRead(
                self._port_handler,
                self._packet_handler,
                ADDR_PRESENT_POSITION,  # Start address
                2,                       # Data length (2 bytes for position)
            )
            for sid in range(1, 7):
                self._sync_reader.addParam(sid)

        # Try reading with retries
        for attempt in range(1 + num_retry):
            result = self._sync_reader.txRxPacket()
            if result == 0:  # COMM_SUCCESS
                break
        else:
            # All retries failed — fall back to individual reads
            return self.read_all_positions()

        # Extract positions from the response
        positions = {}
        for name in JOINT_NAMES:
            sid = JOINT_TO_ID[name]
            if self._sync_reader.isAvailable(sid, ADDR_PRESENT_POSITION, 2):
                positions[name] = self._sync_reader.getData(
                    sid, ADDR_PRESENT_POSITION, 2
                )
            else:
                # This servo didn't respond — read individually
                positions[name] = self.read_position(sid)

        return positions

    def sync_read_positions_list(self, num_retry: int = 1) -> list[int]:
        """Sync read all positions as ordered list [id1, ..., id6]."""
        positions = self.sync_read_positions(num_retry)
        return [positions[name] for name in JOINT_NAMES]

    def benchmark_read(self, iterations: int = 100) -> None:
        """
        Compare individual reads vs sync reads.

        Run this to see the speed difference on your system.
        This is great for understanding WHY sync read matters.
        """
        self._check_connected()

        # Individual reads
        start = time.perf_counter()
        for _ in range(iterations):
            self.read_all_positions()
        individual_time = time.perf_counter() - start

        # Sync reads
        start = time.perf_counter()
        for _ in range(iterations):
            self.sync_read_positions()
        sync_time = time.perf_counter() - start

        print(f"\n=== Read Benchmark ({iterations} iterations) ===")
        print(f"Individual reads: {individual_time:.3f}s "
              f"({individual_time/iterations*1000:.1f}ms per read, "
              f"{iterations/individual_time:.0f} Hz)")
        print(f"Sync reads:       {sync_time:.3f}s "
              f"({sync_time/iterations*1000:.1f}ms per read, "
              f"{iterations/sync_time:.0f} Hz)")
        print(f"Sync is {individual_time/sync_time:.1f}x faster")

    # ── Speed Control ────────────────────────────────────────────────
    #
    # Without speed control:
    #   move(2048) → servo RACES to position as fast as it can
    #
    # With speed control:
    #   move_at_speed("shoulder_pan", 2048, speed=200) → servo moves SMOOTHLY
    #
    # Speed value meanings:
    #   0     = no limit (full speed, ~62 RPM for STS3215)
    #   1     = slowest
    #   4095  = fastest controlled speed
    #

    def set_speed(self, joint: str | int, speed: int) -> None:
        """
        Set the moving speed for a servo.

        This sets how fast the servo moves to its next goal position.
        The speed persists until you change it again.

        Args:
            joint: Joint name or servo ID
            speed: Speed value (0=max speed, 1-4095 for controlled speed)
        """
        self._check_connected()
        sid = self._resolve_id(joint)
        speed = max(0, min(4095, speed))
        self._packet_handler.write2ByteTxRx(
            self._port_handler, sid, ADDR_GOAL_SPEED, speed
        )

    def set_all_speeds(self, speed: int) -> None:
        """Set the same speed for all servos."""
        for sid in range(1, 7):
            self.set_speed(sid, speed)

    def move_at_speed(
        self, joint: str | int, position: int, speed: int
    ) -> None:
        """
        Move a servo to a position at a specific speed.

        Args:
            joint: Joint name or servo ID
            position: Target position (0-4095)
            speed: Movement speed (0=max, 1-4095)
        """
        self.set_speed(joint, speed)
        self.move(joint, position)

    def move_all_at_speed(
        self, positions: list[int], speed: int
    ) -> None:
        """Move all servos to positions at a controlled speed."""
        self.set_all_speeds(speed)
        self.move_all(positions)

    # ── Acceleration Control ─────────────────────────────────────────
    #
    # Acceleration controls how the servo RAMPS UP and SLOWS DOWN.
    #
    # Without acceleration control:
    #   Motor: STOPPED → FULL SPEED → STOPPED (jerky!)
    #
    # With acceleration = 20:
    #   Motor: STOPPED → gradually speeds up → gradually slows down → STOPPED (smooth!)
    #
    # Think of it like driving a car:
    #   - High acceleration (254) = sports car, instant response
    #   - Low acceleration (20) = luxury car, smooth and gentle
    #   - No acceleration (0) = instant, like a light switch
    #

    def set_acceleration(self, joint: str | int, acceleration: int) -> None:
        """
        Set the acceleration for a servo.

        Controls how quickly the servo ramps up and slows down.
        Lower values = smoother, gentler motion.
        Higher values = snappier, more responsive motion.

        Args:
            joint: Joint name or servo ID
            acceleration: 0-254 (0=instant, 254=max acceleration rate)
        """
        self._check_connected()
        sid = self._resolve_id(joint)
        acceleration = max(0, min(254, acceleration))
        # Need to disable torque to write acceleration
        current_torque = self._read_torque_enabled(sid)
        if current_torque:
            self._packet_handler.write1ByteTxRx(
                self._port_handler, sid, ADDR_LOCK, 0
            )
            time.sleep(0.02)
        self._packet_handler.write1ByteTxRx(
            self._port_handler, sid, ADDR_ACCELERATION, acceleration
        )
        time.sleep(0.02)
        if current_torque:
            self._packet_handler.write1ByteTxRx(
                self._port_handler, sid, ADDR_LOCK, 1
            )
            time.sleep(0.02)

    def set_all_accelerations(self, acceleration: int) -> None:
        """Set the same acceleration for all servos."""
        for sid in range(1, 7):
            self.set_acceleration(sid, acceleration)

    def configure_motion(self, speed: int, acceleration: int) -> None:
        """
        Set both speed and acceleration for all servos at once.

        Convenience method for quickly configuring motion profile.

        Common profiles:
            Gentle demo:   configure_motion(SPEED_SLOW, ACCEL_GENTLE)
            Normal use:    configure_motion(SPEED_MEDIUM, ACCEL_MEDIUM)
            Fast response: configure_motion(SPEED_FAST, ACCEL_FAST)
            Full speed:    configure_motion(SPEED_MAX, ACCEL_INSTANT)

        Args:
            speed: Speed for all servos (0=max, 1-4095)
            acceleration: Acceleration for all servos (0-254)
        """
        self.set_all_speeds(speed)
        self.set_all_accelerations(acceleration)
        print(f"Motion configured: speed={speed}, acceleration={acceleration}")

    # ── Writing (Single Servo) ───────────────────────────────────────

    def move(self, joint: str | int, position: int) -> None:
        """
        Move a single servo to a target position.

        Args:
            joint: Joint name or servo ID
            position: Target position (0-4095)
        """
        self._check_connected()
        sid = self._resolve_id(joint)
        position = self._clamp_position(position)
        self._packet_handler.write2ByteTxRx(
            self._port_handler, sid, ADDR_GOAL_POSITION, position
        )

    # ── Writing (Full Arm) ───────────────────────────────────────────

    def move_all(self, positions: list[int] | dict[str, int]) -> None:
        """
        Move all 6 servos simultaneously.

        Args:
            positions: Either a list of 6 positions [id1, id2, ..., id6]
                      or a dict {"shoulder_pan": 2048, ...}
        """
        self._check_connected()

        if isinstance(positions, dict):
            for name, pos in positions.items():
                self.move(name, pos)
        elif isinstance(positions, list):
            if len(positions) != 6:
                raise ValueError(f"Expected 6 positions, got {len(positions)}")
            for sid, pos in enumerate(positions, 1):
                self.move(sid, pos)
        else:
            raise TypeError(f"Expected list or dict, got {type(positions)}")

    def smooth_move(
        self,
        target: list[int] | dict[str, int],
        duration: float = 2.0,
        steps: int = 40,
    ) -> None:
        """
        Smoothly interpolate from current position to target.

        Uses a smooth-step easing function so the motion starts slow,
        speeds up, then slows down — like natural human movement.

        Args:
            target: Target positions (list of 6 or dict)
            duration: Time to complete the move in seconds
            steps: Number of interpolation steps (more = smoother)
        """
        self._check_connected()

        # Normalize target to list
        if isinstance(target, dict):
            target_list = [target.get(name, self.read_position(name)) for name in JOINT_NAMES]
        else:
            target_list = target

        # Read current positions
        current = self.read_all_positions_list()

        # Interpolate with smooth-step easing
        for s in range(1, steps + 1):
            t = s / steps
            # Smooth-step: starts slow, speeds up, slows down
            t = t * t * (3 - 2 * t)
            interpolated = [
                int(current[i] + (target_list[i] - current[i]) * t)
                for i in range(6)
            ]
            self.move_all(interpolated)
            time.sleep(duration / steps)

    # ── Torque Control ───────────────────────────────────────────────

    def enable_torque(self, joint: str | int | None = None) -> None:
        """
        Enable torque (servo holds its position).

        Args:
            joint: Specific joint, or None for all servos
        """
        self._check_connected()
        if joint is None:
            for sid in range(1, 7):
                self._set_torque(sid, True)
            print("Torque ENABLED on all servos")
        else:
            sid = self._resolve_id(joint)
            self._set_torque(sid, True)
            print(f"Torque ENABLED on {ID_TO_JOINT[sid]}")

    def disable_torque(self, joint: str | int | None = None) -> None:
        """
        Disable torque (servo moves freely).

        Args:
            joint: Specific joint, or None for all servos
        """
        self._check_connected()
        if joint is None:
            for sid in range(1, 7):
                self._set_torque(sid, False)
            print("Torque DISABLED on all servos")
        else:
            sid = self._resolve_id(joint)
            self._set_torque(sid, False)
            print(f"Torque DISABLED on {ID_TO_JOINT[sid]}")

    def _set_torque(self, sid: int, enable: bool) -> None:
        """Low-level torque control with proper lock handling.
        
        CRITICAL: When enabling torque, we MUST set goal = current position
        FIRST. Otherwise the servo races to whatever old goal was stored
        in memory (could be 0) at full speed. This caused the violent
        movement bug discovered on 2026-03-10.
        """
        if enable:
            # SAFETY: Set goal to current position BEFORE enabling torque
            pos, _, _ = self._packet_handler.read2ByteTxRx(
                self._port_handler, sid, ADDR_PRESENT_POSITION
            )
            time.sleep(0.02)
            self._packet_handler.write2ByteTxRx(
                self._port_handler, sid, ADDR_GOAL_POSITION, pos
            )
            time.sleep(0.02)
            # Now safe to enable torque — servo stays in place
            self._packet_handler.write1ByteTxRx(
                self._port_handler, sid, ADDR_TORQUE_ENABLE, 1
            )
            time.sleep(0.02)
            self._packet_handler.write1ByteTxRx(
                self._port_handler, sid, ADDR_LOCK, 1
            )
            time.sleep(0.02)
        else:
            self._packet_handler.write1ByteTxRx(
                self._port_handler, sid, ADDR_TORQUE_ENABLE, 0
            )
            time.sleep(0.02)
            self._packet_handler.write1ByteTxRx(
                self._port_handler, sid, ADDR_LOCK, 0
            )
            time.sleep(0.02)

    def _read_torque_enabled(self, sid: int) -> bool:
        """Check if torque is enabled on a servo."""
        val, _, _ = self._packet_handler.read1ByteTxRx(
            self._port_handler, sid, ADDR_TORQUE_ENABLE
        )
        return val == 1

    # ── Utility Methods ──────────────────────────────────────────────

    def set_max_torque(self, joint: str | int, value: int) -> None:
        """
        Set maximum torque limit for a servo.

        Args:
            joint: Joint name or ID
            value: Torque limit (0-1000, where 1000 = 100%)
        """
        self._check_connected()
        sid = self._resolve_id(joint)
        value = max(0, min(1000, value))
        self._packet_handler.write2ByteTxRx(
            self._port_handler, sid, ADDR_MAX_TORQUE, value
        )

    def center_all(self, duration: float = 3.0) -> None:
        """Smoothly move all servos to center position (2048)."""
        print("Moving all joints to center...")
        self.smooth_move([POSITION_CENTER] * 6, duration=duration)

    def go_to(self, position_name: str, duration: float = 2.0) -> None:
        """
        Move to a named preset position.

        Available positions: "center", "rest", "raised", "folded"
        """
        presets = {
            "center": [2048, 2048, 2048, 2048, 2048, 2048],
            "raised": [2048, 1400, 1400, 1600, 2048, 2500],
            "folded": [2048, 2400, 2600, 1400, 2048, 2048],
            "wave_ready": [2048, 1400, 1400, 1500, 2048, 2800],
        }
        if position_name not in presets:
            raise ValueError(
                f"Unknown position '{position_name}'. "
                f"Available: {list(presets.keys())}"
            )
        self.smooth_move(presets[position_name], duration=duration)

    # ── Internal Helpers ─────────────────────────────────────────────

    def _resolve_id(self, joint: str | int) -> int:
        """Convert joint name or ID to servo ID."""
        if isinstance(joint, str):
            if joint not in JOINT_TO_ID:
                raise ValueError(
                    f"Unknown joint '{joint}'. "
                    f"Available: {JOINT_NAMES}"
                )
            return JOINT_TO_ID[joint]
        elif isinstance(joint, int):
            if joint < 1 or joint > 6:
                raise ValueError(f"Servo ID must be 1-6, got {joint}")
            return joint
        else:
            raise TypeError(f"Expected str or int, got {type(joint)}")

    @staticmethod
    def _clamp_position(pos: int) -> int:
        """Clamp position to valid range."""
        return max(POSITION_MIN, min(POSITION_MAX, pos))

    # ── Context Manager ──────────────────────────────────────────────

    def __enter__(self):
        """Support 'with' statement for automatic cleanup."""
        self.connect()
        return self

    def __exit__(self, *args):
        """Automatically disconnect when exiting 'with' block."""
        self.disconnect()

    def __repr__(self):
        status = "connected" if self._connected else "disconnected"
        return f"ServoController(port='{self.port}', status={status})"


# ─── Quick Test ──────────────────────────────────────────────────────

if __name__ == "__main__":
    """
    Run this file directly to test your servo connection:
        python servo.py --port COM6
        python servo.py --port COM6 --test benchmark
        python servo.py --port COM6 --test speed
        python servo.py --port COM6 --test acceleration
        python servo.py --port COM6 --test all
    """
    import argparse

    parser = argparse.ArgumentParser(description="SO-101 Servo Test")
    parser.add_argument("--port", type=str, default="COM6", help="Serial port")
    parser.add_argument(
        "--test",
        type=str,
        default="status",
        choices=["status", "benchmark", "speed", "acceleration", "all"],
        help="Which test to run",
    )
    args = parser.parse_args()

    with ServoController(args.port) as arm:

        # ── TEST: Status ─────────────────────────────────────────────
        if args.test in ("status", "all"):
            print("\n" + "=" * 55)
            print("  TEST: Full Arm Status")
            print("=" * 55)

            # Individual reads
            print("\n[Individual reads]")
            snapshot = arm.read_all()
            print(snapshot)

            # Sync reads
            print("\n[Sync read]")
            positions = arm.sync_read_positions()
            for name, pos in positions.items():
                angle = (pos / 4095) * 360
                print(f"  {name:>15}: {pos:>4} ({angle:.1f}°)")

        # ── TEST: Benchmark ──────────────────────────────────────────
        if args.test in ("benchmark", "all"):
            print("\n" + "=" * 55)
            print("  TEST: Read Speed Benchmark")
            print("  (Individual reads vs Sync reads)")
            print("=" * 55)
            arm.benchmark_read(iterations=100)

        # ── TEST: Speed Control ──────────────────────────────────────
        if args.test in ("speed", "all"):
            print("\n" + "=" * 55)
            print("  TEST: Speed Control")
            print("  Watch how speed changes the movement feel")
            print("=" * 55)

            response = input("\nEnable torque for speed test? (y/n): ")
            if response.lower() == "y":
                arm.enable_torque()
                rest = arm.read_all_positions_list()

                # Move to center first
                arm.smooth_move([POSITION_CENTER] * 6, duration=2.0)
                time.sleep(1)

                # Slow speed
                print("\n→ SLOW speed (200) — watch how gentle it is")
                arm.move_all_at_speed([1600, 2048, 2048, 2048, 2048, 2048], SPEED_SLOW)
                time.sleep(3)
                arm.move_all_at_speed([2500, 2048, 2048, 2048, 2048, 2048], SPEED_SLOW)
                time.sleep(3)

                # Medium speed
                print("→ MEDIUM speed (600) — normal operating speed")
                arm.move_all_at_speed([1600, 2048, 2048, 2048, 2048, 2048], SPEED_MEDIUM)
                time.sleep(2)
                arm.move_all_at_speed([2500, 2048, 2048, 2048, 2048, 2048], SPEED_MEDIUM)
                time.sleep(2)

                # Fast speed
                print("→ FAST speed (1500) — quick but controlled")
                arm.move_all_at_speed([1600, 2048, 2048, 2048, 2048, 2048], SPEED_FAST)
                time.sleep(1.5)
                arm.move_all_at_speed([2500, 2048, 2048, 2048, 2048, 2048], SPEED_FAST)
                time.sleep(1.5)

                # Max speed
                print("→ MAX speed (0) — full speed, no limit")
                arm.set_all_speeds(SPEED_MAX)
                arm.move_all([1600, 2048, 2048, 2048, 2048, 2048])
                time.sleep(1)
                arm.move_all([2500, 2048, 2048, 2048, 2048, 2048])
                time.sleep(1)

                # Return
                print("\nReturning to rest...")
                arm.set_all_speeds(SPEED_MEDIUM)
                arm.smooth_move(rest, duration=2.0)

        # ── TEST: Acceleration ───────────────────────────────────────
        if args.test in ("acceleration", "all"):
            print("\n" + "=" * 55)
            print("  TEST: Acceleration Control")
            print("  Watch how acceleration changes the start/stop feel")
            print("=" * 55)

            response = input("\nEnable torque for acceleration test? (y/n): ")
            if response.lower() == "y":
                arm.enable_torque()
                rest = arm.read_all_positions_list()
                arm.set_all_speeds(SPEED_MAX)

                arm.smooth_move([POSITION_CENTER] * 6, duration=2.0)
                time.sleep(1)

                # Gentle acceleration
                print("\n→ GENTLE acceleration (20) — luxury car feel")
                arm.set_all_accelerations(ACCEL_GENTLE)
                arm.move_all([1600, 2048, 2048, 2048, 2048, 2048])
                time.sleep(3)
                arm.move_all([2500, 2048, 2048, 2048, 2048, 2048])
                time.sleep(3)

                # Fast acceleration
                print("→ FAST acceleration (200) — sports car feel")
                arm.set_all_accelerations(ACCEL_FAST)
                arm.move_all([1600, 2048, 2048, 2048, 2048, 2048])
                time.sleep(2)
                arm.move_all([2500, 2048, 2048, 2048, 2048, 2048])
                time.sleep(2)

                # Instant acceleration
                print("→ INSTANT acceleration (254) — snappiest response")
                arm.set_all_accelerations(ACCEL_INSTANT)
                arm.move_all([1600, 2048, 2048, 2048, 2048, 2048])
                time.sleep(1.5)
                arm.move_all([2500, 2048, 2048, 2048, 2048, 2048])
                time.sleep(1.5)

                # Return
                print("\nReturning to rest...")
                arm.set_all_accelerations(ACCEL_MEDIUM)
                arm.smooth_move(rest, duration=2.0)

        print("\n" + "=" * 55)
        print("  ALL TESTS COMPLETE")
        print("=" * 55)
