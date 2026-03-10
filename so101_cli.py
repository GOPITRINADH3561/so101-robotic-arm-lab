"""
SO-101 CLI — One command for everything
========================================

Usage:
    python so101_cli.py status --port COM6
    python so101_cli.py status --port COM5 --id leader
    python so101_cli.py find-ports
    python so101_cli.py calibrate --port COM6 --id follower
    python so101_cli.py temps --port COM6
    python so101_cli.py home --port COM6
    python so101_cli.py benchmark --port COM6

Author: Gopi Trinadh
Project: SO-101 Robotic Arm Lab
"""
import sys
import os

# Add src to path so imports work
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from so101.servo import ServoController
from so101.arm import ArmController, find_port, find_all_ports


def print_help():
    print("""
  ╔══════════════════════════════════════════╗
  ║        SO-101 Robotic Arm CLI            ║
  ╚══════════════════════════════════════════╝

  COMMANDS:

    find-ports                   Detect leader/follower ports (unplug method)
    status   --port PORT         Full arm status with positions & temps
    benchmark --port PORT        Compare individual vs sync read speed
    temps    --port PORT         Temperature report
    calibrate --port PORT        Interactive joint-by-joint calibration
    quick-cal --port PORT        Quick calibration (move all joints)
    home     --port PORT         Move all joints to home position

  OPTIONS:

    --port PORT     Serial port (COM5, COM6, /dev/ttyUSB0)
    --id NAME       Arm name: leader or follower (default: follower)

  EXAMPLES:

    python so101_cli.py find-ports
    python so101_cli.py status --port COM6
    python so101_cli.py status --port COM5 --id leader
    python so101_cli.py benchmark --port COM6
    python so101_cli.py calibrate --port COM6 --id follower
    python so101_cli.py temps --port COM6
    python so101_cli.py home --port COM6
""")


def parse_args():
    args = sys.argv[1:]
    if not args or args[0] in ["-h", "--help", "help"]:
        print_help()
        sys.exit(0)

    command = args[0]
    port = None
    arm_id = "follower"

    i = 1
    while i < len(args):
        if args[i] == "--port" and i + 1 < len(args):
            port = args[i + 1]
            i += 2
        elif args[i] == "--id" and i + 1 < len(args):
            arm_id = args[i + 1]
            i += 2
        else:
            i += 1

    return command, port, arm_id


def main():
    command, port, arm_id = parse_args()

    # ── find-ports: no port needed ───────────────────────────────
    if command == "find-ports":
        response = input("Detect [1] single arm or [2] both arms? (1/2): ")
        if response.strip() == "2":
            find_all_ports()
        else:
            find_port(f"{arm_id.upper()} arm")
        return

    # ── All other commands need a port ───────────────────────────
    if not port:
        print("Error: --port required")
        print("Example: python so101_cli.py status --port COM6")
        print("Run 'python so101_cli.py find-ports' to detect ports")
        sys.exit(1)

    # ── Commands using ServoController (Layer 1) ─────────────────
    if command == "benchmark":
        with ServoController(port) as arm:
            arm.benchmark_read()
        return

    # ── Commands using ArmController (Layer 2) ───────────────────
    arm = ArmController.from_port(port, arm_id)

    try:
        if command == "status":
            arm.print_status()

        elif command == "temps":
            arm.print_temperatures()

        elif command == "calibrate":
            arm.calibrate()

        elif command == "quick-cal":
            arm.quick_calibrate()

        elif command == "home":
            arm.home()
            import time
            time.sleep(2)

        else:
            print(f"Unknown command: {command}")
            print_help()

    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
