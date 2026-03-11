"""
Teleoperation — SO-101 Arm Lab
================================

Leader arm controls follower arm. Move the leader, follower mirrors.

Usage:
    python scripts/teleop.py --leader /dev/ttyACM0 --follower /dev/ttyACM1
    python scripts/teleop.py --leader COM5 --follower COM6

Author: Gopi Trinadh
Project: SO-101 Robotic Arm Lab
"""
import scservo_sdk as scs
import time
import argparse


def teleop(leader_port, follower_port, fps=30):
    lp = scs.PortHandler(leader_port)
    fp = scs.PortHandler(follower_port)
    lp.openPort(); lp.setBaudRate(1000000)
    fp.openPort(); fp.setBaudRate(1000000)
    pkt = scs.PacketHandler(0)

    print(f"Leader:   {leader_port}")
    print(f"Follower: {follower_port}")
    print(f"FPS:      {fps}")
    print()
    print("Teleop running! Move the leader arm. Ctrl+C to stop.")

    try:
        while True:
            for sid in range(1, 7):
                pos, _, _ = pkt.read2ByteTxRx(lp, sid, 56)
                pkt.write2ByteTxRx(fp, sid, 42, pos)
            time.sleep(1.0 / fps)
    except KeyboardInterrupt:
        print("\nStopped.")

    for sid in range(1, 7):
        pkt.write1ByteTxRx(fp, sid, 40, 0)
        time.sleep(0.02)

    lp.closePort()
    fp.closePort()
    print("Disconnected.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SO-101 Teleoperation")
    parser.add_argument("--leader", required=True, help="Leader port (COM5 or /dev/ttyACM0)")
    parser.add_argument("--follower", required=True, help="Follower port (COM6 or /dev/ttyACM1)")
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()
    teleop(args.leader, args.follower, args.fps)
