"""
Teleoperation — SO-101 Arm Lab
Leader arm controls follower arm in real-time.

Usage:
    python scripts/teleop.py --leader /dev/ttyACM0 --follower /dev/ttyACM1
    python scripts/teleop.py --leader COM5 --follower COM6
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

    # Read both positions
    leader_pos = []
    follower_pos = []
    for sid in range(1, 7):
        l, _, _ = pkt.read2ByteTxRx(lp, sid, 56)
        time.sleep(0.05)
        f, _, _ = pkt.read2ByteTxRx(fp, sid, 56)
        time.sleep(0.05)
        leader_pos.append(l)
        follower_pos.append(f)
        gap = abs(l - f)
        print(f"  Servo {sid}: Leader={l} Follower={f} Gap={gap}")

    # Set goal=current before enabling torque (SAFETY)
    for sid in range(1, 7):
        pos, _, _ = pkt.read2ByteTxRx(fp, sid, 56)
        time.sleep(0.05)
        pkt.write2ByteTxRx(fp, sid, 42, pos)
        time.sleep(0.05)

    # Enable torque
    for sid in range(1, 7):
        pkt.write1ByteTxRx(fp, sid, 40, 1)
        time.sleep(0.05)

    # Slowly match follower to leader (3 seconds)
    print("\nMatching follower to leader slowly...")
    steps = 90
    for s in range(1, steps + 1):
        t = s / steps
        t = t * t * (3 - 2 * t)
        for sid in range(6):
            pos = int(follower_pos[sid] + (leader_pos[sid] - follower_pos[sid]) * t)
            pkt.write2ByteTxRx(fp, sid + 1, 42, pos)
        time.sleep(0.033)

    print("Matched! Teleop running. Ctrl+C to stop.\n")

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
    parser.add_argument("--leader", required=True)
    parser.add_argument("--follower", required=True)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()
    teleop(args.leader, args.follower, args.fps)