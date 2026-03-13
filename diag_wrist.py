import scservo_sdk as scs
import time

p = scs.PortHandler("COM6")
p.openPort(); p.setBaudRate(1000000)
pkt = scs.PacketHandler(0)

print("=== WRIST ROLL (Servo 5) FULL DIAGNOSIS ===")
print()

pos,_,_ = pkt.read2ByteTxRx(p, 5, 56); time.sleep(0.1)
goal,_,_ = pkt.read2ByteTxRx(p, 5, 42); time.sleep(0.1)
load,_,_ = pkt.read2ByteTxRx(p, 5, 60); time.sleep(0.1)
temp,_,_ = pkt.read1ByteTxRx(p, 5, 63); time.sleep(0.1)
volt,_,_ = pkt.read1ByteTxRx(p, 5, 62); time.sleep(0.1)
torque,_,_ = pkt.read1ByteTxRx(p, 5, 40); time.sleep(0.1)
minp,_,_ = pkt.read2ByteTxRx(p, 5, 21); time.sleep(0.1)
maxp,_,_ = pkt.read2ByteTxRx(p, 5, 23); time.sleep(0.1)
offset,_,_ = pkt.read2ByteTxRx(p, 5, 33); time.sleep(0.1)
maxt,_,_ = pkt.read2ByteTxRx(p, 5, 46); time.sleep(0.1)
speed,_,_ = pkt.read2ByteTxRx(p, 5, 44); time.sleep(0.1)
accel,_,_ = pkt.read1ByteTxRx(p, 5, 41); time.sleep(0.1)
moving,_,_ = pkt.read1ByteTxRx(p, 5, 66); time.sleep(0.1)
current,_,_ = pkt.read2ByteTxRx(p, 5, 69); time.sleep(0.1)

load_mag = load & 0x3FF
load_dir = "CW" if load & 0x400 else "CCW"

print(f"  Position:     {pos}")
print(f"  Goal:         {goal}")
print(f"  Load:         {load_mag} ({load_dir})")
print(f"  Temperature:  {temp} C")
print(f"  Voltage:      {volt/10} V")
print(f"  Torque:       {'ON' if torque else 'OFF'}")
print(f"  Min Limit:    {minp}")
print(f"  Max Limit:    {maxp}")
print(f"  Offset:       {offset}")
print(f"  Max Torque:   {maxt}")
print(f"  Speed:        {speed}")
print(f"  Acceleration: {accel}")
print(f"  Moving:       {moving}")
print(f"  Current:      {current}")
print()

print("Testing movement...")
pkt.write2ByteTxRx(p, 5, 42, pos); time.sleep(0.1)
pkt.write2ByteTxRx(p, 5, 44, 100); time.sleep(0.1)
pkt.write1ByteTxRx(p, 5, 40, 1); time.sleep(0.3)

print(f"  Moving +100 from {pos}...")
pkt.write2ByteTxRx(p, 5, 42, pos + 100)
time.sleep(1.5)
pos2,_,_ = pkt.read2ByteTxRx(p, 5, 56)
print(f"  After +100: {pos2} (moved {pos2-pos})")

print(f"  Moving -100 from {pos2}...")
pkt.write2ByteTxRx(p, 5, 42, pos2 - 100)
time.sleep(1.5)
pos3,_,_ = pkt.read2ByteTxRx(p, 5, 56)
print(f"  After -100: {pos3} (moved {pos3-pos2})")

print(f"  Returning to {pos}...")
pkt.write2ByteTxRx(p, 5, 42, pos)
time.sleep(1.5)
pos4,_,_ = pkt.read2ByteTxRx(p, 5, 56)
print(f"  Final: {pos4}")

pkt.write1ByteTxRx(p, 5, 40, 0)
p.closePort()
print()
print("Done.")
