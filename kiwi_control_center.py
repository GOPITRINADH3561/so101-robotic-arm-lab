"""
KIWI Control Center v3 — All Fixes
Author: Gopi Trinadh | SO-101 Robotic Arm Lab
"""
import json,os,sys,time,math,threading
from pathlib import Path
from datetime import datetime
from flask import Flask,render_template_string,jsonify,request
from flask_socketio import SocketIO

sys.path.insert(0,os.path.join(os.path.dirname(__file__),"src"))
try: import scservo_sdk as scs; HAS_SDK=True
except: HAS_SDK=False
try:
    from lerobot.robots.so101_follower import SO101Follower
    from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
    from lerobot.teleoperators.so101_leader import SO101Leader
    from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig
    HAS_LEROBOT=True
except: HAS_LEROBOT=False

app=Flask(__name__);app.config["SECRET_KEY"]="kiwi";socketio=SocketIO(app,cors_allowed_origins="*")
JOINTS=["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"]

# PORT LOCK — prevents status refresh from conflicting with gestures/controls
port_lock = threading.Lock()

state={"leader_port":"","follower_port":"","teleop_running":False,"recording":False,
    "trajectory":[],"stop_event":threading.Event(),"temp_history":{n:[] for n in JOINTS},
    "busy":False}
try:
    with open(".ports.json") as f: c=json.load(f); state["leader_port"]=c.get("leader",""); state["follower_port"]=c.get("follower","")
except: pass

def auto_detect_ports():
    if not HAS_SDK: return []
    try:
        import serial.tools.list_ports
        ports = [p.device for p in serial.tools.list_ports.comports()]
    except: return []
    found = []
    for port in ports:
        try:
            with port_lock:
                p = scs.PortHandler(port)
                if not p.openPort():
                    continue
                p.setBaudRate(1000000)
                pkt = scs.PacketHandler(0)
                # Try pinging servo 1 with retries
                ok = False
                for attempt in range(3):
                    _, result, _ = pkt.ping(p, 1)
                    if result == 0:
                        ok = True
                        break
                    time.sleep(0.1)
                p.closePort()
                if ok:
                    found.append(port)
        except:
            try: p.closePort()
            except: pass
    return found

def read_servos(port):
    if not HAS_SDK or not port or state["busy"]: return None
    try:
        with port_lock:
            p=scs.PortHandler(port);p.openPort();p.setBaudRate(1000000);pkt=scs.PacketHandler(0);data=[]
            for i,n in enumerate(JOINTS):
                sid=i+1;pos,_,_=pkt.read2ByteTxRx(p,sid,56);time.sleep(.03);ld,_,_=pkt.read2ByteTxRx(p,sid,60);time.sleep(.03)
                tmp,_,_=pkt.read1ByteTxRx(p,sid,63);time.sleep(.03);vlt,_,_=pkt.read1ByteTxRx(p,sid,62);time.sleep(.03)
                tq,_,_=pkt.read1ByteTxRx(p,sid,40);time.sleep(.03)
                data.append({"name":n,"position":pos,"angle":round((pos/4095)*360,1),"load":ld&0x3FF,"temp":tmp,"voltage":round(vlt/10,1),"torque":bool(tq),"pct":round((pos/4095)*100,1)})
                state["temp_history"][n].append(tmp)
                if len(state["temp_history"][n])>60: state["temp_history"][n].pop(0)
            p.closePort();return data
    except: return None

def run_gesture(port, gesture_name):
    state["busy"] = True
    try:
        with port_lock:
            p = scs.PortHandler(port)
            p.openPort()
            p.setBaudRate(1000000)
            pkt = scs.PacketHandler(0)

            def read_cur():
                cur = []
                for sid in range(1,7):
                    pos,_,_ = pkt.read2ByteTxRx(p,sid,56); time.sleep(.02); cur.append(pos)
                return cur

            def move(target, dur=1.5):
                cur = read_cur()
                for sid in range(6): pkt.write2ByteTxRx(p,sid+1,42,cur[sid]); time.sleep(.02)
                for sid in range(1,7): pkt.write1ByteTxRx(p,sid,40,1); time.sleep(.02)
                steps = max(10, int(dur*25))
                for s in range(1, steps+1):
                    t = s/steps; t = t*t*(3-2*t)
                    for sid in range(6): pkt.write2ByteTxRx(p,sid+1,42,int(cur[sid]+(target[sid]-cur[sid])*t))
                    time.sleep(dur/steps)

            H = [2048, 870, 3088, 2841, 2048, 2030]
            if gesture_name=="wave":
                move([2048,1400,1400,1600,2048,2800],1.0)
                for _ in range(3):
                    move([2048,1400,1400,1600,2500,2800],0.3)
                    move([2048,1400,1400,1600,1600,2200],0.3)
                move(H,1.0)
            elif gesture_name=="nod":
                move([2048,1600,1800,1800,2048,2048],0.8)
                for _ in range(3):
                    move([2048,1800,2000,1600,2048,2048],0.3)
                    move([2048,1400,1600,2000,2048,2048],0.3)
                move(H,0.8)
            elif gesture_name=="shake":
                move([2048,1600,1800,2048,2048,2048],0.8)
                for _ in range(3):
                    move([1700,1600,1800,2048,2048,2048],0.3)
                    move([2400,1600,1800,2048,2048,2048],0.3)
                move(H,0.8)
            elif gesture_name=="thumbsup":
                move([2048,1400,1400,1600,2048,2800],1.5); time.sleep(1.5); move(H,1.5)
            elif gesture_name=="point":
                move([2048,1400,1400,2048,2048,2048],1.5); time.sleep(1.5); move(H,1.5)
            elif gesture_name=="dance":
                move([2048,1600,1800,1800,2048,2400],1.0)
                for tv in range(120):
                    a=tv*2*math.pi/30; e=math.sin(tv/120*math.pi)
                    ps=[int(2048+e*300*math.sin(a)),int(1600+e*200*math.sin(a-.8)),int(1800+e*250*math.cos(a)),
                        int(1800+e*200*math.sin(a-2)),int(2048+e*350*math.sin(a*1.5)),int(2400+e*200*math.sin(a*1.2))]
                    for sid,pos in enumerate(ps,1): pkt.write2ByteTxRx(p,sid,42,pos)
                    time.sleep(.04)
                move(H,1.5)
            elif gesture_name=="home":
                move(H,2.0)

            for sid in range(1,7): pkt.write1ByteTxRx(p,sid,40,0); time.sleep(.02)
            p.closePort()
    except Exception as e:
        try: p.closePort()
        except: pass
        socketio.emit("err",{"e":str(e)})
    state["busy"] = False

@app.route("/")
def index(): return render_template_string(HTML)
@app.route("/api/ports")
def api_ports():
    try:
        import serial.tools.list_ports
        all_ports=[{"d":p.device,"desc":p.description} for p in serial.tools.list_ports.comports()]
    except: all_ports=[]
    return jsonify({"ports":all_ports,"leader":state["leader_port"],"follower":state["follower_port"]})
@app.route("/api/auto_detect",methods=["POST"])
def api_auto_detect():
    found=auto_detect_ports()
    if len(found)>=2: state["leader_port"]=found[0]; state["follower_port"]=found[1]
    elif len(found)==1: state["follower_port"]=found[0]
    if found:
        with open(".ports.json","w") as f: json.dump({"leader":state["leader_port"],"follower":state["follower_port"]},f)
    return jsonify({"found":found,"leader":state["leader_port"],"follower":state["follower_port"]})
@app.route("/api/set_ports",methods=["POST"])
def api_set_ports():
    d=request.json;state["leader_port"]=d.get("leader","");state["follower_port"]=d.get("follower","")
    with open(".ports.json","w") as f: json.dump({"leader":state["leader_port"],"follower":state["follower_port"]},f)
    return jsonify({"ok":True})
@app.route("/api/status/<role>")
def api_status(role):
    port=state["follower_port"] if role=="follower" else state["leader_port"]
    return jsonify({"servos":read_servos(port),"temps":state["temp_history"]})
@app.route("/api/teleop/start",methods=["POST"])
def api_teleop_start():
    if state["teleop_running"]: return jsonify({"error":"Already running"})
    if not HAS_LEROBOT: return jsonify({"error":"LeRobot not installed"})
    lp2=state["leader_port"];fp2=state["follower_port"]
    if not lp2 or not fp2: return jsonify({"error":"Set ports first"})
    state["teleop_running"]=True;state["busy"]=True;state["stop_event"].clear()
    def run():
        dropped=0
        try:
            leader=SO101Leader(SO101LeaderConfig(port=lp2,id="leader_arm"))
            follower=SO101Follower(SO101FollowerConfig(port=fp2,id="follower_arm"))
            leader.connect();follower.connect()
            while not state["stop_event"].is_set():
                try:
                    t0=time.perf_counter();action=leader.get_action();follower.send_action(action);dt=time.perf_counter()-t0
                    socketio.emit("td",{"fps":round(1/max(.001,dt)),"d":dropped,"a":{k:round(v,1) for k,v in action.items()}})
                    time.sleep(max(0,.033-dt))
                except ConnectionError: dropped+=1
                except: break
            leader.disconnect();follower.disconnect()
        except Exception as e: socketio.emit("te",{"e":str(e)})
        state["teleop_running"]=False;state["busy"]=False;socketio.emit("ts",{})
    threading.Thread(target=run,daemon=True).start();return jsonify({"ok":True})
@app.route("/api/teleop/stop",methods=["POST"])
def api_teleop_stop(): state["stop_event"].set();return jsonify({"ok":True})
@app.route("/api/estop",methods=["POST"])
def api_estop():
    state["stop_event"].set()
    for port in [state["leader_port"],state["follower_port"]]:
        if not port or not HAS_SDK: continue
        try:
            with port_lock:
                p=scs.PortHandler(port);p.openPort();p.setBaudRate(1000000);pkt=scs.PacketHandler(0)
                for sid in range(1,7): pkt.write1ByteTxRx(p,sid,40,0)
                p.closePort()
        except: pass
    state["busy"]=False
    return jsonify({"ok":True})
@app.route("/api/gesture/<gn>",methods=["POST"])
def api_gesture(gn):
    port=state["follower_port"]
    if not port or not HAS_SDK: return jsonify({"error":"No follower port"})
    if state["busy"]: return jsonify({"error":"Arm is busy"})
    threading.Thread(target=run_gesture,args=(port,gn),daemon=True).start()
    return jsonify({"ok":True})
@app.route("/api/compliant/<action>",methods=["POST"])
def api_compliant(action):
    port=state["follower_port"]
    if not port or not HAS_SDK: return jsonify({"error":"No port"})
    try:
        with port_lock:
            p=scs.PortHandler(port);p.openPort();p.setBaudRate(1000000);pkt=scs.PacketHandler(0)
            if action=="on":
                for sid in range(1,7): pkt.write1ByteTxRx(p,sid,40,0);time.sleep(.05)
            else:
                for sid in range(1,7):
                    pos,_,_=pkt.read2ByteTxRx(p,sid,56);time.sleep(.05)
                    pkt.write2ByteTxRx(p,sid,42,pos);time.sleep(.05)
                for sid in range(1,7): pkt.write1ByteTxRx(p,sid,40,1);time.sleep(.05)
            p.closePort()
        return jsonify({"ok":True})
    except Exception as e: return jsonify({"error":str(e)})
@app.route("/api/record/start",methods=["POST"])
def api_rec_start():
    port=state["follower_port"]
    if not port or not HAS_SDK: return jsonify({"error":"No port"})
    if state["busy"]: return jsonify({"error":"Busy"})
    state["recording"]=True;state["busy"]=True;state["trajectory"]=[];state["stop_event"].clear()
    def rec():
        try:
            with port_lock:
                p=scs.PortHandler(port);p.openPort();p.setBaudRate(1000000);pkt=scs.PacketHandler(0)
                for sid in range(1,7): pkt.write1ByteTxRx(p,sid,40,0);time.sleep(.05)
                t0=time.time()
                while not state["stop_event"].is_set():
                    ps=[]
                    for sid in range(1,7): pos,_,_=pkt.read2ByteTxRx(p,sid,56);ps.append(pos)
                    state["trajectory"].append({"t":round(time.time()-t0,3),"p":ps})
                    socketio.emit("rf",{"c":len(state["trajectory"]),"t":round(time.time()-t0,1)})
                    time.sleep(.033)
                p.closePort()
        except Exception as e: socketio.emit("err",{"e":str(e)})
        state["recording"]=False;state["busy"]=False;socketio.emit("rs",{"c":len(state["trajectory"])})
    threading.Thread(target=rec,daemon=True).start();return jsonify({"ok":True})
@app.route("/api/record/stop",methods=["POST"])
def api_rec_stop(): state["stop_event"].set();return jsonify({"ok":True})
@app.route("/api/record/replay",methods=["POST"])
def api_rec_replay():
    if not state["trajectory"]: return jsonify({"error":"No recording"})
    port=state["follower_port"]
    if not port or not HAS_SDK: return jsonify({"error":"No port"})
    if state["busy"]: return jsonify({"error":"Busy"})
    state["busy"]=True
    def play():
        try:
            with port_lock:
                p=scs.PortHandler(port);p.openPort();p.setBaudRate(1000000);pkt=scs.PacketHandler(0)
                for sid in range(1,7):
                    pos,_,_=pkt.read2ByteTxRx(p,sid,56);time.sleep(.05);pkt.write2ByteTxRx(p,sid,42,pos);time.sleep(.05)
                for sid in range(1,7): pkt.write1ByteTxRx(p,sid,40,1);time.sleep(.05)
                t0=time.time()
                for f in state["trajectory"]:
                    while time.time()-t0<f["t"]: time.sleep(.005)
                    for sid,pos in enumerate(f["p"],1): pkt.write2ByteTxRx(p,sid,42,pos)
                time.sleep(.5)
                for sid in range(1,7): pkt.write1ByteTxRx(p,sid,40,0);time.sleep(.02)
                p.closePort()
            socketio.emit("rdone",{})
        except Exception as e: socketio.emit("err",{"e":str(e)})
        state["busy"]=False
    threading.Thread(target=play,daemon=True).start();return jsonify({"ok":True})
@app.route("/api/record/save",methods=["POST"])
def api_rec_save():
    if not state["trajectory"]: return jsonify({"error":"No recording"})
    os.makedirs("recordings",exist_ok=True)
    path=f"recordings/rec_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with open(path,"w") as f: json.dump({"frames":state["trajectory"]},f)
    return jsonify({"ok":True,"path":path})
@app.route("/api/calibration_status")
def api_cal_status():
    home=Path.home()
    fp=home/".cache/huggingface/lerobot/calibration/robots/so101_follower/follower_arm.json"
    lp2=home/".cache/huggingface/lerobot/calibration/teleoperators/so101_leader/leader_arm.json"
    return jsonify({"follower":fp.exists(),"leader":lp2.exists(),
        "fcmd":f"lerobot-calibrate --robot.type=so101_follower --robot.port={state['follower_port'] or '<PORT>'} --robot.id=follower_arm",
        "lcmd":f"lerobot-calibrate --teleop.type=so101_leader --teleop.port={state['leader_port'] or '<PORT>'} --teleop.id=leader_arm"})

HTML=r"""<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>KIWI Control Center</title>
<link href="https://fonts.googleapis.com/css2?family=DM+Sans:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500;600&display=swap" rel="stylesheet">
<script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.4/socket.io.min.js"></script>
<style>
:root{--bg:#E5DDD5;--cd:#EDEAE5;--inn:#E5DDD5;--bdr:#D5CEC6;--hov:#F2F0EC;--ink1:#2C2520;--ink2:#6B6058;--ink3:#9C9488;--ac:#3D348B;--sg:#4A7C59;--tr:#C4553A;--hn:#C08B30;--r:16px;--r2:12px;--r3:8px}
*{margin:0;padding:0;box-sizing:border-box}body{font-family:'DM Sans',sans-serif;background:var(--bg);color:var(--ink1);-webkit-font-smoothing:antialiased}
header{position:sticky;top:0;z-index:100;height:56px;display:flex;align-items:center;justify-content:space-between;padding:0 32px;background:var(--cd);border-bottom:1px solid var(--bdr)}
.logo{display:flex;align-items:center;gap:12px}.lm{width:32px;height:32px;border-radius:10px;background:var(--ac);display:grid;place-items:center}
.lm svg{width:18px;height:18px;fill:none;stroke:#fff;stroke-width:2;stroke-linecap:round;stroke-linejoin:round}
.logo h1{font-size:16px;font-weight:700}.logo small{font-size:10px;color:var(--ink3);display:block;margin-top:-2px}
.hr{display:flex;align-items:center;gap:16px}.hs{font-size:11px;color:var(--ink3);display:flex;align-items:center;gap:5px}
.dot{width:6px;height:6px;border-radius:50%}.dot.on{background:var(--sg)}.dot.off{background:var(--bdr)}
.estop{background:var(--tr);color:#fff;border:none;height:32px;padding:0 16px;border-radius:var(--r3);font-size:11px;font-weight:700;cursor:pointer;transition:.2s}.estop:hover{filter:brightness(1.1)}
main{display:grid;grid-template-columns:1fr 1fr 280px;gap:16px;padding:20px 32px 40px;max-width:1440px;margin:0 auto}
@media(max-width:1200px){main{grid-template-columns:1fr 1fr}.side{grid-column:1/-1;display:grid;grid-template-columns:1fr 1fr 1fr;gap:16px}}
@media(max-width:768px){main{grid-template-columns:1fr;padding:12px}.side{grid-template-columns:1fr}}
.s2{grid-column:span 2}.s3{grid-column:1/-1}@media(max-width:768px){.s2,.s3{grid-column:span 1}}
.cd{background:var(--cd);border:1px solid var(--bdr);border-radius:var(--r);padding:20px;transition:.2s;overflow:hidden}.cd:hover{background:var(--hov)}
.ch{display:flex;align-items:center;justify-content:space-between;margin-bottom:16px}.ct{font-size:13px;font-weight:700}
.pill{font-size:10px;font-weight:600;padding:3px 10px;border-radius:20px}.pill-on{background:rgba(74,124,89,.12);color:var(--sg)}.pill-off{background:rgba(196,85,58,.1);color:var(--tr)}
.sl{display:flex;flex-direction:column;gap:2px}
.si{display:grid;grid-template-columns:100px 1fr 44px 38px;align-items:center;gap:10px;padding:10px 8px;border-radius:var(--r3)}.si:hover{background:var(--bg)}
.sn{font-size:12px;color:var(--ink3)}.bar{height:5px;background:var(--bdr);border-radius:3px;overflow:hidden}
.fill{height:100%;border-radius:3px;background:var(--ac);transition:width .5s ease}
.sv{font-family:'JetBrains Mono',monospace;font-size:11px;font-weight:600;text-align:right;color:var(--ink2)}
.st{font-family:'JetBrains Mono',monospace;font-size:11px;font-weight:600;text-align:right}.tok{color:var(--sg)}.twn{color:var(--hn)}.tht{color:var(--tr)}
.tw{display:flex;flex-direction:column;align-items:center;gap:14px;padding:8px 0}
.tb{width:100px;height:100px;border-radius:50%;border:2.5px solid var(--bdr);background:var(--bg);display:grid;place-items:center;cursor:pointer;transition:.3s}
.tb:hover{border-color:var(--ac);background:var(--hov)}.tb.on{border-color:var(--tr);background:rgba(196,85,58,.06)}
.tb span{font-size:13px;font-weight:700;letter-spacing:.5px;color:var(--ac)}.tb.on span{color:var(--tr)}
.ts{display:flex;gap:28px}.tstat{text-align:center}
.tn{font-family:'JetBrains Mono',monospace;font-size:22px;font-weight:700;color:var(--ac)}
.tl{font-size:9px;color:var(--ink3);text-transform:uppercase;letter-spacing:1.5px;margin-top:2px}
.jg{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;width:100%}
.jb{background:var(--bg);border:1px solid var(--bdr);border-radius:var(--r3);padding:8px;text-align:center}
.jb .jn{font-size:9px;color:var(--ink3);text-transform:uppercase;letter-spacing:.5px}.jb .jv{font-family:'JetBrains Mono',monospace;font-size:15px;font-weight:700;color:var(--ac);margin-top:2px}
.btn{display:inline-flex;align-items:center;gap:5px;padding:8px 16px;border:none;border-radius:var(--r3);font-size:11px;font-weight:600;cursor:pointer;transition:.2s;font-family:inherit}
.btn:hover{transform:translateY(-1px)}.btn:active{transform:translateY(0)}
.ba{background:var(--ac);color:#fff}.bs{background:var(--sg);color:#fff}.bt{background:var(--tr);color:#fff}.bh{background:var(--hn);color:#fff}
.bg{background:var(--bg);color:var(--ink2);border:1px solid var(--bdr)}.bg:hover{background:var(--hov)}
.row{display:flex;gap:8px;flex-wrap:wrap}
.field{margin-bottom:10px}.fl{font-size:10px;font-weight:600;color:var(--ink3);margin-bottom:4px}
.fi{width:100%;background:var(--bg);border:1.5px solid var(--bdr);border-radius:var(--r3);padding:9px 12px;color:var(--ink1);font-family:'JetBrains Mono',monospace;font-size:12px;outline:none;transition:.2s}.fi:focus{border-color:var(--ac)}
canvas{display:block;background:transparent;width:100%;max-width:100%}
.toast{position:fixed;bottom:24px;left:50%;transform:translateX(-50%) translateY(50px);padding:10px 24px;border-radius:var(--r2);font-size:12px;font-weight:500;z-index:1000;opacity:0;transition:.4s;pointer-events:none;background:var(--ink1);color:var(--cd)}.toast.show{opacity:1;transform:translateX(-50%) translateY(0)}
.side{display:flex;flex-direction:column;gap:16px}

/* GESTURE GRID — square tiles with SVG icons */
.gg{display:grid;grid-template-columns:repeat(7,1fr);gap:8px}
.gi{aspect-ratio:1;background:var(--bg);border:1px solid var(--bdr);border-radius:var(--r2);display:flex;flex-direction:column;align-items:center;justify-content:center;gap:6px;cursor:pointer;transition:.25s}
.gi:hover{background:var(--hov);border-color:var(--ac);transform:translateY(-2px)}.gi:active{transform:translateY(0)}
.gi svg{width:22px;height:22px;stroke:var(--ink3);fill:none;stroke-width:1.5;stroke-linecap:round;stroke-linejoin:round;transition:.25s}
.gi:hover svg{stroke:var(--ac)}
.gi span{font-size:10px;font-weight:600;color:var(--ink2)}.gi:hover span{color:var(--ac)}

/* RECORD + CALIBRATION BOX */
.rb{font-family:'JetBrains Mono',monospace;font-size:18px;font-weight:600;text-align:center;padding:14px;border-radius:var(--r3);background:var(--bg);border:1px solid var(--bdr);margin-bottom:12px;color:var(--ink3)}
.rb.rec{color:var(--tr);animation:bk 1s ease infinite}.rb.done{color:var(--sg)}
@keyframes bk{0%,100%{opacity:1}50%{opacity:.3}}
.cmd{background:var(--bg);border:1px solid var(--bdr);border-radius:var(--r3);padding:8px 12px;font-family:'JetBrains Mono',monospace;font-size:9px;color:var(--ink2);cursor:pointer;transition:.2s;margin-top:6px;word-break:break-all}
.cmd:hover{background:var(--hov);border-color:var(--ac)}.cmd:active{background:var(--ac);color:#fff}
.cal-ok{color:var(--sg);font-size:11px;font-weight:600}.cal-no{color:var(--tr);font-size:11px;font-weight:600}

main>*{animation:up .4s ease both}
main>:nth-child(1){animation-delay:.03s}main>:nth-child(2){animation-delay:.06s}main>:nth-child(3){animation-delay:.09s}
main>:nth-child(4){animation-delay:.12s}main>:nth-child(5){animation-delay:.15s}main>:nth-child(6){animation-delay:.18s}
main>:nth-child(7){animation-delay:.21s}main>:nth-child(8){animation-delay:.24s}
@keyframes up{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:translateY(0)}}
::selection{background:rgba(61,52,139,.15)}
</style></head><body>

<header>
<div class="logo"><div class="lm"><svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="10"/><path d="M8 14s1.5 2 4 2 4-2 4-2"/><line x1="9" y1="9" x2="9.01" y2="9"/><line x1="15" y1="9" x2="15.01" y2="9"/></svg></div><div><h1>KIWI Control Center</h1><small>SO-101 Arm Lab</small></div></div>
<div class="hr"><div class="hs">Leader <span class="dot" id="ld"></span></div><div class="hs">Follower <span class="dot" id="fd"></span></div><button class="estop" onclick="estop()">STOP</button></div>
</header>

<main>
<div class="cd"><div class="ch"><span class="ct">Follower Arm</span><span class="pill pill-off" id="fb">offline</span></div><div class="sl" id="fs"><div style="color:var(--ink3);text-align:center;padding:28px">Connecting...</div></div></div>
<div class="cd"><div class="ch"><span class="ct">Leader Arm</span><span class="pill pill-off" id="lb">offline</span></div><div class="sl" id="ls"><div style="color:var(--ink3);text-align:center;padding:28px">Connecting...</div></div></div>

<div class="side">
<div class="cd"><div class="ch"><span class="ct">Teleoperation</span></div>
<div class="tw"><div class="tb" id="tr" onclick="toggleTeleop()"><span id="tl">START</span></div>
<div class="ts"><div class="tstat"><div class="tn" id="tf">--</div><div class="tl">FPS</div></div><div class="tstat"><div class="tn" id="td2">0</div><div class="tl">Drop</div></div></div>
<div class="jg" id="tj"></div></div></div>

<div class="cd"><div class="ch"><span class="ct">Quick Controls</span></div>
<div class="row"><button class="btn bs" onclick="compliant('on')">Free Move</button><button class="btn bg" onclick="compliant('off')">Lock</button><button class="btn bg" onclick="gesture('home')">Home</button><button class="btn bh" onclick="if(confirm('Reset?'))fetch('/api/reset/both',{method:'POST'}).then(()=>toast('Reset'))">Reset</button></div></div>

<div class="cd"><div class="ch"><span class="ct">Ports</span></div>
<div class="field"><div class="fl">Leader</div><input class="fi" id="lp"></div>
<div class="field"><div class="fl">Follower</div><input class="fi" id="fp"></div>
<div class="row"><button class="btn ba" onclick="savePorts()">Save</button><button class="btn bg" onclick="scanPorts()">Scan</button><button class="btn bs" onclick="autoDetect()">Auto Detect</button></div></div>
</div>

<div class="cd s2"><div class="ch"><span class="ct">Temperature History</span></div><canvas id="tc" height="75"></canvas></div>

<!-- TEACHING MODE + CALIBRATION -->
<div class="cd"><div class="ch"><span class="ct">Teaching Mode</span></div>
<div class="rb" id="rd">Ready</div>
<div class="row" style="justify-content:center;margin-bottom:14px"><button class="btn bt" id="reb" onclick="toggleRec()">Record</button><button class="btn ba" id="pb" onclick="replay()" disabled>Replay</button><button class="btn bg" onclick="saveRec()">Save</button></div>
<div style="border-top:1px solid var(--bdr);padding-top:12px"><span class="ct" style="font-size:11px">Calibration Commands</span><div id="cal-info" style="margin-top:8px">Checking...</div></div></div>

<!-- GESTURES — square icons -->
<div class="cd s2"><div class="ch"><span class="ct">Gestures</span></div>
<div class="gg">
<div class="gi" onclick="gesture('wave')"><svg viewBox="0 0 24 24"><path d="M18 11V6a2 2 0 0 0-2-2 2 2 0 0 0-2 2"/><path d="M14 10V4a2 2 0 0 0-2-2 2 2 0 0 0-2 2v2"/><path d="M10 10.5V6a2 2 0 0 0-2-2 2 2 0 0 0-2 2v8"/><path d="M18 8a2 2 0 1 1 4 0v6a8 8 0 0 1-8 8h-2c-2.8 0-4.5-.86-5.99-2.34l-3.6-3.6a2 2 0 0 1 2.83-2.82L7 15"/></svg><span>Wave</span></div>
<div class="gi" onclick="gesture('thumbsup')"><svg viewBox="0 0 24 24"><path d="M7 10v12"/><path d="M15 5.88 14 10h5.83a2 2 0 0 1 1.92 2.56l-2.33 8A2 2 0 0 1 17.5 22H4a2 2 0 0 1-2-2v-8a2 2 0 0 1 2-2h2.76a2 2 0 0 0 1.79-1.11L12 2a3.13 3.13 0 0 1 3 3.88Z"/></svg><span>Approve</span></div>
<div class="gi" onclick="gesture('point')"><svg viewBox="0 0 24 24"><path d="m9 11-6 6v3h9l3-3"/><path d="m22 12-4.6 4.6a2 2 0 0 1-2.8 0l-5.2-5.2a2 2 0 0 1 0-2.8L14 4"/></svg><span>Point</span></div>
<div class="gi" onclick="gesture('nod')"><svg viewBox="0 0 24 24"><polyline points="18 15 12 9 6 15"/></svg><span>Nod</span></div>
<div class="gi" onclick="gesture('shake')"><svg viewBox="0 0 24 24"><path d="M2 12h4"/><path d="M18 12h4"/><path d="m15 5-3 3-3-3"/><path d="m15 19-3-3-3 3"/></svg><span>Shake</span></div>
<div class="gi" onclick="gesture('dance')"><svg viewBox="0 0 24 24"><circle cx="12" cy="4" r="2"/><path d="M4.05 11.55 8 13.54a2 2 0 0 0 2.04-.07L12 12l1.93 1.47a2 2 0 0 0 2.04.07l3.98-1.99"/><path d="m7.5 15.5 1-3.5"/><path d="m16.5 15.5-1-3.5"/><path d="M9 22v-5l-3.5-2"/><path d="M15 22v-5l3.5-2"/></svg><span>Dance</span></div>
<div class="gi" onclick="gesture('home')"><svg viewBox="0 0 24 24"><path d="M3 9l9-7 9 7v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/><polyline points="9 22 9 12 15 12 15 22"/></svg><span>Home</span></div>
</div></div>
</main>

<div class="toast" id="toast"></div>
<script>
const S=io();let TO=false,RC=false;
const api=(u,m='GET',b)=>{const o={method:m,headers:{'Content-Type':'application/json'}};if(b)o.body=JSON.stringify(b);return fetch(u,o).then(r=>r.json())};
const toast=m=>{const t=document.getElementById('toast');t.textContent=m;t.classList.add('show');setTimeout(()=>t.classList.remove('show'),2500)};
const copyCmd=t=>{navigator.clipboard.writeText(t);toast('Copied!')};
function renderS(id,bid,data){const b=document.getElementById(bid);if(!data){b.textContent='offline';b.className='pill pill-off';return}
b.textContent='online';b.className='pill pill-on';
document.getElementById(id).innerHTML=data.map(s=>{const tc=s.temp>=50?'tht':s.temp>=40?'twn':'tok';
return '<div class="si"><span class="sn">'+s.name.replace('_',' ')+'</span><div class="bar"><div class="fill" style="width:'+s.pct+'%"></div></div><span class="sv">'+s.position+'</span><span class="st '+tc+'">'+s.temp+'&deg;</span></div>'}).join('')}
function drawT(h){const cv=document.getElementById('tc');if(!cv)return;const ctx=cv.getContext('2d');cv.width=cv.offsetWidth*2;cv.height=150;ctx.scale(2,2);
const w=cv.offsetWidth,ht=75;ctx.clearRect(0,0,w,ht);
const cs=['#3D348B','#4A7C59','#C08B30','#C4553A','#7B68AE','#9C9488'];
const js=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'];
js.forEach((j,i)=>{const d=h[j]||[];if(d.length<2)return;ctx.beginPath();ctx.strokeStyle=cs[i];ctx.lineWidth=2;ctx.globalAlpha=.5;d.forEach((t,x)=>{const px=(x/(d.length-1))*w,py=ht-((t-20)/40)*ht;x===0?ctx.moveTo(px,py):ctx.lineTo(px,py)});ctx.stroke();ctx.globalAlpha=1})}
function refresh(){api('/api/status/follower').then(d=>{renderS('fs','fb',d.servos);if(d.temps)drawT(d.temps);document.getElementById('fd').className='dot '+(d.servos?'on':'off')});
api('/api/status/leader').then(d=>{renderS('ls','lb',d.servos);document.getElementById('ld').className='dot '+(d.servos?'on':'off')})}
function scanPorts(){api('/api/ports').then(d=>{document.getElementById('lp').value=d.leader;document.getElementById('fp').value=d.follower})}
function savePorts(){api('/api/set_ports','POST',{leader:document.getElementById('lp').value,follower:document.getElementById('fp').value}).then(()=>{toast('Ports saved');refresh();checkCal()})}
function autoDetect(){toast('Scanning...');api('/api/auto_detect','POST').then(d=>{if(d.found.length==0){toast('No arms found');return}
document.getElementById('lp').value=d.leader||'';document.getElementById('fp').value=d.follower||'';toast('Found '+d.found.length+' arm(s): '+d.found.join(', '));refresh();checkCal()})}
function toggleTeleop(){if(!TO){api('/api/teleop/start','POST').then(d=>{if(d.error){toast(d.error);return}TO=true;document.getElementById('tr').classList.add('on');document.getElementById('tl').textContent='STOP'})}else{api('/api/teleop/stop','POST')}}
S.on('td',d=>{document.getElementById('tf').textContent=d.fps;document.getElementById('td2').textContent=d.d;
const js=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'];
document.getElementById('tj').innerHTML=js.map(j=>{const v=d.a[j+'.pos'];return '<div class="jb"><div class="jn">'+j.split('_').pop()+'</div><div class="jv">'+(v!==undefined?v.toFixed(1):'--')+'</div></div>'}).join('')});
S.on('ts',()=>{TO=false;document.getElementById('tr').classList.remove('on');document.getElementById('tl').textContent='START'});
S.on('te',d=>toast(d.e));S.on('err',d=>toast(d.e));
function toggleRec(){if(!RC){api('/api/record/start','POST').then(d=>{if(d.error){toast(d.error);return}RC=true;document.getElementById('reb').textContent='Stop';document.getElementById('rd').className='rb rec'})}
else{api('/api/record/stop','POST');RC=false;document.getElementById('reb').textContent='Record';document.getElementById('pb').disabled=false}}
S.on('rf',d=>{document.getElementById('rd').textContent=d.c+' frames \u00b7 '+d.t+'s'});
S.on('rs',d=>{document.getElementById('rd').textContent=d.c+' frames';document.getElementById('rd').className='rb done'});
function replay(){api('/api/record/replay','POST');toast('Replaying...')}
function saveRec(){api('/api/record/save','POST').then(d=>{if(d.path)toast('Saved: '+d.path)})}
S.on('rdone',()=>toast('Replay complete'));
function gesture(n){api('/api/gesture/'+n,'POST').then(d=>{if(d.error)toast(d.error);else toast(n.charAt(0).toUpperCase()+n.slice(1))})}
function compliant(a){api('/api/compliant/'+a,'POST').then(d=>{if(d.error)toast(d.error);else toast(a==='on'?'Free movement':'Locked')})}
function estop(){api('/api/estop','POST');toast('Emergency stop')}
function checkCal(){api('/api/calibration_status').then(d=>{
document.getElementById('cal-info').innerHTML=
'<div style="margin-bottom:6px"><span class="'+(d.follower?'cal-ok':'cal-no')+'">'+(d.follower?'Follower: Calibrated':'Follower: Not calibrated')+'</span><div class="cmd" onclick="copyCmd(\''+d.fcmd+'\')">'+d.fcmd+'<span style="float:right;opacity:.4">click to copy</span></div></div>'+
'<div><span class="'+(d.leader?'cal-ok':'cal-no')+'">'+(d.leader?'Leader: Calibrated':'Leader: Not calibrated')+'</span><div class="cmd" onclick="copyCmd(\''+d.lcmd+'\')">'+d.lcmd+'<span style="float:right;opacity:.4">click to copy</span></div></div>'})}
scanPorts();refresh();checkCal();setInterval(()=>{if(!TO)refresh()},3000);
</script></body></html>"""

if __name__=="__main__":
    import webbrowser
    print("\n  KIWI Control Center")
    print("  http://localhost:5000\n")
    threading.Timer(1.5,lambda:webbrowser.open("http://localhost:5000")).start()
    socketio.run(app,host="0.0.0.0",port=5000,debug=False,allow_unsafe_werkzeug=True)


