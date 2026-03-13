"""
KIWI Control Center — Final Edition
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
state={"leader_port":"","follower_port":"","teleop_running":False,"recording":False,"trajectory":[],"stop_event":threading.Event(),"temp_history":{n:[] for n in JOINTS}}
try:
    with open(".ports.json") as f: c=json.load(f); state["leader_port"]=c.get("leader",""); state["follower_port"]=c.get("follower","")
except: pass

def read_servos(port):
    if not HAS_SDK or not port: return None
    try:
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

@app.route("/")
def index(): return render_template_string(HTML)
@app.route("/api/ports")
def api_ports():
    import serial.tools.list_ports
    return jsonify({"ports":[{"d":p.device,"desc":p.description} for p in serial.tools.list_ports.comports()],"leader":state["leader_port"],"follower":state["follower_port"]})
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
    if state["teleop_running"]: return jsonify({"error":"Running"})
    if not HAS_LEROBOT: return jsonify({"error":"LeRobot missing"})
    lp2=state["leader_port"];fp2=state["follower_port"]
    if not lp2 or not fp2: return jsonify({"error":"Set ports"})
    state["teleop_running"]=True;state["stop_event"].clear()
    def run():
        dropped=0
        try:
            leader=SO101Leader(SO101LeaderConfig(port=lp2,id="leader_arm"));follower=SO101Follower(SO101FollowerConfig(port=fp2,id="follower_arm"))
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
        state["teleop_running"]=False;socketio.emit("ts",{})
    threading.Thread(target=run,daemon=True).start();return jsonify({"ok":True})
@app.route("/api/teleop/stop",methods=["POST"])
def api_teleop_stop(): state["stop_event"].set();return jsonify({"ok":True})
@app.route("/api/estop",methods=["POST"])
def api_estop():
    state["stop_event"].set()
    for port in [state["leader_port"],state["follower_port"]]:
        if not port or not HAS_SDK: continue
        try:
            p=scs.PortHandler(port);p.openPort();p.setBaudRate(1000000);pkt=scs.PacketHandler(0)
            for sid in range(1,7): pkt.write1ByteTxRx(p,sid,40,0)
            p.closePort()
        except: pass
    return jsonify({"ok":True})
@app.route("/api/gesture/<n>",methods=["POST"])
def api_gesture(name):
    port=state["follower_port"]
    if not port or not HAS_SDK: return jsonify({"error":"No port"})
    def run():
        try:
            p=scs.PortHandler(port);p.openPort();p.setBaudRate(1000000);pkt=scs.PacketHandler(0)
            def sm(tgt,dur=1.5):
                cur=[]
                for sid in range(1,7): pos,_,_=pkt.read2ByteTxRx(p,sid,56);time.sleep(.02);cur.append(pos)
                for sid in range(6): pkt.write2ByteTxRx(p,sid+1,42,cur[sid]);time.sleep(.02)
                for sid in range(1,7): pkt.write1ByteTxRx(p,sid,40,1);time.sleep(.02)
                for s in range(1,41):
                    t=s/40;t=t*t*(3-2*t)
                    for sid in range(6): pkt.write2ByteTxRx(p,sid+1,42,int(cur[sid]+(tgt[sid]-cur[sid])*t))
                    time.sleep(dur/40)
            H=[2048]*6
            if name=="wave":
                sm([2048,1400,1400,1600,2048,2800],1.0)
                for _ in range(3): sm([2048,1400,1400,1600,2500,2800],.3);sm([2048,1400,1400,1600,1600,2200],.3)
                sm(H,1.0)
            elif name=="nod":
                sm([2048,1600,1800,1800,2048,2048],.8)
                for _ in range(3): sm([2048,1800,2000,1600,2048,2048],.3);sm([2048,1400,1600,2000,2048,2048],.3)
                sm(H,.8)
            elif name=="shake":
                sm([2048,1600,1800,2048,2048,2048],.8)
                for _ in range(3): sm([1700,1600,1800,2048,2048,2048],.3);sm([2400,1600,1800,2048,2048,2048],.3)
                sm(H,.8)
            elif name=="thumbsup": sm([2048,1400,1400,1600,2048,2800],1.5);time.sleep(1.5);sm(H,1.5)
            elif name=="point": sm([2048,1400,1400,2048,2048,2048],1.5);time.sleep(1.5);sm(H,1.5)
            elif name=="dance":
                sm([2048,1600,1800,1800,2048,2400],1.0)
                for tv in range(120):
                    a=tv*2*math.pi/30;e=math.sin(tv/120*math.pi)
                    ps=[int(2048+e*300*math.sin(a)),int(1600+e*200*math.sin(a-.8)),int(1800+e*250*math.cos(a)),int(1800+e*200*math.sin(a-2)),int(2048+e*350*math.sin(a*1.5)),int(2400+e*200*math.sin(a*1.2))]
                    for sid,pos in enumerate(ps,1): pkt.write2ByteTxRx(p,sid,42,pos)
                    time.sleep(.04)
                sm(H,1.5)
            elif name=="home": sm(H,2.0)
            for sid in range(1,7): pkt.write1ByteTxRx(p,sid,40,0);time.sleep(.02)
            p.closePort()
        except Exception as e: socketio.emit("ge",{"e":str(e)})
    threading.Thread(target=run,daemon=True).start();return jsonify({"ok":True})
@app.route("/api/compliant/<action>",methods=["POST"])
def api_compliant(action):
    port=state["follower_port"]
    if not port or not HAS_SDK: return jsonify({"error":"No port"})
    try:
        p=scs.PortHandler(port);p.openPort();p.setBaudRate(1000000);pkt=scs.PacketHandler(0)
        if action=="on":
            for sid in range(1,7): pkt.write1ByteTxRx(p,sid,40,0);time.sleep(.05)
        else:
            for sid in range(1,7):
                pos,_,_=pkt.read2ByteTxRx(p,sid,56);time.sleep(.05);pkt.write2ByteTxRx(p,sid,42,pos);time.sleep(.05)
            for sid in range(1,7): pkt.write1ByteTxRx(p,sid,40,1);time.sleep(.05)
        p.closePort();return jsonify({"ok":True})
    except Exception as e: return jsonify({"error":str(e)})
@app.route("/api/record/start",methods=["POST"])
def api_rec_start():
    port=state["follower_port"]
    if not port or not HAS_SDK: return jsonify({"error":"No port"})
    state["recording"]=True;state["trajectory"]=[];state["stop_event"].clear()
    def rec():
        try:
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
        except Exception as e: socketio.emit("re2",{"e":str(e)})
        state["recording"]=False;socketio.emit("rs",{"c":len(state["trajectory"])})
    threading.Thread(target=rec,daemon=True).start();return jsonify({"ok":True})
@app.route("/api/record/stop",methods=["POST"])
def api_rec_stop(): state["stop_event"].set();return jsonify({"ok":True})
@app.route("/api/record/replay",methods=["POST"])
def api_rec_replay():
    if not state["trajectory"]: return jsonify({"error":"Empty"})
    port=state["follower_port"]
    if not port or not HAS_SDK: return jsonify({"error":"No port"})
    def play():
        try:
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
            p.closePort();socketio.emit("rdone",{})
        except Exception as e: socketio.emit("rpe",{"e":str(e)})
    threading.Thread(target=play,daemon=True).start();return jsonify({"ok":True})
@app.route("/api/record/save",methods=["POST"])
def api_rec_save():
    if not state["trajectory"]: return jsonify({"error":"Empty"})
    os.makedirs("recordings",exist_ok=True);path=f"recordings/rec_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    with open(path,"w") as f: json.dump({"frames":state["trajectory"]},f)
    return jsonify({"ok":True,"path":path})

HTML=r"""<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>KIWI Control Center</title>
<link href="https://fonts.googleapis.com/css2?family=DM+Sans:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500;600&display=swap" rel="stylesheet">
<script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.7.4/socket.io.min.js"></script>
<style>
/*
  COLOR SYSTEM — 5 shades of warm stone, 1 accent, 3 status
  bg     #E5DDD5  — page background
  card   #EDEAE5  — card background
  inner  #E5DDD5  — inputs, gesture tiles, inner boxes (same as bg)
  border #D5CEC6  — all borders
  hover  #F2F0EC  — card hover, button hover
  ink1   #2C2520  — primary text
  ink2   #6B6058  — secondary text
  ink3   #9C9488  — muted text
  accent #3D348B  — indigo, primary action
  sage   #4A7C59  — success/online
  terra  #C4553A  — danger/stop/record
  honey  #C08B30  — warning
*/
:root{
  --bg:#E5DDD5;--card:#EDEAE5;--inner:#E5DDD5;--bdr:#D5CEC6;--hover:#F2F0EC;
  --ink1:#2C2520;--ink2:#6B6058;--ink3:#9C9488;
  --accent:#3D348B;--sage:#4A7C59;--terra:#C4553A;--honey:#C08B30;
  --r:16px;--r2:12px;--r3:8px
}
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:'DM Sans',sans-serif;background:var(--bg);color:var(--ink1);-webkit-font-smoothing:antialiased}

/* HEADER */
header{position:sticky;top:0;z-index:100;height:56px;display:flex;align-items:center;justify-content:space-between;padding:0 32px;background:var(--card);border-bottom:1px solid var(--bdr)}
.logo{display:flex;align-items:center;gap:12px}
.logo-mark{width:32px;height:32px;border-radius:10px;background:var(--accent);display:grid;place-items:center}
.logo-mark svg{width:18px;height:18px;fill:none;stroke:#fff;stroke-width:2;stroke-linecap:round;stroke-linejoin:round}
.logo h1{font-size:16px;font-weight:700;color:var(--ink1)}
.logo small{font-size:10px;color:var(--ink3);display:block;margin-top:-2px}
.hdr-r{display:flex;align-items:center;gap:16px}
.hdr-s{font-size:11px;color:var(--ink3);display:flex;align-items:center;gap:5px}
.dot{width:6px;height:6px;border-radius:50%}.dot.on{background:var(--sage)}.dot.off{background:var(--bdr)}
.estop{background:var(--terra);color:#fff;border:none;height:32px;padding:0 16px;border-radius:var(--r3);font-size:11px;font-weight:700;cursor:pointer;transition:.2s}
.estop:hover{filter:brightness(1.1)}

/* PAGE */
main{display:grid;grid-template-columns:1fr 1fr 280px;gap:16px;padding:20px 32px 40px;max-width:1440px;margin:0 auto}
@media(max-width:1200px){main{grid-template-columns:1fr 1fr}.side{grid-column:1/-1;display:grid;grid-template-columns:1fr 1fr 1fr;gap:16px}}
@media(max-width:768px){main{grid-template-columns:1fr;padding:12px}.side{grid-template-columns:1fr}}
.span2{grid-column:span 2}@media(max-width:768px){.span2{grid-column:span 1}}

/* CARD */
.card{background:var(--card);overflow:hidden;border:1px solid var(--bdr);border-radius:var(--r);padding:20px;transition:.2s}
.card:hover{background:var(--hover)}
.card-head{display:flex;align-items:center;justify-content:space-between;margin-bottom:16px}
.card-title{font-size:13px;font-weight:700;color:var(--ink1)}
.pill{font-size:10px;font-weight:600;padding:3px 10px;border-radius:20px}
.pill-on{background:rgba(74,124,89,0.12);color:var(--sage)}
.pill-off{background:rgba(196,85,58,0.1);color:var(--terra)}

/* SERVO LIST */
.srv-list{display:flex;flex-direction:column;gap:2px}
.srv{display:grid;grid-template-columns:100px 1fr 44px 38px;align-items:center;gap:10px;padding:10px 8px;border-radius:var(--r3)}
.srv:hover{background:var(--bg)}
.srv-name{font-size:12px;color:var(--ink3)}
.srv-bar{height:5px;background:var(--bdr);border-radius:3px;overflow:hidden}
.srv-fill{height:100%;border-radius:3px;background:var(--accent);transition:width .5s ease}
.srv-val{font-family:'JetBrains Mono',monospace;font-size:11px;font-weight:600;text-align:right;color:var(--ink2)}
.srv-tmp{font-family:'JetBrains Mono',monospace;font-size:11px;font-weight:600;text-align:right}
.t-ok{color:var(--sage)}.t-wn{color:var(--honey)}.t-ht{color:var(--terra)}

/* TELEOP */
.tele-wrap{display:flex;flex-direction:column;align-items:center;gap:14px;padding:8px 0}
.tele-btn{width:100px;height:100px;border-radius:50%;border:2.5px solid var(--bdr);background:var(--bg);display:grid;place-items:center;cursor:pointer;transition:.3s}
.tele-btn:hover{border-color:var(--accent);background:var(--hover)}
.tele-btn.on{border-color:var(--terra);background:rgba(196,85,58,0.06)}
.tele-btn span{font-size:13px;font-weight:700;letter-spacing:.5px;color:var(--accent)}
.tele-btn.on span{color:var(--terra)}
.tele-stats{display:flex;gap:28px}
.tele-stat{text-align:center}
.tele-num{font-family:'JetBrains Mono',monospace;font-size:22px;font-weight:700;color:var(--accent)}
.tele-lbl{font-size:9px;color:var(--ink3);text-transform:uppercase;letter-spacing:1.5px;margin-top:2px}
.joint-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;width:100%}
.joint-box{background:var(--bg);border:1px solid var(--bdr);border-radius:var(--r3);padding:8px;text-align:center}
.joint-box .jn{font-size:9px;color:var(--ink3);text-transform:uppercase;letter-spacing:.5px}
.joint-box .jv{font-family:'JetBrains Mono',monospace;font-size:15px;font-weight:700;color:var(--accent);margin-top:2px}

/* GESTURES */
.gest-grid{display:grid;grid-template-columns:repeat(7,1fr);gap:10px}
.gest{background:var(--bg);border:1px solid var(--bdr);border-radius:var(--r2);padding:18px 8px;text-align:center;cursor:pointer;transition:.25s}
.gest:hover{background:var(--hover);border-color:var(--accent);transform:translateY(-2px)}
.gest:active{transform:translateY(0)}
.gest svg{width:24px;height:24px;stroke:var(--ink3);fill:none;stroke-width:1.5;stroke-linecap:round;stroke-linejoin:round;margin:0 auto 8px;display:block;transition:.25s}
.gest:hover svg{stroke:var(--accent)}
.gest span{font-size:11px;font-weight:600;color:var(--ink2);display:block;transition:.25s}
.gest:hover span{color:var(--accent)}

/* BUTTONS */
.btn{display:inline-flex;align-items:center;gap:5px;padding:8px 16px;border:none;border-radius:var(--r3);font-size:11px;font-weight:600;cursor:pointer;transition:.2s;font-family:inherit}
.btn:hover{transform:translateY(-1px)}.btn:active{transform:translateY(0)}
.btn svg{width:13px;height:13px;fill:none;stroke:currentColor;stroke-width:2;stroke-linecap:round;stroke-linejoin:round}
.btn-accent{background:var(--accent);color:#fff}
.btn-sage{background:var(--sage);color:#fff}
.btn-terra{background:var(--terra);color:#fff}
.btn-honey{background:var(--honey);color:#fff}
.btn-ghost{background:var(--bg);color:var(--ink2);border:1px solid var(--bdr)}.btn-ghost:hover{background:var(--hover)}

/* INPUT */
.field{margin-bottom:10px}
.field-label{font-size:10px;font-weight:600;color:var(--ink3);margin-bottom:4px}
.field-input{width:100%;background:var(--bg);border:1.5px solid var(--bdr);border-radius:var(--r3);padding:9px 12px;color:var(--ink1);font-family:'JetBrains Mono',monospace;font-size:12px;outline:none;transition:.2s}
.field-input:focus{border-color:var(--accent)}

/* RECORD */
.rec-box{font-family:'JetBrains Mono',monospace;font-size:18px;font-weight:600;text-align:center;padding:14px;border-radius:var(--r3);background:var(--bg);border:1px solid var(--bdr);margin-bottom:12px;color:var(--ink3)}
.rec-box.rec{color:var(--terra);animation:blink 1s ease infinite}
.rec-box.done{color:var(--sage)}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.3}}

/* CANVAS — transparent, no background */
canvas{display:block;border-radius:var(--r3);background:transparent;width:100%;max-width:100%}

/* TOAST */
.toast{position:fixed;bottom:24px;left:50%;transform:translateX(-50%) translateY(50px);padding:10px 24px;border-radius:var(--r2);font-size:12px;font-weight:500;z-index:1000;opacity:0;transition:.4s;pointer-events:none;background:var(--ink1);color:var(--card)}
.toast.show{opacity:1;transform:translateX(-50%) translateY(0)}

.side{display:flex;flex-direction:column;gap:16px}
.row{display:flex;gap:8px;flex-wrap:wrap}

/* ENTER ANIMATION */
main>*{animation:up .4s ease both}
main>:nth-child(1){animation-delay:.03s}main>:nth-child(2){animation-delay:.06s}main>:nth-child(3){animation-delay:.09s}
main>:nth-child(4){animation-delay:.12s}main>:nth-child(5){animation-delay:.15s}main>:nth-child(6){animation-delay:.18s}main>:nth-child(7){animation-delay:.21s}
@keyframes up{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:translateY(0)}}
::selection{background:rgba(61,52,139,0.15)}
</style></head><body>

<header>
  <div class="logo">
    <div class="logo-mark"><svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="10"/><path d="M8 14s1.5 2 4 2 4-2 4-2"/><line x1="9" y1="9" x2="9.01" y2="9"/><line x1="15" y1="9" x2="15.01" y2="9"/></svg></div>
    <div><h1>KIWI Control Center</h1><small>SO-101 Arm Lab</small></div>
  </div>
  <div class="hdr-r">
    <div class="hdr-s">Leader <span class="dot" id="ld"></span></div>
    <div class="hdr-s">Follower <span class="dot" id="fd"></span></div>
    <button class="estop" onclick="estop()">STOP</button>
  </div>
</header>

<main>
  <!-- FOLLOWER -->
  <div class="card">
    <div class="card-head"><span class="card-title">Follower Arm</span><span class="pill pill-off" id="fb">offline</span></div>
    <div class="srv-list" id="fs"><div style="color:var(--ink3);text-align:center;padding:28px">Connecting...</div></div>
  </div>

  <!-- LEADER -->
  <div class="card">
    <div class="card-head"><span class="card-title">Leader Arm</span><span class="pill pill-off" id="lb">offline</span></div>
    <div class="srv-list" id="ls"><div style="color:var(--ink3);text-align:center;padding:28px">Connecting...</div></div>
  </div>

  <!-- SIDEBAR -->
  <div class="side">
    <div class="card">
      <div class="card-head"><span class="card-title">Teleoperation</span></div>
      <div class="tele-wrap">
        <div class="tele-btn" id="tr" onclick="toggleTeleop()"><span id="tl">START</span></div>
        <div class="tele-stats">
          <div class="tele-stat"><div class="tele-num" id="tf">--</div><div class="tele-lbl">FPS</div></div>
          <div class="tele-stat"><div class="tele-num" id="td2">0</div><div class="tele-lbl">Drop</div></div>
        </div>
        <div class="joint-grid" id="tj"></div>
      </div>
    </div>

    <div class="card">
      <div class="card-head"><span class="card-title">Quick Controls</span></div>
      <div class="row">
        <button class="btn btn-sage" onclick="compliant('on')"><svg viewBox="0 0 24 24"><path d="M18 8h1a4 4 0 0 1 0 8h-1M2 8h16v9a4 4 0 0 1-4 4H6a4 4 0 0 1-4-4V8z"/></svg>Free Move</button>
        <button class="btn btn-ghost" onclick="compliant('off')"><svg viewBox="0 0 24 24"><rect x="3" y="11" width="18" height="11" rx="2"/><path d="M7 11V7a5 5 0 0 1 10 0v4"/></svg>Lock</button>
        <button class="btn btn-ghost" onclick="gesture('home')"><svg viewBox="0 0 24 24"><path d="M3 9l9-7 9 7v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/></svg>Home</button>
      </div>
    </div>

    <div class="card">
      <div class="card-head"><span class="card-title">Ports</span></div>
      <div class="field"><div class="field-label">Leader</div><input class="field-input" id="lp"></div>
      <div class="field"><div class="field-label">Follower</div><input class="field-input" id="fp"></div>
      <div class="row"><button class="btn btn-accent" onclick="savePorts()">Save</button><button class="btn btn-ghost" onclick="scanPorts()">Scan</button></div>
    </div>
  </div>

  <!-- TEMPERATURE -->
  <div class="card span2">
    <div class="card-head"><span class="card-title">Temperature History</span></div>
    <canvas id="tc" height="75"></canvas>
  </div>

  <!-- RECORD -->
  <div class="card">
    <div class="card-head"><span class="card-title">Teaching Mode</span></div>
    <div class="rec-box" id="rd">Ready</div>
    <div class="row" style="justify-content:center">
      <button class="btn btn-terra" id="rb" onclick="toggleRec()"><svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="10"/></svg>Record</button>
      <button class="btn btn-accent" id="pb" onclick="replay()" disabled><svg viewBox="0 0 24 24"><polygon points="5 3 19 12 5 21"/></svg>Replay</button>
      <button class="btn btn-ghost" onclick="saveRec()"><svg viewBox="0 0 24 24"><path d="M19 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h11l5 5v11a2 2 0 0 1-2 2z"/></svg>Save</button>
    </div>
  </div>

  <!-- GESTURES -->
  <div class="card span2">
    <div class="card-head"><span class="card-title">Gestures</span></div>
    <div class="gest-grid">
      <div class="gest" onclick="gesture('wave')"><svg viewBox="0 0 24 24"><path d="M18 11V6a2 2 0 0 0-2-2 2 2 0 0 0-2 2"/><path d="M14 10V4a2 2 0 0 0-2-2 2 2 0 0 0-2 2v2"/><path d="M10 10.5V6a2 2 0 0 0-2-2 2 2 0 0 0-2 2v8"/><path d="M18 8a2 2 0 1 1 4 0v6a8 8 0 0 1-8 8h-2c-2.8 0-4.5-.86-5.99-2.34l-3.6-3.6a2 2 0 0 1 2.83-2.82L7 15"/></svg><span>Wave</span></div>
      <div class="gest" onclick="gesture('thumbsup')"><svg viewBox="0 0 24 24"><path d="M7 10v12"/><path d="M15 5.88 14 10h5.83a2 2 0 0 1 1.92 2.56l-2.33 8A2 2 0 0 1 17.5 22H4a2 2 0 0 1-2-2v-8a2 2 0 0 1 2-2h2.76a2 2 0 0 0 1.79-1.11L12 2a3.13 3.13 0 0 1 3 3.88Z"/></svg><span>Approve</span></div>
      <div class="gest" onclick="gesture('point')"><svg viewBox="0 0 24 24"><path d="m9 11-6 6v3h9l3-3"/><path d="m22 12-4.6 4.6a2 2 0 0 1-2.8 0l-5.2-5.2a2 2 0 0 1 0-2.8L14 4"/></svg><span>Point</span></div>
      <div class="gest" onclick="gesture('nod')"><svg viewBox="0 0 24 24"><path d="M12 2v4"/><path d="m4.93 10.93 2.83 2.83"/><path d="M2 18h4"/><path d="m19.07 10.93-2.83 2.83"/><path d="M22 18h-4"/><path d="M12 22v-4"/></svg><span>Nod</span></div>
      <div class="gest" onclick="gesture('shake')"><svg viewBox="0 0 24 24"><path d="M2 12h4"/><path d="M18 12h4"/><path d="m15 5-3 3-3-3"/><path d="m15 19-3-3-3 3"/></svg><span>Shake</span></div>
      <div class="gest" onclick="gesture('dance')"><svg viewBox="0 0 24 24"><circle cx="12" cy="4" r="2"/><path d="M4.05 11.55 8 13.54a2 2 0 0 0 2.04-.07L12 12l1.93 1.47a2 2 0 0 0 2.04.07l3.98-1.99"/><path d="m7.5 15.5 1-3.5"/><path d="m16.5 15.5-1-3.5"/><path d="M9 22v-5l-3.5-2"/><path d="M15 22v-5l3.5-2"/></svg><span>Dance</span></div>
      <div class="gest" onclick="gesture('home')"><svg viewBox="0 0 24 24"><path d="M3 9l9-7 9 7v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"/><polyline points="9 22 9 12 15 12 15 22"/></svg><span>Home</span></div>
    </div>
  </div>
</main>

<div class="toast" id="toast"></div>

<script>
const S=io();let TO=false,RC=false;
const api=(u,m='GET',b)=>{const o={method:m,headers:{'Content-Type':'application/json'}};if(b)o.body=JSON.stringify(b);return fetch(u,o).then(r=>r.json())};
const toast=m=>{const t=document.getElementById('toast');t.textContent=m;t.classList.add('show');setTimeout(()=>t.classList.remove('show'),2500)};

function renderS(id,bid,data){
  const b=document.getElementById(bid);
  if(!data){b.textContent='offline';b.className='pill pill-off';return}
  b.textContent='online';b.className='pill pill-on';
  document.getElementById(id).innerHTML=data.map(s=>{
    const tc=s.temp>=50?'t-ht':s.temp>=40?'t-wn':'t-ok';
    return '<div class="srv"><span class="srv-name">'+s.name.replace('_',' ')+'</span><div class="srv-bar"><div class="srv-fill" style="width:'+s.pct+'%"></div></div><span class="srv-val">'+s.position+'</span><span class="srv-tmp '+tc+'">'+s.temp+'&deg;</span></div>';
  }).join('');
}

function drawT(h){
  const cv=document.getElementById('tc');if(!cv)return;
  const ctx=cv.getContext('2d');
  cv.width=cv.offsetWidth*2;cv.height=150;ctx.scale(2,2);
  const w=cv.offsetWidth,ht=75;
  ctx.clearRect(0,0,w,ht);
  const cs=['#3D348B','#4A7C59','#C08B30','#C4553A','#7B68AE','#9C9488'];
  const js=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'];
  js.forEach((j,i)=>{
    const d=h[j]||[];if(d.length<2)return;
    ctx.beginPath();ctx.strokeStyle=cs[i];ctx.lineWidth=2;ctx.globalAlpha=0.5;
    d.forEach((t,x)=>{const px=(x/(d.length-1))*w,py=ht-((t-20)/40)*ht;x===0?ctx.moveTo(px,py):ctx.lineTo(px,py)});
    ctx.stroke();ctx.globalAlpha=1;
  });
}

function refresh(){
  api('/api/status/follower').then(d=>{renderS('fs','fb',d.servos);if(d.temps)drawT(d.temps);document.getElementById('fd').className='dot '+(d.servos?'on':'off')});
  api('/api/status/leader').then(d=>{renderS('ls','lb',d.servos);document.getElementById('ld').className='dot '+(d.servos?'on':'off')});
}

function scanPorts(){api('/api/ports').then(d=>{document.getElementById('lp').value=d.leader;document.getElementById('fp').value=d.follower})}
function savePorts(){api('/api/set_ports','POST',{leader:document.getElementById('lp').value,follower:document.getElementById('fp').value}).then(()=>toast('Ports saved'))}

function toggleTeleop(){
  if(!TO){api('/api/teleop/start','POST').then(d=>{if(d.error){toast(d.error);return}TO=true;document.getElementById('tr').classList.add('on');document.getElementById('tl').textContent='STOP'})}
  else{api('/api/teleop/stop','POST')}
}
S.on('td',d=>{
  document.getElementById('tf').textContent=d.fps;document.getElementById('td2').textContent=d.d;
  const js=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'];
  document.getElementById('tj').innerHTML=js.map(j=>{const v=d.a[j+'.pos'];return '<div class="joint-box"><div class="jn">'+j.split('_').pop()+'</div><div class="jv">'+(v!==undefined?v.toFixed(1):'--')+'</div></div>'}).join('');
});
S.on('ts',()=>{TO=false;document.getElementById('tr').classList.remove('on');document.getElementById('tl').textContent='START'});
S.on('te',d=>toast(d.e));

function toggleRec(){
  if(!RC){api('/api/record/start','POST').then(d=>{if(d.error){toast(d.error);return}RC=true;document.getElementById('rb').innerHTML='<svg viewBox="0 0 24 24"><rect x="6" y="6" width="12" height="12" rx="1"/></svg>Stop';document.getElementById('rd').className='rec-box rec'})}
  else{api('/api/record/stop','POST');RC=false;document.getElementById('rb').innerHTML='<svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="10"/></svg>Record';document.getElementById('pb').disabled=false}
}
S.on('rf',d=>{document.getElementById('rd').textContent=d.c+' frames \u00b7 '+d.t+'s'});
S.on('rs',d=>{document.getElementById('rd').textContent=d.c+' frames';document.getElementById('rd').className='rec-box done'});
function replay(){api('/api/record/replay','POST');toast('Replaying...')}
function saveRec(){api('/api/record/save','POST').then(d=>{if(d.path)toast('Saved')})}
S.on('rdone',()=>toast('Done'));

function gesture(n){api('/api/gesture/'+n,'POST');toast(n.charAt(0).toUpperCase()+n.slice(1))}
function compliant(a){api('/api/compliant/'+a,'POST').then(()=>toast(a==='on'?'Free movement':'Locked'))}
function estop(){api('/api/estop','POST');toast('Emergency stop')}

scanPorts();refresh();setInterval(()=>{if(!TO)refresh()},3000);
</script>
</body></html>"""

if __name__=="__main__":
    import webbrowser
    print("\n  KIWI Control Center")
    print("  http://localhost:5000\n")
    threading.Timer(1.5,lambda:webbrowser.open("http://localhost:5000")).start()
    socketio.run(app,host="0.0.0.0",port=5000,debug=False,allow_unsafe_werkzeug=True)

