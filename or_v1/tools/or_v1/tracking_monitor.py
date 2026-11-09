#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
Live VR tracking check. Listens to the bridge's mirror stream (UDP 5007), runs the SAME
conversion as dora's quest-receiver, and prints what the robot is being asked to do.

    .venv/bin/python tools/or_v1/tracking_monitor.py            # while dora + bridge run

Columns (robot arm_origin frame, cm):  fwd = toward the cart, left, up
  * "rate" should be ~60 Hz, "valid" L1 R1
  * Press ENTER to set a zero point, then move one hand ~20 cm forward / up / right:
    the matching column must change by ~20 and the others stay ~0.
  * dist = distance from the arm's shoulder; > ~60 cm is beyond reach (arm will lag).
"""
import json, socket, sys, select, time
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from dora_openarm_vr.quest_receiver import QuestPoseProcessor  # noqa: E402

SHOULDER = {"left": np.array([0.0, 0.093, 0.0]), "right": np.array([0.0, -0.093, 0.0])}
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", int(sys.argv[1]) if len(sys.argv) > 1 else 5007))
sock.setblocking(False)
proc = QuestPoseProcessor()
zero = None
watch = [sock, sys.stdin] if sys.stdin.isatty() else [sock]
n, t_rate, rate, last = 0, time.time(), 0.0, None
print("waiting for bridge packets on 5007 ... (ENTER = set zero)")
while True:
    r, _, _ = select.select(watch, [], [], 0.5)
    if sys.stdin in r:
        if not sys.stdin.readline():
            watch = [sock]
        if last is not None:
            zero = {k: v.copy() for k, v in last.items()}
            print("---- zero set ----")
    if sock not in r:
        if time.time() - t_rate > 2:
            print("  (no packets — is tools/or_v1/vr_web_bridge.sh running?)"); t_rate = time.time()
        continue
    while True:
        try:
            msg = json.loads(sock.recv(65535))
        except BlockingIOError:
            break
        n += 1
    pr, pl, _ = proc.process(msg)
    last = {"right": pr[:3].astype(float), "left": pl[:3].astype(float)}
    now = time.time()
    if now - t_rate >= 0.5:
        rate = n / (now - t_rate); n, t_rate = 0, now
        out = []
        for side in ("left", "right"):
            p = last[side] - (zero[side] if zero else 0)
            dist = np.linalg.norm(last[side] - SHOULDER[side]) * 100
            out.append(f"{side[0].upper()}: fwd{p[0]*100:+6.1f} left{p[1]*100:+6.1f} up{p[2]*100:+6.1f} "
                       f"dist{dist:5.0f}{'!' if dist > 60 else ' '} trig{msg['lt' if side == 'left' else 'rt']:.2f}")
        print(f"rate {rate:4.0f}Hz valid L{int(msg['vl'] == 0)} R{int(msg['vr'] == 0)} | " + " | ".join(out), flush=True)
