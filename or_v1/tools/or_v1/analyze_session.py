#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
Analyse a VR session recorded by the bridge (tools/or_v1/start_vr.sh -> out/sessions/*.csv).

Finds what makes the robot "move on its own":
  * tracking dropouts (controller/head invalid) and their durations
  * raw controller jumps (> 5 cm or > 30 deg between consecutive frames = tracking glitch)
  * Y presses (re-calibration)
  * replays the recorded targets through the dora IK and lists joint flips (> 0.15 rad/tick)

    .venv/bin/python tools/or_v1/analyze_session.py out/sessions/vr_<time>.csv [--no-ik]
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("--no-ik", action="store_true")
ap.add_argument("--ik", default="--max-iters 30 --dt 0.1 --damping 0.05 --posture-cost 0.0 "
                                "--lm-damping 0.005 --no-joint-braking")
a = ap.parse_args()

rows = np.genfromtxt(a.csv, delimiter=",", names=True)
t = rows["t"]
dt = np.diff(t)
print(f"{a.csv}: {len(t)} frames, {t[-1]:.1f} s, {len(t) / max(t[-1], 1e-6):.0f} Hz "
      f"(max gap {dt.max() * 1000:.0f} ms)")


def spans(mask):
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = i
        if not m and start is not None:
            out.append((t[start], t[i - 1])); start = None
    if start is not None:
        out.append((t[start], t[-1]))
    return out


for side, k in (("right", "raw_rok"), ("left", "raw_lok"), ("head", "raw_hok")):
    sp = spans(rows[k] < 0.5)
    tot = sum(b - a_ for a_, b in sp)
    print(f"  {side:5s} tracking lost: {len(sp)} times, {tot:.1f} s total"
          + (f"  e.g. {', '.join(f'{a_:.1f}-{b:.1f}s' for a_, b in sp[:5])}" if sp else ""))

for side, p in (("right", "raw_r"), ("left", "raw_l")):
    P = np.stack([rows[p + c] for c in "xyz"], 1)
    Q = np.stack([rows[p + "q" + c] for c in "xyzw"], 1)
    ok = rows[p + "ok"] > 0.5
    jumps = []
    for i in range(1, len(t)):
        if ok[i] and ok[i - 1]:
            dp = np.linalg.norm(P[i] - P[i - 1])
            da = np.degrees((R.from_quat(Q[i]) * R.from_quat(Q[i - 1]).inv()).magnitude())
            if dp > 0.05 or da > 30:
                jumps.append((t[i], dp, da))
    print(f"  {side:5s} raw jumps (>5cm or >30deg in one frame): {len(jumps)}"
          + (f"  e.g. {', '.join(f'{x:.1f}s({d*100:.0f}cm,{g:.0f}deg)' for x, d, g in jumps[:5])}" if jumps else ""))

ypress = t[1:][(rows["y_btn"][1:] > 0.5) & (rows["y_btn"][:-1] < 0.5)]
print(f"  Y presses at: {', '.join(f'{x:.1f}s' for x in ypress) or 'none'}; "
      f"calibrated from {t[np.argmax(rows['calibrated'] > 0.5)]:.1f}s" if (rows["calibrated"] > 0.5).any()
      else "  never calibrated (Y not pressed)")

if a.no_ik:
    sys.exit()
from dora_openarm_vr.quest_receiver import QuestPoseProcessor  # noqa: E402
from openarm_control.config import register_common_args, setup_from_args  # noqa: E402
from openarm_control.ik_params import register_ik_args, ik_params_from_args  # noqa: E402
from openarm_control.kinematics import Kinematics  # noqa: E402

XML = str(ROOT / "third_party/openarm_mujoco/v1/dental_or/dental_or_scene_v1.0.xml")
p2 = argparse.ArgumentParser(); register_common_args(p2); register_ik_args(p2)
kin = Kinematics(setup_from_args(p2.parse_args(f"--xml {XML} --keyframe home --mode bimanual {a.ik}".split())),
                 ik_params_from_args(p2.parse_args(f"--xml {XML} --keyframe home --mode bimanual {a.ik}".split())))
proc = QuestPoseProcessor()
IDENT = {"x": 0, "y": 0, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1}
prev, flips = None, []
for i in range(len(t)):
    if rows["vl"][i] < 0.5 or rows["vr"][i] < 0.5 or rows["calibrated"][i] < 0.5:
        continue
    pk = {"rf": IDENT,
          "rc": {k: rows["r" + k][i] for k in ("x", "y", "z", "qx", "qy", "qz", "qw")},
          "lc": {k: rows["l" + k][i] for k in ("x", "y", "z", "qx", "qy", "qz", "qw")}}
    pr, pl, _ = proc.process(pk)
    kin.set_target("right", pr[:7]); kin.set_target("left", pl[:7])
    kin.set_gripper("right", 0); kin.set_gripper("left", 0)
    res = kin.solve()
    if res is None:
        continue
    q = np.concatenate([res[0:7], res[8:15]])
    if prev is not None and np.abs(q - prev).max() > 0.15:
        flips.append((t[i], np.abs(q - prev).max(), int(np.abs(q - prev).argmax())))
    prev = q
print(f"  IK replay: {len(flips)} joint flips (>0.15 rad/tick)"
      + (f"  e.g. {', '.join(f'{x:.1f}s({d:.2f}rad, {'RL'[j // 7]}{j % 7 + 1})' for x, d, j in flips[:6])}" if flips else ""))
