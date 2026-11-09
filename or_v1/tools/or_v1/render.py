#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""Offscreen snapshots of the OR v1 scene at `home` (optionally after N s of physics).
    MUJOCO_GL=egl .venv/bin/python tools/or_v1/render.py out/or_v1 [--sim 2.0]"""
import argparse, sys
from pathlib import Path
import mujoco
from PIL import Image

XML = Path(__file__).resolve().parents[2] / "third_party/openarm_mujoco/v1/dental_or/dental_or_scene_v1.0.xml"
ap = argparse.ArgumentParser(); ap.add_argument("prefix"); ap.add_argument("--sim", type=float, default=0.0)
ap.add_argument("--cams", default="wide,staging,robot_over,tray_top")
a = ap.parse_args()
m = mujoco.MjModel.from_xml_path(str(XML)); d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, m.key("home").id); mujoco.mj_forward(m, d)
while d.time < a.sim:
    mujoco.mj_step(m, d)
r = mujoco.Renderer(m, 720, 1080)
Path(a.prefix).parent.mkdir(parents=True, exist_ok=True)
for c in a.cams.split(","):
    r.update_scene(d, camera=c); Image.fromarray(r.render()).save(f"{a.prefix}_{c}.png"); print(f"{a.prefix}_{c}.png")
