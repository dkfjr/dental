#!/usr/bin/env python3
"""Regenerate the `home` keyframe in sim/main_v3_teleop.xml.
Run after editing the teleop scene. Sets arms to a ready pose (joint4=pi/2),
keeps everything else at authored rest.  Usage: python teleop/tools/build_home_keyframe.py"""
import re, pathlib, mujoco, numpy as np
SIM = pathlib.Path(__file__).resolve().parents[2] / "sim"
src = SIM / "main_v3_teleop.xml"
HOME_CTRL = [0,0,0,1.570796,0,0,0,0.0, 0,0,0,1.570796,0,0,0,0.0]
text = re.sub(r"\n  <keyframe>.*?</keyframe>\n", "\n", src.read_text(), flags=re.S)
src.write_text(text)
m = mujoco.MjModel.from_xml_path(str(src)); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
q = d.qpos.copy()
for a in range(m.nu): q[m.jnt_qposadr[m.actuator_trnid[a,0]]] = HOME_CTRL[a]
key = f'\n  <keyframe>\n    <key name="home" qpos="{" ".join(f"{v:.5f}" for v in q)}" ctrl="{" ".join(f"{v:.5f}" for v in HOME_CTRL)}"/>\n  </keyframe>\n'
src.write_text(text.replace("\n</mujoco>", key + "</mujoco>"))
print("home keyframe rebuilt in", src.name)
