#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""Physics smoke test: right arm picks an instrument off the drape (position actuators,
contacts, gravity, friction all live). Prints instrument lift height.
    .venv/bin/python tools/or_v1/grasp_test.py [instrument] [--video out.mp4]"""
import argparse, sys
from pathlib import Path
import mujoco, numpy as np

ROOT = Path(__file__).resolve().parents[2]
XML = ROOT / "third_party/openarm_mujoco/v1/dental_or/dental_or_scene_v1.0.xml"
ap = argparse.ArgumentParser(); ap.add_argument("instr", nargs="?", default="elevator_cryer")
ap.add_argument("--close-time", type=float, default=0.7, help="seconds to go from open to closed");
ap.add_argument("--video"); ap.add_argument("--side", default="right", choices=("left", "right")); ap.add_argument("--open", type=float, default=0.044, help="finger opening ctrl before closing (0.044 = full)"); ap.add_argument("--grip-x", type=float, default=-0.035,
                                          help="grasp point along the instrument (m, local x; handle<0)")
a = ap.parse_args()
m = mujoco.MjModel.from_xml_path(str(XML)); d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, m.key("home").id); mujoco.mj_forward(m, d)
side = a.side
qa = [m.jnt_qposadr[m.joint(f"openarm_{side}_joint{i}").id] for i in range(1, 8)]
da = [m.jnt_dofadr[m.joint(f"openarm_{side}_joint{i}").id] for i in range(1, 8)]
act = [m.actuator(f"{side}_joint{i}_ctrl").id for i in range(1, 8)]
fing = m.actuator(f"{side}_finger1_ctrl").id
lo = m.actuator_ctrlrange[act, 0]; hi = m.actuator_ctrlrange[act, 1]
tcp = m.body(f"openarm_{side}_hand_tcp").id; sid = m.site(f"{side}_ee_control_point").id
ib = m.body(f"instr_{a.instr}").id

def ik(target, yaw_axis, q0, iters=400):
    """TCP at target, approach down, finger-closing axis across the instrument."""
    dd = mujoco.MjData(m); dd.qpos[:] = d.qpos; dd.qpos[qa] = q0
    for _ in range(iters):
        mujoco.mj_kinematics(m, dd); mujoco.mj_comPos(m, dd)
        R = dd.site_xmat[sid].reshape(3, 3)
        e_ori = np.cross(-R[:, 2], [0, 0, -1.0]) + 0.5 * np.cross(R[:, 1], yaw_axis)
        e = np.hstack([target - dd.xpos[tcp], 0.5 * e_ori])
        jp = np.zeros((3, m.nv)); jr = np.zeros((3, m.nv)); mujoco.mj_jac(m, dd, jp, jr, dd.xpos[tcp], tcp)
        J = np.vstack([jp[:, da], 0.5 * jr[:, da]])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
        dd.qpos[qa] = np.clip(dd.qpos[qa] + 0.5 * dq, lo, hi)
    mujoco.mj_kinematics(m, dd)
    return dd.qpos[qa].copy(), np.linalg.norm(target - dd.xpos[tcp])

R_i = d.xmat[ib].reshape(3, 3); long_ax = R_i[:, 0]
gp = d.xpos[ib] + R_i @ np.array([a.grip_x, 0, 0])
z_top = d.xpos[ib][2] + m.geom_size[m.geom(f"col_{a.instr}").id][2]
across = np.cross([0, 0, 1.0], long_ax)            # fingers close across the long axis
# the finger slide axis is the hand's local y; align it with `across`, try both signs
best = None
for sgn in (1, -1):
    q = d.qpos[qa].copy()
    for k in range(8):   # restarts
        qs, e = ik(gp + [0, 0, 0.10], sgn * across, q if k == 0 else np.random.default_rng(k).uniform(lo, hi))
        if best is None or e < best[1]:
            best = (qs, e, sgn)
q_pre, e_pre, sgn = best
# v1 fingertips reach 15.4 mm beyond the TCP: keep them 1.5 mm above the drape (z=0.912)
TABLE_TOP, TIP = 0.912, 0.0154
q_grasp, e_g = ik(np.array([gp[0], gp[1], max(gp[2] + 0.004, TABLE_TOP + TIP + 0.0015)]), sgn * across, q_pre)
q_lift, e_l = ik(gp + [0, 0, 0.12], sgn * across, q_grasp)
print(f"IK errors: pre={e_pre*1000:.1f}mm grasp={e_g*1000:.1f}mm lift={e_l*1000:.1f}mm")

frames = []; r = mujoco.Renderer(m, 480, 640) if a.video else None
max_pen = {"finger-instr": 0.0, "instr-table": 0.0}
def run(q_to, grip, T, grip_T=None):
    q_from = d.ctrl[act].copy(); g_from = d.ctrl[fing]; t0 = d.time
    while d.time - t0 < T:
        s = min(1.0, (d.time - t0) / (0.7 * T)); s = s * s * (3 - 2 * s)
        d.ctrl[act] = (1 - s) * q_from + s * q_to
        sg = 1.0 if not grip_T else min(1.0, (d.time - t0) / grip_T)
        d.ctrl[fing] = g_from + sg * (grip - g_from)
        mujoco.mj_step(m, d)
        for c in d.contact[:d.ncon]:
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if ib in (b1, b2):
                other = m.body(b2 if b1 == ib else b1).name
                k = "finger-instr" if "finger" in other or "hand" in other else ("instr-table" if other == "or_env" else None)
                if k:
                    max_pen[k] = max(max_pen[k], -c.dist)
        if r is not None and len(frames) < 5000 and int(d.time / m.opt.timestep) % 17 == 0:
            r.update_scene(d, camera="camera_ceiling"); frames.append(r.render())

def report(tag):
    fj = [d.qpos[m.jnt_qposadr[m.joint(f"openarm_{side}_finger_joint{i}").id]] for i in (1, 2)]
    con = sorted({m.geom(c.geom1).name + "|" + m.geom(c.geom2).name for c in d.contact[:d.ncon]
                  if "openarm" in (m.body(m.geom_bodyid[c.geom1]).name + m.body(m.geom_bodyid[c.geom2]).name)})
    print(f"  [{tag}] tcp={np.round(d.xpos[tcp],3)} target={np.round(gp,3)} instr={np.round(d.xpos[ib],3)} "
          f"fingers={np.round(fj,4)} arm_contacts={con[:6]}")

z0 = d.xpos[ib][2]
run(q_pre, a.open, 2.0); report("pre")
run(q_grasp, a.open, 1.5); report("down")
run(q_grasp, -0.005, max(1.0, a.close_time + 0.3), grip_T=a.close_time); report("closed")
run(q_lift, -0.005, 2.0); run(q_lift, -0.005, 1.0); report("lift")
dz = d.xpos[ib][2] - z0
touch = {m.geom(c.geom1).name + "|" + m.geom(c.geom2).name for c in d.contact[:d.ncon]
         if ib in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2])}
print(f"  max penetration: finger-instr {max_pen['finger-instr']*1000:.1f} mm, instr-table {max_pen['instr-table']*1000:.1f} mm")
print(f"{a.instr}: lifted {dz*100:.1f} cm  -> {'PASS' if dz > 0.05 else 'FAIL'}; contacts: {sorted(touch)}")
if a.video and frames:
    import cv2
    h, w = frames[0].shape[:2]; vw = cv2.VideoWriter(a.video, cv2.VideoWriter_fourcc(*"mp4v"), 30, (w, h))
    for f in frames: vw.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
    vw.release(); print("video:", a.video)
