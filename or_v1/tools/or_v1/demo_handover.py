#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
Scripted demo (no VR): LEFT arm picks the 10 ml syringe off the tray and presents it toward
the doctor, using movej — Cartesian waypoints (world coordinates) are converted to joint
targets by IK, then the arm moves in joint space with smooth (quintic) interpolation while
MuJoCo physics runs (position actuators, contacts, gravity). Records a two-view video.

    MUJOCO_GL=egl .venv/bin/python tools/or_v1/demo_handover.py
    -> out/videos/demo_movej_left_handover.mp4   + printed waypoint table / result
"""
import argparse
from pathlib import Path

import cv2
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
XML = ROOT / "third_party/openarm_mujoco/v1/dental_or/dental_or_scene_v1.0.xml"
ap = argparse.ArgumentParser()
ap.add_argument("--instr", default="syringe_10ml")
ap.add_argument("--grip-x", type=float, default=-0.015, help="grasp point along the instrument (local x, m)")
ap.add_argument("--open", type=float, default=0.035, help="finger opening before grasp (0.044 = full)")
ap.add_argument("--handoff", type=float, nargs=3, default=(0.36, 0.43, 1.16),
                help="world TCP position where the syringe is presented to the doctor (doctor is at -x)")
ap.add_argument("--close-time", type=float, default=1.2, help="gripper open->closed duration (s)")
ap.add_argument("--out", default=str(ROOT / "out/videos/demo_movej_left_handover.mp4"))
a = ap.parse_args()

m = mujoco.MjModel.from_xml_path(str(XML)); d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, m.key("home").id); mujoco.mj_forward(m, d)
SIDE = "left"
qa = np.array([m.jnt_qposadr[m.joint(f"openarm_{SIDE}_joint{i}").id] for i in range(1, 8)])
da = np.array([m.jnt_dofadr[m.joint(f"openarm_{SIDE}_joint{i}").id] for i in range(1, 8)])
act = np.array([m.actuator(f"{SIDE}_joint{i}_ctrl").id for i in range(1, 8)])
fing = m.actuator(f"{SIDE}_finger1_ctrl").id
lo, hi = m.actuator_ctrlrange[act, 0], m.actuator_ctrlrange[act, 1]
tcp = m.body(f"openarm_{SIDE}_hand_tcp").id
sid = m.site(f"{SIDE}_ee_control_point").id
ib = m.body(f"instr_{a.instr}").id
OPEN, CLOSED, TABLE_TOP, TIP = a.open, -0.005, 0.912, 0.0154


def ik(target, approach, finger_axis, q0, iters=500, restarts=8):
    """movej waypoint: TCP at `target` (world), gripper pointing along `approach`,
    finger-closing axis along `finger_axis`. Damped least squares within joint limits."""
    best = None
    rng = np.random.default_rng(0)
    for k in range(restarts):
        dd = mujoco.MjData(m); dd.qpos[:] = d.qpos
        dd.qpos[qa] = q0 if k == 0 else rng.uniform(lo, hi)
        for _ in range(iters):
            mujoco.mj_kinematics(m, dd); mujoco.mj_comPos(m, dd)
            Rs = dd.site_xmat[sid].reshape(3, 3)
            e_ori = np.cross(-Rs[:, 2], approach) + 0.5 * np.cross(Rs[:, 1], finger_axis)
            e = np.hstack([target - dd.xpos[tcp], 0.5 * e_ori])
            jp = np.zeros((3, m.nv)); jr = np.zeros((3, m.nv))
            mujoco.mj_jac(m, dd, jp, jr, dd.xpos[tcp], tcp)
            J = np.vstack([jp[:, da], 0.5 * jr[:, da]])
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
            dd.qpos[qa] = np.clip(dd.qpos[qa] + 0.5 * dq, lo, hi)
        mujoco.mj_kinematics(m, dd)
        Rs = dd.site_xmat[sid].reshape(3, 3)
        err = np.linalg.norm(target - dd.xpos[tcp]); ang = np.degrees(np.arccos(np.clip(-Rs[:, 2] @ approach, -1, 1)))
        score = err + 0.001 * ang + 0.02 * np.linalg.norm(dd.qpos[qa] - q0)   # prefer close to previous pose
        if best is None or score < best[0]:
            best = (score, dd.qpos[qa].copy(), err, ang)
    return best[1], best[2], best[3]


# ── waypoints (world coordinates) ──
R_i = d.xmat[ib].reshape(3, 3)
gp = d.xpos[ib] + R_i @ np.array([a.grip_x, 0, 0])
across = np.cross([0, 0, 1.0], R_i[:, 0]); across /= np.linalg.norm(across)
down = np.array([0, 0, -1.0])
grasp_z = max(gp[2] + 0.004, TABLE_TOP + TIP + 0.012)   # +12 mm: arm sags ~6 mm under gravity; fingertips must clear the drape
toward_doctor = np.array([-1.0, 0, 0])                      # doctor sits at -x
# present tilted ~30 deg down (like a hand-over); fully horizontal lets the syringe slide out
present_dir = toward_doctor + np.array([0, 0, -0.58]); present_dir /= np.linalg.norm(present_dir)
handoff = np.array(a.handoff)
wps = [
    ("above syringe", gp + [0, 0, 0.10], down, across, OPEN),
    ("down to grasp", np.array([gp[0], gp[1], grasp_z]), down, across, OPEN),
    ("close gripper", None, None, None, CLOSED),
    ("lift", gp + [0, 0, 0.07], down, across, CLOSED),
    ("carry", (gp + [0, 0, 0.12] + handoff) / 2 + [0, -0.05, 0.03], (down + toward_doctor) / np.sqrt(2),
     np.array([0, 0, 1.0]), CLOSED),
    ("present to doctor", handoff, present_dir, np.array([0, 0, 1.0]), CLOSED),
    ("hold", None, None, None, CLOSED),
]
q = d.qpos[qa].copy()
plan = []
print(f"{'waypoint':20s} {'TCP target (world, m)':28s} IK err   approach err")
for name, tgt, appr, fax, grip in wps:
    if tgt is not None:
        # small moves continue from the current arm pose only (no random restarts) so movej
        # never flips the arm into a different IK branch while holding the instrument
        q, err, ang = ik(tgt, appr, fax, q, restarts=8 if name == "above syringe" else 1)
        print(f"{name:20s} {np.array2string(tgt, precision=3):28s} {err*1000:5.1f}mm  {ang:5.1f}deg")
    else:
        print(f"{name:20s} {'(joints unchanged)':28s}")
    plan.append((name, q.copy(), grip))

# ── movej execution with physics + video ──
W, H = 960, 540
ren = mujoco.Renderer(m, H, W)
cams = []
for lookat, dist, az, el in (((0.45, 0.60, 1.05), 1.55, 205, -20),      # robot + doctor + patient
                             ((0.58, 0.72, 0.98), 0.75, 250, -38)):     # tray close-up
    c = mujoco.MjvCamera(); c.lookat[:] = lookat; c.distance = dist; c.azimuth = az; c.elevation = el
    cams.append(c)
out = Path(a.out); out.parent.mkdir(parents=True, exist_ok=True)
vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), 30, (2 * W, H))
FRAME_EVERY = int(round(1 / 30 / m.opt.timestep))
step = 0


def frame(label):
    tiles = []
    for c in cams:
        ren.update_scene(d, c); tiles.append(ren.render())
    img = cv2.cvtColor(np.hstack(tiles), cv2.COLOR_RGB2BGR)
    cv2.putText(img, f"movej (left arm): {label}", (18, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cv2.putText(img, f"t={d.time:5.2f}s", (18, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    vw.write(img)


def movej(q_to, grip_to, T, label):
    global step
    q_from = d.ctrl[act].copy(); g_from = d.ctrl[fing]; t0 = d.time
    while d.time - t0 < T:
        s = min(1.0, (d.time - t0) / T)
        s = 10 * s**3 - 15 * s**4 + 6 * s**5                  # quintic: zero vel/acc at both ends
        d.ctrl[act] = q_from + s * (q_to - q_from)
        d.ctrl[fing] = g_from + s * (grip_to - g_from)
        mujoco.mj_step(m, d); step += 1
        if step % FRAME_EVERY == 0:
            frame(label)



movej(d.ctrl[act].copy(), OPEN, 1.0, "home")
durations = {"above syringe": 2.5, "down to grasp": 1.8, "close gripper": a.close_time, "lift": 1.8, "carry": 2.2,
             "present to doctor": 2.2, "hold": 2.0}
z0 = d.xpos[ib][2]
for name, qt, grip in plan:
    movej(qt, grip, durations[name], name)
    fj = d.qpos[m.jnt_qposadr[m.joint(f"openarm_{SIDE}_finger_joint1").id]]
    touch = sorted({m.geom(c.geom1 if m.geom_bodyid[c.geom2] == ib else c.geom2).name
                    for c in d.contact[:d.ncon] if ib in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2])})
    print(f"  after {name:18s} tcp={np.round(d.xpos[tcp], 3)} instr={np.round(d.xpos[ib], 3)} finger={fj:.4f} touching={touch[:4]}")
lift = d.xpos[ib][2] - z0
dist = np.linalg.norm(d.xpos[ib] - d.xpos[tcp])
vw.release()
ok = lift > 0.10 and dist < 0.08
print(f"\nsyringe raised {lift*100:.1f} cm, {dist*100:.1f} cm from the gripper at the end -> "
      f"{'PASS (held through the handover)' if ok else 'FAIL'}")
print(f"video: {out}  ({step * m.opt.timestep:.1f} s)")
