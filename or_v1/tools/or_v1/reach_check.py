#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""Can each arm reach each instrument for a top-down grasp?
For every instrument, solve IK (damped least squares, joint limits, 12 restarts) so the
gripper TCP (`openarm_<side>_hand_tcp`, between the fingertips) sits 1 cm above the
instrument's collider top with the approach axis pointing down. Reports TCP position
error and tilt from vertical. OK = err < 1 cm and tilt < 30 deg.
    .venv/bin/python tools/or_v1/reach_check.py"""
from pathlib import Path
import mujoco, numpy as np

XML = Path(__file__).resolve().parents[2] / "third_party/openarm_mujoco/v1/dental_or/dental_or_scene_v1.0.xml"
m = mujoco.MjModel.from_xml_path(str(XML)); d = mujoco.MjData(m)
HOME = m.key("home").id

def arm(side):
    j = [m.joint(f"openarm_{side}_joint{i}").id for i in range(1, 8)]
    return [m.jnt_qposadr[i] for i in j], [m.jnt_dofadr[i] for i in j], [m.jnt_range[i] for i in j]

RNG = np.random.default_rng(0)


def solve(side, target, down_w=0.3, iters=300, restarts=12):
    best = (9.0, 180.0)
    for k in range(restarts):
        r = _solve(side, target, down_w, iters, seed=k)
        if (r[0] > 0.01, r[1]) < (best[0] > 0.01, best[1]):
            best = r
    return best


def _solve(side, target, down_w, iters, seed):
    mujoco.mj_resetDataKeyframe(m, d, HOME)
    qa, da, rng = arm(side); sid = m.site(f"{side}_ee_control_point").id
    tcp = m.body(f"openarm_{side}_hand_tcp").id
    lo = np.array([r[0] for r in rng]); hi = np.array([r[1] for r in rng])
    if seed:
        d.qpos[qa] = RNG.uniform(lo, hi)
    for _ in range(iters):
        mujoco.mj_kinematics(m, d); mujoco.mj_comPos(m, d)
        p = d.xpos[tcp]; R = d.site_xmat[sid].reshape(3, 3)
        ep = target - p
        z = -R[:, 2]                                  # approach axis (site -z = gripper pointing dir)
        ez = np.cross(z, np.array([0, 0, -1.0]))       # rotate z toward -Z
        jp = np.zeros((3, m.nv)); jr = np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, jp, jr, d.xpos[tcp], tcp)
        J = np.vstack([jp[:, da], down_w * jr[:, da]]); e = np.hstack([ep, down_w * ez])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
        q = d.qpos[qa] + 0.5 * dq
        d.qpos[qa] = np.clip(q, lo, hi)
    mujoco.mj_kinematics(m, d)
    p = d.xpos[tcp]; z = -d.site_xmat[sid].reshape(3, 3)[:, 2]
    return np.linalg.norm(target - p), np.degrees(np.arccos(np.clip(-z[2], -1, 1)))

# which way does the approach axis point? (sanity: report at home)
mujoco.mj_resetDataKeyframe(m, d, HOME); mujoco.mj_forward(m, d)
print("home ee z-axis (left):", np.round(d.site_xmat[m.site('left_ee_control_point').id].reshape(3,3)[:,2], 2))
rows = []
for b in range(m.nbody):
    n = m.body(b).name
    if not n.startswith("instr_"): continue
    mujoco.mj_resetDataKeyframe(m, d, HOME); mujoco.mj_forward(m, d)
    g = m.geom(f"col_{n[6:]}").id
    top = d.geom_xpos[g].copy(); top[2] += m.geom_size[g][2] + 0.01
    res = {s: solve(s, top) for s in ("left", "right")}
    best = min(res, key=lambda s: (res[s][0] > 0.01, res[s][1]))
    ok = res[best][0] < 0.01 and res[best][1] < 30
    rows.append(ok)
    print(f"{n[6:]:20s} xy=({top[0]:.2f},{top[1]:.2f})  L err={res['left'][0]*100:5.1f}cm tilt={res['left'][1]:4.0f}°"
          f"  R err={res['right'][0]*100:5.1f}cm tilt={res['right'][1]:4.0f}°  -> {'OK ('+best+')' if ok else 'NO'}")
print(f"reachable top-down: {sum(rows)}/{len(rows)}")
