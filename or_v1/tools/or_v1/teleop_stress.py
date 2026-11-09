#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
Offline teleop stress test: a scripted 30 s operator motion (OpenXR controller poses at
60 Hz) -> AttentionMapper -> encode -> dora quest_receiver maths -> the dora IK (same
openarm_control Kinematics + CLI args) -> joint trajectory. Reports how "jumpy" the arms
are (max joint change per tick, ticks with a >0.15 rad jump = visible flip) and how well
the wrist follows (tracking error).

    .venv/bin/python tools/or_v1/teleop_stress.py --neutral home --ik "<ik args>"
"""
import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from dental_vla.attention_map import AttentionMapper  # noqa: E402
from dora_openarm_vr.quest_receiver import QuestPoseProcessor  # noqa: E402
from openarm_control.config import register_common_args, setup_from_args  # noqa: E402
from openarm_control.ik_params import register_ik_args, ik_params_from_args  # noqa: E402
from openarm_control.kinematics import Kinematics  # noqa: E402

XML = str(ROOT / "third_party/openarm_mujoco/v1/dental_or/dental_or_scene_v1.0.xml")
CURRENT = "--max-iters 30 --dt 0.1 --damping 0.05 --posture-cost 0.0 --lm-damping 0.005 --no-joint-braking"


def operator(t):
    """OpenXR poses of both controllers at time t. Starts in the viewer's home pose (ㄴ)."""
    base = {"left": np.array([-0.22, 1.05, -0.30]), "right": np.array([0.22, 1.05, -0.30])}
    key = [  # time, dpos(right; left mirrored in x), pitch, roll(deg)
        (0, (0, 0, 0), 0, 0), (3, (0, 0, 0), 0, 0),
        (7, (0, -0.28, 0.25), -70, 0), (11, (0, 0, 0), 0, 0),            # down toward attention, back
        (16, (0, 0.10, -0.35), -30, 0), (20, (0.20, 0.05, -0.30), -30, 0),  # reach over tray, sideways
        (25, (0.05, 0.0, -0.30), -75, 40), (30, (0, 0, 0), 0, 0),           # wrist flex + roll, return
    ]
    for (t0, p0, pi0, r0), (t1, p1, pi1, r1) in zip(key, key[1:]):
        if t0 <= t <= t1:
            s = (t - t0) / (t1 - t0); s = s * s * (3 - 2 * s)
            dp = (1 - s) * np.array(p0) + s * np.array(p1)
            pitch, roll = (1 - s) * pi0 + s * pi1, (1 - s) * r0 + s * r1
            break
    else:
        dp, pitch, roll = np.zeros(3), 0, 0
    tremor = 0.002 * np.sin(2 * np.pi * np.array([7.1, 8.3, 9.7]) * t)
    out = {}
    for side, sx in (("right", 1), ("left", -1)):
        p = base[side] + np.array([sx * dp[0], dp[1], dp[2]]) + tremor
        q = R.from_euler("xz", [pitch, -sx * roll], degrees=True)          # OpenXR: x right, z back
        out[side] = (p, q.as_quat())
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--neutral", default="home", choices=("home", "attention"))
    ap.add_argument("--ik", default=CURRENT)
    a = ap.parse_args()
    p2 = argparse.ArgumentParser(); register_common_args(p2); register_ik_args(p2)
    args = p2.parse_args(f"--xml {XML} --keyframe home --mode bimanual {a.ik}".split())
    kin = Kinematics(setup_from_args(args), ik_params_from_args(args))

    am = AttentionMapper(neutral=a.neutral)
    head = R.identity().as_quat()
    if a.neutral == "attention":      # operator stands at attention with controllers pointing down
        am.calibrate(head, {s: (p + np.array([0, -0.28, 0.25]), R.from_euler("x", -90, degrees=True).as_quat())
                            for s, (p, _) in operator(0).items()})
    else:
        am.calibrate(head, operator(0))
    proc = QuestPoseProcessor()
    IDENT = {"x": 0, "y": 0, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1}

    M = mujoco.MjModel.from_xml_path(XML); D = mujoco.MjData(M)
    qa = {s: [M.jnt_qposadr[M.joint(f"openarm_{s}_joint{i}").id] for i in range(1, 8)] for s in ("left", "right")}
    sid = {s: M.site(f"{s}_ee_control_point").id for s in ("left", "right")}
    oid = M.site("arm_origin").id
    prev, jumps, maxstep, errs = None, 0, 0.0, []
    for k in range(int(30 * 60)):
        t = k / 60
        pk = {"rf": IDENT}
        for s, (p, q) in operator(t).items():
            pk["lc" if s == "left" else "rc"] = am.encode(*am.target(s, p, q))
        pr, pl, _ = proc.process(pk)
        kin.set_target("right", pr[:7]); kin.set_target("left", pl[:7])
        kin.set_gripper("right", 0); kin.set_gripper("left", 0)
        res = kin.solve()
        if res is None:
            continue
        q = np.concatenate([res[0:7], res[8:15]])
        if prev is not None:
            dq = np.abs(q - prev).max(); maxstep = max(maxstep, dq); jumps += dq > 0.15
        prev = q
        # wrist tracking error
        mujoco.mj_resetDataKeyframe(M, D, M.key("home").id)
        D.qpos[qa["right"]] = res[0:7]; D.qpos[qa["left"]] = res[8:15]; mujoco.mj_kinematics(M, D)
        Ro = D.site_xmat[oid].reshape(3, 3)
        for s, tgt in (("right", pr), ("left", pl)):
            errs.append(np.linalg.norm(Ro.T @ (D.site_xpos[sid[s]] - D.site_xpos[oid]) - tgt[:3]))
    errs = np.array(errs) * 1000
    print(f"neutral={a.neutral:9s} ik='{a.ik}'\n   max joint step/tick {maxstep:.3f} rad | flips(>0.15 rad/tick) {jumps} "
          f"| wrist err median {np.median(errs):.1f} mm, p95 {np.percentile(errs, 95):.1f} mm, max {errs.max():.1f} mm")


if __name__ == "__main__":
    main()
