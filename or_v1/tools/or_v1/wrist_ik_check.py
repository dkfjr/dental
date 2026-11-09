#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""How far can the teleop IK pitch the gripper down over the tray?
Targets are given directly in the arm_origin frame (x fwd to the cart, y left, z up),
gripper pitched down by 0/30/60/90 deg, solved by the SAME openarm_control IK the dora
node uses. Compare IK argument sets:
    .venv/bin/python tools/or_v1/wrist_ik_check.py "<ik args A>" "<ik args B>" ..."""
import argparse, sys
import mujoco, numpy as np
from scipy.spatial.transform import Rotation as R
from openarm_control.config import register_common_args, setup_from_args
from openarm_control.ik_params import register_ik_args, ik_params_from_args
from openarm_control.kinematics import Kinematics

XML = "third_party/openarm_mujoco/v1/dental_or/dental_or_scene_v1.0.xml"
R0 = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float)      # neutral: gripper points forward
PTS = {"right": [(0.30, -0.12, -0.25), (0.38, -0.10, -0.32), (0.25, -0.18, -0.38)]}
M = mujoco.MjModel.from_xml_path(XML); D = mujoco.MjData(M)
qa = [M.jnt_qposadr[M.joint(f"openarm_right_joint{i}").id] for i in range(1, 8)]
sid, oid = M.site("right_ee_control_point").id, M.site("arm_origin").id

def fk(j):
    mujoco.mj_resetDataKeyframe(M, D, M.key("home").id); D.qpos[qa] = j[:7]; mujoco.mj_forward(M, D)
    Ro = D.site_xmat[oid].reshape(3, 3)
    return Ro.T @ (D.site_xpos[sid] - D.site_xpos[oid]), Ro.T @ D.site_xmat[sid].reshape(3, 3)

for ikargs in sys.argv[1:] or ["--max-iters 30 --dt 0.1 --damping 0.05 --posture-cost 0.0 --lm-damping 0.005 --no-joint-braking"]:
    ap = argparse.ArgumentParser(); register_common_args(ap); register_ik_args(ap)
    args = ap.parse_args(f"--xml {XML} --keyframe home --mode bimanual {ikargs}".split())
    print(f"== IK args: {ikargs}")
    for p in PTS["right"]:
        row = []
        for th in (0, 30, 60, 90):
            Rt = R.from_euler("y", th, degrees=True).as_matrix() @ R0
            qt = R.from_matrix(Rt).as_quat()
            pose = np.array([*p, qt[3], qt[0], qt[1], qt[2]], np.float32)
            kin = Kinematics(setup_from_args(args), ik_params_from_args(args))
            left = np.array([0.30, 0.15, -0.25, *pose[3:]], np.float32)
            for _ in range(150):
                kin.set_target("right", pose); kin.set_target("left", left)
                kin.set_gripper("right", 0); kin.set_gripper("left", 0); res = kin.solve()
            pp, Rs = fk(res[:8])
            down = np.degrees(np.arcsin(np.clip(Rs[2, 2], -1, 1)))
            row.append(f"{th:2d}->{down:4.0f}deg/{np.linalg.norm(pp - p) * 1000:3.0f}mm")
        print(f"  target {p}: " + "  ".join(row))
