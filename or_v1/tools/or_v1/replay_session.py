#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
Physics replay of a recorded VR session: feeds the bridge packets from out/sessions/*.csv
through the same chain as dora (quest_receiver maths + OneEuro smoother, openarm_control IK,
gripper slew limit, MuJoCo position actuators with contacts) and reports every instrument
"kick" (speed > 0.3 m/s) with the finger contact forces at that moment. Optional close-up
video that follows the gripper.

    LD_LIBRARY_PATH=~/.local/nvidia-590/lib MUJOCO_GL=egl \
      .venv/bin/python tools/or_v1/replay_session.py out/sessions/vr_<t>.csv --video out/replay.mp4
"""
import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from dora_openarm_vr.quest_receiver import QuestPoseProcessor  # noqa: E402
from dora_openarm_vr.smoothing import OneEuroPoseSmoother  # noqa: E402
from openarm_control.config import register_common_args, setup_from_args  # noqa: E402
from openarm_control.ik_params import register_ik_args, ik_params_from_args  # noqa: E402
from openarm_control.kinematics import Kinematics  # noqa: E402
from openarm_mujoco.v2.joint_resolver import JointResolver  # noqa: E402

XML = str(ROOT / "third_party/openarm_mujoco/v1/dental_or/dental_or_scene_v1.0.xml")
IK = ("--max-iters 30 --dt 0.1 --damping 0.05 --posture-cost 0.0 --lm-damping 0.005 "
      "--no-joint-braking")
OPEN, CLOSED = 0.044, -0.005


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--video")
    ap.add_argument("--two-view", action="store_true",
                    help="1920x540: left = camera_ceiling (same as the live recording), right = close-up following the gripper")
    ap.add_argument("--start", type=float, default=0.0)
    ap.add_argument("--end", type=float, default=1e9)
    ap.add_argument("--close-time", type=float, default=0.5, help="gripper slew: full close (s)")
    ap.add_argument("--open-time", type=float, default=0.25)
    ap.add_argument("--xml", default=XML)
    ap.add_argument("--floor", action="store_true", help="apply the virtual floor to the recorded targets")
    a = ap.parse_args()

    r = np.genfromtxt(a.csv, delimiter=",", names=True, invalid_raise=False)
    m = mujoco.MjModel.from_xml_path(a.xml); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("home").id); mujoco.mj_forward(m, d)
    jr = JointResolver(m)
    p2 = argparse.ArgumentParser(); register_common_args(p2); register_ik_args(p2)
    args = p2.parse_args(f"--xml {a.xml} --keyframe home --mode bimanual {IK}".split())
    kin = Kinematics(setup_from_args(args), ik_params_from_args(args))
    proc = QuestPoseProcessor()
    sm = {s: OneEuroPoseSmoother(min_cutoff=2.0, beta=0.04, d_cutoff=1.5, max_linear_speed=1.0,
                                 max_angular_speed=6.0) for s in ("left", "right")}
    IDENT = {"x": 0, "y": 0, "z": 0, "qx": 0, "qy": 0, "qz": 0, "qw": 1}
    instr = [b for b in range(m.nbody) if m.body(b).name.startswith("instr_")]
    finger_bodies = {m.body(n).id for n in (f"openarm_{s}_{f}_finger" for s in ("left", "right")
                                            for f in ("left", "right"))}
    hand = {s: m.body(f"openarm_{s}_hand_tcp").id for s in ("left", "right")}
    grip = {"left": OPEN, "right": OPEN}
    cs, os_ = (OPEN - CLOSED) / a.close_time, (OPEN - CLOSED) / a.open_time

    ren = vw = None
    if a.video:
        import cv2
        W, H = (960, 540) if a.two_view else (640, 480)
        ren = mujoco.Renderer(m, H, W)
        cam = mujoco.MjvCamera(); cam.distance = 0.55 if a.two_view else 0.45; cam.elevation = -38; cam.azimuth = 200
        vw = cv2.VideoWriter(a.video, cv2.VideoWriter_fourcc(*"mp4v"), 30, ((2 * W) if a.two_view else W, H))
        look = None
    steps = int(round((1 / 60) / m.opt.timestep))
    kicks, k_last = [], -1.0
    prev_t = None
    for i in range(len(r)):
        t = float(r["t"][i])
        if prev_t is not None:
            n = max(1, int(round((t - prev_t) / m.opt.timestep)))
        else:
            n = steps
        prev_t = t
        pk = {"rf": IDENT, "v": 0,
              "rc": {k: float(r["r" + k][i]) for k in ("x", "y", "z", "qx", "qy", "qz", "qw")},
              "lc": {k: float(r["l" + k][i]) for k in ("x", "y", "z", "qx", "qy", "qz", "qw")}}
        started = r["calibrated"][i] > 0.5
        if started:
            pr, pl, _ = proc.process(pk)
            if a.floor:
                from dental_vla.attention_map import apply_floor
                from scipy.spatial.transform import Rotation as Rot
                for arr in (pr, pl):
                    arr[:3] = apply_floor(arr[:3], Rot.from_quat([arr[4], arr[5], arr[6], arr[3]]))
            tgt = {"right": sm["right"].smooth(t, pr), "left": sm["left"].smooth(t, pl)}
            kin.set_target("right", tgt["right"]); kin.set_target("left", tgt["left"])
            kin.set_gripper("right", 0); kin.set_gripper("left", 0)
            res = kin.solve()
            if res is not None:
                for s, trig, off in (("right", r["rt"][i], 0), ("left", r["lt"][i], 8)):
                    goal = OPEN + float(np.clip(trig, 0, 1)) * (CLOSED - OPEN)
                    step = (cs if goal < grip[s] else os_) / 60
                    grip[s] += max(-step, min(step, goal - grip[s]))
                    q = np.array(res[off:off + 8], dtype=float); q[7] = grip[s]
                    jr.set_ctrl(d.ctrl, q, s)
        for _ in range(n):
            mujoco.mj_step(m, d)
        if not (a.start <= t <= a.end):
            continue
        # kicks: instrument speed > 0.3 m/s
        for b in instr:
            v = np.linalg.norm(d.cvel[b][3:])
            if v > 0.3 and t - k_last > 0.3:
                who = {}
                for ci in range(d.ncon):
                    c = d.contact[ci]
                    b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
                    if b in (b1, b2):
                        other = m.body(b2 if b1 == b else b1).name.replace("openarm_", "")
                        f6 = np.zeros(6); mujoco.mj_contactForce(m, d, ci, f6)
                        who[other] = round(who.get(other, 0.0) + abs(f6[0]), 1)   # normal force (N)
                kicks.append((t, m.body(b).name[6:], v, who))
                k_last = t
        if vw is not None and i % 2 == 0:
            import cv2
            # follow the arm whose controller is tracked / moving (left if both)
            side = "left" if (r["raw_lok"][i] > 0.5 or r["raw_rok"][i] < 0.5) else "right"
            tgt = d.xpos[hand[side]].copy()
            look = tgt if look is None else 0.85 * look + 0.15 * tgt       # smooth camera
            cam.lookat[:] = look
            ren.update_scene(d, cam)
            close = cv2.cvtColor(ren.render(), cv2.COLOR_RGB2BGR)
            if a.two_view:
                ren.update_scene(d, camera="camera_ceiling")
                wide = cv2.cvtColor(ren.render(), cv2.COLOR_RGB2BGR)
                img = np.hstack([wide, close])
                cv2.putText(img, "VR teleop (replay of recorded session)", (18, 36),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
                cv2.putText(img, f"t={t:5.1f}s  trigger L={r['lt'][i]:.2f} R={r['rt'][i]:.2f}", (18, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            else:
                img = close
                cv2.putText(img, f"t={t:5.1f}s trigL={r['lt'][i]:.2f} gripL={grip['left']:.3f}", (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            vw.write(img)
    if vw is not None:
        vw.release()
    print(f"replayed {len(r)} packets ({r['t'][-1]:.1f} s)")
    for t, n, v, who in kicks[:20]:
        print(f"  KICK t={t:5.1f}s {n:18s} speed {v:4.2f} m/s  touching: {who}")
    print(f"  {len(kicks)} kicks")


if __name__ == "__main__":
    main()
