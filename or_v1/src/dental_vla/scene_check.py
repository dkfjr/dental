#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
Headless validator for a Dental-VLA MuJoCo scene.

Loads the scene, resets to the `home` keyframe, checks the OpenArm `arm_origin`
teleop-reference site exists, steps the simulation briefly to confirm the
instruments rest stably on the tray, and (optionally) renders preview PNGs.

    python -m dental_vla.scene_check                 # validate default scene
    python -m dental_vla.scene_check --render out/   # + save preview images
    MUJOCO_GL=egl python -m dental_vla.scene_check --render out/
"""
from __future__ import annotations
import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np

DEFAULT_SCENE = (Path(__file__).resolve().parents[2]
                 / "third_party/openarm_mujoco/v2/dental/dental_scene.xml")
CAMERAS = ["overview", "ceiling", "operator"]


def check(scene: Path, render_dir: Path | None) -> int:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)

    ok = True

    # 1) home keyframe
    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if kid < 0:
        print("[FAIL] no 'home' keyframe"); ok = False
    else:
        mujoco.mj_resetDataKeyframe(model, data, kid)
    mujoco.mj_forward(model, data)

    # 2) arm_origin site (teleop reference frame)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "arm_origin")
    if sid < 0:
        print("[FAIL] missing 'arm_origin' site — VR/IK frame is undefined"); ok = False
    else:
        print(f"[ok] arm_origin at z={data.site_xpos[sid][2]:.3f}")

    # 3) actuator count (8 per arm x 2)
    print(f"[ok] nq={model.nq} nv={model.nv} nu={model.nu} nbody={model.nbody}")
    if model.nu != 16:
        print(f"[warn] expected 16 actuators (8/arm), got {model.nu}")

    # 4) stability: step 1 s and confirm nothing falls off the table
    free_z0 = _free_z(model, data)
    for _ in range(1000):
        mujoco.mj_step(model, data)
    free_z1 = _free_z(model, data)
    drift = float(np.max(np.abs(np.array(free_z1) - np.array(free_z0))))
    lowest = min(free_z1)
    print(f"[{'ok' if lowest > 0.35 else 'FAIL'}] instruments rest on tray "
          f"(lowest z={lowest:.3f}, max drift over 1s={drift:.3f})")
    if lowest <= 0.35:
        ok = False

    # 5) optional render
    if render_dir is not None:
        render_dir.mkdir(parents=True, exist_ok=True)
        try:
            from PIL import Image
            r = mujoco.Renderer(model, 1000, 1200)
            mujoco.mj_resetDataKeyframe(model, data, max(kid, 0))
            mujoco.mj_forward(model, data)
            for cam in CAMERAS:
                r.update_scene(data, camera=cam)
                out = render_dir / f"scene_{cam}.png"
                Image.fromarray(r.render()).save(out)
                print(f"[ok] rendered {out}")
        except Exception as e:  # rendering needs a GL backend; validation still passed
            print(f"[warn] render skipped: {e}")

    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


def _free_z(model, data) -> list[float]:
    zs = []
    for j in range(model.njnt):
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
            zs.append(float(data.qpos[model.jnt_qposadr[j] + 2]))
    return zs


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("scene", nargs="?", default=str(DEFAULT_SCENE))
    ap.add_argument("--render", metavar="DIR", default=None)
    args = ap.parse_args()
    rc = check(Path(args.scene),
               Path(args.render) if args.render else None)
    sys.exit(rc)


if __name__ == "__main__":
    main()
