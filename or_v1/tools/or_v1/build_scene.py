#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
Build the OR (operating-room) teleop scene: OpenArm **v1.0** (black) placed where the
Blender scene `scenes/dental_or_v1.blend` puts the robot, plus the Blender environment
meshes exported by `blender_export_env.py`.

    # 1) robot only -> posed geom list for Blender (robot import into the .blend)
    .venv/bin/python tools/or_v1/build_scene.py robot-json
    # 2) after `blender -b scenes/dental_or_v1.blend -P tools/or_v1/blender_export_env.py`
    .venv/bin/python tools/or_v1/build_scene.py scene

Outputs (third_party/openarm_mujoco/v1/dental_or/):
    openarm_v1_black.xml      robot, body_link0 moved/rotated, all materials black
    dental_or_scene_v1.0.xml  full scene (include robot + env + instruments + home key)
    robot_home_geoms.json     world poses of every robot visual geom at `home`

Frame convention kept from the v2 dental scene: the teleop target frame is the
`arm_origin` site, which lives in body_link0 at +0.698 m (arm mount height).
"""
from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
V1 = ROOT / "third_party/openarm_mujoco/v1"
OUT = V1 / "dental_or"
ENV_DIR = OUT / "env"

# Robot placement (2026-09-29, user request): facing the instrument cart (+y) instead of
# the patient (-x). Chosen by reach_check over a grid: body at (0.84, 0.47), arm mount
# (= arm_origin) lowered 10 cm to z=1.22 -> 17/22 instruments reachable top-down, both arms,
# no contact at `home`. (Blender original: (0.93, 0.35), z 1.32, facing -x -> 13/22, right arm only.)
BASE_YAW_DEG = 90.0
BASE_POS = (0.84, 0.47, 1.22 - 0.698)
BASE_QUAT = (math.cos(math.radians(BASE_YAW_DEG) / 2), 0.0, 0.0, math.sin(math.radians(BASE_YAW_DEG) / 2))

# "ㄴ" ready pose (upper arm down, forearm forward) — same joint convention as v2.
HOME_ARM_LEFT = [0.0, -0.30, 0.0, 1.60, 0.0, 0.0, 0.0]
HOME_ARM_RIGHT = [0.0, 0.30, 0.0, 1.60, 0.0, 0.0, 0.0]
FINGER_OPEN = 0.044

# Pedestal under the v1 body plate: plate centre is 3 cm behind body_link0 (body frame -x)
PEDESTAL_XY = (BASE_POS[0] - 0.03 * math.cos(math.radians(BASE_YAW_DEG)),
               BASE_POS[1] - 0.03 * math.sin(math.radians(BASE_YAW_DEG)))
PEDESTAL_TOP = BASE_POS[2]

BLACK = "0.035 0.035 0.038 1"
GUNMETAL = "0.11 0.11 0.12 1"


def fmt(v) -> str:
    return " ".join(f"{x:.6g}" for x in v)


# ─────────────────────────────── robot ────────────────────────────────
def build_robot() -> Path:
    tree = ET.parse(V1 / "openarm_bimanual.xml")
    root = tree.getroot()
    root.set("model", "openarm_v1_black")

    root.find("compiler").set("meshdir", "../meshes")
    # the scene sets its own option/timestep
    opt = root.find("option")
    if opt is not None:
        root.remove(opt)

    for mat in root.find("asset").findall("material"):
        name = mat.get("name")
        if name == "matte_black":
            mat.set("rgba", BLACK)
            mat.set("reflectance", "0.02")
            mat.set("shininess", "0.35")
            mat.set("specular", "0.25")
        elif name == "metal_silver":  # joint rings/caps -> dark gunmetal, still reads as metal
            mat.set("rgba", GUNMETAL)
            mat.set("reflectance", "0.15")
            mat.set("shininess", "0.8")
            mat.set("specular", "0.5")

    # ── grasp contacts (2026-09-29) ──
    # Replay of real VR sessions showed instruments launched at 4-6 m/s by only ~5 N of squeeze:
    # the v1 finger collision hulls have slanted inner faces, so closing on a round/thin
    # instrument wedges it upward ("watermelon seed"). Fix: flat, high-friction, slightly
    # compliant PADS on the inner fingertip faces are the only finger geometry instruments touch.
    #   bit 1: robot / furniture      bit 2: instruments + pads
    #   instruments  contype=2 conaffinity=2   (see instrument geoms in build_scene)
    #   furniture    contype=1 conaffinity=3   robot links   contype=1 conaffinity=3
    #   finger hulls contype=1 conaffinity=1   -> never touch instruments
    #   pads         contype=2 conaffinity=2
    for g in root.iter("geom"):
        if g.get("class") == "collision":
            g.set("conaffinity", "1" if "finger_collision" in (g.get("name") or "") else "3")
    for side in ("left", "right"):
        for fname, sign in (("left_finger", -1), ("right_finger", +1)):
            fb = root.find(f".//body[@name='openarm_{side}_{fname}']")
            ET.SubElement(fb, "geom", name=f"openarm_{side}_{fname}_pad", type="box",
                          pos=fmt((0, sign * (0.0078 - 0.0010), 0.0665)), size=fmt((0.012, 0.0015, 0.014)),
                          contype="2", conaffinity="2", priority="3", condim="4",
                          friction="2.0 0.05 0.005", solref="0.008 1", solimp="0.9 0.95 0.001",
                          rgba="0.9 0.2 0.2 0.5", group="3", mass="0.001")

    wb = root.find("worldbody")
    body0 = wb.find("body[@name='openarm_body_link0']")
    body0.set("pos", fmt(BASE_POS))
    body0.set("quat", fmt(BASE_QUAT))
    # drop the upstream demo cameras (they are world-fixed at the origin)
    for cam in wb.findall("camera"):
        wb.remove(cam)
    # the scene defines the full-length keyframe
    kf = root.find("keyframe")
    if kf is not None:
        root.remove(kf)

    OUT.mkdir(parents=True, exist_ok=True)
    dst = OUT / "openarm_v1_black.xml"
    ET.indent(tree, space="  ")
    tree.write(dst, encoding="unicode")
    return dst


def robot_home_qpos() -> list[float]:
    return (HOME_ARM_LEFT + [FINGER_OPEN, FINGER_OPEN]
            + HOME_ARM_RIGHT + [FINGER_OPEN, FINGER_OPEN])


def robot_home_ctrl() -> list[float]:
    return HOME_ARM_LEFT + [FINGER_OPEN] + HOME_ARM_RIGHT + [FINGER_OPEN]


def dump_robot_json() -> Path:
    """World pose of each robot visual geom at home (for importing into Blender)."""
    import mujoco
    import numpy as np

    robot_xml = build_robot()
    # wrap with a keyframe-free scene so we can set qpos directly
    m = mujoco.MjModel.from_xml_path(str(robot_xml))
    d = mujoco.MjData(m)
    d.qpos[:18] = robot_home_qpos()
    mujoco.mj_forward(m, d)

    # mesh name -> (file, scale) from the XML (MjModel does not keep file names)
    asset = ET.parse(robot_xml).getroot().find("asset")
    mesh_src = {me.get("name"): (me.get("file"), [float(s) for s in me.get("scale", "1 1 1").split()])
                for me in asset.findall("mesh")}
    geoms = []
    for g in range(m.ngeom):
        if m.geom_group[g] != 2 or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g])
        f, scale = mesh_src[mname]
        # MuJoCo re-centres meshes on their inertial frame; undo it so the raw
        # OBJ file can be placed with a single world transform.
        mpos = m.mesh_pos[m.geom_dataid[g]]
        mquat = m.mesh_quat[m.geom_dataid[g]]
        R_g = d.geom_xmat[g].reshape(3, 3)
        R_m = np.zeros(9)
        mujoco.mju_quat2Mat(R_m, mquat)
        R_m = R_m.reshape(3, 3)
        # world_from_file = world_from_geom * inv(file_to_geom); file_to_geom: x_g = R_m^T (x_f - p_m)
        R = R_g @ R_m.T
        t = d.geom_xpos[g] - R @ mpos
        mat = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MATERIAL, m.geom_matid[g])
        geoms.append({
            "name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or f"geom{g}",
            "file": str((V1 / "meshes" / f).resolve().relative_to(ROOT)),  # repo-relative
            "scale": scale,
            "matrix": np.vstack([np.hstack([R, t[:, None]]), [0, 0, 0, 1]]).tolist(),
            "material": mat,
        })
    dst = OUT / "robot_home_geoms.json"
    dst.write_text(json.dumps({"base_pos": BASE_POS, "base_yaw_deg": BASE_YAW_DEG,
                              "pedestal_xy": PEDESTAL_XY, "geoms": geoms}, indent=1))
    print(f"[robot-json] {len(geoms)} visual geoms -> {dst}")
    return dst


# ─────────────────────────────── scene ────────────────────────────────
def build_scene() -> Path:
    import mujoco

    build_robot()
    manifest = json.loads((ENV_DIR / "manifest.json").read_text())

    mj = ET.Element("mujoco", model="dental_or_scene_v1.0")
    ET.SubElement(mj, "include", file="openarm_v1_black.xml")
    # Grasp physics (2026-09-29): with dt=0.002 and default contacts, light instruments
    # squeezed between the position-servoed fingers and the table sank 5-9 mm = through a
    # 3-8 mm instrument. dt=0.001 (upstream v1 default) allows stiff solref=0.002;
    # elliptic cones + impratio make friction grasps hold instead of slipping.
    ET.SubElement(mj, "option", timestep="0.001", integrator="implicitfast",
                  cone="elliptic", impratio="10")
    vis = ET.SubElement(mj, "visual")
    ET.SubElement(vis, "global", offwidth="1920", offheight="1080", azimuth="200", elevation="-22")
    ET.SubElement(vis, "headlight", active="1", ambient="0.38 0.38 0.40",
                  diffuse="0.50 0.50 0.52", specular="0.15 0.15 0.15")
    ET.SubElement(vis, "quality", shadowsize="8192", offsamples="8")
    ET.SubElement(vis, "map", shadowclip="3.0", shadowscale="0.7", znear="0.01")
    ET.SubElement(vis, "rgba", haze="0.16 0.18 0.22 1")
    ET.SubElement(mj, "statistic", center="0.4 0.5 1.0", extent="1.6")

    asset = ET.SubElement(mj, "asset")
    for mname, rgba in manifest["materials"].items():
        spec = manifest["material_props"].get(mname, {})
        ET.SubElement(asset, "material", name=f"env_{mname}", rgba=fmt(rgba),
                      specular=f"{spec.get('specular', 0.3):.3g}",
                      shininess=f"{spec.get('shininess', 0.3):.3g}",
                      reflectance=f"{spec.get('reflectance', 0.0):.3g}")

    wb = ET.SubElement(mj, "worldbody")
    ET.SubElement(wb, "light", name="ceiling_a", pos="-1.1 0.1 2.9", dir="0.2 0 -1", diffuse="0.45 0.45 0.45", castshadow="false")
    ET.SubElement(wb, "light", name="ceiling_b", pos="1.2 0.1 2.9", dir="-0.2 0 -1", diffuse="0.45 0.45 0.45", castshadow="false")
    ET.SubElement(wb, "light", name="surgical", pos="0.36 0.15 2.34", dir="-0.2 0.2 -1", diffuse="0.55 0.55 0.52", specular="0.3 0.3 0.3")
    ET.SubElement(wb, "geom", name="floor_col", type="plane", size="4 4 0.1", rgba="0 0 0 0", group="3",
                  contype="1", conaffinity="3")

    # static environment: visual meshes (no contact) + primitive colliders
    static = ET.SubElement(wb, "body", name="or_env")
    for part in manifest["static"]:
        mesh = f"env_{part['name']}"
        ET.SubElement(asset, "mesh", name=mesh, file=f"../dental_or/env/{part['file']}")
        ET.SubElement(static, "geom", name=f"vis_{part['name']}", type="mesh", mesh=mesh,
                      material=f"env_{part['material']}", contype="0", conaffinity="0", group="1")
    px, py = PEDESTAL_XY
    ET.SubElement(static, "geom", name="oa1_pedestal_column", type="box",
                  pos=fmt((px, py, PEDESTAL_TOP / 2)), size=fmt((0.05, 0.05, PEDESTAL_TOP / 2)), quat=fmt(BASE_QUAT),
                  rgba=BLACK, group="1", contype="1", conaffinity="3")
    ET.SubElement(static, "geom", name="oa1_pedestal_foot", type="box",
                  pos=fmt((px, py, 0.012)), size=fmt((0.20, 0.17, 0.012)), quat=fmt(BASE_QUAT),
                  rgba=GUNMETAL, group="1", contype="1", conaffinity="3")
    for box in manifest["colliders"]:
        ET.SubElement(static, "geom", name=f"col_{box['name']}", type="box",
                      pos=fmt(box["center"]), size=fmt(box["half"]),
                      rgba="1 0.3 0.1 0.35", group="3", friction="1 0.01 0.001",
                      contype="1", conaffinity="3", priority="2", solref="0.002 1", solimp="0.97 0.995 0.0005")

    # instruments: free bodies, visual mesh + convex-hull collider
    key_free = []
    for ins in manifest["instruments"]:
        b = ET.SubElement(wb, "body", name=f"instr_{ins['name']}", pos=fmt(ins["pos"]), quat=fmt(ins["quat"]))
        ET.SubElement(b, "freejoint", name=f"instr_{ins['name']}")
        for k, part in enumerate(ins["parts"]):
            mesh = f"instr_{ins['name']}_{k}"
            ET.SubElement(asset, "mesh", name=mesh, file=f"../dental_or/env/{part['file']}")
            ET.SubElement(b, "geom", name=f"vis_{ins['name']}_{k}", type="mesh", mesh=mesh,
                          material=f"env_{part['material']}", contype="0", conaffinity="0",
                          group="1", mass="0")
        # collision: per-primitive boxes following the real shape (rings open, shafts thin);
        # falls back to one bbox if the object has no recorded primitives
        cols = ins.get("cols") or [{"pos": ins["col_center"], "quat": [1, 0, 0, 0], "half": ins["col_half"]}]
        vols = [8 * c["half"][0] * c["half"][1] * c["half"][2] for c in cols]
        for k, (cb, v) in enumerate(zip(cols, vols)):
            ET.SubElement(b, "geom", name=f"col_{ins['name']}" if k == 0 else f"col_{ins['name']}_{k}",
                          type="box", pos=fmt(cb["pos"]), quat=fmt(cb["quat"]), size=fmt(cb["half"]),
                          mass=f"{ins['mass'] * v / sum(vols):.5g}", rgba="0.2 0.5 1 0.4",
                          group="3", friction="1.2 0.02 0.002", condim="4", priority="2",
                          contype="2", conaffinity="2",
                          solref="0.002 1", solimp="0.97 0.995 0.0005")
        ET.SubElement(b, "site", name=f"{ins['name']}_R", pos=fmt(ins["col_center"]), size="0.004",
                      rgba="0.1 0.3 1 1", group="4")
        key_free += list(ins["pos"]) + list(ins["quat"])

    for s in manifest.get("sites", []):
        ET.SubElement(wb, "site", name=s["name"], pos=fmt(s["pos"]), size=f"{s.get('size', 0.02)}",
                      rgba=s.get("rgba", "0 1 0 0.3"), group="4")

    ET.SubElement(wb, "body", name="cam_target", pos="0.70 0.70 1.00", mocap="true")
    ET.SubElement(wb, "camera", name="wide", pos="2.05 -2.15 2.05", fovy="46", mode="targetbody", target="cam_target")
    ET.SubElement(wb, "camera", name="staging", pos="-0.95 -2.1 1.75", fovy="40", mode="targetbody", target="cam_target")
    ET.SubElement(wb, "camera", name="robot_over", pos="1.85 0.35 2.05", fovy="55", mode="targetbody", target="cam_target")
    # dora-openarm-mujoco publishes this name when run with --render (recorded by video_recorder)
    ET.SubElement(wb, "camera", name="camera_ceiling", pos="1.85 -1.05 1.95", fovy="50",
                  mode="targetbody", target="cam_target", resolution="960 540")
    # second live view (recorded next to camera_ceiling): tray close-up from across the cart,
    # looking back at the grippers. Name reused from dora-openarm-mujoco's fixed camera list.
    ET.SubElement(wb, "body", name="tray_target", pos="0.80 0.74 0.95", mocap="true")
    ET.SubElement(wb, "camera", name="camera_head_left", pos="0.80 1.42 1.40", fovy="52",
                  mode="targetbody", target="tray_target", resolution="960 540")
    ET.SubElement(wb, "camera", name="tray_top", pos="0.83 0.85 2.25", xyaxes="0 -1 0 1 0 0", fovy="42")

    kf = ET.SubElement(mj, "keyframe")
    ET.SubElement(kf, "key", name="home", qpos=fmt(robot_home_qpos() + key_free), ctrl=fmt(robot_home_ctrl()))

    dst = OUT / "dental_or_scene_v1.0.xml"
    tree = ET.ElementTree(mj)
    ET.indent(tree, space="  ")
    header = ("<!-- GENERATED by tools/or_v1/build_scene.py — do not hand-edit.\n"
              "     Source: scenes/dental_or_v1.blend (env) + openarm_mujoco v1 (robot, black).\n"
              "     Teleop frame: site `arm_origin` (body_link0 +0.698 m). -->\n")
    dst.write_text(header + ET.tostring(mj, encoding="unicode"))

    m = mujoco.MjModel.from_xml_path(str(dst))
    print(f"[scene] {dst}\n        nq={m.nq} nv={m.nv} nu={m.nu} nbody={m.nbody} ngeom={m.ngeom} nmesh={m.nmesh}")
    return dst


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["robot", "robot-json", "scene"])
    a = ap.parse_args()
    {"robot": build_robot, "robot-json": dump_robot_json, "scene": build_scene}[a.what]()


if __name__ == "__main__":
    main()
