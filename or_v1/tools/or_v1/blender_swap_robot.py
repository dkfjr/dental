# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
Blender: replace the OpenArm v2 reference with OpenArm v1.0 (black), posed at `home`.

    blender -b scenes/dental_or_v1.blend -P tools/or_v1/blender_swap_robot.py

Reads third_party/openarm_mujoco/v1/dental_or/robot_home_geoms.json
(made by `build_scene.py robot-json`), so Blender and MuJoCo show the same robot.
The robot collection stays a *reference* — the export script skips it (MuJoCo uses
the official MJCF robot).
"""
import json
import math
from pathlib import Path

import bpy
from mathutils import Matrix

ROOT = Path(bpy.path.abspath("//")).resolve().parent
JSON = ROOT / "third_party/openarm_mujoco/v1/dental_or/robot_home_geoms.json"
COLL = "OPENARM_V1_REFERENCE"


def srgb_to_lin(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def material(name, srgb, rough, metal):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    bsdf = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    lin = [srgb_to_lin(c) for c in srgb]
    bsdf.inputs["Base Color"].default_value = (*lin, 1.0)
    bsdf.inputs["Roughness"].default_value = rough
    bsdf.inputs["Metallic"].default_value = metal
    m.diffuse_color = (*lin, 1.0)
    return m


# 1) remove the v2 robot + its pedestal
old = bpy.data.collections.get("OPENARM_REFERENCE")
if old:
    for o in list(old.objects):
        bpy.data.objects.remove(o, do_unlink=True)
    bpy.data.collections.remove(old)
for o in [o for o in bpy.data.objects if o.name.startswith("COL_Robot_")]:
    bpy.data.objects.remove(o, do_unlink=True)

# 2) import v1 geoms
coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
if coll.name not in bpy.context.scene.collection.children:
    bpy.context.scene.collection.children.link(coll)
for o in list(coll.objects):
    bpy.data.objects.remove(o, do_unlink=True)

mats = {
    "matte_black": material("OA1_MatteBlack", (0.035, 0.035, 0.038), 0.55, 0.0),
    "metal_silver": material("OA1_Gunmetal", (0.11, 0.11, 0.12), 0.3, 0.8),
}
spec = json.loads(JSON.read_text())
cache = {}
for g in spec["geoms"]:
    f = str(ROOT / g["file"])  # paths are repo-relative
    if f not in cache:
        bpy.ops.wm.obj_import(filepath=f, forward_axis="Y", up_axis="Z")
        imp = bpy.context.selected_objects[0]
        cache[f] = imp.data
        bpy.data.objects.remove(imp, do_unlink=True)
    ob = bpy.data.objects.new(g["name"], cache[f])
    coll.objects.link(ob)
    sx, sy, sz = g["scale"]
    ob.matrix_world = Matrix(g["matrix"]) @ Matrix.Diagonal((sx, sy, sz, 1.0))
    # per-object material (linked to object, not mesh, because meshes are shared)
    if len(ob.data.materials) == 0:
        ob.data.materials.append(None)
    ob.material_slots[0].link = "OBJECT"
    ob.material_slots[0].material = mats.get(g["material"], mats["matte_black"])

# 3) pedestal column under body_link0 (black), floor -> base of the v1 body
bz = spec["base_pos"][2]
bx, by = spec["pedestal_xy"]
for n in ("oa1_pedestal_column", "oa1_pedestal_foot"):
    if n in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects[n], do_unlink=True)
yaw = math.radians(spec.get("base_yaw_deg", 180.0))
bpy.ops.mesh.primitive_cube_add(size=1, location=(bx, by, bz / 2), scale=(0.10, 0.10, bz),
                                rotation=(0, 0, yaw))
col = bpy.context.object
col.name = "oa1_pedestal_column"
bpy.ops.mesh.primitive_cube_add(size=1, location=(bx, by, 0.012), scale=(0.40, 0.34, 0.024),
                                rotation=(0, 0, yaw))
foot = bpy.context.object
foot.name = "oa1_pedestal_foot"
for o in (col, foot):
    for c in o.users_collection:
        c.objects.unlink(o)
    coll.objects.link(o)
    o.data.materials.append(mats["matte_black"] if o is col else mats["metal_silver"])

bpy.data.texts["README_EDITING"].from_string(
    "OpenArm v1.0 (black) is a locked visual reference generated from MJCF\n"
    "(tools/or_v1/blender_swap_robot.py); do not edit it here.\n"
    "Edit CHARACTERS_EDIT, ROOM_EDIT, PROPS_EDIT, OR_* collections freely, then run\n"
    "tools/or_v1/export_all.sh to regenerate the MuJoCo scene.\n")
bpy.ops.wm.save_mainfile()
print(f"[swap] v1 robot: {len(spec['geoms'])} geoms imported into {COLL}")
