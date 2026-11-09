# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
Blender -> MuJoCo export of the OR environment.

    blender -b scenes/dental_or_v1.blend -P tools/or_v1/blender_export_env.py

Writes third_party/openarm_mujoco/v1/dental_or/env/:
    <object>__<material>.obj   world-space (static) or body-local (instrument) triangles
    manifest.json              materials, static parts, box colliders, instruments, sites

Rules
  * collection OPENARM_V1_REFERENCE is skipped (MuJoCo uses the official robot MJCF)
  * objects whose name starts with INSTR_ become free bodies (graspable instruments);
    a custom property `mass` (kg) is honoured, default 0.03
  * objects with custom property `collider` = True (or matching COLLIDER_NAMES) get a
    world AABB box collider; everything else is visual only
"""
import json
import re
from pathlib import Path

import bpy
from mathutils import Matrix, Vector

ROOT = Path(bpy.path.abspath("//")).resolve().parent
OUT = ROOT / "third_party/openarm_mujoco/v1/dental_or/env"
SKIP_COLLECTIONS = {"OPENARM_V1_REFERENCE", "MJCF_COLLISION_PROXIES"}
# furniture that the arms must not pass through
COLLIDER_NAMES = re.compile(
    r"^(bl_Cart_Shelf_.*|bl_Cart_Post_.*|bl_Tray_.*|TRAY_.*|bl_Chair_.*|bl_Backrest_g|bl_Headrest_g|"
    r"bl_Seat_Pan_g|bl_Leg_Rest_g|OR_PatientMonitor|OR_ScrubSink|OR_SupplyCabinet|OR_RightWall|"
    r"wall_back|wall_side|oa1_pedestal_.*|patient_.*_mesh)$")


def lin_to_srgb(c):
    return 12.92 * c if c <= 0.0031308 else 1.055 * c ** (1 / 2.4) - 0.055


def mat_props(m):
    """(rgba sRGB, props) from a Principled BSDF; follows a linked RGB node if any."""
    rgb, rough, metal = (0.7, 0.7, 0.7), 0.5, 0.0
    if m and m.node_tree:
        bsdf = next((n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED"), None)
        if bsdf:
            inp = bsdf.inputs["Base Color"]
            col = inp.default_value
            if inp.is_linked:
                src = inp.links[0].from_node
                if src.type == "RGB":
                    col = src.outputs[0].default_value
                elif "Color" in src.inputs and not src.inputs["Color"].is_linked:
                    col = src.inputs["Color"].default_value
                else:
                    col = m.diffuse_color
            rgb = tuple(lin_to_srgb(c) for c in col[:3])
            rough = bsdf.inputs["Roughness"].default_value
            metal = bsdf.inputs["Metallic"].default_value
            alpha = bsdf.inputs["Alpha"].default_value
            return [*rgb, alpha], {"specular": 0.2 + 0.6 * metal + 0.2 * (1 - rough),
                                   "shininess": max(0.05, 1 - rough),
                                   "reflectance": 0.25 * metal}
    return [*rgb, 1.0], {"specular": 0.3, "shininess": 0.3, "reflectance": 0.0}


def safe(s):
    return re.sub(r"[^A-Za-z0-9_]+", "_", s)


def write_obj(path, verts, norms, tris):
    with open(path, "w") as f:
        for v in verts:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for n in norms:
            f.write(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n")
        for a, b, c in tris:
            f.write(f"f {a}//{a} {b}//{b} {c}//{c}\n")


def export_object(ob, depsgraph, frame=None):
    """Split an evaluated mesh by material slot. Returns [(file, material_name, material)].
    Vertices are world coordinates, or expressed in `frame` (4x4 world pose) if given."""
    ev = ob.evaluated_get(depsgraph)
    me = ev.to_mesh()
    me.calc_loop_triangles()
    to_frame = frame.inverted() if frame is not None else Matrix.Identity(4)
    mw = to_frame @ ob.matrix_world
    nm = mw.to_3x3().inverted().transposed()
    by_mat = {}
    for tri in me.loop_triangles:
        by_mat.setdefault(tri.material_index, []).append(tri)
    out = []
    for mi, tris in by_mat.items():
        mat = ob.material_slots[mi].material if mi < len(ob.material_slots) else None
        mname = safe(mat.name) if mat else "default"
        verts, norms, faces = [], [], []
        for tri in tris:
            idx = []
            for li in tri.loops:
                lo = me.loops[li]
                verts.append(tuple(mw @ me.vertices[lo.vertex_index].co))
                n = (nm @ Vector(lo.normal)).normalized()
                norms.append(tuple(n))
                idx.append(len(verts))
            if mw.determinant() < 0:
                idx.reverse()
            faces.append(idx)
        if len(faces) == 0:
            continue
        fname = f"{safe(ob.name)}__{mname}.obj"
        write_obj(OUT / fname, verts, norms, faces)
        out.append((fname, mname, mat))
    ev.to_mesh_clear()
    return out


def world_aabb(ob):
    pts = [ob.matrix_world @ Vector(c) for c in ob.bound_box]
    lo = Vector([min(p[i] for p in pts) for i in range(3)])
    hi = Vector([max(p[i] for p in pts) for i in range(3)])
    return lo, hi


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for f in OUT.glob("*.obj"):
        f.unlink()
    dg = bpy.context.evaluated_depsgraph_get()
    materials, mprops, static, colliders, instruments, sites = {}, {}, [], [], [], []

    def reg(mname, mat):
        if mname not in materials:
            materials[mname], mprops[mname] = mat_props(mat)

    for ob in bpy.context.scene.objects:
        if any(c.name in SKIP_COLLECTIONS for c in ob.users_collection):
            continue
        if ob.type == "EMPTY" and ob.name.startswith("SITE_"):
            sites.append({"name": ob.name[5:], "pos": list(ob.matrix_world.translation),
                          "size": ob.empty_display_size})
            continue
        if ob.type != "MESH" or ob.hide_render or len(ob.data.polygons) == 0:
            continue
        if ob.name.startswith("INSTR_"):
            # body frame = object rotation, origin at the local bbox centre -> oriented box collider
            bb = [Vector(c) for c in ob.bound_box]
            lo = Vector([min(p[i] for p in bb) for i in range(3)])
            hi = Vector([max(p[i] for p in bb) for i in range(3)])
            sc = ob.matrix_world.to_scale()
            rot = ob.matrix_world.to_quaternion()
            center = ob.matrix_world @ ((lo + hi) / 2)
            frame = Matrix.Translation(center) @ rot.to_matrix().to_4x4()
            parts = export_object(ob, dg, frame=frame)
            for _, mname, mat in parts:
                reg(mname, mat)
            half = [max(0.0015, (hi[i] - lo[i]) * sc[i] / 2) for i in range(3)]
            # per-primitive oriented boxes recorded by blender_build_tray.py (local object frame)
            cols = []
            if "colliders" in ob:
                c0 = (lo + hi) / 2
                for cb in json.loads(ob["colliders"]):
                    cols.append({"pos": [cb["c"][i] - c0[i] for i in range(3)], "quat": cb["q"], "half": cb["h"]})
            instruments.append({
                "name": safe(ob.name[6:]).lower(),
                "pos": [center[0], center[1], center[2] + 0.0005],
                "quat": [rot.w, rot.x, rot.y, rot.z],
                "parts": [{"file": f, "material": m} for f, m, _ in parts],
                "col_center": [0, 0, 0], "col_half": half,
                "mass": float(ob.get("mass", 0.03)),
                "cols": cols,
            })
            continue
        for f, mname, mat in export_object(ob, dg):
            reg(mname, mat)
            static.append({"name": f[:-4], "file": f, "material": mname})
        if ob.get("collider") or COLLIDER_NAMES.match(ob.name):
            lo, hi = world_aabb(ob)
            colliders.append({"name": safe(ob.name), "center": list((lo + hi) / 2),
                              "half": [max(0.002, (hi[i] - lo[i]) / 2) for i in range(3)]})

    manifest = {"materials": materials, "material_props": mprops, "static": static,
                "colliders": colliders, "instruments": instruments, "sites": sites}
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=1))
    print(f"[export] static={len(static)} colliders={len(colliders)} "
          f"instruments={len(instruments)} sites={len(sites)} materials={len(materials)} -> {OUT}")


main()
