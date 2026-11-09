# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
Blender: dental extraction instrument set on a blue surgical drape, laid out like the
reference photo (docs/ref_instrument_tray.jpg).

    blender -b scenes/dental_or_v1.blend -P tools/or_v1/blender_build_tray.py

Replaces the old tray + instruments on the cart top with:
  * DRAPE (blue non-woven, overhanging the cart edges)
  * INSTR_* objects — one joined mesh per instrument, real-world size. The exporter turns
    every INSTR_* into a MuJoCo free body (graspable), everything else is static.

Photo -> cart mapping: photo top->bottom = world +x -> -x (bottom row toward the patient
chair); photo right (ring handles) = low y, toward the robot; photo left = high y.
Each instrument is built along local +X (working end at +X) and then rotated so +X points
to world +y (tips away from the robot), plus the small tilt seen in the photo.
"""
import json
import math

import bmesh
import bpy
from mathutils import Euler, Matrix, Vector

TOP_Z = 0.912            # cart top shelf (world)
CART = (0.13, 1.33, 0.60, 1.10)   # xmin xmax ymin ymax
DRAPE_T = 0.0015
SURF = TOP_Z + DRAPE_T   # instruments rest here
COLL = "PROPS_TRAY_PHOTO"


# ───────────────────────── materials ─────────────────────────
def srgb_lin(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def mat(name, srgb, rough=0.5, metal=0.0, alpha=1.0):
    m = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.use_nodes = True
    b = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    lin = [srgb_lin(c) for c in srgb]
    b.inputs["Base Color"].default_value = (*lin, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    b.inputs["Alpha"].default_value = alpha
    m.diffuse_color = (*lin, alpha)
    if alpha < 1:
        m.surface_render_method = "BLENDED"
    return m


M = {
    "steel": mat("TR_Steel", (0.78, 0.80, 0.82), 0.22, 1.0),
    "steel_dark": mat("TR_SteelSatin", (0.62, 0.64, 0.66), 0.45, 1.0),
    "gold": mat("TR_GoldTC", (0.83, 0.68, 0.30), 0.25, 1.0),
    "drape": mat("TR_DrapeBlue", (0.20, 0.42, 0.80), 0.9),
    "gauze": mat("TR_Gauze", (0.96, 0.96, 0.94), 0.95),
    "paper": mat("TR_PaperWhite", (0.93, 0.93, 0.90), 0.8),
    "teal": mat("TR_LabelTeal", (0.35, 0.72, 0.74), 0.7),
    "clear": mat("TR_ClearPlastic", (0.88, 0.92, 0.95), 0.1, 0.0, 0.45),
    "plunger": mat("TR_PlungerWhite", (0.95, 0.95, 0.95), 0.4),
    "rubber": mat("TR_RubberBlack", (0.05, 0.05, 0.05), 0.6),
    "pink": mat("TR_HubPink", (0.93, 0.55, 0.70), 0.4),
    "green": mat("TR_HubGreen", (0.25, 0.70, 0.45), 0.4),
    "thread": mat("TR_SilkBlack", (0.03, 0.03, 0.03), 0.7),
    "ruler": mat("TR_RulerMark", (0.15, 0.15, 0.15), 0.5),
}


# ───────────────────────── primitive helpers (local frame) ─────────────────────────
class Part:
    """Accumulates primitives into one bmesh, one material slot per material key."""

    def __init__(self):
        self.bm = bmesh.new()
        self.slots = []
        self.cols = []          # oriented collision boxes in the local frame (exported to MuJoCo)

    def _col(self, c, q, half):
        q = q.normalized()
        self.cols.append({"c": [float(v) for v in c], "q": [q.w, q.x, q.y, q.z],
                          "h": [max(0.003, float(v)) for v in half]})   # >=6 mm: thin shafts must not slip through the fingers

    def _slot(self, key):
        if key not in self.slots:
            self.slots.append(key)
        return self.slots.index(key)

    def _add(self, geom_fn, key, matrix):
        tmp = bmesh.new()
        geom_fn(tmp)
        bmesh.ops.transform(tmp, matrix=matrix, verts=tmp.verts)
        si = self._slot(key)
        for f in tmp.faces:
            f.material_index = si
        me = bpy.data.meshes.new("tmp")
        tmp.to_mesh(me)
        tmp.free()
        self.bm.from_mesh(me)
        bpy.data.meshes.remove(me)

    def box(self, c, size, key="steel", rot=(0, 0, 0)):
        S = Matrix.Diagonal((*size, 1))
        self._add(lambda b: bmesh.ops.create_cube(b, size=1.0),
                  key, Matrix.Translation(c) @ Euler(rot).to_matrix().to_4x4() @ S)
        self._col(c, Euler(rot).to_quaternion(), [v / 2 for v in size])

    def cyl(self, p0, p1, r0, r1=None, key="steel", seg=16):
        p0, p1 = Vector(p0), Vector(p1)
        d = p1 - p0
        L = d.length
        q = Vector((0, 0, 1)).rotation_difference(d.normalized())
        m = Matrix.Translation((p0 + p1) / 2) @ q.to_matrix().to_4x4()
        self._add(lambda b: bmesh.ops.create_cone(b, cap_ends=True, segments=seg, radius1=r0,
                                                  radius2=r1 if r1 is not None else r0, depth=L),
                  key, m)
        rm = max(r0, r1 if r1 is not None else r0)
        if L > 1e-6:
            self._col((p0 + p1) / 2, q, [rm, rm, L / 2])

    def ell(self, c, radii, key="steel", rot=(0, 0, 0), seg=24):
        m = Matrix.Translation(c) @ Euler(rot).to_matrix().to_4x4() @ Matrix.Diagonal((*radii, 1))
        self._add(lambda b: bmesh.ops.create_uvsphere(b, u_segments=seg, v_segments=seg // 2, radius=1.0),
                  key, m)
        self._col(c, Euler(rot).to_quaternion(), [0.8 * v for v in radii])

    def lathe(self, profile, key="steel", seg=40, c=(0, 0, 0)):
        """Surface of revolution about local Z from an (r, z) polyline (open, e.g. a cup)."""
        def fn(b):
            rows = []
            for i in range(seg):
                a = 2 * math.pi * i / seg
                rows.append([b.verts.new((r * math.cos(a), r * math.sin(a), z)) for r, z in profile])
            for i in range(seg):
                A, B = rows[i], rows[(i + 1) % seg]
                for j in range(len(profile) - 1):
                    b.faces.new((A[j], B[j], B[j + 1], A[j + 1]))
        self._add(fn, key, Matrix.Translation(c))
        # cup-like: bottom plate + 8 wall panels (keeps the inside open)
        rmax = max(r for r, _ in profile)
        top = max(z for _, z in profile)
        c = Vector(c)
        self._col(c + Vector((0, 0, 0.0015)), Euler((0, 0, 0)).to_quaternion(), [0.7 * rmax, 0.7 * rmax, 0.0015])
        for i in range(8):
            a = 2 * math.pi * i / 8
            self._col(c + Vector((0.95 * rmax * math.cos(a), 0.95 * rmax * math.sin(a), top / 2)),
                      Euler((0, 0, a)).to_quaternion(), [0.0015, 0.40 * rmax, top / 2])

    def ring(self, c, R, r, key="steel", seg=28):
        """Finger ring lying flat (axis = local Z)."""
        def fn(b):
            verts = []
            for i in range(seg):
                a = 2 * math.pi * i / seg
                row = []
                for j in range(8):
                    t = 2 * math.pi * j / 8
                    row.append(b.verts.new(((R + r * math.cos(t)) * math.cos(a),
                                            (R + r * math.cos(t)) * math.sin(a), r * math.sin(t))))
                verts.append(row)
            for i in range(seg):
                for j in range(8):
                    b.faces.new((verts[i][j], verts[(i + 1) % seg][j],
                                 verts[(i + 1) % seg][(j + 1) % 8], verts[i][(j + 1) % 8]))
        self._add(fn, key, Matrix.Translation(c))
        c = Vector(c)
        for i in range(4):                      # 4 arcs, hole stays open
            a = math.pi / 2 * i
            self._col(c + Vector((R * math.cos(a), R * math.sin(a), 0)), Euler((0, 0, a)).to_quaternion(),
                      [r, 0.72 * R, r])

    def finish(self, name, coll, world, mass):
        me = bpy.data.meshes.new(name)
        self.bm.to_mesh(me)
        self.bm.free()
        for k in self.slots:
            me.materials.append(M[k])
        for p in me.polygons:
            p.use_smooth = True
        ob = bpy.data.objects.new(name, me)
        coll.objects.link(ob)
        ob.matrix_world = world
        ob["mass"] = mass
        ob["colliders"] = json.dumps(self.cols)
        return ob


# local frame: +X = working end, Z up, instrument lying on z>=0 (z=0 is the drape)
def ring_handles(p, x0, gap, key="steel", R=0.0105, r=0.0022, h=0.004):
    """Two finger rings at x0 (handle end), shanks converging to the box joint at x=0."""
    for s in (+1, -1):
        c = (x0, s * gap, h)
        p.ring(c, R, r, key)
        p.cyl((x0 + R, s * gap * 0.7, h), (-0.004, s * 0.0022, h), 0.0022, 0.0018, "steel")
    p.box((0, 0, h), (0.012, 0.009, 0.005), "steel")          # box joint


def scissors(p, gold=True):
    ring_handles(p, -0.085, 0.017, "gold" if gold else "steel")
    for s, dz in ((1, 0.0045), (-1, 0.0032)):   # two curved blades
        pts = [(0.004, 0.0), (0.035, 0.0015 * s), (0.060, 0.006), (0.078, 0.012)]
        for a, b in zip(pts, pts[1:]):
            p.cyl((a[0], a[1], dz), (b[0], b[1], dz), 0.0026, 0.0017)


def hemostat(p, jaw=0.045, ring_x=-0.095, gold=False):
    ring_handles(p, ring_x, 0.016, "gold" if gold else "steel")
    p.box((jaw / 2 + 0.004, 0, 0.004), (jaw, 0.0045, 0.0035))
    p.cyl((jaw + 0.004, 0, 0.004), (jaw + 0.012, 0, 0.004), 0.0018, 0.0010)
    # ratchet
    for s in (1, -1):
        p.box((ring_x + 0.02, s * 0.006, 0.004), (0.010, 0.002, 0.003))


def thumb_forceps(p, L=0.15, width=0.010, serrated_tip=True):
    for s in (1, -1):
        p.box((0, s * width / 2, 0.004), (L * 0.75, 0.0035, 0.0018), "steel", (0, 0, -s * 0.035))
        p.box((L * 0.40, s * 0.0016, 0.004), (L * 0.2, 0.0022, 0.0015), "steel", (0, 0, -s * 0.01))
    p.box((-L / 2 + 0.005, 0, 0.004), (0.012, width + 0.003, 0.004))


def elevator(p, L=0.17, handle=0.10, tip="straight"):
    hx = -L / 2 + handle / 2
    p.ell((hx, 0, 0.014), (handle / 2, 0.016, 0.014), "steel_dark")
    p.ell((hx - handle * 0.18, 0, 0.018), (handle * 0.32, 0.019, 0.017), "steel_dark")
    p.cyl((hx + handle / 2 - 0.004, 0, 0.012), (L / 2 - 0.012, 0, 0.006), 0.0045, 0.0025, "gold" if tip == "gold" else "steel")
    p.cyl((L / 2 - 0.012, 0, 0.006), (L / 2, 0, 0.004), 0.0025, 0.0009)


def double_ended(p, L=0.18, hook=True):
    p.cyl((-0.05, 0, 0.005), (0.05, 0, 0.005), 0.0045, key="steel_dark", seg=12)
    for s in (1, -1):
        p.cyl((s * 0.05, 0, 0.005), (s * 0.075, 0, 0.004), 0.0035, 0.0015)
        end = (s * (L / 2), 0.004 * s if hook else 0, 0.0035)
        p.cyl((s * 0.075, 0, 0.004), end, 0.0015, 0.0008)


def explorer(p):
    p.cyl((-0.055, 0, 0.0045), (0.035, 0, 0.0045), 0.0042, key="steel_dark", seg=12)
    p.cyl((0.035, 0, 0.0045), (0.06, 0, 0.004), 0.003, 0.0012)
    p.cyl((0.06, 0, 0.004), (0.072, 0.012, 0.004), 0.0012, 0.0008)          # hooked end
    p.cyl((-0.055, 0, 0.0045), (-0.075, 0, 0.004), 0.003, 0.0012)
    p.cyl((-0.075, 0, 0.004), (-0.08, -0.004, 0.003), 0.0012, 0.0006)


def scalpel(p, band=None):
    p.box((-0.02, 0, 0.0025), (0.12, 0.009, 0.0025), "steel_dark")
    for i in range(7):   # ruler ticks
        p.box((-0.07 + i * 0.012, 0.003, 0.0038), (0.0012, 0.003, 0.0004), "ruler")
    if band:
        p.box((-0.055, 0, 0.0028), (0.004, 0.0094, 0.0028), band)
    p.box((0.047, 0, 0.0025), (0.015, 0.005, 0.0018))
    # curved blade (No.15 style)
    p.cyl((0.054, 0, 0.0025), (0.066, 0.0025, 0.0025), 0.003, 0.0022)
    p.cyl((0.066, 0.0025, 0.0025), (0.074, 0.0065, 0.0025), 0.0022, 0.0006)


def mirror(p):
    p.cyl((-0.085, 0, 0.0055), (0.035, 0, 0.0055), 0.0045, 0.004, "steel_dark", 12)
    p.cyl((0.035, 0, 0.0055), (0.065, 0.004, 0.005), 0.0022, 0.0015)
    p.cyl((0.064, 0.004, 0.006), (0.066, 0.004, 0.014), 0.012, 0.012, "steel", 24)   # head
    p.cyl((0.066, 0.004, 0.0145), (0.0662, 0.004, 0.0147), 0.0105, 0.0105, "clear", 24)


def syringe(p, L=0.105, r=0.0085, needle=0.03, hub="pink", bent=True, big=True):
    zc = r * 1.8                 # rests on the finger flange (widest part), like a real syringe
    p.cyl((-L / 2, 0, zc), (L / 2, 0, zc), r, key="clear", seg=20)                    # barrel
    p.cyl((-L / 2, 0, zc), (-L / 2 - 0.002, 0, zc), r * 1.8, key="clear", seg=20)    # finger flange
    p.cyl((-L / 2 - 0.002, 0, zc), (-L / 2 - 0.05, 0, zc), r * 0.35, key="plunger", seg=8)
    p.cyl((-L / 2 - 0.05, 0, zc), (-L / 2 - 0.053, 0, zc), r * 1.3, key="plunger", seg=16)
    p.cyl((L / 2 - 0.015, 0, zc), (L / 2 - 0.009, 0, zc), r * 0.93, key="rubber", seg=16)
    p.cyl((L / 2, 0, zc), (L / 2 + 0.008, 0, zc), r * 0.3, r * 0.22, key="clear", seg=10)
    p.cyl((L / 2 + 0.008, 0, zc), (L / 2 + 0.016, 0, zc), r * 0.3 if big else r * 0.25, key=hub, seg=10)
    tip = Vector((L / 2 + 0.016 + needle, 0, zc))
    if bent:
        mid = Vector((L / 2 + 0.016 + needle * 0.6, 0, zc))
        p.cyl((L / 2 + 0.016, 0, zc), mid, 0.0005, key="steel", seg=6)
        p.cyl(mid, mid + Vector((needle * 0.4, 0.012, 0)), 0.0005, key="steel", seg=6)
    else:
        p.cyl((L / 2 + 0.016, 0, zc), tip, 0.0004, key="steel", seg=6)


def needle(p, hub="green"):
    p.cyl((-0.012, 0, 0.0035), (0.0, 0, 0.0035), 0.0035, 0.0028, key=hub, seg=10)
    p.cyl((-0.020, 0, 0.0035), (-0.012, 0, 0.0035), 0.0030, key="clear", seg=10)   # cap end
    p.cyl((0.0, 0, 0.0035), (0.025, 0, 0.0035), 0.0005, key="steel", seg=6)


def cup(p, R=0.034, H=0.05, t=0.0012):
    # outer bottom -> wall -> rolled rim -> inner wall -> inner bottom (open top)
    prof = [(0.0, 0.0), (R * 0.90, 0.0), (R * 0.93, 0.003), (R, H - 0.002), (R + 0.0015, H),
            (R - t, H + 0.0005), (R * 0.93 - t, 0.004), (R * 0.88 - t, t + 0.0005), (0.0, t + 0.0005)]
    p.lathe(prof, "steel")


def periosteal(p):
    """EP23-style: long flat handle + tapered shank + small rounded blade (photo: diagonal)."""
    p.box((-0.035, 0, 0.004), (0.115, 0.016, 0.006), "steel_dark")
    p.box((-0.035, 0, 0.0072), (0.100, 0.011, 0.0008), "steel")
    p.cyl((0.022, 0, 0.004), (0.068, 0, 0.004), 0.004, 0.0025)
    p.ell((0.072, 0, 0.004), (0.006, 0.004, 0.0015))


def gauze(p):
    for i in range(6):
        p.box((0, 0, 0.0015 + i * 0.0026), (0.125 - i * 0.001, 0.068 - i * 0.0006, 0.0024), "gauze",
              (0, 0, (i % 2 - 0.5) * 0.02))


def blister(p):
    p.box((0, 0, 0.0008), (0.075, 0.068, 0.0016), "paper")
    p.box((0, 0, 0.006), (0.052, 0.034, 0.009), "clear")
    p.box((0.004, 0, 0.004), (0.028, 0.012, 0.004), "plunger")


def suture_pack(p):
    p.box((0, 0, 0.0008), (0.075, 0.028, 0.0015), "paper")
    p.box((-0.022, 0, 0.0017), (0.026, 0.028, 0.0003), "teal")
    for i in range(3):
        p.box((0.01 + i * 0.012, 0, 0.0017), (0.008, 0.012, 0.0003), "ruler")
    p.ring((0.05, 0.004, 0.001), 0.008, 0.0005, "thread", seg=20)


# ───────────────────────── build ─────────────────────────
def world(x, y, yaw_deg, z=SURF):
    """Local +X -> world direction at `yaw_deg` measured from world +y (toward -x positive)."""
    yaw = math.radians(90 + yaw_deg)
    return Matrix.Translation((x, y, z)) @ Matrix.Rotation(yaw, 4, "Z")


def clear_old():
    kill = [o for o in bpy.data.objects if (
        o.name.startswith(("bl_Tray_", "bl_Suction", "bl_Suture_Packet")) or
        any(k in o.name for k in ("Curette", "Elevator", "Forceps", "Mirror", "NeedleHolder",
                                  "Probe", "Retractor", "RootPicker", "Scalpel", "SutureScissors",
                                  "Syringe", "TissueForceps")) and o.name.startswith("bl_"))]
    for o in kill:
        bpy.data.objects.remove(o, do_unlink=True)
    old = bpy.data.collections.get(COLL)
    if old:
        for o in list(old.objects):
            bpy.data.objects.remove(o, do_unlink=True)
    return len(kill)


def build_drape(coll):
    x0, x1, y0, y1 = CART
    x0, x1, y0, y1 = x0 - 0.004, x1 + 0.004, y0 - 0.004, y1 + 0.004   # cover the shelf corners
    over = 0.16
    nx, ny = 90, 44
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=nx, y_segments=ny, size=0.5)
    W, D = (x1 - x0) + 2 * over, (y1 - y0) + 2 * over
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    import random
    rnd = random.Random(7)
    waves = [(rnd.uniform(6, 16), rnd.uniform(6, 16), rnd.uniform(0, 6.28), rnd.uniform(0.0006, 0.0014))
             for _ in range(6)]
    for v in bm.verts:
        x = cx + v.co.x * W
        y = cy + v.co.y * D
        dx = max(x0 - x, x - x1, 0.0)
        dy = max(y0 - y, y - y1, 0.0)
        d = max(dx, dy)
        z = TOP_Z + DRAPE_T
        # fold down over the edges with a soft roll
        x_c = min(max(x, x0), x1)
        y_c = min(max(y, y0), y1)
        if d > 0:
            k = min(d / 0.012, 1.0)
            out = 0.012 * math.sin(k * math.pi / 2)
            drop = d - 0.012 * k + 0.012 * (1 - math.cos(k * math.pi / 2))
            ox = (x - x_c) / (d + 1e-9)
            oy = (y - y_c) / (d + 1e-9)
            x, y = x_c + ox * (out + 0.002), y_c + oy * (out + 0.002)
            z = TOP_Z + DRAPE_T - drop
            # hanging folds
            z += 0.006 * math.sin((x + y) * 38) * min(d / over, 1)
            x += ox * 0.006 * math.sin((x - y) * 45) * min(d / over, 1)
            y += oy * 0.006 * math.sin((x - y) * 45) * min(d / over, 1)
        else:
            # subtle creases on the top (kept below the instruments' contact height)
            z += sum(a * (0.5 + 0.5 * math.sin(fx * x + fy * y + ph)) for fx, fy, ph, a in waves) * 0.5
        v.co = Vector((x, y, z))
    me = bpy.data.meshes.new("DRAPE_blue")
    bm.to_mesh(me)
    bm.free()
    me.materials.append(M["drape"])
    for p in me.polygons:
        p.use_smooth = True
    ob = bpy.data.objects.new("DRAPE_blue", me)
    coll.objects.link(ob)
    sol = ob.modifiers.new("thickness", "SOLIDIFY")
    sol.thickness = DRAPE_T
    sol.offset = -1
    return ob


def build():
    n_old = clear_old()
    coll = bpy.data.collections.get(COLL) or bpy.data.collections.new(COLL)
    if coll.name not in bpy.context.scene.collection.children:
        bpy.context.scene.collection.children.link(coll)
    build_drape(coll)

    # right column (photo top -> bottom). (name, builder, x, y, yaw_deg, mass)
    Y = 0.765                      # column centre line; ring handles toward y≈0.66
    items = [
        ("scissors_dean",      lambda p: scissors(p, gold=True),               1.245, Y + 0.005, -9, 0.045),
        ("needle_holder",      lambda p: hemostat(p, 0.040, -0.098, False),    1.165, Y + 0.012, -4, 0.050),
        ("forceps_adson",      lambda p: thumb_forceps(p, 0.14, 0.009),        1.115, Y + 0.010, 1, 0.020),
        ("hemostat",           lambda p: hemostat(p, 0.035, -0.085, False),    1.065, Y + 0.018, -5, 0.040),
        ("periosteal_ep23",    periosteal,                                     1.015, Y + 0.010, -22, 0.040),
        ("explorer",           explorer,                                       0.965, Y + 0.012, 0, 0.015),
        ("elevator_cryer",     lambda p: elevator(p, 0.165, 0.10, "gold"),      0.905, Y + 0.020, 2, 0.060),
        ("elevator_301",       lambda p: elevator(p, 0.160, 0.09),              0.860, Y + 0.022, 0, 0.055),
        ("elevator_34s",       lambda p: elevator(p, 0.170, 0.09),              0.815, Y + 0.020, 1, 0.055),
        ("curette_double_a",   lambda p: double_ended(p, 0.18, True),           0.765, Y + 0.010, 1, 0.018),
        ("curette_double_b",   lambda p: double_ended(p, 0.175, False),         0.745, Y + 0.010, 0, 0.018),
        ("scalpel_no3_ruler",  lambda p: scalpel(p, None),                      0.680, Y + 0.020, -4, 0.020),
        ("scalpel_no3_green",  lambda p: scalpel(p, "green"),                   0.655, Y + 0.025, 1, 0.020),
        ("dressing_forceps",   lambda p: thumb_forceps(p, 0.17, 0.008),         0.600, Y + 0.020, -5, 0.020),
        ("mouth_mirror",       mirror,                                         0.525, Y + 0.030, -22, 0.020),
        ("syringe_10ml",       lambda p: syringe(p, 0.085, 0.0085, 0.032, "pink", True),
                                                                               0.450, Y + 0.030, -16, 0.030),
    ]
    # left group (photo left) — high y
    left = [
        ("gauze",              gauze,                                          0.945, 0.965, 0, 0.010),
        ("blister_pack",       blister,                                        0.840, 0.965, 0, 0.008),
        ("syringe_1ml",        lambda p: syringe(p, 0.075, 0.0045, 0.013, "clear", False, False),
                                                                               0.575, 0.905, -90, 0.008),
        ("cup_stainless",      cup,                                            0.495, 1.000, 0, 0.080),
        ("needle_green",       needle,                                         0.520, 1.050, -80, 0.003),
        ("suture_silk_3_0",    suture_pack,                                    1.120, 0.890, 88, 0.004),
    ]
    made = []
    for name, fn, x, y, yaw, mass in items + left:
        p = Part()
        fn(p)
        ob = p.finish(f"INSTR_{name}", coll, world(x, y, yaw), mass)
        made.append(ob.name)
    bpy.data.texts["README_EDITING"].write(
        f"\nPROPS_TRAY_PHOTO: generated by tools/or_v1/blender_build_tray.py ({len(made)} items).\n"
        "INSTR_* objects become graspable free bodies in MuJoCo; move them freely here.\n")
    bpy.ops.wm.save_mainfile()
    print(f"[tray] removed {n_old} old objects; built drape + {len(made)} instruments")


build()
