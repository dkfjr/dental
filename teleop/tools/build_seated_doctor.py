#!/usr/bin/env python3
"""의사 메시를 '앉은 자세'용 3조각으로 절단한다 (Blender 필요).

왜 필요한가
-----------
`blender/dental_chair_scene.blend` 에는 **아마추어(리그)가 없다**. 의사는 서 있는
자세로 baked 된 메시라서 MuJoCo XML에서는 위치/회전만 가능하고 다리를 굽힐 수 없다.
그래서 몸통 메시를 골반/무릎 높이에서 잘라 3조각으로 만들고, 각 조각에 강체변환을
걸어 앉은 자세를 만든다. 절단면은 Blender의 holes_fill 로 막아 속이 비어 보이지 않게 한다.

관절을 새로 추가하지 않으므로 **nq(=156)가 그대로**이고 `home` 키프레임도 유효하다.

실행
----
    blender -b -P teleop/tools/build_seated_doctor.py

출력 (sim/assets/doctor_rig/)
    DocSeat_torso.obj   z >= 0.90   상체
    DocSeat_thigh.obj   0.52..0.90  허벅지(+엉덩이)
    DocSeat_shin.obj    z <  0.52   정강이+발

room_v3_teleop.xml 쪽 변환식 (메시 로컬좌표 기준, 골반 H=(0,0.03,0.90) 무릎 K=(0,0.037,0.52))
    상체   : 평행이동 (0, 0, -0.355)              # 골반이 좌면 위 z=0.545 로 내려앉음
    허벅지 : quat Rx(-90°) + 이동 (0, -0.87, 0.575) # H 둘레 -90° 회전 -> 앞으로 수평
    정강이 : 이동 (0, -0.387, 0.018)              # 허벅지 회전 후 무릎에서 +90° 되돌림 = 회전 상쇄
"""
import bpy, bmesh, os
from mathutils import Vector

ROOT  = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC   = os.path.join(ROOT, 'sim/assets/doctor_rig/DocPart2_torso_only.obj')
OUT   = os.path.join(ROOT, 'sim/assets/doctor_rig')
# 절단면은 '가랑이'(다리가 갈라지는 높이)에 두고, 회전축은 그보다 위인 '실제 고관절'에 둔다.
# 이렇게 해야 회전하는 조각이 다리뿐이고 골반/엉덩이 덩어리는 몸통에 남는다.
# (골반에서 자르면 엉덩이까지 통째로 앞으로 돌아가 넓적한 판때기가 된다)
Z_HIP, Z_KNEE = 0.80, 0.50
# 조각을 맞대면(butt joint) 상체를 앞으로 숙일 때 관절부가 벌어져 절단면 캡이 드러난다.
# 그래서 각 조각을 OVERLAP 만큼 더 길게 잘라 서로 겹치게 한다 -> 벌어져도 속이 안 보인다.
OVERLAP = 0.16   # 손이 z 0.654 까지 내려오므로 상체 절단면을 0.64 로 낮춰 손 전체를 포함

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.obj_import(filepath=SRC, forward_axis='Y', up_axis='Z')
src = [o for o in bpy.context.scene.objects if o.type == 'MESH']
bpy.ops.object.select_all(action='DESELECT')
for o in src:
    o.select_set(True)
bpy.context.view_layer.objects.active = src[0]
if len(src) > 1:
    bpy.ops.object.join()
base = bpy.context.view_layer.objects.active
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)


def slab(name, zmin, zmax, drop_islands=False, split_lhand=False):
    """[zmin, zmax] 구간만 남긴 사본을 만들고 절단면을 막아 OBJ로 저장."""
    obj = base.copy()
    obj.data = base.data.copy()
    obj.name = name
    bpy.context.collection.objects.link(obj)
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    for z, side in ((zmax, 'upper'), (zmin, 'lower')):
        if z is None:
            continue
        bm.normal_update()
        bmesh.ops.bisect_plane(
            bm, geom=list(bm.verts) + list(bm.edges) + list(bm.faces), dist=1e-6,
            plane_co=Vector((0, 0, z)), plane_no=Vector((0, 0, 1)),
            clear_outer=(side == 'upper'), clear_inner=(side == 'lower'))
        bm.normal_update()
        edges = [e for e in bm.edges if e.is_boundary
                 and abs(e.verts[0].co.z - z) < 1e-4 and abs(e.verts[1].co.z - z) < 1e-4]
        if edges:
            res = bmesh.ops.holes_fill(bm, edges=edges, sides=0)
            bmesh.ops.triangulate(bm, faces=[f for f in res.get('faces', []) if f.is_valid])
    bm.to_mesh(obj.data)
    bm.free()
    # --- 다리 조각에서 '손' 떼어내기 ---------------------------------------
    # 서 있는 자세에서 손은 허벅지 높이(z 0.50~0.80)에 있다. 그대로 자르면 손이
    # 허벅지 조각에 딸려 들어가고, 허벅지를 앞으로 회전시키면 무릎에 손이 붙는다.
    # 슬랩 안에서 손은 팔과 끊겨 '떠 있는 섬'이 되므로, 연결 성분으로 분리해
    # 작은 섬(=손)은 버리고 다리만 남긴다. 손은 상체 조각(z>=0.70)이 이미 포함한다.
    if split_lhand:
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True); bpy.context.view_layer.objects.active = obj
        bpy.ops.mesh.separate(type='LOOSE')
        parts = [o for o in bpy.context.selected_objects if o.type == 'MESH']
        def info(o):
            vs = o.data.vertices
            cx = sum(v.co.x for v in vs) / len(vs); zt = max(v.co.z for v in vs)
            return cx, zt
        hand = [o for o in parts if info(o)[0] < -0.10 and info(o)[1] < 0.95]
        body = [o for o in parts if o not in hand]
        print(f"   {name}: 연결성분 {len(parts)} -> 몸통 {len(body)}, 왼손섬 {len(hand)}")
        if hand:
            bpy.ops.object.select_all(action='DESELECT')
            for o in hand: o.select_set(True)
            bpy.context.view_layer.objects.active = hand[0]
            if len(hand) > 1: bpy.ops.object.join()
            h = bpy.context.view_layer.objects.active; h.name = 'DocSeat_lhand'
            bpy.ops.wm.obj_export(filepath=os.path.join(OUT, 'DocSeat_lhand.obj'),
                                  export_selected_objects=True, export_materials=False,
                                  export_uv=False, forward_axis='Y', up_axis='Z')
            zz=[v.co.z for v in h.data.vertices]
            print(f"   DocSeat_lhand   verts={len(h.data.vertices)} z[{min(zz):.3f},{max(zz):.3f}]")
            h.hide_set(True)
        bpy.ops.object.select_all(action='DESELECT')
        for o in body: o.select_set(True)
        bpy.context.view_layer.objects.active = body[0]
        if len(body) > 1: bpy.ops.object.join()
        obj = bpy.context.view_layer.objects.active; obj.name = name
    if drop_islands:
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True); bpy.context.view_layer.objects.active = obj
        bpy.ops.mesh.separate(type='LOOSE')
        parts = [o for o in bpy.context.selected_objects if o.type == 'MESH']
        def cx(o):
            vs = o.data.vertices
            return sum(v.co.x for v in vs) / max(len(vs), 1)
        keep = [o for o in parts if abs(cx(o)) < 0.14]      # 다리 = 정중선 근처
        drop = [o for o in parts if o not in keep]          # 손 = 바깥쪽(|x| ~ 0.18+)
        for o in drop:
            bpy.data.objects.remove(o, do_unlink=True)
        print(f"   {name}: 연결성분 {len(parts)}개 -> 다리 {len(keep)}개 유지, 손 등 {len(drop)}개 제거")
        bpy.ops.object.select_all(action='DESELECT')
        for o in keep: o.select_set(True)
        bpy.context.view_layer.objects.active = keep[0]
        if len(keep) > 1: bpy.ops.object.join()
        obj = bpy.context.view_layer.objects.active
        obj.name = name
    path = os.path.join(OUT, name + '.obj')
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.wm.obj_export(filepath=path, export_selected_objects=True,
                          export_materials=False, export_uv=False,
                          forward_axis='Y', up_axis='Z', apply_modifiers=False)
    zs = [v.co.z for v in obj.data.vertices]
    print(f"{name:16s} verts={len(obj.data.vertices):6d} z[{min(zs):.3f},{max(zs):.3f}] -> {path}")
    obj.hide_set(True)


# 상체 조각 안에는 '왼손'이 z 0.64~0.88 에 떠 있는 섬으로 들어 있다(팔뚝은 DocPart2_left_hand 로 별도).
# 이걸 상체에 그대로 두면 상체를 20° 숙일 때 팔 전체가 앞으로 딸려와 "가슴에서 손이 나오는" 모양이 된다.
# 그래서 섬을 떼어 DocSeat_lhand 로 따로 뽑고, XML 에서 어깨 둘레로 -20° 되돌려 중력 방향으로 늘어뜨린다.
slab('DocSeat_torso', Z_HIP - OVERLAP, None, split_lhand=True)
slab('DocSeat_thigh', Z_KNEE, Z_HIP, drop_islands=True)   # 0.50~0.80 (손 섬 제거)
slab('DocSeat_shin',  None,   Z_KNEE + OVERLAP/2, drop_islands=True)  # ~0.55 (손 섬 제거)
