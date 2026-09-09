#!/usr/bin/env python3
"""의사 리그 메시들의 열린 절단면(구멍)을 막는다 (Blender 필요).

원본 `asset/` 리그는 팔을 upper_arm / forearm / hand 로 쪼개면서 절단면을 막지 않았다.
그래서 팔꿈치·손목에서 메시 안쪽이 들여다보여 "깨진" 것처럼 보인다
(측정: 팔꿈치 19.8 mm, 손목 12.4 mm 간극). 몸통도 오른팔을 떼어낸 자리가 뚫려 있다.

실행:  blender -b -P teleop/tools/cap_doctor_holes.py
출력:  sim/assets/doctor_rig/DocCap_*.obj  (원본 파일은 건드리지 않음)
"""
import bpy, bmesh, os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RIG  = os.path.join(ROOT, 'sim/assets/doctor_rig')
# 주의: 경계가 복잡한 구멍(어깨·머리)을 holes_fill 로 메우면 면을 가로지르는 커다란
# 삼각형 파편이 생겨 오히려 깨져 보인다. 그래서 **평면에 가깝고 작은 경계만** 메운다.
# 얼굴 피부 패치는 메우지 않고 법선 오프셋만 준다 (cap_mask 와의 z-파이팅 제거용).
JOBS = [('DocPart_forearm.obj',  'DocCap_forearm',  True),
        ('DocPart2_left_hand.obj','DocCap_left_hand', True),
        ('DocPart3_face_skin.obj','DocCap_faceskin', False)]

for src, out, do_fill in JOBS:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.obj_import(filepath=os.path.join(RIG, src), forward_axis='Y', up_axis='Z')
    objs = [o for o in bpy.context.scene.objects if o.type == 'MESH']
    bpy.ops.object.select_all(action='DESELECT')
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    if len(objs) > 1:
        bpy.ops.object.join()
    obj = bpy.context.view_layer.objects.active
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.normal_update()
    before = sum(1 for e in bm.edges if e.is_boundary)
    if before and do_fill:
        # 경계 루프가 평면에 가깝고(두께<8mm) 작을 때만 메운다
        loops=[e for e in bm.edges if e.is_boundary]
        zs=[v.co.z for e in loops for v in e.verts]
        if loops and (max(zs)-min(zs))>0.30:
            loops=[]                      # 전신을 감싸는 큰 경계는 건드리지 않음
        res = bmesh.ops.holes_fill(bm, edges=loops, sides=0) if loops else {'faces':[]}
        bmesh.ops.triangulate(bm, faces=[f for f in res.get('faces', []) if f.is_valid])
        bm.normal_update()
    after = sum(1 for e in bm.edges if e.is_boundary)
    # 얼굴 피부 패치는 cap_mask 표면과 거의 같은 위치라 z-파이팅으로 얼굴이 찢어져 보인다.
    # 법선 방향으로 2 mm 밀어내 확실히 바깥에 오게 한다.
    if 'face_skin' in src:
        bm.normal_update()
        for v in bm.verts:
            v.co += v.normal * 0.002
    bm.to_mesh(obj.data)
    bm.free()

    path = os.path.join(RIG, out + '.obj')
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.wm.obj_export(filepath=path, export_selected_objects=True, export_materials=False,
                          export_uv=False, forward_axis='Y', up_axis='Z', apply_modifiers=False)
    print(f"{src:26s} 열린 모서리 {before:5d} -> {after:5d}   {out}.obj")
