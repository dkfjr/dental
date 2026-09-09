#!/usr/bin/env python3
"""환자 메시를 '누운 자세'용 2조각으로 만든다 (Blender 필요).

환자를 캡슐/타원체 프리미티브로 두면 사람으로 안 보인다. 의사와 같은 인체 메시
(`DocPart_torso.obj` = 머리·양팔 포함 전신 1.819 m)를 재사용해 앙와위로 눕힌다.
의사와 구분되도록 0.96배로 줄이고(신장 1.746 m) XML 에서 환자복 재질을 입힌다.

앉은 의사와 같은 원리: 절단면은 가랑이, 회전축은 실제 고관절.
  상체 = z >= 0.768 -> 고관절 둘레로 x축 -72° 회전 (등받이 경사 18° 에 맞춤)
  다리 = z <  0.768 -> 고관절 둘레로 x축 -90° 회전 (시트면에 수평으로)

실행:  blender -b -P teleop/tools/build_supine_patient.py
출력:  sim/assets/doctor_rig/PatSupine_head.obj / _torso.obj / _legs.obj
       (머리와 상체는 같은 강체변환을 쓰고 재질만 다르게 준다)
"""
import bpy, bmesh, os
from mathutils import Vector

ROOT  = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC   = os.path.join(ROOT, 'sim/assets/doctor_rig/DocPart_torso.obj')
OUT   = os.path.join(ROOT, 'sim/assets/doctor_rig')
SCALE = 0.96          # 1.819 m -> 1.746 m (성인 표준 범위)
Z_CUT = 0.80 * SCALE  # 가랑이
OVERLAP = 0.16        # 조각을 겹치게 잘라 관절부가 벌어져도 속이 안 보이게

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.obj_import(filepath=SRC, forward_axis='Y', up_axis='Z')
src=[o for o in bpy.context.scene.objects if o.type=='MESH']
bpy.ops.object.select_all(action='DESELECT')
for o in src: o.select_set(True)
bpy.context.view_layer.objects.active=src[0]
if len(src)>1: bpy.ops.object.join()
base=bpy.context.view_layer.objects.active
base.scale=(SCALE,SCALE,SCALE)
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
zs=[v.co.z for v in base.data.vertices]
print(f"환자 신장 = {max(zs)-min(zs):.3f} m   절단면 z={Z_CUT:.3f}")

def slab(name, zmin, zmax, drop_islands=False):
    obj=base.copy(); obj.data=base.data.copy(); obj.name=name
    bpy.context.collection.objects.link(obj)
    bm=bmesh.new(); bm.from_mesh(obj.data)
    for z,side in ((zmax,'upper'),(zmin,'lower')):
        if z is None: continue
        bm.normal_update()
        bmesh.ops.bisect_plane(bm, geom=list(bm.verts)+list(bm.edges)+list(bm.faces), dist=1e-6,
                               plane_co=Vector((0,0,z)), plane_no=Vector((0,0,1)),
                               clear_outer=(side=='upper'), clear_inner=(side=='lower'))
        bm.normal_update()
        e=[x for x in bm.edges if x.is_boundary and abs(x.verts[0].co.z-z)<1e-4 and abs(x.verts[1].co.z-z)<1e-4]
        if e:
            r=bmesh.ops.holes_fill(bm, edges=e, sides=0)
            bmesh.ops.triangulate(bm, faces=[f for f in r.get('faces',[]) if f.is_valid])
    bm.to_mesh(obj.data); bm.free()
    # 다리 조각에 딸려 들어간 손을 떼어낸다 (의사와 동일 — 상체 조각이 손을 포함한다)
    if drop_islands:
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True); bpy.context.view_layer.objects.active=obj
        bpy.ops.mesh.separate(type='LOOSE')
        parts=[o for o in bpy.context.selected_objects if o.type=='MESH']
        cx=lambda o: sum(v.co.x for v in o.data.vertices)/max(len(o.data.vertices),1)
        keep=[o for o in parts if abs(cx(o))<0.14*SCALE/0.96]
        for o in [o for o in parts if o not in keep]: bpy.data.objects.remove(o,do_unlink=True)
        print(f"   {name}: 연결성분 {len(parts)} -> 다리 {len(keep)} 유지, 손 {len(parts)-len(keep)} 제거")
        bpy.ops.object.select_all(action='DESELECT')
        for o in keep: o.select_set(True)
        bpy.context.view_layer.objects.active=keep[0]
        if len(keep)>1: bpy.ops.object.join()
        obj=bpy.context.view_layer.objects.active; obj.name=name
    path=os.path.join(OUT,name+'.obj')
    bpy.ops.object.select_all(action='DESELECT'); obj.select_set(True)
    bpy.context.view_layer.objects.active=obj
    bpy.ops.wm.obj_export(filepath=path, export_selected_objects=True, export_materials=False,
                          export_uv=False, forward_axis='Y', up_axis='Z', apply_modifiers=False)
    zz=[v.co.z for v in obj.data.vertices]
    print(f"{name:20s} verts={len(obj.data.vertices):6d} z[{min(zz):.3f},{max(zz):.3f}] -> {path}")
    obj.hide_set(True)

Z_NECK = 1.50 * SCALE     # 목 — 머리는 피부/머리색 재질을 따로 입히려고 분리
slab('PatSupine_head',  Z_NECK - OVERLAP/2, None)
slab('PatSupine_torso', Z_CUT - OVERLAP,  Z_NECK)
slab('PatSupine_legs',  None,             Z_CUT, drop_islands=True)
