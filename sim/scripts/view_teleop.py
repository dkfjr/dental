#!/usr/bin/env python3
"""텔레옵 씬을 MuJoCo 뷰어에 `home` 자세로 띄운다.

    python3 sim/scripts/view_teleop.py            # staging 카메라로 시작
    python3 sim/scripts/view_teleop.py wide       # 카메라 지정 (staging|wide|free)
    python3 sim/scripts/view_teleop.py --step     # 물리 스텝 실행(기본은 정지)

기본은 물리 스텝을 돌리지 않는 정적 뷰(느슨한 기구가 안 튀게). 마우스로 회전/줌.
"""
import os, sys, time, pathlib

os.environ.setdefault("MUJOCO_GL", "glfw")   # GUI 창 (오프스크린이면 render_staging.py 사용)
import mujoco, mujoco.viewer

def _free_cam_from(m, d, cam_name, v):
    """이름 있는 카메라의 구도를 그대로 '자유 카메라'로 옮긴다.

    고정(fixed) 카메라로 띄우면 MuJoCo 뷰어에서 마우스 회전/줌이 전혀 먹지 않는다.
    그래서 같은 위치·방향을 자유 카메라 파라미터(lookat/distance/azimuth/elevation)로
    변환해서 넣어 준다. 첫 화면은 동일하고, 바로 마우스로 돌리고 당길 수 있다.
    """
    import numpy as np
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
    if cid < 0:
        return False
    cpos = d.cam_xpos[cid].copy()
    # targetbody 카메라면 그 바디를, 아니면 카메라가 보는 방향으로 2 m 앞을 중심으로 삼는다
    tb = int(m.cam_targetbodyid[cid])
    if tb >= 0:
        look = d.xpos[tb].copy()
    else:
        look = cpos - d.cam_xmat[cid].reshape(3, 3)[:, 2] * 2.0
    vv = cpos - look
    dist = float(np.linalg.norm(vv))
    v.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    v.cam.lookat[:] = look
    v.cam.distance = dist
    # MuJoCo 자유 카메라는  pos = lookat - distance * forward,
    # forward = (cos(el)cos(az), cos(el)sin(az), sin(el))  이므로 forward = -vv/dist.
    # (부호를 빼먹으면 화면이 위아래·앞뒤로 뒤집힌다)
    fwd = -vv / max(dist, 1e-9)
    v.cam.azimuth = float(np.degrees(np.arctan2(fwd[1], fwd[0])))
    v.cam.elevation = float(np.degrees(np.arcsin(np.clip(fwd[2], -1, 1))))
    return True


args = sys.argv[1:]
do_step = "--step" in args
lock_cam = "--fixed" in args   # 고정 카메라(마우스 조작 불가)로 띄우고 싶을 때
cam_name = next((a for a in args if not a.startswith("-")), "staging")

XML = pathlib.Path(__file__).resolve().parents[1] / "main_v3_teleop.xml"
m = mujoco.MjModel.from_xml_path(str(XML))
d = mujoco.MjData(m)

kid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")
if kid >= 0:
    mujoco.mj_resetDataKeyframe(m, d, kid)
    d.ctrl[:] = m.key_ctrl[kid]
mujoco.mj_forward(m, d)

with mujoco.viewer.launch_passive(m, d) as v:
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
    if cid >= 0 and lock_cam:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        v.cam.fixedcamid = cid
    elif cid >= 0:
        _free_cam_from(m, d, cam_name, v)
    print(f"[view_teleop] {XML.name} · keyframe=home · camera={cam_name if cid >= 0 else 'free'}"
          f" · {'물리 ON' if do_step else '물리 OFF(정적)'}  — 창을 닫으면 종료", flush=True)
    print("           마우스: 좌드래그 회전 / 우드래그 이동 / 휠 줌  ·  [ ] 카메라 전환  ·  Esc 자유카메라", flush=True)
    while v.is_running():
        if do_step:
            mujoco.mj_step(m, d)
        else:
            mujoco.mj_forward(m, d)   # home 자세 유지
        v.sync()
        time.sleep(m.opt.timestep if do_step else 0.02)

sys.stdout.flush()
os._exit(0)   # GLFW/EGL 해제 단계에서 나는 코어덤프 회피 (정상 종료 후)
