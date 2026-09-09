#!/usr/bin/env python3
"""스테이징 구도를 오프스크린으로 렌더해 PNG 시트로 저장 (뷰어/GPU 창 없이 확인용).

    python sim/scripts/render_staging.py [출력경로.png]

기본 4뷰: staging 카메라 / wide 카메라 / 탑뷰 / 의사 클로즈업.
WSL·헤드리스에서는 MUJOCO_GL=egl 로 실행할 것.
"""
import os, sys, pathlib, mujoco
from PIL import Image

os.environ.setdefault("MUJOCO_GL", "egl")
XML = pathlib.Path(__file__).resolve().parents[1] / "main_v3_teleop.xml"
OUT = sys.argv[1] if len(sys.argv) > 1 else str(XML.parent / "output" / "staging.png")
pathlib.Path(OUT).parent.mkdir(parents=True, exist_ok=True)

m = mujoco.MjModel.from_xml_path(str(XML))
d = mujoco.MjData(m)
kid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")
if kid >= 0:
    mujoco.mj_resetDataKeyframe(m, d, kid)
mujoco.mj_forward(m, d)

W, H = int(m.vis.global_.offwidth), int(m.vis.global_.offheight)
r = mujoco.Renderer(m, H, W)
tiles = []
for cam in ("staging", "wide"):
    r.update_scene(d, camera=cam)
    tiles.append(Image.fromarray(r.render()))
for az, el, dist, look in ((90, -89, 3.6, [-0.1, 0.20, 0.95]),
                           (150, -14, 2.1, [-0.80, 0.55, 0.85])):
    c = mujoco.MjvCamera()
    c.type = mujoco.mjtCamera.mjCAMERA_FREE
    c.azimuth, c.elevation, c.distance = az, el, dist
    c.lookat[:] = look
    r.update_scene(d, camera=c)
    tiles.append(Image.fromarray(r.render()))

sheet = Image.new("RGB", (W * 2, H * 2))
for i, t in enumerate(tiles):
    sheet.paste(t, (W * (i % 2), H * (i // 2)))
sheet.save(OUT)
print("saved", OUT)

sys.stdout.flush()
os._exit(0)   # EGL 해제 단계에서 나는 무해한 예외 출력 회피
