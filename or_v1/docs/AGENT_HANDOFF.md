# Dental VLA — 작업 에이전트 인수인계

작성 2026-09-29 / 대상: **이 저장소(`~/dental-vla`)만** 담당할 작업 에이전트
범위 밖: `~/openarm_ws/chemistry`, `~/leisaac_openarm` (Isaac Sim 화학실험 과제). 절대 건드리지 말 것.

> 이 문서는 **작성 시점에 실제로 확인한 것**만 적었다. 확인하지 않은 것은 "미확인"이라고 표시했다.
> 이전 대화 기록에 의존하지 않는다.

---

## 0. 지금 바로 해야 할 일 (우선순위)

1. **VR 텔레옵 영상 촬영·공유** — 사용자가 요청한 산출물. Quest 3 로 덴탈 씬을 조작하는 화면을 녹화해 전달한다.
2. **Blender 씬 검토** — `~/Downloads/dental_scene_or_teleop.blend1` 이 현재 환경에 맞는지 판단하고, 맞으면 씬 **v1.0** 으로 반영, 안 맞으면 대체안을 만든다.
3. **새 씬에 VR 적용** — 2번 결과물로 텔레옵이 도는지 확인.

---

## 1. 확인된 환경 (실행해서 확인함)

| 항목 | 상태 |
|---|---|
| 프로젝트 루트 | `~/dental-vla` (현재는 GitHub dkfjr/dental 의 `or_v1/`) (git 저장소 **아님** — `git log` 비어 있음) |
| 파이썬 | `<repo>/or_v1/.venv/bin/python` |
| MuJoCo | **3.12.0** (venv 및 여러 conda env) |
| dora (python) | import 됨. **CLI 는 PATH 에 없음** → `uv run dora ...` 로 실행 |
| Blender | **설치됨** `/snap/bin/blender` |
| 표준 씬 | `third_party/openarm_mujoco/v2/dental/dental_scene.xml` (11.7 KB) |
| 씬 검증 | **PASS** (아래 결과 그대로 재현됨) |

```
$ PYTHONPATH=src .venv/bin/python -m dental_vla.scene_check
[ok] arm_origin at z=0.698
[ok] nq=46 nv=42 nu=16 nbody=32
[ok] instruments rest on tray (lowest z=0.420, max drift over 1s=0.001)
PASS
```

`config/out/` 에 과거 dora 세션 디렉토리가 5개 남아 있다 → **텔레옵이 최소 한 번은 돌았던 흔적**이다.

---

## 2. 씬 구조 (README 기준, 씬 XML 로 교차 확인함)

환자를 사이에 두고 술자에게 기구를 전달하는 **덴탈 어시스트** 구조. 사양서 v1.4 기준.

```
 +y  ASSISTANT ─ [ TRAY 기구 ]        (Arm B: 트레이·S 만)
      [ S staging ]
  0   ── OpenArm base ──  [ V 전달존 ] ── 환자 턱/가슴
 -y  OPERATOR ── 환자 얼굴(NO-FLY) ── 술자 손   (Arm A: V·S 진입)
```

- 로봇: **OpenArm v2 양팔** (pedestal 위). 화학실험 쪽 v1.0 과 다른 버전이니 혼동 금지
- 기구 4종(미러·겸자·엘리베이터·시린지), 각각 파지 사이트 `R`(로봇)·`O`(술자)·`W`(working end)
- 존: `V_zone`(전달, 초록) / `nofly_zone`(얼굴 위, 빨강) / `stage`(2슬롯) / 트레이
- 랜드마크: `landmark_C`(턱) `landmark_E`(눈) `surgical_site`(구강) `surgeon_hand`(전달 목표)
- 기구·환자는 **기하 프리미티브**다(1차). 마네킹·구강 정밀화는 2차 과제.

**바꾸면 안 되는 것**: `arm_origin` 사이트(z=0.698). 텔레옵 타깃 프레임의 기준이라 이름·위치를 바꾸면 VR 이 어긋난다.

---

## 3. 실행 방법 (README 에 적힌 것. 하드웨어 없이 되는 것만 이번에 확인함)

```bash
cd ~/dental-vla

# 씬 검증 — 하드웨어·dora 없이 (확인함: PASS)
PYTHONPATH=src .venv/bin/python -m dental_vla.scene_check
MUJOCO_GL=egl PYTHONPATH=src .venv/bin/python -m dental_vla.scene_check --render out/

# 씬 눈으로 보기
.venv/bin/python -m mujoco.viewer --mjcf third_party/openarm_mujoco/v2/dental/dental_scene.xml

# VR 텔레옵  (Quest 3 ─UDP:5006→ udp-receiver → ik → mujoco-viewer)   ※ 미확인
uv pip install "dora-rs-cli==0.5.0" "dora-rs==0.5.0"
uv run dora build config/dataflow-mujoco.yaml --uv
uv run dora run   config/dataflow-mujoco.yaml --uv

# Quest 없이 시험 — 뷰어가 뜬 뒤 다른 터미널에서
PYTHONPATH=src python3 -m dental_vla.mock_quest_sender
```

**미확인**: dora 빌드/실행, Quest 연결. 이번에 하드웨어를 붙이지 않아 검증하지 못했다.
"돌아간다"고 보고하지 말고 직접 돌려본 뒤 결과를 적을 것.

---

## 4. 이 PC 를 쓸 때 지켜야 할 것

- **GPU 를 다른 작업이 쓰고 있다.** 작업 전 반드시 확인:
  ```bash
  nvidia-smi --query-compute-apps=pid,used_memory,process_name --format=csv
  ```
  학습(`train_trace.py` 등)이 돌고 있으면 GPU 를 크게 먹는 작업(렌더·시뮬 대량 실행)은 끝난 뒤에 한다.
  MuJoCo 뷰어 정도는 가볍지만, 배치 렌더는 피한다.
- **디스크 여유가 적다.** `df -h /home` 로 확인하고, 대용량 산출물(영상·데이터셋)을 만들기 전에 공간을 확보한다.
- `~/openarm_ws`, `~/leisaac_openarm`, `~/Downloads/chemistry01_*` 는 **다른 과제**다. 읽지도 고치지도 말 것.
- `~/Downloads/_usd_보관/` 은 화학실험 쪽에서 정리해 둔 폴더다. 건드리지 말 것.

---

## 5. 과제 1 — VR 텔레옵 영상

**목표**: Quest 3 로 덴탈 씬의 OpenArm 을 조작하는 모습을 녹화해 공유.

절차 제안:
1. `dental-view` 로 씬이 정상인지 먼저 눈으로 확인
2. `uv run dora run config/dataflow-mujoco.yaml --uv` 로 텔레옵 기동
3. Quest 없이 먼저 `mock_quest_sender` 로 팔이 움직이는지 확인 → **이 단계 영상도 함께 남기면 좋다**(하드웨어 없이 재현 가능한 증거)
4. Quest 연결 후 실제 조작 녹화
5. 화면 녹화 도구는 이 PC 에 무엇이 있는지 확인 후 선택 (`which ffmpeg obs-studio`)

기록할 것: 실행 명령, dora 세션 ID(`config/out/` 에 생김), 녹화 파일 경로, 실패 시 오류 원문.

---

## 6. 과제 2 — Blender 파일 검토 (`~/Downloads/dental_scene_or_teleop.blend1`)

**지금까지 확인한 사실**
- 크기 7.3 MB, 형식은 **Zstandard 압축 데이터** (Blender 4.x 의 압축 .blend)
- 확장자가 **`.blend1`** 이다 → Blender 가 저장할 때 만드는 **직전 버전 백업 파일**이다. 원본 `.blend` 는 Downloads 에 없다.
- 내용은 **미확인**. 열어보지 않았다.

**해야 할 판단**
1. `blender --background ~/Downloads/dental_scene_or_teleop.blend1 --python-expr "..."` 로 내용을 덤프해 무엇이 들어 있는지 본다(오브젝트 목록·스케일·단위·원점).
2. 현재 MuJoCo 씬과 맞는지 본다. 최소 확인 항목:
   - **단위/스케일**: MuJoCo 씬은 미터 단위. Blender 가 cm 나 임의 스케일이면 변환 필요
   - **좌표계/업축**: Blender 는 +Z up, MuJoCo 도 +Z up 이지만 원점과 전방축이 맞는지
   - **`arm_origin` 기준(z=0.698)** 과 정합되는지
   - 로봇이 포함돼 있는지, 아니면 환경(체어·환자·트레이)만인지
   - 메시 폴리곤 수 — MuJoCo 충돌용으로 쓰려면 볼록분해가 필요할 수 있다
3. **맞으면**: 메시를 내보내 씬 v1.0 으로 반영. **안 맞으면**: 무엇이 안 맞는지 적고 대체안(기존 프리미티브 유지 + 필요한 부분만 메시 교체 등)을 제시한다.

**주의**: 원본 `.blend` 가 없으므로 `.blend1` 을 직접 수정하지 말고 **복사본으로 작업**한다.
```bash
cp ~/Downloads/dental_scene_or_teleop.blend1 ~/dental-vla/scenes/dental_v1.blend
```

**버전 규칙**: 사용자가 "1.0 버전으로" 요청했다. 새 씬은
`third_party/openarm_mujoco/v2/dental/dental_scene_v1.0.xml` 처럼 **버전을 파일명에 남기고**,
기존 `dental_scene.xml` 은 지우지 말고 둔다(되돌릴 수 있어야 한다).

---

## 7. 과제 3 — 새 씬에 VR 적용

`config/dataflow-mujoco.yaml` 의 `--xml` 이 씬을 **상대경로**로 가리킨다. 새 씬으로 바꾸려면
그 한 줄만 고치면 된다. 고치기 전에 원본을 복사해 둘 것.

적용 후 반드시:
1. `scene_check` 가 PASS 하는지
2. `arm_origin` 이 여전히 z=0.698 인지
3. `mock_quest_sender` 로 팔이 의도대로 움직이는지
4. 그다음에 Quest 실기 확인

---

## 8. 보고 형식

작업 끝에 아래를 남긴다.

```
## 한 일
- 실제 실행한 명령
- 수정한 파일 (경로 + 무엇을 왜)

## 확인된 것 / 확인 못 한 것
- PASS/FAIL 을 근거(로그·수치)와 함께
- 하드웨어가 없어 못 한 것은 NOT_RUN 으로 명시

## 남은 문제
```

**하지 말 것**: 실행하지 않은 것을 "된다"고 적기 · 수치를 추정으로 채우기 · 범위 밖 디렉토리 수정.

---

## 9. 참고 문서

- `README.md` — 빠른 시작, 단축어, 좌표/프레임 규약
- `docs/ARCHITECTURE.md` — Unity(LH) → MuJoCo(RH) 변환 등 파이프라인 상세
- `docs/scene_*.png`, `docs/render_*.png` — 씬 렌더 이미지
- `config/dataflow-mujoco.yaml` — 텔레옵 데이터플로우
- `config/dataflow-mujoco-data-collection.yaml` — 데이터 수집용
- `tools/build_scene.py` — home 키프레임 생성/갱신
