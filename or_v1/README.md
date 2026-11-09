# Dental VLA — MuJoCo assist scene + VR teleoperation

주경태 담당(**MuJoCo: 기본 씬 구성 + VR 텔레옵**) 시작 지점.
`enactic/dora-openarm-vr`(Meta Quest 3 → dora-rs → MuJoCo) 파이프라인을 그대로
재사용하고, MuJoCo 뷰어 노드의 `--xml`만 **덴탈 어시스트 씬**으로 바꾼 것.

씬은 사양서 v1.4(§9 transfer volume · §10 hand-off · §20-2 Arm A/B 분리)에 맞춰
**환자를 사이에 두고 술자에게 기구를 전달하는 어시스트 구조**로 구성돼 있다.

![overview](docs/scene_overview.png)

---

## 표준 씬 (단일)

`third_party/openarm_mujoco/v2/dental/dental_scene.xml` — **이 하나만 쓴다.**

Top-down 배치:
```
 +y  ASSISTANT ─ [ TRAY 기구 ]        (Arm B: 트레이·S 만)
      [ S staging ]
  0   ── OpenArm base ──  [ V 전달존 ] ── 환자 턱/가슴
 -y  OPERATOR ── 환자 얼굴(NO-FLY) ── 술자 손   (Arm A: V·S 진입)
```

- **OpenArm v2 양팔** (pedestal) — 어시스트 로봇
- **환자**(리클라인, 얼굴·구강=술야, 흉부 드레이프) + 랜드마크 `landmark_C`(턱)·`landmark_E`(눈)·`surgical_site`(구강)
- **술자**(operator zone에서 받는 손/전완) + `surgeon_hand`(전달 목표)
- **`V_zone`**(전달 volume, 초록) / **`nofly_zone`**(얼굴 위, 빨강, sharps 금지)
- **`stage`**(2슬롯 staging cradle S) / **트레이**(static zone)
- **기구 4종**(미러·겸자·엘리베이터·시린지), 각 기구에 파지영역 사이트
  `R`(로봇 파지, 파랑)·`O`(술자 파지, 주황)·`W`(working end, 빨강) — 사양 §20-3
  - 겸자 R은 **힌지**(핸들 쥐면 beak 닫힘), 시린지 R은 배럴 중앙 등 부록 B 규약 반영

> 기구·환자는 아직 기하 프리미티브(로드맵 1차). 2차에서 마네킹·구강 구조 정밀화.

## 빠른 시작

```bash
cd ~/dental-vla

# 씬 검증 (하드웨어·dora 없이)
pip install mujoco numpy scipy pillow
PYTHONPATH=src python3 -m dental_vla.scene_check            # PASS/FAIL
MUJOCO_GL=egl PYTHONPATH=src python3 -m dental_vla.scene_check --render out/   # 이미지까지

# 씬만 눈으로 보기 (드래그 회전/확대)
.venv/bin/python -m mujoco.viewer --mjcf third_party/openarm_mujoco/v2/dental/dental_scene.xml
```

## VR MuJoCo 텔레옵

파이프라인(`dora-openarm-vr` 그대로): `Quest 3 ─UDP:5006→ udp-receiver → ik → mujoco-viewer(덴탈 씬)`

```bash
# 첫 실행 준비
uv pip install "dora-rs-cli==0.5.0" "dora-rs==0.5.0"

# 빌드 → 실행  (config의 --xml이 dental_scene.xml을 상대경로로 가리킴 — 환경변수 불필요)
uv run dora build config/dataflow-mujoco.yaml --uv
uv run dora run   config/dataflow-mujoco.yaml --uv
```

**Quest 없이 테스트** — 뷰어가 뜨면 다른 터미널에서:
```bash
PYTHONPATH=src python3 -m dental_vla.mock_quest_sender      # 가짜 컨트롤러
```

## 단축어 (~/.bashrc)

```bash
export DENTAL_VLA=$HOME/dental-vla
alias dental-view='cd $DENTAL_VLA && .venv/bin/python -m mujoco.viewer --mjcf third_party/openarm_mujoco/v2/dental/dental_scene.xml'
alias dental-teleop='cd $DENTAL_VLA && uv run dora run config/dataflow-mujoco.yaml --uv'
alias dental-mock='cd $DENTAL_VLA && PYTHONPATH=$DENTAL_VLA/src python3 -m dental_vla.mock_quest_sender'
```
- `dental-view` → 표준 씬 보기 · `dental-teleop` → VR 텔레옵 · `dental-mock` → 가짜 손

## 좌표/프레임 (중요)

- 텔레옵 타깃은 `arm_origin` 사이트 프레임 기준(z=0.698). 이름 변경 금지.
- Unity(LH)→MuJoCo(RH) 변환은 리시버가 담당. 상세는 `docs/ARCHITECTURE.md`.

## 구성
```
config/       dataflow-mujoco.yaml (+ data-collection), test_metadata.yaml
src/dental_vla/  mock_quest_sender.py, scene_check.py
tools/        build_scene.py  (home 키프레임 자동 생성/갱신)
third_party/openarm_mujoco/  OpenArm v2 에셋 (Apache-2.0) + v2/dental/dental_scene.xml
docs/         ARCHITECTURE.md, scene_*.png
```

## 다음 단계
1. 벤치마크 채점기: V/no-fly/R-준수/자세/슬립 술어 → 선택·파지·제시 실패 분리 측정(사양 §14-2, §20-3)
2. 파지영역 조회 테이블(부록 B → `grasp_regions.yaml`)
3. 2차 씬: 덴탈 마네킹·구강 구조

## 라이선스
코드 Apache-2.0. `third_party/openarm_mujoco/` 에셋 © Enactic, Inc. (Apache-2.0).

---

## OR 씬 v1.0 — Blender 씬 + OpenArm **v1.0 (검정)** (2026-09-29)

Blender에서 편집 → MuJoCo로 자동 변환 → 같은 dora VR 파이프라인으로 텔레옵.

| 파일 | 역할 |
|---|---|
| `scenes/dental_or_v1.blend` | 편집용 원본 (Downloads의 `.blend1` 복사본에서 시작) |
| `third_party/openarm_mujoco/v1/` | 공식 OpenArm v1 MJCF (enactic/openarm_mujoco, `UPSTREAM_COMMIT`) |
| `third_party/openarm_mujoco/v1/dental_or/dental_or_scene_v1.0.xml` | **생성된** 씬 (직접 수정 금지) |
| `tools/or_v1/` | 로봇 교체 · 사진 기준 기구 트레이 생성 · Blender→MuJoCo 내보내기 · 렌더 |
| `config/dataflow-mujoco-or-v1.yaml` | 이 씬용 VR 텔레옵 + 녹화 (기존 `dataflow-mujoco.yaml`은 그대로 둠) |

```bash
# Blender에서 씬 수정 후 MuJoCo 씬 재생성
tools/or_v1/export_all.sh
# VR 텔레옵 (+ out/videos/teleop_or_v1*.mp4 자동 녹화)
DISPLAY=:1 uv run dora run config/dataflow-mujoco-or-v1.yaml --uv
# Quest 없이: 다른 터미널에서
PYTHONPATH=src .venv/bin/python -m dental_vla.mock_quest_sender
# 스냅샷
MUJOCO_GL=egl .venv/bin/python tools/or_v1/render.py out/or_v1 --sim 1
```

- Blender에서 `INSTR_` 로 시작하는 오브젝트는 MuJoCo에서 **잡을 수 있는 free body**가 됩니다(방향 맞춘 박스 충돌체, 커스텀 속성 `mass`).
- 로봇 위치: 몸체 `body_link0` (0.93, 0.35, 0.622), −x(환자 쪽)를 봄, `arm_origin` z=1.32. v1과 v2의 관절·액추에이터·사이트 이름이 같아서 IK/뷰어 노드는 수정하지 않았습니다.

### VR 텔레옵 (웹/CloudXR, 2026-09-29) — 전체 인수인계: `docs/HANDOFF_OR_V1_VR.md`
```bash
tools/or_v1/start_vr.sh                  # dora + 웹 브리지 한 번에 (Ctrl+C로 둘 다 종료)
# Quest 브라우저: https://<PC-IP>:48322/client/#/sim -> Start
# 차렷 자세 + 컨트롤러 끝을 아래로 향한 채 왼쪽 Y = 보정(로봇 차렷). 이후 손 이동은 0.73배, 컨트롤러 방향 = 그리퍼 방향
# 트리거 = 그리퍼 닫기(양손 동일, 비례)
tools/or_v1/start_vr.sh --record out/session.csv   # 컨트롤러 데이터 기록
```
- 영상: `out/videos/teleop_or_v1_<날짜_시각>.mp4` (dora 종료 시 완성)
- 스크립트 데모(movej, 왼팔 주사기 전달): `MUJOCO_GL=egl .venv/bin/python tools/or_v1/demo_handover.py` → `out/videos/demo_movej_left_handover.mp4`
- 점검 도구: `tools/or_v1/vr_probe.sh`(헤드셋/컨트롤러 수신), `tools/or_v1/tracking_monitor.py`(손→로봇 목표 실시간), `tools/or_v1/reach_check.py`, `tools/or_v1/grasp_test.py`
