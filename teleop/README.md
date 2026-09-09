# Dental_VLA — VR 텔레옵 트랙 (드롭인)

기존 v3 데모 씬(의자·의사·트레이·기구)은 그대로 두고, **OpenArm을 VR로 조종**하는 트랙.
`enactic/dora-openarm-vr` 파이프라인(Quest 3 → UDP → IK → MuJoCo)을 재사용하고,
MuJoCo 노드의 `--xml`만 이 프로젝트의 텔레옵 씬으로 바꾼 것.

## 추가/변경된 파일
- `sim/main_v3_teleop.xml` — `main_v3_blender`와 동일 + **① 공식 OpenArm torso(body_link0) 추가 ② `home` 키프레임**
- `sim/openarm_bimanual_v3_teleop.xml` — 네 팔 + torso 블록, `arm_origin`을 팔 중점(0.79 0.5 1.20)으로 이동(텔레옵 좌표 기준)
- `sim/scripts/view_teleop.py` — 뷰어를 home 자세로 여는 실행 스크립트
- `sim/room_v3_teleop.xml` — room 사본 + **씬 스테이징**(아래 참조)
- `sim/scripts/render_staging.py` — 구도를 오프스크린 PNG로 확인(뷰어 없이)
- `teleop/` — dora config · mock 송신기 · 키프레임 재생성 도구 · **의사 메시 절단 도구**

> 네 원본(`main_v3_blender.xml`, `openarm_bimanual_v3.xml`, `episode_v3.py` 등)은 **안 건드림.**

## 1) MuJoCo로 씬 띄우기 (명령어)
```bash
cd ~/projects/Dental_VLA          # 네 프로젝트 루트
source .venv/bin/activate
python sim/scripts/view_teleop.py   # home 자세로 뷰어가 뜸 (마우스로 회전)
```
또는 그냥: `python -m mujoco.viewer --mjcf sim/main_v3_teleop.xml` (뷰어에서 keyframe=home 선택)

## 2) VR 텔레옵
```bash
cd teleop
uv pip install "dora-rs-cli==0.5.0" "dora-rs==0.5.0"
uv run dora build config/dataflow-mujoco.yaml --uv
uv run dora run   config/dataflow-mujoco.yaml --uv
```
Quest 없이 테스트 — 뷰어 뜨면 다른 터미널에서:
```bash
cd teleop && PYTHONPATH=. python -m dental_vla_teleop.mock_quest_sender
```

## 씬 수정 후
팔 자세/씬을 바꿨으면 키프레임 재생성:
```bash
python teleop/tools/build_home_keyframe.py
```

## 알아둘 것 (텔레옵 정확도)
- `arm_origin`을 팔 중점으로 옮겨 좌표 기준은 맞췄지만, **라이브 IK 추적 품질은 dora 런타임에서 직접 확인** 필요(이 씬의 베이스가 카트 위 z=1.20이라 도달범위가 표준 pedestal과 다름). 팔이 목표를 못 따라가면 IK 인자(`--damping`, `--max-iters`)나 타깃 스케일 조정.
- 느슨한 기구 freejoint 일부가 물리상 미세하게 흔들릴 수 있음(데모는 weld로 잡음). 텔레옵 관찰엔 무관.

---

## 명령어 세팅 (설치 완료 — 2026-09-03)

`~/.bashrc`에 이미 설치돼 있다. (설치 전에 `~/dental-vla`를 가리키는 **낡은 중복 블록 3개**가
있어서 `dental-view`가 깨져 있었음 → 제거하고 하나로 통합. 백업: `~/.bashrc.bak.*`)

프로젝트에 `.venv`가 없어 **시스템 `python3`**(mujoco 3.12.0)를 쓴다 — `$DENTAL_PY`로 지정.

| 명령어 | 하는 일 | 상태 |
|--------|---------|------|
| `dental-view` | GUI 뷰어, `home` 자세 + `staging` 카메라 | ✅ 확인됨 |
| `dental-view wide` / `free` | 카메라 바꿔서 열기 | ✅ |
| `dental-view --step` | 물리까지 돌리며 보기(기본은 정적) | ⚠ 기구 freejoint 버그 있음(아래) |
| `dental-scene` | 순수 `mujoco.viewer` — 키프레임/카메라를 UI에서 선택 | ✅ |
| `dental-shot` | 창 없이 4분할 구도 PNG 저장 후 열기 → `sim/output/staging.png` | ✅ |
| `dental-setup` | dora 설치 + 노드 빌드 (**텔레옵 최초 1회**) | ⏳ 미실행 |
| `dental-teleop` | VR 텔레옵 파이프라인 실행 | ⏳ `dental-setup` 필요 |
| `dental-mock` | Quest 없이 가짜 컨트롤러 UDP 송신 (새 터미널) | ✅ |
| `dental-keyframe` | `home` 키프레임 재생성 | ✅ |
| `dental-doctor` | 앉은 의사 메시 재절단(Blender) | ✅ |
| `dental-help` | 위 목록을 `~/.bashrc`에서 그대로 출력 | ✅ |

GUI는 `DISPLAY=${DISPLAY:-:1}` + `MUJOCO_GL=glfw`로 고정돼 있어 X 없는 셸에서도 알아서 `:1`을 쓴다.
헤드리스로만 확인하려면 `dental-shot`(EGL 오프스크린).

> `dental-teleop`을 지금 실행하면 `error: Failed to spawn: dora` 가 난다. **정상** —
> `dental-setup`을 먼저 한 번 돌려야 한다(네트워크로 dora + enactic 노드 3종 설치).

---

## 씬 스테이징 (2026-09-03 추가)

목표 구도 = **의사(앉음) ─ 환자 ─ OpenArm(의사 마주봄)**, 환자를 사이에 두고 전달.
사양서 §9(오퍼레이터·환자·어시스트) / §10 hand-off 기준.

이전에는 "Blender 원본에서 해야 한다"고 봤지만, 실제로 확인해 보니

- 의자 부품(`Backrest`/`Headrest`/`Seat_Pan`/`Leg_Rest`)이 **각각 별도 OBJ**라
  MuJoCo XML만으로 **리클라인이 가능**했다 (baked 메시라도 힌지 둘레 회전은 됨).
- 의사는 `.blend`에도 **아마추어가 없어서**(순수 baked 메시) 리깅 대신
  **메시를 가랑이/무릎에서 절단**해 앉은 자세를 만들었다.
  절단면을 골반(z=0.90)에 두면 엉덩이 덩어리째 앞으로 돌아가 **넓적한 판때기**가 되고 절단면 캡이
  드러난다. 그래서 절단면은 가랑이(z=0.80)로 내리고 **회전축만 실제 고관절(z=0.92)에 따로 둔다** —
  회전축이 절단면 위에 있어도 되므로 회전하는 건 다리뿐이고 골반은 몸통에 남는다.

따라서 **Blender 원본 재작업 없이** 스테이징을 끝냈다. OBJ 재익스포트도 불필요.

### 무엇을 바꿨나 (전부 `room_v3_teleop.xml` / `main_v3_teleop.xml`)

| # | 항목 | 내용 |
|---|------|------|
| 1 | **의자 리클라인** | 등받이+헤드레스트를 하단 힌지 `(0, 0.230, 0.878)` 둘레로 **-44° 회전**. 수직대비 28°(업라이트) → **72°(앙와위)**. baked 메시라도 자식 body를 힌지에 두고 geom을 `-힌지`만큼 되밀면 힌지 회전이 된다 |
| 2 | **레그레스트 정렬** | 발쪽으로 7.9° 들려 있고 시트면보다 78mm 낮던 단차 제거 → 수평 + 시트면(z=0.90) 정렬 |
| 3 | **의사 앉힘** | 몸통 메시를 **가랑이 z=0.80 / 무릎 z=0.50**에서 3분할 후 강체변환. 회전축은 절단면이 아니라 **실제 고관절 z=0.92**. 허벅지는 수평이 아니라 **15° 하향**(하이스툴 자세). 고관절 월드 `(-0.926, 0.58, 0.636)`, 무릎 z=0.515, 발바닥 z=0.015 |
| 4 | **의사·스툴 배치** | 환자 반대편(x<0)으로 이동해 로봇(x=+0.79)과 **마주봄**. 스툴 좌면 z 0.588→**0.530** = 엉덩이 절단면(z=0.516)이 좌면에 묻혀 안 보이는 높이. 발끝~의자축 0.536 m > 베이스반경 0.45 |
| 5 | **환자 프리미티브** | 리클라인된 등받이 위 앙와위. **신장 1.76 m · 어깨폭 0.35 · 가슴두께 0.23**. 몸통/골반/머리는 **방향 있는 타원체**(구형 캡슐이면 앞뒤 두께가 좌우 폭과 같아져 통나무처럼 보임). 팔은 팔걸이 위. `patient_mouth` = 구강 기준점 |
| 6 | **카메라** | `cam_target`을 전달구역 `(0, 0.30, 1.05)`로 이동. `staging` 카메라 신규(3자 한 프레임) |

### 지켜진 제약

- **관절을 하나도 추가하지 않았다** → `nq=156` 불변 → **`home` 키프레임 그대로 유효**
  (`build_home_keyframe.py` 재실행 불필요)
- 환자 지오메트리는 전부 `contype="0" conaffinity="0"` **시각 전용** → 기존 물리·`episode_v3.py`에 영향 없음
- `arm_origin`(0.79 0.5 1.20) 미변경 → **텔레옵 좌표 기준 그대로**
- 원본(`main_v3_blender.xml` · `room_v3_blender.xml` · `openarm_bimanual_v3.xml` · `episode_v3.py`) 미변경
- 수정 전 스냅샷: `sim/.staging_backup/`

### 재현 / 확인

```bash
blender -b -P teleop/tools/build_seated_doctor.py   # 의사 3분할 메시 재생성 (이미 커밋돼 있으면 불필요)
python sim/scripts/render_staging.py                # 구도 PNG -> sim/output/staging.png
python sim/scripts/view_teleop.py                   # 뷰어 (카메라 목록에 staging 추가됨)
```

### 2026-09-04 개정 — 사양서 §9 전달구역 충족

"서로 물건을 주고받을 수 있는 거리"가 필수 요건이 되어 전면 재배치했다.

**먼저 잡은 계산 오류**: 로봇 도달반경을 손목(`ee_base_link`)까지만 재서 0.527 m 로 봤는데,
그리퍼 손끝 오프셋 0.172 m 를 빠뜨린 것이었다. 실제 **0.699 m**. 이 때문에 "구조적으로 불가능"
이라고 잘못 결론냈었다. (OpenArm 자체는 공식 openarm_v20 그대로 — 원본 대비 링크 pos 0건,
관절 range 0건 차이, 메시 scale 1)

**의자 실물 규격 보정** — Blender 에셋이 과대했고 이게 의사를 밀어내던 진짜 원인:

| 항목 | 전 | 후 | 실물 규격 |
|---|---|---|---|
| 베이스 지름 | 0.900 | **0.650** | 0.55~0.70 |
| 시트면 높이 | 0.900 | **0.780** | 0.35~0.80 |
| 전장 | 2.106 | **1.903** | 1.85~2.00 |

**전달구역 V (사양서 §9-2)** — 환자 실측으로 산출. `C_x+30~250mm` · `|y|≤120mm` ·
`Π_chest+50mm ≤ z` (= **가슴에 맞닿지 않고 50 mm 이상 띄움**). 월드 x[-0.12,+0.12]
y[0.21,0.46] z[1.17,1.37].

**배치 해** — V 안에서 양쪽이 80% 신전 이내로 도달하는 공통영역 **44점** 확보:

| | 값 |
|---|---|
| 로봇 베이스 | x 0.79 → **0.42** (`arm_origin`도 함께 이동 — 사이트와 두 팔이 같이 움직이므로 텔레옵 좌표 기준 정합성 유지) |
| 의사 고관절 | (**-0.50, 0.48**, 0.636), 상체 **20° 전방 경사**(고관절 둘레 회전, 메시 재절단 없이 body 하나로) |
| 전달지점 `handoff` 사이트 | (-0.105, 0.411, 1.253) — 의사 0.80 신전 / 로봇 0.76 신전 |

**환자를 인체 메시로 교체** — 캡슐/타원체로는 사람으로 안 보여서, 의사와 같은
`DocPart_torso.obj`(머리·양팔 포함 전신)를 0.96배(**신장 1.746 m**)로 줄여 가랑이·목에서 3분할,
앙와위 강체변환. 평행이동은 "등받이 파고듦 0 / 시트 파고듦 0" 연립해(오차 0.000 mm).
헤드레스트는 레일을 따라 -0.367 m 슬라이드해 뒤통수를 받친다(머리와 5.2 mm).

**관통 검사**: 의사 상체↔환자 머리 55.6 mm, 로봇↔등받이 316.7 mm, 로봇↔환자 155.0 mm.
의사 허벅지↔환자 몸통 1.6 mm 는 무릎이 의자 밑을 지나는 정상 자세(가려짐).

**물리 안정화** (스테이징과 무관한 기존 버그 수정):
`integrator=implicitfast` + 기구 freejoint `armature=1e-5` + 의사 geom 시각전용화.
30초 시뮬에서 기구 이동량 **45,973 mm → 21.7 mm**, `max|qacc|` 2.7e8 → 5.8e2, 발산 없음.

**미해결**: 사양서 §9-2의 상한 `z ≤ E_z-20mm`(환자 시선 아래)는 이 환자 에셋으로 만족 불가.
몸통 두께(0.305 m)가 머리와 거의 같아 "흉부+50mm ~ 시선-20mm" 구간이 -82 mm 로 음수가 된다.
헤드레스트로 머리를 더 높이거나 체형이 다른 환자 에셋이 필요하다.

### 남은 것

- 의사 오른팔(5DOF 체인)은 여전히 **아래로 늘어진 자세** — 전달 자세는 `episode_v3.py`가
  스크립트로 잡는 몫이라 스테이징에서는 건드리지 않았다.
- **기존 버그(스테이징과 무관)**: `main_v3_teleop.xml`·`main_v3_blender.xml` 모두 물리 스텝을 돌리면
  t≈0.08s에 `QACC` 폭주(`DOF 48` = `instr_elevator_cryer74`의 freejoint). 트레이 위 기구가 초기
  관통 상태로 보인다. `view_teleop.py`는 `mj_step`을 돌리지 않아 영향 없지만, 텔레옵으로
  실제 물리를 돌릴 때는 해당 기구의 초기 위치/충돌 지오메트리를 손봐야 한다.
