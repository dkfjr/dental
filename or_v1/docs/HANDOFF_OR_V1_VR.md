# Dental VLA — OR 씬 v1.0 + VR 텔레옵 인수인계

작성 2026-09-29 · 대상: `~/dental-vla` 를 이어받는 에이전트/사람
범위 밖(건드리지 말 것): `~/openarm_ws`, `~/leisaac_openarm`, `~/Downloads/chemistry01_*`, `~/Downloads/_usd_보관`

> 아래 내용은 전부 이 날짜에 **실제로 실행해 확인한 것**이다. 확인 못 한 것은 "미확인"으로 표시했다.

---

## 0. 한눈에 보기

| 항목 | 상태 |
|---|---|
| 씬 | Blender(`scenes/dental_or_v1.blend`)에서 편집 → MuJoCo MJCF 자동 생성 |
| 로봇 | **OpenArm v1.0, 검정**, 기구 카트를 정면으로 바라봄 (원래 v2) |
| 기구 카트 | 실제 발치 세트 사진(`docs/ref_instrument_tray.jpg`) 배치대로 파란 드레이프 + 기구 22종, 실제 크기, 잡을 수 있는 물체(free body) |
| VR | Quest 3 → **웹 브라우저(CloudXR)** → 브리지 → dora → MuJoCo. 실제 Quest로 조작·집기 성공(사용자 확인) |
| 녹화 | 텔레옵 중 자동 녹화, **두 시점**(3인칭 + 트레이 정면 확대) 1920×540 |
| 자동 동작 데모 | movej로 왼팔이 10ml 주사기를 집어 의사 쪽으로 전달 — 성공 |
| 알려진 한계 | 납작하게 누운 얇은 기구(탐침·큐렛·메스)는 집기 어려움. 22개 중 17개만 위에서 닿음 |

---

## 1. VR 텔레옵 실행 (가장 자주 쓰는 것)

### 1-0. 재부팅 전 / 후 (NVIDIA 드라이버) — 먼저 확인
2026-09-29 16:07 apt 가 드라이버 **595.91.07** 을 설치했지만 커널 모듈은 **590.48.01** 로 떠 있다(재부팅 안 함).
확인: `cat /proc/driver/nvidia/version` (커널) vs `ls /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.*` (라이브러리)

| | 재부팅 전 (지금: 커널 590 / 라이브러리 595) | **재부팅 후 (커널 595 / 라이브러리 595)** |
|---|---|---|
| `start_vr.sh` | 자동으로 `~/.local/nvidia-590/lib` 사용 | **자동으로 시스템 595 사용** — 할 일 없음 |
| CloudXR 수동 실행 | `LD_LIBRARY_PATH` 앞에 `$HOME/.local/nvidia-590/lib:` 필요 | **붙이면 안 됨**(590 라이브러리 + 595 커널 = 다시 불일치). §1-2 명령 그대로 |
| `MUJOCO_GL=egl` 스크립트(데모·재생) | 앞에 `LD_LIBRARY_PATH=~/.local/nvidia-590/lib` | 붙이지 말 것 |
| CloudXR 서버 | 떠 있음(590 라이브러리로 재시작해 둠) | **꺼짐** → `start_vr.sh` 가 자동으로 켬 |
| `nvidia-smi` | `Driver/library version mismatch` | 정상, `595.91.07` 표시 → 이걸로 확인 |

재부팅 후 점검 순서:
```bash
nvidia-smi --query-gpu=name,driver_version --format=csv   # 595.91.07 이 나와야 함
cd ~/dental-vla && tools/or_v1/start_vr.sh                 # "NVIDIA kernel ... != system libs" 메시지가 안 나와야 정상
```
- `nvidia-smi` 가 안 되면(595 커널 모듈이 빌드/로드 안 됨): `dkms status`, `lsmod | grep nvidia` 확인. 이건 시스템 문제라 sudo 필요.
- 재부팅 후 `~/.local/nvidia-590/` 폴더는 필요 없다(지워도 됨, 약 1 GB). `start_vr.sh` 는 커널 버전이 다를 때만 `~/.local/nvidia-<커널버전>/lib` 를 찾는다.

### 1-1. 한 줄로 (권장)
```bash
cd ~/dental-vla && tools/or_v1/start_vr.sh
```
이 스크립트가 순서대로 하는 일:
1. NVIDIA 드라이버 불일치 감지 → 있으면 `~/.local/nvidia-<커널버전>/lib` 사용 (§5-2)
2. **CloudXR(웹 VR 서버, 포트 48322)이 꺼져 있으면 자동으로 켬**
3. dora 데이터플로우 실행(백그라운드): 수신 → IK → 그리퍼 → MuJoCo 뷰어 → 녹화
4. 웹 브리지 실행(포그라운드): Quest 컨트롤러 → dora
5. Ctrl+C 한 번으로 dora·브리지 종료 (CloudXR 은 계속 켜둠)

옵션: `--neck-drop 0`(헤드셋을 목에 걸 때, neck 매핑에서만 의미), `--record <csv>`(기본으로 자동 기록됨)

### 1-2. 수동으로 하나씩 (디버깅용)
```bash
# (a) CloudXR 웹 VR 서버 — 한 번만. 떠 있으면 생략 (확인: ss -tln | grep 48322)
cd ~/vr_teleop && \
  PATH=$HOME/vr_teleop/cloudxr_service_venv/bin:/usr/local/cuda-12.8/bin:$PATH \
  LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64 \
  setsid nohup python -m isaacteleop.cloudxr --accept-eula --host-client > ~/dental-vla/out/cloudxr.log 2>&1 &
#   ※ 재부팅 전(드라이버 불일치)에만 LD_LIBRARY_PATH 앞에 $HOME/.local/nvidia-590/lib: 를 붙일 것.
#     재부팅 후(595)에는 붙이면 안 된다 (§1-0)

# (b) 시뮬레이션 + IK + 녹화 (터미널 1)
cd ~/dental-vla && export VIRTUAL_ENV=$PWD/.venv PATH=$PWD/.venv/bin:$PATH DISPLAY=:1
dora run config/dataflow-mujoco-or-v1.yaml

# (c) 웹 VR 브리지 (터미널 2)
cd ~/dental-vla && tools/or_v1/vr_web_bridge.sh --print --record out/sessions/manual.csv
```

### 1-3. Quest 쪽 (웹 사이트 띄우기)
1. Quest 3 착용 → **Meta Quest 브라우저** 실행
2. 주소: **`https://192.168.0.10:48322/client/#/sim`** (PC IP 는 `hostname -I` 첫 번째 값)
3. 인증서 경고 → "고급 → 계속" 으로 허용
4. **Connect / Start** → VR 세션 진입 (PC 에서 영상 스트림이 열려야 함)
5. 헤드셋·PC 가 같은 네트워크(192.168.0.x)여야 함

### 1-4. 조작 방법
| 동작 | 입력 |
|---|---|
| **시작(보정)** | MuJoCo 뷰어의 로봇처럼 **ㄴ자**(윗팔 내리고 팔꿈치 90°, 컨트롤러 앞으로)로 서서 **컨트롤러 옆 그립 버튼**(아무 손) 또는 왼쪽 Y. 누르기 전엔 로봇 정지 |
| 재보정 | 왼쪽 **Y** |
| 팔 이동 | 손 이동 × 0.73 (사람 팔 0.60 m → 로봇 0.437 m) |
| 손목 방향 | 컨트롤러가 가리키는 방향 = 그리퍼 방향 |
| 그리퍼 | **트리거** 당김 = 닫힘 (양손 동일, 비례, 끝까지 닫는 데 ≥0.5 s) |
| 한쪽 컨트롤러 미인식 | 보이는 손만 먼저 움직임. 나중에 보이면 그 손 그립으로 합류 |

브리지 터미널 메시지: `보정 완료 — 따라 움직입니다: left, right`, 미인식이면 `⚠ 오른쪽 컨트롤러가 인식되지 않습니다`.

### 1-5. 결과물 위치
- 영상: `out/videos/teleop_or_v1_<날짜_시각>.mp4` (1920×540, 왼쪽 3인칭 / 오른쪽 트레이 확대). **dora 종료 시 파일 완성**
- 컨트롤러 원본 기록: `out/sessions/vr_<날짜_시각>.csv` (47열: 가공 목표 + 원본 헤드/컨트롤러 포즈 + 버튼)
- dora 로그: `out/dora.log`, 세션별 노드 로그 `config/out/<세션ID>/log_*.txt`

---

## 2. 구조

```
Quest 3 (브라우저) ──WebRTC──▶ CloudXR runtime (isaacteleop, :48322/:49100)
                                     │ OpenXR (헤드·컨트롤러)
                                     ▼
          src/dental_vla/isaacteleop_bridge.py  (+ attention_map.py)
                                     │ UDP JSON 127.0.0.1:5006  (+ 5007 미러)
                                     ▼
 dora: udp-receiver ─▶ ik ─▶ gripper(gripper_remap.py) ─▶ mujoco-viewer ─▶ recorder(video_recorder.py)
       (enactic)       (enactic)  (우리)                     (enactic)        (우리)
```

- enactic 노드(`dora-openarm-quest-receiver`, `dora-openarm-ik`, `dora-openarm-mujoco`, `dora-openarm-quitter`)는 **수정하지 않음**. 우리 쪽에서 앞(브리지)·중간(gripper)·뒤(recorder) 노드로 보정.
- OpenArm v1 과 v2 는 관절·액추에이터·`arm_origin`·`*_ee_control_point` 이름이 같아 enactic 노드가 그대로 동작.
- 원본 enactic 방식(전용 APK + 헤드셋을 목에 걸기)과 다름: 이 PC 는 **웹(CloudXR)** 방식. APK 는 이 PC 에 없음.

### 파일
| 경로 | 역할 |
|---|---|
| `scenes/dental_or_v1.blend` | 편집 원본 (Downloads 의 `dental_scene_or_teleop.blend1` 복사본에서 시작) |
| `third_party/openarm_mujoco/v1/` | 공식 OpenArm v1 MJCF (enactic/openarm_mujoco `08e2fb3`) |
| `third_party/openarm_mujoco/v1/dental_or/dental_or_scene_v1.0.xml` | **생성물**, 직접 수정 금지 |
| `third_party/openarm_mujoco/v1/dental_or/openarm_v1_black.xml` | 생성물: 검정 로봇 + 배치 + 손가락 패드 |
| `config/dataflow-mujoco-or-v1.yaml` | 이 씬용 dora 데이터플로우 (`dataflow-mujoco.yaml` 은 원래 v2 씬용, 손대지 않음) |
| `src/dental_vla/isaacteleop_bridge.py` | CloudXR→UDP 브리지 (원본: `~/projects/Dental_VLA/teleop/`, 여기서 수정) |
| `src/dental_vla/attention_map.py` | 보정 기반 매핑 + 가상 바닥 |
| `src/dental_vla/gripper_remap.py` | v1 그리퍼 방향·비례·속도 제한 |
| `src/dental_vla/video_recorder.py` | 두 시점 mp4 녹화 노드 |
| `tools/or_v1/*.sh, *.py` | 빌드·실행·검증 도구 (§4) |

---

## 3. 씬 수정 흐름 (Blender → MuJoCo)

```bash
cd ~/dental-vla
tools/or_v1/export_all.sh             # .blend 편집 후: 환경 메시 내보내기 + MJCF 생성
tools/or_v1/export_all.sh --rebuild   # 로봇 교체 + 트레이 재생성까지 (트레이 수작업 편집은 덮어써짐)
```
- Blender 규칙: 이름이 `INSTR_` 로 시작하는 오브젝트 → MuJoCo **free body**(잡을 수 있음). 커스텀 속성 `mass`, `colliders`(부품별 충돌 상자, 트레이 빌더가 기록).
- `OPENARM_V1_REFERENCE` 컬렉션은 참조용(내보내지 않음). 로봇은 공식 MJCF 사용.
- 로봇 배치(`tools/or_v1/build_scene.py`): `BASE_YAW_DEG=90`(카트 방향), `BASE_POS=(0.84, 0.47, 0.522)` → `arm_origin` 월드 z=1.22. 도달성 검사로 결정(`reach_check.py`: 위에서 잡기 17/22, 양팔).
- 물리: timestep 0.001, implicitfast, elliptic cone, impratio 10. 기구·테이블 접촉 solref 0.002(관통 방지).
- 충돌 비트: 가구/링크 contype1 conaffinity3, 기구 2/2, **손가락 원래 형상 1/1(기구와 안 닿음)**, **손가락 패드 2/2(기구는 패드로만 잡힘)**.

---

## 4. 검증·분석 도구

| 명령 | 용도 |
|---|---|
| `LD_LIBRARY_PATH=~/.local/nvidia-590/lib MUJOCO_GL=egl .venv/bin/python tools/or_v1/demo_handover.py` | **movej 자동 데모**(왼팔 주사기 전달) → `out/videos/demo_movej_left_handover.mp4`. 마지막 줄 PASS/FAIL. ※ `LD_LIBRARY_PATH=...nvidia-590...` 는 재부팅 전에만, 재부팅 후엔 빼고 실행 |
| `.venv/bin/python tools/or_v1/analyze_session.py out/sessions/<csv>` | 세션: 추적 끊김, 원본 점프, 보정 시각, IK 튐 |
| `... tools/or_v1/replay_session.py <csv> --floor --two-view --video out/x.mp4` | 기록된 조작을 물리 재생(튕김 탐지, 두 시점 영상). ※ 원본과 타이밍이 ~1 s 어긋날 수 있음 |
| `.venv/bin/python tools/or_v1/tracking_monitor.py` | 실행 중 손→로봇 목표 실시간 확인(브리지 5007 미러) |
| `tools/or_v1/vr_probe.sh` | 헤드셋/컨트롤러/손 추적이 PC 에 들어오는지 |
| `.venv/bin/python tools/or_v1/reach_check.py` | 기구별 도달성 |
| `.venv/bin/python tools/or_v1/grasp_test.py <instr> --side left ...` | 단일 파지 시험 |
| `.venv/bin/python tools/or_v1/teleop_stress.py` | 합성 조작 궤적으로 IK 튐 검사 |
| `PYTHONPATH=src .venv/bin/python -m dental_vla.mock_quest_sender` | 헤드셋 없이 가짜 컨트롤러(브리지 대신 5006 으로 직접) |

영상 편집(앞/뒤 자르기)은 OpenCV 로 재인코딩(ffmpeg 없음, H.264 인코더 없음 → mp4v).

---

## 5. 문제 해결 (실제로 겪은 것)

### 5-1. 씬이 안 뜸 + `error: Failed to spawn: dora`
원인: `uv run` 이 `.venv` 를 **통째로 재생성**해 dora·enactic 노드가 사라짐(2026-09-29 16:33, uv 업데이트 후). 복구:
```bash
tools/or_v1/install_teleop.sh      # 검증된 커밋으로 재설치
```
**`uv run` 을 쓰지 말 것.** `start_vr.sh` 는 `.venv/bin/dora` 를 직접 쓴다.

### 5-2. `GLX: Failed to create context` / 웹 Connect 계속 실패 / `NV_ENC_ERR_UNSUPPORTED_DEVICE`
원인: 16:07 apt 가 NVIDIA 드라이버 595 설치, 커널 모듈은 590.48 그대로(재부팅 안 함) → 새 프로세스의 OpenGL·CUDA·NVENC 실패.
- 확인: `nvidia-smi` → `Driver/library version mismatch`, `cat /proc/driver/nvidia/version`
- 재부팅 없이 해결(root 불필요): 커널과 같은 버전 유저 라이브러리를 개인 폴더에 풀어 사용
  ```bash
  tools/or_v1/nvidia_userspace.sh    # ~/.local/nvidia-<ver>/lib 생성 (NVIDIA .run --extract-only)
  ```
  `start_vr.sh` 가 자동 적용. **CloudXR 도 이 라이브러리로 재시작해야 웹 연결(NVENC)이 된다** (§1-2 ※).
- 재부팅하면 버전이 맞아 자동으로 불필요해짐. 재발 방지 권장: `sudo apt-mark hold nvidia-driver-595` (미적용)

### 5-3. 로봇이 안 움직임
- 브리지가 안 떠 있음 → `pgrep -af isaacteleop_bridge`
- 보정 안 함 → 그립(또는 Y) 눌러야 시작. CSV `calibrated` 열로 확인
- 컨트롤러 미인식 → `vr_probe.sh`: head=OK 인데 L/R 이 `--` 면 컨트롤러 절전/배터리. 브라우저 VR 세션이 몰입 모드인지
- 헤드셋을 목에 걸면 착용 센서로 세션이 멈출 수 있음(센서 테이프)

### 5-4. 기구가 튕겨 나감 (수정 완료, 재생 검증)
원인과 대응:
1. 트리거를 0.05 s 에 당김 → 손가락이 때리듯 닫힘 → `gripper_remap.py` 속도 제한(닫힘 ≥0.5 s)
2. 손이 낮아 목표가 드레이프 아래 → 팔이 트레이를 파고듦(재생에서 최대 12만 N) → `attention_map.apply_floor` 가상 바닥(손끝 ≥ 드레이프+3 mm)
3. v1 손가락 안쪽 면이 비스듬해 둥근 기구를 쐐기처럼 튕김 → **평평한 고마찰 패드**(friction 2.0, solref 0.008)만 기구와 접촉
- 실제 조작 기록 4개 재생: 튕김 15/11/15/4 회 → 4/7/1/2 회, 속도 최대 5.7 → 1.6 m/s

### 5-5. 팔이 "제멋대로"
- 차렷(팔 곧게 폄) 기준 매핑은 특이점 → 폐기, **ㄴ자(home) 기준**으로 변경
- 보정 전엔 로봇 정지, 시작 순간 목표 튐 제거
- 합성 궤적 테스트: 튐 0 회. 실제 원인이 남아 있으면 `analyze_session.py` 로 CSV 확인

---

## 6. 결정 기록 (왜 이렇게 했나)
- **로봇 방향**: 원래 환자(−x)를 봤으나 카트가 로봇 오른쪽 → 사용자 요청으로 카트(+y)를 보게 90° 회전, 몸체 (0.84,0.47), 10 cm 낮춤. 도달 13/22(오른팔만) → 17/22(양팔)
- **브리지 축 부호**: 새 씬은 `arm_origin` 이 몸체와 함께 회전(공식 규약) → 예전 v3 씬용 `sx=-1,sz=+1` 이 180° 뒤집혀 있었음 → attention 매핑이 직접 arm_origin 목표를 만들어 인코딩
- **기준점**: enactic 원본은 목에 건 헤드셋을 고정 기준으로 씀. 머리 실시간 포즈를 기준으로 하면 고개만 돌려도 팔이 움직여서 고정 방식으로
- **IK**: enactic IK 그대로, 인자는 `--max-iters 30 --dt 0.1 --damping 0.05 --posture-cost 0.0 --lm-damping 0.005 --no-joint-braking`, `--xml` 필수(없으면 데모 셀 기준으로 풂)
- **양팔 IK**: 양쪽 목표가 모두 있어야 계산 → 안 보이는 팔에는 home/마지막 목표를 계속 보냄

## 7. 미확인 / 남은 일
- 얇은 기구 파지: v1 손끝이 TCP 보다 15.4 mm 앞 → 누운 3~8 mm 기구를 끝으로만 조임. 제안: 기구 거치대(1~2 cm 띄움) — **사용자 결정 대기**
- 사진 왼쪽 그룹(거즈·블리스터·1 ml 주사기·컵·니들) 도달 불가 — 배치 조정 여부 결정 대기
- 환자·의사는 시각 전용(충돌 없음)
- 손목 꺾임 체감은 ㄴ자 매핑 이후 실기 재확인 필요
- 실시간 두 시점 녹화는 mock 으로만 확인(실제 세션 영상은 다음 조작부터)
- 시스템은 여전히 reboot-required 상태
