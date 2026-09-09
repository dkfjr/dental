# dental

발치 시술 보조 로봇의 MuJoCo 시뮬레이션과 Meta Quest 3 VR 텔레오퍼레이션.

의사(앉은 자세) — 환자(누운 자세) — OpenArm 양팔 로봇이 마주 보고,
서로 기구를 주고받을 수 있는 거리에 배치된 씬입니다.

## 구성

```
sim/
├── main_v3_teleop.xml              메인 씬 (아래 둘을 include)
├── room_v3_teleop.xml              의자·의사·환자·카트·기구 19종
├── openarm_bimanual_v3_teleop.xml  OpenArm 양팔 + 사각 기둥 받침
├── assets/                         메시 (visual, collision, doctor, doctor_rig)
└── scripts/                        뷰어·렌더·도달성 검사

teleop/
├── dental_vla_teleop/
│   ├── isaacteleop_bridge.py       IsaacTeleop -> dora UDP 브리지
│   ├── quest_adb_bridge.py
│   └── mock_quest_sender.py
├── config/dataflow-mujoco.yaml     dora 데이터플로우
└── tools/                          Blender 에셋 생성 스크립트

bin/                                단축 명령어
```

## 실행

`bin/` 을 PATH 에 넣거나 `~/.local/bin/` 에 복사한 뒤:

```bash
export DENTAL_VLA=/path/to/dental      # 이 저장소 경로

dental-view          # 씬 GUI 뷰어 (staging 카메라)
dental-view wide     # 카메라 지정
dental-view --step   # 물리 ON
dental-teleop        # 텔레옵 씬
dental-vr            # CloudXR + dora + Quest 3 브리지 (VR 텔레옵)
dental-mock          # Quest 없이 가짜 입력으로 테스트
dental-shot          # 오프스크린 렌더
```

마우스: 좌드래그 회전 / 우드래그 이동 / 휠 줌 · `[` `]` 카메라 전환 · `Esc` 자유 카메라

## VR 텔레옵

Quest 3 를 CloudXR 로 무선 연결합니다. `dental-vr` 이 CloudXR 환경을 읽고,
dora 데이터플로우를 띄운 뒤 IsaacTeleop 브리지를 실행합니다.

| 입력 | 동작 |
|---|---|
| 컨트롤러 이동 | 팔 위치 |
| 트리거 | 그리퍼 열기·닫기 |
| 그립 + 조이스틱 | 손목 회전 |
| A / X | 자세 초기화 |

브리지 옵션:

| 옵션 | 기본 | 설명 |
|---|---|---|
| `--move-mode` | `off` | `off` 컨트롤러만 / `only` 조이스틱만 / `add` 둘 다 |
| `--wrist-mode` | `joystick` | `controller` 로 두면 컨트롤러 자세를 그대로 추종 |
| `--wrist-rate` | 1.6 | 손목 회전 속도 (rad/s) |
| `--move-rate` | 0.25 | 조이스틱 이동 속도 (m/s) |
| `--sx --sy --sz` | -1 1 1 | 좌표축 부호 |

## 씬 사양

| | |
|---|---|
| nq / nu | 156 / 16 |
| body / geom | 61 / 185 |
| 키프레임 | `home` (텔레옵 시작 자세), `handoff` (전달 자세) |
| 로봇 | OpenArm bimanual — 치수·관절 이름 원본 유지 |
| 기구 | 19종, 회의록 §6.3 순서대로 트레이 한 줄 배치 |
| 통합기 | `implicitfast` |

`home` 자세는 옆에서 봤을 때 **ㄴ 모양** 입니다 — 상완이 수직(옆면 투영각 0.0~0.7°),
전완이 수평. 로봇 어깨 구조상 3차원 완전 수직은 불가능해(최소 이탈 21.5°),
옆면 투영 기준으로 맞추고 좌우 벌어짐으로 기둥을 피합니다.

## 검증

```
10초 물리   NaN 없음, max|qacc| 1.45e+03 (발산 없음)
팔 관통     0.2mm
기구 도달   19종 전부, 신전율 0.66~0.78
의자 간섭   1.2%
```

## 제외한 것

MuJoCo 실험실(연구실) 환경 씬은 이 저장소에 없습니다. 이 저장소는
의사-환자-OpenArm 연동 씬과 VR 텔레옵만 담습니다.
