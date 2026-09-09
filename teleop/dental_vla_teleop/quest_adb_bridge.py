#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""Meta Quest 3 (ADB/oculus_reader APK) → dora-openarm-vr UDP 브리지.

왜 필요한가
-----------
`enactic/dora-openarm-vr` 의 `dora-openarm-quest-receiver` 는 Quest 앱이 **UDP 로 JSON**
패킷을 쏘는 것을 전제한다. 그런데 이 PC 에 이미 있는 Quest 앱은
`rail-berkeley/oculus_reader` 계열(LabUtopia `meta_quest_teleop`)로, **ADB 로 컨트롤러
포즈·버튼을 스트리밍**한다. 전송 방식만 다를 뿐 담고 있는 데이터는 동등하다.

이 브리지가 ADB 스트림을 읽어 dora 가 기대하는 UDP JSON 으로 변환해 보낸다.
덕분에 enactic 쪽 Unity 앱을 새로 빌드하지 않고 **가지고 있는 APK 그대로** 쓸 수 있고,
UDP 가 127.0.0.1 로 흐르므로 **방화벽 설정도 필요 없다**.

좌표계
------
oculus_reader 는 OpenXR(X 오른쪽, Y 위, Z 뒤) 4x4 행렬을 준다.
dora 수신기는 Unity 왼손 좌표(X 오른쪽, Y 위, Z 앞)를 기대하므로 z 부호를 뒤집는다.
z 축만 반전하는 손잡이 변환에서 쿼터니언은 (qx,qy,qz,qw) -> (-qx,-qy,qz,qw).
헤드셋을 쓰고 실제로 움직여 보면서 축이 어긋나면 --flip / --swap 으로 맞출 수 있다.

사용법
------
    # 1) Quest 를 USB 로 연결(또는 adb connect <IP>:5555), 개발자 모드 ON
    # 2) 파이프라인 실행:  dental-teleop
    # 3) 다른 터미널에서:
    python -m dental_vla_teleop.quest_adb_bridge
    python -m dental_vla_teleop.quest_adb_bridge --flip-z 0   # z 반전 끄기
    python -m dental_vla_teleop.quest_adb_bridge --print      # 값 확인용
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time

import numpy as np

# LabUtopia 의 meta_quest_teleop 은 site-packages 에 없으므로 경로를 직접 잡아준다
_CANDIDATES = [
    os.path.expanduser("~/LabUtopia/meta_quest_teleop"),
    os.path.expanduser("~/meta_quest_teleop"),
]
for _p in _CANDIDATES:
    if os.path.isdir(os.path.join(_p, "meta_quest_teleop")) and _p not in sys.path:
        sys.path.insert(0, _p)

try:
    from meta_quest_teleop.reader import MetaQuestReader  # type: ignore
except Exception as exc:  # pragma: no cover - 환경 의존
    print(f"[bridge] meta_quest_teleop 임포트 실패: {exc}", file=sys.stderr)
    print(f"[bridge] 찾아본 경로: {_CANDIDATES}", file=sys.stderr)
    raise SystemExit(1)


def mat_to_pose(mat: np.ndarray, flip_z: bool) -> dict:
    """4x4 (OpenXR) -> dora 가 기대하는 {x,y,z,qx,qy,qz,qw} (Unity 왼손)."""
    from scipy.spatial.transform import Rotation

    p = mat[:3, 3].astype(float)
    q = Rotation.from_matrix(mat[:3, :3]).as_quat()  # (x, y, z, w)
    if flip_z:
        p = np.array([p[0], p[1], -p[2]])
        q = np.array([-q[0], -q[1], q[2], q[3]])
    return {"x": float(p[0]), "y": float(p[1]), "z": float(p[2]),
            "qx": float(q[0]), "qy": float(q[1]), "qz": float(q[2]), "qw": float(q[3])}


IDENT = {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="dora 수신기 주소")
    ap.add_argument("--port", type=int, default=5006)
    ap.add_argument("--rate", type=float, default=60.0, help="송신 Hz")
    ap.add_argument("--flip-z", type=int, default=1, help="1=OpenXR z 반전(기본), 0=그대로")
    ap.add_argument("--ip", default=None, help="adb over Wi-Fi 로 붙을 Quest IP")
    ap.add_argument("--print", dest="show", action="store_true", help="송신값 출력")
    args = ap.parse_args()

    print("[bridge] Quest 연결 중... (USB 연결 + 개발자 모드 + 헤드셋에서 USB 디버깅 허용)")
    reader = MetaQuestReader(ip_address=args.ip) if args.ip else MetaQuestReader()
    reader.run()
    time.sleep(1.5)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)
    period = 1.0 / max(args.rate, 1.0)
    t0 = time.time()
    n_sent = n_valid = 0
    last_report = t0
    print(f"[bridge] {dest[0]}:{dest[1]} 로 {args.rate:.0f} Hz 송신 시작. Ctrl+C 로 종료.")

    try:
        while True:
            loop_start = time.time()
            t = loop_start - t0
            lm = reader.get_hand_controller_transform_openxr("left")
            rm = reader.get_hand_controller_transform_openxr("right")
            ok = lm is not None and rm is not None

            lc = mat_to_pose(lm, bool(args.flip_z)) if lm is not None else dict(IDENT)
            rc = mat_to_pose(rm, bool(args.flip_z)) if rm is not None else dict(IDENT)
            # 기준 프레임: 오른손 첫 포즈를 원점으로 쓰지 않고, 헤드셋 원점(항등)을 그대로 준다.
            rf = dict(IDENT)

            def _f(fn, *a):
                try:
                    v = fn(*a)
                    return float(v) if v is not None else 0.0
                except Exception:
                    return 0.0

            def _b(name):
                try:
                    return bool(reader.get_button_state(name))
                except Exception:
                    return False

            ljs = reader.get_joystick_value("left") or (0.0, 0.0)
            rjs = reader.get_joystick_value("right") or (0.0, 0.0)

            pkt = {
                "t": t,
                "lc": lc, "rc": rc, "rf": rf,
                "lt": _f(reader.get_trigger_value, "left"),
                "rt": _f(reader.get_trigger_value, "right"),
                "lg": _f(reader.get_grip_value, "left"),
                "rg": _f(reader.get_grip_value, "right"),
                "lsx": float(ljs[0]), "lsy": float(ljs[1]),
                "rsx": float(rjs[0]), "rsy": float(rjs[1]),
                "a": _b("A"), "b": _b("B"), "x": _b("X"), "y": _b("Y"),
                "v": 0 if ok else 2, "vl": 0 if lm is not None else 2,
                "vr": 0 if rm is not None else 2,
            }
            sock.sendto(json.dumps(pkt).encode("utf-8"), dest)
            n_sent += 1
            n_valid += int(ok)

            if args.show and n_sent % 30 == 0:
                print(f"  R{rc['x']:+.3f},{rc['y']:+.3f},{rc['z']:+.3f} "
                      f"trig {pkt['rt']:.2f}  L{lc['x']:+.3f},{lc['y']:+.3f},{lc['z']:+.3f} "
                      f"trig {pkt['lt']:.2f}  valid={ok}")
            if loop_start - last_report > 5.0:
                print(f"[bridge] {n_sent} 패킷 송신, 포즈 유효 {100.0*n_valid/max(n_sent,1):.0f}%")
                last_report = loop_start
                if n_valid == 0:
                    print("[bridge] 포즈가 계속 무효입니다 — 헤드셋을 착용하고 컨트롤러를 "
                          "트래킹 범위 안에서 움직여 보세요.")

            time.sleep(max(0.0, period - (time.time() - loop_start)))
    except KeyboardInterrupt:
        print("\n[bridge] 종료")
    finally:
        try:
            reader.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
