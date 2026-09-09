#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""IsaacTeleop(CloudXR, 무선) → dora-openarm-vr UDP 브리지.

왜 이 방식인가
--------------
이 PC 에는 이미 NVIDIA IsaacTeleop + CloudXR 서비스가 떠 있고(포트 48322),
Quest 3 는 브라우저로 ``https://<PC-IP>:48322/client/#/sim`` 에 접속해 **무선**으로
연결된다. Isaac Sim 연결에 쓰던 바로 그 경로다.

반면 `enactic/dora-openarm-vr` 의 수신 노드는 Quest 앱이 **UDP JSON** 을 쏘는 것을
전제한다. 그래서 이 브리지가 IsaacTeleop 의 DeviceIO 세션에서 컨트롤러 포즈·트리거를
읽어 dora 가 기대하는 UDP JSON 으로 바꿔 보낸다.

  Quest 3 ──무선──▶ CloudXR/IsaacTeleop ──DeviceIO──▶ [이 브리지] ──UDP:5006──▶ dora
                                                                              │
                                                              udp-receiver ─▶ ik ─▶ mujoco

장점: Quest 앱을 새로 빌드/사이드로드할 필요가 없고, UDP 가 127.0.0.1 로만 흐르므로
방화벽을 열지 않아도 된다.

실행
----
IsaacTeleop 은 자체 venv 에 설치돼 있으므로 **그 파이썬으로** 실행해야 한다:

    "$CXR_PY"          # IsaacTeleop 이 설치된 python
        teleop/dental_vla_teleop/isaacteleop_bridge.py --print

좌표계
------
IsaacTeleop 은 OpenXR 규약(X 오른쪽, Y 위, Z 뒤)으로 포즈를 준다.
dora 수신기는 Unity 왼손 좌표(X 오른쪽, Y 위, Z 앞)를 기대하므로 z 부호를 뒤집는다.
z 축만 반전하는 손잡이 변환에서 쿼터니언은 (qx,qy,qz,qw) -> (-qx,-qy,qz,qw).
실제로 움직여 보고 축이 어긋나면 --flip-z 0 으로 끄고 비교할 수 있다.
"""
from __future__ import annotations

import argparse
import json
import signal
import socket
import sys
import time

import numpy as np

import isaacteleop.deviceio as deviceio
import isaacteleop.oxr as oxr

IDENT = {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}


def parse_rotfix(spec: str):
    """'x90,z-90' 같은 문자열을 회전행렬로. 그리퍼가 엉뚱하게 꺾일 때 맞추는 용도."""
    import numpy as np
    from scipy.spatial.transform import Rotation
    R = np.eye(3)
    if not spec:
        return R
    for tok in spec.split(","):
        tok = tok.strip()
        if not tok:
            continue
        ax, deg = tok[0].lower(), float(tok[1:])
        R = R @ Rotation.from_euler(ax, deg, degrees=True).as_matrix()
    return R


def to_unity(position, orientation, S, lift=0.0, Rfix=None, no_ori=False,
             quat_override=False, shift=None) -> dict:
    """OpenXR pose -> dora 수신기가 기대하는 pose dict.

    S = diag(sx, sy, sz) 부호 행렬. 위치는 S·p, 회전은 S·R·S 로 옮긴다
    (det(S·R·S) = det(R) = 1 이므로 항상 올바른 회전이 된다).

    수신기 내부 사슬은 다음과 같다 (quest_receiver.py 실측):
        p_rh   = [x, y, -z]                       # LH -> RH
        p_rel  = R_ref^-1 (p_ctrl - p_ref)        # 기준 프레임 기준 변위
        p_out  = R_FRAME p_rel + [-0.085, 0, -0.14]
        R_FRAME = [[0,0,-1], [-1,0,0], [0,1,0]]
    따라서 로봇 프레임에서
        X <-  (브리지 z)      Y <- -(브리지 x)      Z <-  (브리지 y)
    가 되므로, 앞뒤(X)·좌우(Y)를 뒤집으려면 브리지의 z, x 부호를 바꾸면 된다.
    """
    import numpy as np
    from scipy.spatial.transform import Rotation

    p = np.array([float(position.x), float(position.y), float(position.z)])
    if quat_override:                     # orientation 이 scipy Rotation 객체
        q = np.asarray(orientation.as_quat(), dtype=float)
    else:
        q = np.array([float(orientation.x), float(orientation.y),
                      float(orientation.z), float(orientation.w)])
    Sm = np.diag(S).astype(float)
    p2 = Sm @ p
    if no_ori:
        q2 = np.array([0.0, 0.0, 0.0, 1.0])      # 방향 무시 = 손목이 컨트롤러를 따라 꺾이지 않음
    else:
        R = Rotation.from_quat(q).as_matrix()
        if Rfix is not None:
            R = R @ Rfix                          # 컨트롤러 -> 그리퍼 정렬 보정
        R2 = Sm @ R @ Sm
        q2 = Rotation.from_matrix(R2).as_quat()
    # lift: 로봇 프레임 Z 는 브리지 y 에서 오므로, y 를 올리면 작업영역이 위로 간다
    p2[1] += lift
    if shift is not None:
        p2 = p2 + np.asarray(shift, dtype=float)   # 조이스틱 누적 이동
    return {"x": float(p2[0]), "y": float(p2[1]), "z": float(p2[2]),
            "qx": float(q2[0]), "qy": float(q2[1]), "qz": float(q2[2]), "qw": float(q2[3])}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="dora 수신기 주소")
    ap.add_argument("--port", type=int, default=5006)
    ap.add_argument("--rate", type=float, default=60.0, help="송신 Hz")
    ap.add_argument("--sx", type=int, default=-1, help="브리지 x 부호 (로봇 좌우)")
    ap.add_argument("--sy", type=int, default=1, help="브리지 y 부호 (로봇 상하)")
    ap.add_argument("--sz", type=int, default=1, help="브리지 z 부호 (로봇 앞뒤)")
    ap.add_argument("--lift", type=float, default=0.0,
                    help="로봇 작업영역을 위로 올림(m). 밑에서 올라오며 의자에 걸릴 때 사용")
    ap.add_argument("--pose", choices=("grip", "aim"), default="grip",
                    help="컨트롤러의 어느 포즈를 쓸지")
    ap.add_argument("--rot-fix", default="",
                    help="컨트롤러->그리퍼 회전 보정, 예: x90 / z-90 / x90,z90")
    ap.add_argument("--no-ori", action="store_true",
                    help="방향을 무시하고 위치만 추종")
    ap.add_argument("--wrist-mode", choices=("joystick", "controller"), default="joystick",
                    help="joystick(기본): 그립버튼+조이스틱으로만 손목을 돌린다. "
                         "controller: 컨트롤러 자세를 그대로 따라간다(제멋대로 꺾일 수 있음)")
    ap.add_argument("--wrist-rate", type=float, default=1.6,
                    help="조이스틱 최대 기울임에서의 손목 회전 속도 (rad/s)")
    ap.add_argument("--wrist-deadzone", type=float, default=0.15,
                    help="조이스틱 데드존")
    ap.add_argument("--move-rate", type=float, default=0.25,
                    help="조이스틱만 기울였을 때 팔이 움직이는 속도 (m/s)")
    ap.add_argument("--move-mode", choices=("off", "only", "add"), default="off",
                    help="조이스틱으로 팔을 옮기는 방식. "
                         "off(기본): 조이스틱 이동 없음, 팔은 컨트롤러만 따라간다. "
                         "only: 컨트롤러 위치를 고정하고 조이스틱으로만 팔을 옮긴다. "
                         "add: 컨트롤러 추종 + 조이스틱 오프셋 (둘 다 동시에 먹는다)")
    ap.add_argument("--stick-y", choices=("forward", "up"), default="forward",
                    help="조이스틱 상하가 앞뒤(forward)인지 위아래(up)인지")
    ap.add_argument("--record", default="",
                    help="세션을 CSV 로 기록 (나중에 분석용)")
    ap.add_argument("--print", dest="show", action="store_true", help="값 출력")
    args = ap.parse_args()

    S = (args.sx, args.sy, args.sz)
    Rfix = parse_rotfix(args.rot_fix)
    # 손목: 그립 버튼을 누른 동안에만 조이스틱으로 돌린다.
    # 이렇게 하면 컨트롤러를 아무렇게나 쥐어도 그리퍼가 멋대로 꺾이지 않는다.
    #   트리거      -> 그리퍼 열고 닫기 (수신기가 직접 처리)
    #   그립 + 조이스틱 X -> 손목 비틀기(roll)
    #   그립 + 조이스틱 Y -> 손목 숙임(pitch)
    #   A(오른손)/X(왼손) -> 손목 자세 초기화
    from scipy.spatial.transform import Rotation as _Rot
    wrist = {"left": _Rot.identity(), "right": _Rot.identity()}
    # 조이스틱만 기울이면 팔이 움직인다(브리지 좌표 누적 오프셋).
    # 수신기 사슬상  로봇X <- 브리지z,  로봇Y <- -브리지x,  로봇Z <- 브리지y  이므로
    #   앞으로(로봇 -X) = 브리지 z 감소,  오른쪽(로봇 +Y) = 브리지 x 감소,  위 = 브리지 y 증가
    move = {"left": np.zeros(3), "right": np.zeros(3)}
    # move-mode=only 일 때 팔을 붙들어 둘 컨트롤러 위치 (A/X 로 현재 위치에 다시 고정)
    hold = {"left": None, "right": None}

    class _P:
        __slots__ = ("x", "y", "z")

        def __init__(self, x, y, z):
            self.x, self.y, self.z = x, y, z

    if args.wrist_mode == "joystick":
        _m = {"off": "조이스틱 이동 없음 (팔 = 컨트롤러)",
              "only": "조이스틱으로만 팔 이동 (컨트롤러 위치 고정)",
              "add": "컨트롤러 + 조이스틱 동시"}[args.move_mode]
        print(f"[bridge] 이동: {_m} / 그립버튼 + 조이스틱 = 손목 / "
              "트리거 = 그리퍼 / A·X = 초기화")
    rec = open(args.record, "w", encoding="utf-8") if args.record else None
    if rec:
        rec.write("t,rx,ry,rz,rqx,rqy,rqz,rqw,rt,rg,lx,ly,lz,lqx,lqy,lqz,lqw,lt,lg,vl,vr\n")
        print(f"[bridge] 세션 기록: {args.record}")
    print(f"[bridge] 축 부호 sx={S[0]} sy={S[1]} sz={S[2]}  lift={args.lift:+.2f} m")

    head_tracker = deviceio.HeadTracker()
    controller_tracker = deviceio.ControllerTracker()
    trackers = [head_tracker, controller_tracker]
    extensions = deviceio.DeviceIOSession.get_required_extensions(trackers)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)
    period = 1.0 / max(args.rate, 1.0)

    print("[bridge] IsaacTeleop 세션 여는 중... "
          "(Quest 브라우저에서 https://<PC-IP>:48322/client/#/sim 접속 상태여야 함)")

    with (
        oxr.OpenXRSession("DentalVLA-DoraBridge", extensions) as oxr_session,
        deviceio.DeviceIOSession.run(trackers, oxr_session.get_handles()) as session,
    ):
        signal.signal(signal.SIGINT, signal.default_int_handler)
        print(f"[bridge] {dest[0]}:{dest[1]} 로 {args.rate:.0f} Hz 송신 시작. Ctrl+C 종료.")
        t0 = time.time()
        n_sent = n_valid = 0
        last_report = t0
        try:
            while True:
                loop_start = time.time()
                session.update()
                t = loop_start - t0

                def read(getter):
                    snap = getter(session)
                    if snap is None:
                        return None, None
                    # 트래커는 ControllerSnapshotTrackedT 를 주고 실제 스냅샷은 .data 안에 있다
                    snap = getattr(snap, "data", snap)
                    if snap is None:
                        return None, None
                    cp = snap.aim_pose if args.pose == "aim" else snap.grip_pose
                    return cp, snap.inputs

                lcp, lin = read(controller_tracker.get_left_controller)
                rcp, rin = read(controller_tracker.get_right_controller)
                l_ok = lcp is not None and lcp.is_valid
                r_ok = rcp is not None and rcp.is_valid

                if args.wrist_mode == "joystick":
                    dt = period
                    dz = args.wrist_deadzone
                    def _ax(v):
                        v = float(v)
                        return 0.0 if abs(v) < dz else (v - dz * (1 if v > 0 else -1)) / (1 - dz)
                    for side, inp in (("left", lin), ("right", rin)):
                        if inp is None:
                            continue
                        jx = _ax(getattr(inp, "thumbstick_x", 0.0))
                        jy = _ax(getattr(inp, "thumbstick_y", 0.0))
                        if float(getattr(inp, "squeeze_value", 0.0)) > 0.5:
                            # 그립 누름 -> 손목 회전
                            if jx or jy:
                                wrist[side] = (_Rot.from_euler(
                                    "zy", [jx * args.wrist_rate * dt,
                                           jy * args.wrist_rate * dt]) * wrist[side])
                        elif (jx or jy) and args.move_mode != "off":
                            # 그립 안 누름 -> 팔 이동
                            r_ = args.move_rate * dt
                            move[side][0] -= jx * r_                    # 로봇 좌우
                            if args.stick_y == "forward":
                                move[side][2] -= jy * r_                # 로봇 앞뒤
                            else:
                                move[side][1] += jy * r_                # 로봇 상하
                        if bool(getattr(inp, "primary_click", False)):
                            wrist[side] = _Rot.identity()
                            move[side][:] = 0.0
                            hold[side] = None                # 현재 위치에 다시 고정

                    # only: 컨트롤러를 움직여도 팔은 따라가지 않는다.
                    # 처음 본 위치(또는 A/X 직후 위치)를 붙들고, 조이스틱 오프셋만 더한다.
                    if args.move_mode == "only":
                        for side, cp, ok in (("left", lcp, l_ok), ("right", rcp, r_ok)):
                            if ok and hold[side] is None:
                                hold[side] = _P(float(cp.pose.position.x),
                                                float(cp.pose.position.y),
                                                float(cp.pose.position.z))

                    lpos = hold["left"] if (args.move_mode == "only" and hold["left"]) \
                        else (lcp.pose.position if l_ok else None)
                    rpos = hold["right"] if (args.move_mode == "only" and hold["right"]) \
                        else (rcp.pose.position if r_ok else None)
                    lc = (to_unity(lpos, wrist["left"], S, args.lift,
                                   Rfix, False, quat_override=True,
                                   shift=move["left"]) if l_ok else dict(IDENT))
                    rc = (to_unity(rpos, wrist["right"], S, args.lift,
                                   Rfix, False, quat_override=True,
                                   shift=move["right"]) if r_ok else dict(IDENT))
                else:
                    lc = (to_unity(lcp.pose.position, lcp.pose.orientation, S, args.lift,
                                   Rfix, args.no_ori) if l_ok else dict(IDENT))
                    rc = (to_unity(rcp.pose.position, rcp.pose.orientation, S, args.lift,
                                   Rfix, args.no_ori) if r_ok else dict(IDENT))

                # 기준 프레임: 헤드셋 포즈. 수신기는 이 프레임 기준으로 컨트롤러를 해석한다.
                head = head_tracker.get_head(session)
                head = getattr(head, "data", head)
                if head is not None and getattr(head, "is_valid", False):
                    rf = to_unity(head.pose.position, head.pose.orientation, S)
                else:
                    rf = dict(IDENT)

                g = lambda o, a, d=0.0: (float(getattr(o, a)) if o is not None else d)
                bl = lambda o, a: (bool(getattr(o, a)) if o is not None else False)

                pkt = {
                    "t": t,
                    "lc": lc, "rc": rc, "rf": rf,
                    "lt": g(lin, "trigger_value"), "rt": g(rin, "trigger_value"),
                    "lg": g(lin, "squeeze_value"), "rg": g(rin, "squeeze_value"),
                    "lsx": g(lin, "thumbstick_x"), "lsy": g(lin, "thumbstick_y"),
                    "rsx": g(rin, "thumbstick_x"), "rsy": g(rin, "thumbstick_y"),
                    "a": bl(rin, "primary_click"), "b": bl(rin, "secondary_click"),
                    "x": bl(lin, "primary_click"), "y": bl(lin, "secondary_click"),
                    "v": 0 if (l_ok and r_ok) else 2,
                    "vl": 0 if l_ok else 2,
                    "vr": 0 if r_ok else 2,
                }
                sock.sendto(json.dumps(pkt).encode("utf-8"), dest)
                if rec:
                    rec.write(f"{t:.4f},{rc['x']:.5f},{rc['y']:.5f},{rc['z']:.5f},"
                              f"{rc['qx']:.5f},{rc['qy']:.5f},{rc['qz']:.5f},{rc['qw']:.5f},"
                              f"{pkt['rt']:.3f},{pkt['rg']:.3f},"
                              f"{lc['x']:.5f},{lc['y']:.5f},{lc['z']:.5f},"
                              f"{lc['qx']:.5f},{lc['qy']:.5f},{lc['qz']:.5f},{lc['qw']:.5f},"
                              f"{pkt['lt']:.3f},{pkt['lg']:.3f},{int(l_ok)},{int(r_ok)}\n")
                n_sent += 1
                n_valid += int(l_ok and r_ok)

                if args.show and n_sent % 30 == 0:
                    print(f"  R({rc['x']:+.3f},{rc['y']:+.3f},{rc['z']:+.3f}) trig {pkt['rt']:.2f}"
                          f"   L({lc['x']:+.3f},{lc['y']:+.3f},{lc['z']:+.3f}) trig {pkt['lt']:.2f}"
                          f"   valid L{int(l_ok)} R{int(r_ok)}")

                if loop_start - last_report > 5.0:
                    pct = 100.0 * n_valid / max(n_sent, 1)
                    print(f"[bridge] {n_sent} 패킷, 포즈 유효 {pct:.0f}%")
                    if n_valid == 0:
                        print("[bridge] 포즈가 계속 무효입니다 — 헤드셋을 착용하고 "
                              "컨트롤러를 켜서 시야 안에서 움직여 보세요.")
                    last_report = loop_start

                time.sleep(max(0.0, period - (time.time() - loop_start)))
        except KeyboardInterrupt:
            print("\n[bridge] 종료")
        finally:
            if rec:
                rec.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
