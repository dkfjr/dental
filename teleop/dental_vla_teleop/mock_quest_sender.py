#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
Mock Meta Quest 3 UDP sender — teleoperate WITHOUT a headset.

Emits the exact JSON packet format that enactic/dora-openarm-vr's
`dora-openarm-quest-receiver` expects, so the full teleop pipeline
(udp-receiver -> ik -> mujoco-viewer) can be exercised on a laptop with no VR
hardware. Useful for smoke-testing the dental scene wiring and for CI.

Packet schema (Unity left-handed world frame, meters / scalar-last quats):
    t                      headset timestamp (s)
    lc, rc, rf             left/right controller + reference pose
                           {x,y,z, qx,qy,qz,qw}
    lt, rt                 left/right index trigger      0.0..1.0
    lg, rg                 left/right grip                0.0..1.0
    lsx,lsy,rsx,rsy        thumbstick axes               -1.0..1.0
    a,b,x,y                buttons                        bool
    v, vl, vr              validity   0=OK 1=STALE 2=INVALID

Motion: both controllers trace a small circle in front of a fixed reference,
with the trigger opening/closing smoothly so you can watch the grippers move.

Usage:
    python -m dental_vla.mock_quest_sender --host 127.0.0.1 --port 5006
    python -m dental_vla.mock_quest_sender --pattern still     # hold still
    python -m dental_vla.mock_quest_sender --rate 50 --radius 0.08
"""
from __future__ import annotations
import argparse
import json
import math
import socket
import time


def _pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0) -> dict:
    return {"x": x, "y": y, "z": z, "qx": qx, "qy": qy, "qz": qz, "qw": qw}


def build_packet(t: float, args) -> dict:
    if args.pattern == "still":
        dx = dy = 0.0
        trig = 0.0
    else:  # "circle"
        ang = 2.0 * math.pi * (t / args.period)
        dx = args.radius * math.cos(ang)
        dy = args.radius * math.sin(ang)
        trig = 0.5 * (1.0 - math.cos(ang))  # 0..1 smooth

    # Reference: where the operator "stood" — fixed. Controllers offset from it.
    rf = _pose(0.0, 1.2, 0.4)
    # Right controller slightly right of the reference, left slightly left.
    rc = _pose(0.15 + dx, 1.2 + 0.10 * math.sin(2 * math.pi * t / args.period),
               0.40 + dy)
    lc = _pose(-0.15 + dx, 1.2 + 0.10 * math.sin(2 * math.pi * t / args.period),
               0.40 + dy)

    return {
        "t": t,
        "rf": rf, "rc": rc, "lc": lc,
        "rt": trig, "lt": trig,
        "rg": 0.0, "lg": 0.0,
        "lsx": 0.0, "lsy": 0.0, "rsx": 0.0, "rsy": 0.0,
        "a": False, "b": False, "x": False, "y": False,
        "v": 0, "vl": 0, "vr": 0,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Mock Quest UDP sender")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5006)
    ap.add_argument("--rate", type=float, default=50.0, help="packets/sec")
    ap.add_argument("--radius", type=float, default=0.08, help="circle radius (m)")
    ap.add_argument("--period", type=float, default=6.0, help="seconds per loop")
    ap.add_argument("--pattern", choices=["circle", "still"], default="circle")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to run (0 = forever)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dt = 1.0 / args.rate
    t0 = time.time()
    n = 0
    print(f"[mock-quest] sending {args.pattern} to udp://{args.host}:{args.port} "
          f"at {args.rate:.0f} Hz  (Ctrl-C to stop)")
    try:
        while True:
            t = time.time() - t0
            if args.duration and t >= args.duration:
                break
            pkt = build_packet(t, args)
            sock.sendto((json.dumps(pkt) + "\n").encode("utf-8"),
                        (args.host, args.port))
            n += 1
            if n % int(args.rate) == 0:
                print(f"[mock-quest] t={t:6.1f}s  sent={n}  trig={pkt['rt']:.2f}",
                      end="\r")
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        print(f"\n[mock-quest] stopped after {n} packets")


if __name__ == "__main__":
    main()
