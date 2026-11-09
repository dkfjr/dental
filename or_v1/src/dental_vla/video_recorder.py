#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
dora node: record JPEG camera frames from `dora-openarm-mujoco --render` into an mp4.

    - id: recorder
      path: ../src/dental_vla/video_recorder.py
      args: "--out ../out/teleop_or_v1.mp4 --fps 30"
      inputs:
        image: mujoco-viewer/camera_ceiling
        image2: mujoco-viewer/camera_head_left     # optional second view, placed on the right

Frames are written as they arrive (≈30 Hz); the file is finalised on STOP / Ctrl-C.
No ffmpeg needed (OpenCV's bundled mp4v encoder).
"""
import argparse
import time
from pathlib import Path

import cv2
import numpy as np
from dora import Node


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="teleop.mp4")
    ap.add_argument("--fps", type=float, default=30.0)
    a = ap.parse_args()
    out = Path(a.out)
    if out.suffix == ".mp4" and out.exists():
        out = out.with_name(f"{out.stem}_{time.strftime('%Y%m%d_%H%M%S')}{out.suffix}")
    out.parent.mkdir(parents=True, exist_ok=True)

    writer, n, second = None, 0, None
    node = Node()
    try:
        for ev in node:
            if ev["type"] == "STOP":
                break
            if ev["type"] != "INPUT":
                continue
            buf = ev["value"].to_numpy(zero_copy_only=False).astype(np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if img is None:
                continue
            if ev["id"] == "image2":
                second = img                      # latest second view; written with the next main frame
                continue
            if second is not None:
                if second.shape[0] != img.shape[0]:
                    second = cv2.resize(second, (int(second.shape[1] * img.shape[0] / second.shape[0]), img.shape[0]))
                img = np.hstack([img, second])
            if writer is None:
                h, w = img.shape[:2]
                writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (w, h))
                print(f"[recorder] {w}x{h} @ {a.fps} fps -> {out.resolve()}", flush=True)
            writer.write(img)
            n += 1
            if n % 300 == 0:
                print(f"[recorder] {n} frames", flush=True)
    finally:
        if writer is not None:
            writer.release()
        print(f"[recorder] done: {n} frames -> {out.resolve()}", flush=True)


if __name__ == "__main__":
    main()
