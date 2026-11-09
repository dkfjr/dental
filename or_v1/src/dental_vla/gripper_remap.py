#!/usr/bin/env python3
# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
dora node: OpenArm **v1** gripper command from the VR triggers (sits between ik and mujoco).

Why: dora-openarm-vr converts the trigger into a *v2* gripper angle in radians, with the
right side mirrored (right: open -45 deg -> closed +10 deg; left: open +45 -> closed -10).
dora-openarm-mujoco writes that number straight into the v1 finger slide actuator
(metres, -0.005 = closed .. 0.044 = open). Result: the right gripper worked inverted and
both sides only switched near 80 % trigger.

This node overwrites qpos[7] of ik/position_{left,right} with a proportional opening:
    trigger 0.0 -> OPEN (0.044 m),  trigger 1.0 -> CLOSED (-0.005 m)
so pulling the trigger closes the gripper on both hands.

Rate limit (2026-09-29): operators snap the trigger 0 -> 1 in ~50 ms. Sent straight to the
finger actuator, the fingers slam shut and knock a syringe off the tray (reproduced in sim:
closing in 0.05 s or 0.15 s fails, 0.4 s and 1.2 s lift it). The command is therefore slewed:
full close takes >= 0.5 s, full open >= 0.25 s (real gripper motors are not faster either).

    - id: gripper
      path: ../src/dental_vla/gripper_remap.py
      inputs:
        position_left:  ik/position_left
        position_right: ik/position_right
        trigger_left:   udp-receiver/trigger_left
        trigger_right:  udp-receiver/trigger_right
      outputs: [position_left, position_right]
"""
import time

import numpy as np
import pyarrow as pa
from dora import Node

OPEN, CLOSED = 0.044, -0.005
CLOSE_SPEED = (OPEN - CLOSED) / 0.5     # m/s  (full stroke in 0.5 s)
OPEN_SPEED = (OPEN - CLOSED) / 0.25
_QPOS = pa.struct({"qpos": pa.list_(pa.float32())})


def _values(value: pa.Array, key: str) -> np.ndarray:
    if pa.types.is_struct(value.type):
        value = value.field(key)[0].values
    return np.array(value, dtype=np.float32)


def main() -> None:
    trig = {"left": 0.0, "right": 0.0}
    cmd = {"left": OPEN, "right": OPEN}          # slewed finger command
    last = {"left": None, "right": None}
    node = Node()
    for ev in node:
        if ev["type"] != "INPUT":
            continue
        eid = ev["id"]
        if eid.startswith("trigger_"):
            v = _values(ev["value"], "trigger")
            if v.size:
                trig[eid.split("_", 1)[1]] = float(np.clip(v[0], 0.0, 1.0))
        elif eid.startswith("position_"):
            side = eid.split("_", 1)[1]
            q = _values(ev["value"], "qpos").copy()
            if q.shape == (8,):
                goal = OPEN + trig[side] * (CLOSED - OPEN)
                now = time.monotonic()
                dt = 0.0 if last[side] is None else min(now - last[side], 0.1)
                last[side] = now
                step = (CLOSE_SPEED if goal < cmd[side] else OPEN_SPEED) * dt
                cmd[side] += max(-step, min(step, goal - cmd[side]))
                q[7] = cmd[side]
            node.send_output(eid, pa.array([{"qpos": q}], type=_QPOS), ev["metadata"])


if __name__ == "__main__":
    main()
