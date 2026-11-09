# Copyright 2026 Dental-VLA project. Apache-2.0.
"""
"Attention-pose" (차렷) calibrated mapping: VR controller -> OpenArm v1 wrist target.

Why: the enactic receiver maps (hand - neck reference) 1:1 into the robot frame. A person's
arm (shoulder -> controller ~0.60 m) is longer than OpenArm v1's (shoulder -> wrist site
0.437 m) and their shoulders are wider, so standing at attention asked the robot for a
point ~0.3 m below its reach and it could never hang its arms.

The operator takes the SAME pose the MuJoCo viewer shows at start (`home`, "ㄴ": upper arms
down, elbows 90 deg, controllers pointing forward) and presses Y. That controller pose is
pinned to the robot's home wrist pose. After that:
    wrist target = wrist_at_home + scale * (hand displacement, in the operator's yaw frame
                   at calibration)                      scale = 0.437 / human_arm
    gripper rot  = (controller rotation since calibration) applied to the home gripper
(neutral="attention" pins to the straight-arm pose instead; avoid: it is a singularity and
the IK flips branches near it.)

Frames: OpenXR input (x right, y up, z back). Output: arm_origin frame (x forward,
y left, z up). `encode()` turns the desired arm_origin pose into the packet the enactic
receiver expects (it inverts the receiver's LH->RH flip, R_FRAME, FRAME_OFFSET_NECK and
R_FIX with an identity reference), so dora itself is unchanged.
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

# dora-openarm-vr quest_receiver constants (see its module docstring)
_R_FRAME = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
_OFFSET = np.array([-0.085, 0.0, -0.14])
_R_FIX = R.from_euler("z", 90, degrees=True)

# OpenArm v1 in the arm_origin frame (tools/or_v1 scene), measured with MuJoCo.
ROBOT_ARM = 0.437                                   # shoulder (link2) -> wrist site
NEUTRAL = {
    # MuJoCo `home` keyframe = "ㄴ" (upper arm down, elbow 90 deg, gripper pointing forward).
    # Default: far from the straight-arm singularity, and it is what the viewer shows at start.
    "home": ({"left": np.array([0.2159, 0.2166, -0.2041]), "right": np.array([0.2159, -0.2166, -0.2041])},
             {"left": R.from_quat([-0.1041, 0.7093, 0.1072, -0.6889]),
              "right": R.from_quat([0.1041, 0.7093, -0.1072, -0.6889])}),
    # all joints 0 = arms hanging straight down (singular: elbow fully extended)
    "attention": ({"left": np.array([0.0, 0.153, -0.436]), "right": np.array([0.0, -0.153, -0.436])},
                  {"left": R.identity(), "right": R.identity()}),
}

# Virtual floor over the instrument cart (arm_origin frame). Without it the operator's hand going
# a little too low sends the wrist target below the drape; the position servos then drive the
# fingers/wrist INTO the tray and 30-60 g instruments squeezed against it are shot away (replay
# of the 2026-09-29 sessions: 4 kN on a mirror, >100 kN from wrist links on a scalpel).
TABLE_Z = 0.912 - 1.22          # drape top in the arm_origin frame (-0.308)
FLOOR_CLEARANCE = 0.003
FLOOR_FROM_X = 0.08             # only in front of the robot (the cart edge is at x=0.13)
# points of the gripper/wrist in the wrist-site frame (-z = pointing direction), metres
_GRIPPER_PTS = np.array([
    [0, 0, -0.1955], [0, 0.02, -0.195], [0, -0.02, -0.195],            # fingertips
    [0.085, 0, -0.11], [-0.085, 0, -0.11], [0, 0.04, -0.13], [0, -0.04, -0.13],   # hand body
    [0.045, 0, -0.06], [-0.045, 0, -0.06], [0, 0.045, -0.06], [0, -0.045, -0.06],  # wrist links
])


def apply_floor(pos: np.ndarray, rot: R) -> np.ndarray:
    """Raise the wrist target so no gripper point goes below the drape (+clearance)."""
    pos = np.array(pos, float)
    if pos[0] < FLOOR_FROM_X:
        return pos
    low = pos[2] + rot.apply(_GRIPPER_PTS)[:, 2].min()
    floor = TABLE_Z + FLOOR_CLEARANCE
    if low < floor:
        pos[2] += floor - low
    return pos


# operator yaw frame (x right, y up, z back) -> robot arm_origin (x fwd, y left, z up)
_M = _R_FRAME


class AttentionMapper:
    def __init__(self, human_arm: float = 0.60, scale: float | None = None, neutral: str = "home"):
        self.scale = scale if scale is not None else ROBOT_ARM / human_arm
        self.wrist0, self.rot0 = NEUTRAL[neutral]
        self.cal: dict[str, tuple[np.ndarray, R]] = {}
        self.R_yaw = R.identity()

    @property
    def ready(self) -> bool:
        return len(self.cal) == 2

    def side_ready(self, side: str) -> bool:
        return side in self.cal

    def calibrate(self, head_quat_xyzw, hands: dict[str, tuple]) -> float:
        """Start / re-centre. hands: side -> ((x,y,z), (qx,qy,qz,qw)) in OpenXR, only the
        controllers that are tracked right now (the other hand can join later via add_hand).
        Returns head yaw (deg)."""
        yaw = R.from_quat(head_quat_xyzw).as_euler("YXZ")[0]
        self.R_yaw = R.from_euler("Y", yaw)
        self.cal = {}
        for side, (p, q) in hands.items():
            self.cal[side] = (np.asarray(p, float), R.from_quat(q))
        return float(np.degrees(yaw))

    def add_hand(self, side: str, p, q) -> None:
        """A controller that was not tracked at calibration joins (same yaw frame)."""
        self.cal[side] = (np.asarray(p, float), R.from_quat(q))

    def target(self, side: str, p, q) -> tuple[np.ndarray, R]:
        """Desired wrist-site pose in the arm_origin frame."""
        p0, q0 = self.cal[side]
        d_op = self.R_yaw.inv().apply(np.asarray(p, float) - p0)          # operator yaw frame
        pos = self.wrist0[side] + self.scale * (_M @ d_op)
        dR_op = self.R_yaw.inv() * R.from_quat(q) * q0.inv() * self.R_yaw  # rotation since cal.
        dR = R.from_matrix(_M @ dR_op.as_matrix() @ _M.T)
        rot = dR * self.rot0[side]
        return apply_floor(pos, rot), rot

    @staticmethod
    def encode(pos: np.ndarray, rot: R) -> dict:
        """arm_origin pose -> quest_receiver packet pose (use with rf = identity)."""
        p_rh = _R_FRAME.T @ (np.asarray(pos) - _OFFSET)
        r_rh = R.from_matrix(_R_FRAME.T) * rot * _R_FIX.inv()
        x, y, z, w = r_rh.as_quat()
        return {"x": float(p_rh[0]), "y": float(p_rh[1]), "z": float(-p_rh[2]),
                "qx": float(-x), "qy": float(-y), "qz": float(z), "qw": float(w)}
