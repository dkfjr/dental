"""v3(마주보기, Blender 실측 메쉬) 씬 애니메이션 — 트레이 -> 핸드오프 -> 반납 왕복.

순서:
  1. 로봇 오른팔이 카트 위 트레이에서 Retractor_Minnesota(로봇 기준 가장 가까운 기구)를 집는다
  2. TRANSFER 지점(카트 모서리, 로봇·의사 둘 다 닿는 공용 지점)까지 들고 가서 내려놓는다
  3. 의사 오른팔이 그 자리에서 받아 잠깐 들고 있는다
  4. 의사가 다시 그 자리에 갖다 놓으면 로봇이 재파지해서 트레이로 반납한다
  5. 1~4를 CYCLES번 반복

scripts/find_waypoints_v3.py 로 REST/PICK/TRANSFER 두 지점만 계산해 둔 단순 버전이라
(구버전 episode.py처럼 ABOVE_TRAY/ABOVE_ZONE 같은 중간 대피 지점은 없음) 팔이 PICK<->TRANSFER
사이를 직선(관절공간) 보간으로 오간다 — output/v3_midpath_*.png 로 경로 중간에 받침대/카트와
안 겹치는 것 확인된 상태.
"""

import mujoco
import numpy as np

import waypoints_v3 as W

CYCLES = 2
FPS = 30

SEG_TIME = {
    "REST->PICK": 1.3,
    "GRASP_CLOSE": 0.5,
    "PICK->TRANSFER": 1.6,          # 이 구간에 맞춰 의사 팔도 REST->TRANSFER로 등장
    "HAND_RECEIVE_PAUSE": 0.3,
    "GRIPPER_OPEN_HANDOFF": 0.5,
    "DOCTOR_HOLD": 0.8,             # 의사가 기구를 들고 있는 시간
    "GRIPPER_CLOSE_REGRASP": 0.5,
    "TRANSFER->PICK": 1.6,          # 이 구간에 맞춰 의사 팔도 대기자세로 퇴장
    "GRASP_OPEN_RELEASE": 0.5,
    "PICK->REST": 1.3,
}

GRIP_OPEN = -0.7854    # 오른팔 그리퍼는 왼팔과 부호가 반대(range가 -0.7854~0)
GRIP_CLOSED = -0.05

ROBOT_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_REST = [0.0] * 7   # 왼팔은 이번 데모에서 안 씀 — 대기 자세로 고정
DOCTOR_JOINTS = [
    "doctor_r_shoulder_yaw",
    "doctor_r_shoulder_pitch",
    "doctor_r_shoulder_roll",
    "doctor_r_elbow_joint",
    "doctor_r_wrist_joint",
]


def min_jerk(t):
    t = np.clip(t, 0.0, 1.0)
    return 10 * t**3 - 15 * t**4 + 6 * t**5


def interp_q(q0, q1, n_steps):
    for i in range(n_steps):
        s = min_jerk((i + 1) / n_steps)
        yield q0 + (q1 - q0) * s


class Sim:
    def __init__(self):
        self.m = mujoco.MjModel.from_xml_path("main_v3_blender.xml")
        self.d = mujoco.MjData(self.m)

        self.qadr = [self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in ROBOT_JOINTS]
        self.act = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"right_joint{i}_ctrl") for i in range(1, 8)]
        self.fing_act = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "right_finger1_ctrl")
        self.fing_joint_adr = self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "openarm_right_finger_joint1")]
        self.fing2_joint_adr = self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "openarm_right_finger_joint2")]

        l_names = [f"openarm_left_joint{i}" for i in range(1, 8)]
        self.l_qadr = [self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in l_names]
        self.l_act = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"left_joint{i}_ctrl") for i in range(1, 8)]

        self.ee_site = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "right_gripper_tip")
        self.instr_body = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "instr_retractor_minnesota")
        self.instr_qadr = self.m.jnt_qposadr[self.m.body_jntadr[self.instr_body]]
        self.instr_dof = self.m.body_dofadr[self.instr_body]
        # 트레이 위 원래 놓여 있던 자세(freejoint 기본값) — release_to_rest()가 여기로
        # 되돌린다. PICK_TARGET은 그리퍼가 트레이를 안 뚫게 트레이보다 약간 높게 잡아서
        # (§find_waypoints_v3.py) 손을 놓는 순간 실제 트레이 표면보다 붕 뜬 높이에서
        # 멈춰버리는 문제가 있었음 — 손을 놓을 때 이 자세로 다시 내려앉게 함.
        self.instr_rest_qpos = self.d.qpos[self.instr_qadr: self.instr_qadr + 7].copy()

        self.doc_qadr = [self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in DOCTOR_JOINTS]
        self.doc_grip_site = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "doctor_r_grip")

        self.renderer = mujoco.Renderer(self.m, height=720, width=1280)
        self.cam = "wide"
        self.frames = []

        self.grasp_offset_local = np.zeros(3)
        self.holding = False
        self.carrier = "gripper"

        for a, v in zip(self.l_qadr, LEFT_REST):
            self.d.qpos[a] = v
        for a, v in zip(self.l_act, LEFT_REST):
            self.d.ctrl[a] = v
        self.set_doctor_arm_q(W.DOCTOR_REST)

    def set_arm_q(self, q):
        for a, act_id, v in zip(self.qadr, self.act, q):
            self.d.qpos[a] = v
            self.d.ctrl[act_id] = v

    def set_gripper(self, g):
        self.d.ctrl[self.fing_act] = g
        self.d.qpos[self.fing_joint_adr] = g
        self.d.qpos[self.fing2_joint_adr] = g

    def set_doctor_arm_q(self, q):
        for a, v in zip(self.doc_qadr, q):
            self.d.qpos[a] = v

    def carry_instrument(self):
        if self.carrier == "gripper":
            site = self.ee_site
            offset = self.grasp_offset_local
        else:
            site = self.doc_grip_site
            offset = np.zeros(3)
        pos = self.d.site_xpos[site].copy()
        mat = self.d.site_xmat[site].reshape(3, 3)
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, mat.flatten())
        self.d.qpos[self.instr_qadr: self.instr_qadr + 3] = pos - mat @ offset
        self.d.qpos[self.instr_qadr + 3: self.instr_qadr + 7] = quat
        self.d.qvel[self.instr_dof: self.instr_dof + 6] = 0

    def release_to_rest(self, seconds):
        """기구를 (그리퍼를 따라가던) 현재 위치에서 트레이 위 원래 자세로 부드럽게
        내려놓는다. 이걸 안 하면 기구가 PICK_TARGET 높이(트레이보다 위)에서 그대로
        얼어붙어 트레이 위에 붕 떠 있는 것처럼 보인다."""
        n_frames = max(1, int(round(seconds * FPS)))
        start_pos = self.d.qpos[self.instr_qadr: self.instr_qadr + 3].copy()
        start_quat = self.d.qpos[self.instr_qadr + 3: self.instr_qadr + 7].copy()
        end_pos = self.instr_rest_qpos[:3]
        end_quat = self.instr_rest_qpos[3:7]
        for i in range(n_frames):
            s = min_jerk((i + 1) / n_frames)
            self.d.qpos[self.instr_qadr: self.instr_qadr + 3] = start_pos + (end_pos - start_pos) * s
            q = start_quat + (end_quat - start_quat) * s
            self.d.qpos[self.instr_qadr + 3: self.instr_qadr + 7] = q / np.linalg.norm(q)
            self.d.qvel[self.instr_dof: self.instr_dof + 6] = 0
            mujoco.mj_forward(self.m, self.d)
            self.capture()

    def capture(self):
        self.renderer.update_scene(self.d, camera=self.cam)
        self.frames.append(self.renderer.render().copy())

    def _tick(self):
        mujoco.mj_forward(self.m, self.d)
        if self.holding:
            self.carry_instrument()
            mujoco.mj_forward(self.m, self.d)
        self.capture()

    def move(self, q_from, q_to, seconds, gripper=None, doctor_from=None, doctor_to=None):
        n_frames = max(1, int(round(seconds * FPS)))
        doctor_from = None if doctor_from is None else np.array(doctor_from)
        doctor_to = None if doctor_to is None else np.array(doctor_to)
        for i, q in enumerate(interp_q(np.array(q_from), np.array(q_to), n_frames)):
            self.set_arm_q(q)
            if gripper is not None:
                self.set_gripper(gripper)
            if doctor_from is not None:
                s = min_jerk((i + 1) / n_frames)
                self.set_doctor_arm_q(doctor_from + (doctor_to - doctor_from) * s)
            self._tick()

    def hold_frames(self, q, seconds, gripper=None, doctor_from=None, doctor_to=None):
        n_frames = max(1, int(round(seconds * FPS)))
        doctor_from = None if doctor_from is None else np.array(doctor_from)
        doctor_to = None if doctor_to is None else np.array(doctor_to)
        for i in range(n_frames):
            self.set_arm_q(q)
            if gripper is not None:
                self.set_gripper(gripper)
            if doctor_from is not None:
                s = min_jerk((i + 1) / n_frames)
                self.set_doctor_arm_q(doctor_from + (doctor_to - doctor_from) * s)
            self._tick()


def run_cycle(sim: Sim):
    R_REST, R_PICK, R_TRANSFER = W.ROBOT_REST, W.ROBOT_PICK, W.ROBOT_TRANSFER
    R_ABOVE_PICK = W.ROBOT_ABOVE_PICK
    D_REST, D_TRANSFER, D_LIFT = W.DOCTOR_REST, W.DOCTOR_TRANSFER, W.DOCTOR_LIFT

    # REST->PICK 직선(관절공간) 보간은 카트 상판(bl_Cart_Shelf_Top, z=0.912)을 스치고
    # 지나간다 — PICK과 xy는 같고 z만 높인 ABOVE_PICK을 거쳐서 카트/트레이 위로 다니게 함.
    t_above = 0.5
    sim.move(R_REST, R_ABOVE_PICK, SEG_TIME["REST->PICK"] * t_above, gripper=GRIP_OPEN)
    sim.move(R_ABOVE_PICK, R_PICK, SEG_TIME["REST->PICK"] * (1 - t_above), gripper=GRIP_OPEN)
    sim.hold_frames(R_PICK, SEG_TIME["GRASP_CLOSE"], gripper=GRIP_CLOSED)
    sim.holding = True
    sim.carrier = "gripper"

    # 의사 팔은 REST->TRANSFER로 곧장 직선(관절공간) 보간하면 트레이 위를 가로질러
    # 지나간다 — 먼저 거의 제자리에서 손 높이만 TRANSFER 높이로 들어올리는 LIFT를
    # 거친 뒤(REST->LIFT), 트레이보다 위에서 옆으로 이동(LIFT->TRANSFER)하게 나눈다.
    # 로봇 팔의 PICK->TRANSFER 구간도 같은 시간비율로 2등분(직선 보간이라 이어붙여도
    # 경로 자체는 그대로)해서 같은 시간 동안 같이 끝나도록 맞춤.
    t_split = 0.4
    r_pick, r_transfer = np.array(R_PICK), np.array(R_TRANSFER)
    r_mid = r_pick + t_split * (r_transfer - r_pick)
    seg = SEG_TIME["PICK->TRANSFER"]
    sim.move(R_PICK, r_mid, seg * t_split, gripper=GRIP_CLOSED, doctor_from=D_REST, doctor_to=D_LIFT)
    sim.move(r_mid, R_TRANSFER, seg * (1 - t_split), gripper=GRIP_CLOSED, doctor_from=D_LIFT, doctor_to=D_TRANSFER)
    sim.hold_frames(R_TRANSFER, SEG_TIME["HAND_RECEIVE_PAUSE"], gripper=GRIP_CLOSED,
                     doctor_from=D_TRANSFER, doctor_to=D_TRANSFER)

    sim.hold_frames(R_TRANSFER, SEG_TIME["GRIPPER_OPEN_HANDOFF"], gripper=GRIP_OPEN,
                     doctor_from=D_TRANSFER, doctor_to=D_TRANSFER)
    sim.carrier = "hand"

    sim.hold_frames(R_TRANSFER, SEG_TIME["DOCTOR_HOLD"], gripper=GRIP_OPEN,
                     doctor_from=D_TRANSFER, doctor_to=D_TRANSFER)

    sim.hold_frames(R_TRANSFER, SEG_TIME["GRIPPER_CLOSE_REGRASP"], gripper=GRIP_CLOSED,
                     doctor_from=D_TRANSFER, doctor_to=D_TRANSFER)
    sim.carrier = "gripper"

    # 복귀 경로도 마지막에 ABOVE_PICK을 한 번 거쳐서 카트 상판 위로 내려오게 함
    # (반대 방향이라 이번엔 r_mid2->ABOVE_PICK->PICK 순서).
    seg2 = SEG_TIME["TRANSFER->PICK"]
    r_mid2 = R_TRANSFER + (1 - t_split) * (r_pick - np.array(R_TRANSFER))
    sim.move(R_TRANSFER, r_mid2, seg2 * (1 - t_split), gripper=GRIP_CLOSED,
              doctor_from=D_TRANSFER, doctor_to=D_LIFT)
    sim.move(r_mid2, R_ABOVE_PICK, seg2 * t_split * 0.5, gripper=GRIP_CLOSED,
              doctor_from=D_LIFT, doctor_to=D_LIFT)
    sim.move(R_ABOVE_PICK, R_PICK, seg2 * t_split * 0.5, gripper=GRIP_CLOSED,
              doctor_from=D_LIFT, doctor_to=D_REST)
    # 그리퍼를 열면서 기구를 트레이 위 원래 자세로 부드럽게 내려놓음 (안 그러면
    # PICK_TARGET 높이=트레이보다 살짝 위인 채로 얼어붙어 붕 떠 보임)
    sim.set_arm_q(R_PICK)
    sim.set_gripper(GRIP_OPEN)
    sim.release_to_rest(SEG_TIME["GRASP_OPEN_RELEASE"])
    sim.holding = False

    sim.move(R_PICK, R_ABOVE_PICK, SEG_TIME["PICK->REST"] * (1 - t_above), gripper=GRIP_OPEN)
    sim.move(R_ABOVE_PICK, R_REST, SEG_TIME["PICK->REST"] * t_above, gripper=GRIP_OPEN)


def main():
    sim = Sim()
    sim.set_arm_q(W.ROBOT_REST)
    mujoco.mj_forward(sim.m, sim.d)
    sim.capture()

    for c in range(CYCLES):
        print(f"cycle {c+1}/{CYCLES}")
        run_cycle(sim)

    import imageio
    out = "output/tray_transfer_v3.mp4"
    imageio.mimsave(out, sim.frames, fps=FPS, quality=8)
    print(f"저장 완료: {out} ({len(sim.frames)} frames, {len(sim.frames)/FPS:.1f}s)")


if __name__ == "__main__":
    main()
