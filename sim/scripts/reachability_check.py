"""헤드리스 도달성 확인. 뷰어 없이 순기구학만으로 EE 좌표를 계산한다.

사용법:
    python reachability_check.py

무작위 샘플링(30000회)으로 목표 근방 최소거리를 찾는다. 정밀 IK가 아니므로
여기서 나온 오차(수 cm)는 상한이 아니라 "이 정도는 최소로 닿는다"는 하한 증거로 읽을 것.
"""

import mujoco
import numpy as np

ARM = "left"
NAMES = [f"openarm_{ARM}_joint{i}" for i in range(1, 8)]
LO = np.array([-3.49, -3.32, -1.57, 0, -1.57, -0.79, -1.57])
HI = np.array([1.40, 0.17, 1.57, 2.44, 1.57, 0.79, 1.57])

TARGETS = {
    "트레이 슬롯1": np.array([-0.35, -0.25, 0.815]),
    "전달구역 중심": np.array([0.20, 0.00, 0.91]),
}


def main():
    m = mujoco.MjModel.from_xml_path("main.xml")
    d = mujoco.MjData(m)
    adr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in NAMES]
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"{ARM}_ee_control_point")

    def ee(q):
        d.qpos[:] = 0
        for a, v in zip(adr, q):
            d.qpos[a] = v
        mujoco.mj_forward(m, d)
        return d.site_xpos[sid].copy()

    rng = np.random.default_rng(0)
    for name, tgt in TARGETS.items():
        best_d, best_q = 1e9, None
        for _ in range(30000):
            q = rng.uniform(LO, HI)
            dist = np.linalg.norm(ee(q) - tgt)
            if dist < best_d:
                best_d, best_q = dist, q
        print(f"{name}: 목표 {tgt}")
        print(f"  무작위 샘플 최소거리 = {best_d:.4f} m")
        print(f"  q = {list(np.round(best_q, 4))}")
        print()


if __name__ == "__main__":
    main()
