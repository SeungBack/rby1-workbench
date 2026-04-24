"""rby1-workbench 기본 제어 예제.

실행:
    python examples/robot_control_basic.py --address 192.168.30.1:50051
"""

import argparse
import logging
import time

import numpy as np

from rby1_workbench import RBY1, RBY1Config

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-8s %(message)s")


def main(address: str, model: str):
    # 1. 설정 및 연결
    cfg = RBY1Config(address=address, model=model)
    robot = RBY1(cfg)
    robot.initialize()   # connect + power_on + servo_on + enable_control_manager

    # 2. Zero → Ready pose (blocking)
    robot.zero_pose(minimum_time=5.0)
    robot.ready_pose(minimum_time=5.0)

    # 3. 헤드 제어
    robot.head.move(yaw=0.2, pitch=-0.1, minimum_time=2.0)
    time.sleep(1.0)
    robot.head.zero()

    # 4. 현재 EE 포즈 읽기 (FK)
    T_right = robot.get_ee_pose("right_arm")
    T_left  = robot.get_ee_pose("left_arm")
    print("Right EE:\n", T_right)
    print("Left EE:\n", T_left)

    # 5. Joint impedance control (blocking, 22번 예제 패턴)
    robot.joint_impedance_control(
        right_arm=np.zeros(len(robot.model.right_arm_idx)),
        stiffness=100.0,
        torque_limit=10.0,
        minimum_time=5.0,
    )

    # 6. 실시간 Cartesian impedance streaming (VR teleop 패턴)
    T_right = robot.get_ee_pose("right_arm")
    T_left  = robot.get_ee_pose("left_arm")
    q_torso = np.asarray(robot.get_state().position)[list(robot.model.torso_idx)]

    stream = robot.create_stream()
    try:
        # 초기화: reset=True로 CartesianImpedance reference 설정
        stream.send(
            torso=q_torso,
            right_arm=T_right,
            left_arm=T_left,
            head=(0.0, 0.0),
            reset=True,
        )

        # 이후 루프: 오른팔만 움직여도 torso/left_arm/head 자동 hold
        dt = cfg.stream.dt
        for i in range(50):
            T_right_new = T_right.copy()
            T_right_new[2, 3] += 0.02 * np.sin(i * 0.2)   # Z 방향 진동

            stream.send(right_arm=T_right_new)
            time.sleep(dt)
    finally:
        stream.cancel()

    print("Done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--address", default="192.168.30.1:50051")
    parser.add_argument("--model", default="a")
    args = parser.parse_args()
    main(args.address, args.model)
