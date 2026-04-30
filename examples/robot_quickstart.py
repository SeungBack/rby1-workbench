"""RBY1 최소 사용 예제.

기본 실행은 연결과 상태 조회만 수행한다.
실제 동작 예제를 보려면 아래 옵션을 사용한다.

실행:
    python examples/robot_quickstart.py
    python examples/robot_quickstart.py --address 192.168.12.1:50051 --model m
    python examples/robot_quickstart.py --run-motion
    python examples/robot_quickstart.py --stream-demo
"""

from __future__ import annotations

import argparse
import logging
import time

import numpy as np

from rby1_workbench import RBY1, load_rby1_config

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)


def _print_usage_banner() -> None:
    log.info("RBY1 최소 예제")
    log.info("  기본: 연결 + 현재 상태 조회")
    log.info("  --run-motion: ready / head move / zero 예제 실행")
    log.info("  --stream-demo: open_stream + send 예제 실행")


def _show_current_state(robot: RBY1) -> None:
    log.info("현재 관절 상태")
    for component in ("torso", "right_arm", "left_arm", "head"):
        q = robot.get_joint_positions(component)
        log.info("  %-10s q_deg=%s", component, np.rad2deg(q).round(1).tolist())

    t_right = robot.get_ee_pose("right_arm")
    t_left = robot.get_ee_pose("left_arm")
    log.info("현재 EE 위치")
    log.info("  right_arm pos(m)=%s", t_right[:3, 3].round(3).tolist())
    log.info("  left_arm  pos(m)=%s", t_left[:3, 3].round(3).tolist())


def _run_motion_demo(robot: RBY1) -> None:
    log.info("Motion demo 시작")
    ok = robot.ready(minimum_time=5.0)
    log.info("  ready -> %s", ok)

    ok = robot.move(
        mode="joint",
        head=np.array([0.2, 0.15], dtype=float),
        minimum_time=2.0,
    )
    log.info("  head joint move -> %s", ok)
    time.sleep(0.5)

    ok = robot.move(
        mode="joint",
        head=np.zeros(2, dtype=float),
        minimum_time=2.0,
    )
    log.info("  head home -> %s", ok)


def _run_stream_demo(robot: RBY1, dt: float) -> None:
    log.info("Stream demo 시작")
    q_torso = robot.get_joint_positions("torso")
    q_head = robot.get_joint_positions("head")
    t_right = robot.get_ee_pose("right_arm")
    t_left = robot.get_ee_pose("left_arm")

    stream = robot.open_stream(mode="cartesian")
    try:
        stream.send(
            torso=q_torso,
            right_arm=t_right,
            left_arm=t_left,
            head=q_head,
        )
        for tick in range(1, 11):
            target = t_right.copy()
            target[2, 3] += 0.02 * np.sin(tick * 0.4)
            stream.send(right_arm=target)
            time.sleep(dt)
        log.info("  stream demo 완료")
    finally:
        stream.close()
        log.info("  stream.close()")


def main() -> None:
    parser = argparse.ArgumentParser(description="RBY1 quickstart example")
    parser.add_argument("--address", default=None, help="로봇 주소 override")
    parser.add_argument("--model", default=None, help="로봇 모델 override")
    parser.add_argument("--config", default=None, help="사용자 rby1.yaml 경로")
    parser.add_argument(
        "--run-motion",
        action="store_true",
        help="ready / joint move 예제 실행",
    )
    parser.add_argument(
        "--stream-demo",
        action="store_true",
        help="open_stream + send 예제 실행",
    )
    args = parser.parse_args()

    _print_usage_banner()

    cfg = load_rby1_config(args.config)
    if args.address:
        cfg.address = args.address
    if args.model:
        cfg.model = args.model

    robot = RBY1(cfg)
    robot.initialize()
    log.info("connected to %s  model=%s", cfg.address, robot.model.model_name)

    _show_current_state(robot)

    if args.run_motion:
        _run_motion_demo(robot)

    if args.stream_demo:
        _run_stream_demo(robot, float(cfg.stream.dt))


if __name__ == "__main__":
    main()
