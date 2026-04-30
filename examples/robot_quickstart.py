"""RBY1 quickstart reference.

핵심 흐름만 한 파일에 모아 둔 입문 예제:
- `status`: 연결 + 상태 조회
- `move`: blocking `move(...)`
- `stream`: `open_stream(...).send(...)`

같은 예제를 direct / client backend 모두에서 실행할 수 있다.
즉 `RBY1(cfg, backend="client", endpoint=...)` 사용 예제도 이 파일에 포함된다.

실행:
    python examples/robot_quickstart.py status
    python examples/robot_quickstart.py move
    python examples/robot_quickstart.py stream
    python examples/robot_quickstart.py status --backend client --endpoint tcp://127.0.0.1:5555
    python examples/robot_quickstart.py stream --backend client --endpoint tcp://127.0.0.1:5555
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
    log.info("RBY1 quickstart")
    log.info("  status : 연결 + 현재 상태 조회")
    log.info("  move   : blocking move(mode='joint')")
    log.info("  stream : open_stream(mode='cartesian')")
    log.info("  client : 위 흐름에 --backend client --endpoint tcp://HOST:PORT 추가")


def _show_current_state(robot: RBY1) -> None:
    log.info("현재 관절 상태")
    for component in ("torso", "right_arm", "left_arm", "head"):
        q = robot.get_joint_positions(component)
        log.info("  %-10s q_rad=%s q_deg=%s", component,q.round(3).tolist()
                , np.rad2deg(q).round(1).tolist())

    t_right = robot.get_ee_pose("right_arm")
    t_left = robot.get_ee_pose("left_arm")
    log.info("현재 EE 위치")
    log.info("  right_arm pos(m)=%s", t_right[:3, 3].round(3).tolist())
    log.info("  left_arm  pos(m)=%s", t_left[:3, 3].round(3).tolist())


def _run_move_demo(robot: RBY1, minimum_time: float) -> None:
    log.info("Move demo 시작")
    ok = robot.ready(minimum_time=5.0)
    log.info("  ready -> %s", ok)

    ok = robot.move(
        mode="joint",
        head=np.array([0.2, 0.15], dtype=float),
        minimum_time=minimum_time,
    )
    log.info("  head joint move -> %s", ok)
    time.sleep(0.5)

    ok = robot.move(
        mode="joint",
        head=np.zeros(2, dtype=float),
        minimum_time=minimum_time,
    )
    log.info("  head home -> %s", ok)


def _run_stream_demo(robot: RBY1, dt: float, seconds: float) -> None:
    log.info("Stream demo 시작")
    q_torso = robot.get_joint_positions("torso")
    q_head = robot.get_joint_positions("head")
    t_right = robot.get_ee_pose("right_arm")
    t_left = robot.get_ee_pose("left_arm")
    n_ticks = max(2, int(round(seconds / dt)))

    stream = robot.open_stream(mode="cartesian")
    try:
        stream.send(
            torso=q_torso,
            right_arm=t_right,
            left_arm=t_left,
            head=q_head,
        )
        for tick in range(1, n_ticks):
            target = t_right.copy()
            target[2, 3] += 0.02 * np.sin(tick * 0.4)
            stream.send(right_arm=target)
            time.sleep(dt)
        log.info("  stream demo 완료 (%d ticks, %.2fs)", n_ticks, dt * n_ticks)
    finally:
        stream.close()
        log.info("  stream.close()")


def _load_robot(args: argparse.Namespace) -> tuple[RBY1, float]:
    cfg = load_rby1_config(args.config)
    if args.address:
        cfg.address = args.address
    if args.model:
        cfg.model = args.model

    endpoint = None
    if args.backend == "client":
        if not args.endpoint:
            raise ValueError("--backend client 사용 시 --endpoint 가 필요합니다.")
        endpoint = args.endpoint

    robot = RBY1(cfg, backend=args.backend, endpoint=endpoint)
    return robot, float(cfg.stream.dt)


def main() -> None:
    parser = argparse.ArgumentParser(description="RBY1 quickstart example")
    parser.add_argument(
        "command",
        nargs="?",
        choices=("status", "move", "stream"),
        default="status",
        help="실행할 quickstart 흐름",
    )
    parser.add_argument("--address", default=None, help="로봇 주소 override")
    parser.add_argument("--model", default=None, help="로봇 모델 override")
    parser.add_argument("--config", default=None, help="사용자 rby1.yaml 경로")
    parser.add_argument(
        "--backend",
        choices=("direct", "client"),
        default="direct",
        help="RBY1 backend 선택",
    )
    parser.add_argument(
        "--endpoint",
        default=None,
        help="client backend endpoint, 예: tcp://127.0.0.1:5555",
    )
    parser.add_argument(
        "--minimum-time",
        type=float,
        default=2.0,
        help="move command에서 사용할 head move minimum_time",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=3.0,
        help="stream command에서 사용할 streaming duration in seconds",
    )
    args = parser.parse_args()

    _print_usage_banner()

    robot, dt = _load_robot(args)

    if args.command == "status":
        robot.connect()
    else:
        robot.initialize()
    log.info("backend=%s  model=%s", args.backend, robot.model.model_name)

    _show_current_state(robot)

    if args.command == "move":
        _run_move_demo(robot, float(args.minimum_time))
    elif args.command == "stream":
        _run_stream_demo(robot, dt, float(args.seconds))


if __name__ == "__main__":
    main()
