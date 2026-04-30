"""rby1-workbench wrapper 기능 테스트.

각 기능을 단계별로 순서대로 테스트하고 결과를 출력한다.

실행:
    python examples/robot_control_basic.py
    python examples/robot_control_basic.py --address 192.168.30.1:50051 --model a
    python examples/robot_control_basic.py --config /path/to/custom_rby1.yaml
    python examples/robot_control_basic.py --skip-streaming   # streaming 단계 건너뜀
"""

from __future__ import annotations

import argparse
import logging
import sys
import threading
import time

import numpy as np

from rby1_workbench import RBY1, load_rby1_config

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-8s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)


# ── 헬퍼 ──────────────────────────────────────────────────────────────────────

def step(n: int, title: str) -> None:
    log.info("=" * 60)
    log.info("STEP %d: %s", n, title)
    log.info("=" * 60)


def ok(msg: str) -> None:
    log.info("  OK  %s", msg)


def fail(msg: str) -> None:
    log.error("  NG  %s", msg)


# ── 각 테스트 단계 ────────────────────────────────────────────────────────────

def test_state_reading(robot: RBY1) -> None:
    step(2, "상태 읽기 (get_joint_positions / get_ee_pose / get_torso_pose)")

    for comp in ("torso", "right_arm", "left_arm", "head"):
        q = robot.get_joint_positions(comp)
        d = robot.dof(comp)
        ok(f"{comp:10s}  dof={d:2d}  q_deg={np.rad2deg(q).round(1).tolist()}")

    T_r = robot.get_ee_pose("right_arm")
    T_l = robot.get_ee_pose("left_arm")
    T_t = robot.get_torso_pose()
    ok(f"right_arm EE      pos(m): {T_r[:3, 3].round(3).tolist()}")
    ok(f"left_arm  EE      pos(m): {T_l[:3, 3].round(3).tolist()}")
    ok(f"link_torso_5      pos(m): {T_t[:3, 3].round(3).tolist()}")


def test_zero_pose(robot: RBY1) -> None:
    step(3, "Zero pose (blocking)")
    ok_flag = robot.zero(minimum_time=5.0)
    (ok if ok_flag else fail)(f"zero -> {ok_flag}")


def test_ready_pose(robot: RBY1) -> None:
    step(4, "Ready pose (blocking)")
    ok_flag = robot.ready(minimum_time=5.0)
    (ok if ok_flag else fail)(f"ready -> {ok_flag}")


def test_head(robot: RBY1) -> None:
    step(5, "Head 제어 (move / zero)")
    robot.move(mode="joint", head=np.array([0.3, -0.2]), minimum_time=2.0)
    ok("move(mode='joint', head=[0.3, -0.2])")
    time.sleep(0.5)
    robot.move(mode="joint", head=np.zeros(2), minimum_time=2.0)
    ok("move(mode='joint', head=[0.0, 0.0])")


def test_joint_impedance(robot: RBY1) -> None:
    step(6, "Joint impedance control — ready pose 자세 hold (blocking)")
    # ready pose 위치 그대로 impedance mode로 전환 (새 위치로 이동하지 않음)
    q_right = robot.get_joint_positions("right_arm")
    q_left = robot.get_joint_positions("left_arm")
    ok_flag = robot.move(
        mode="impedance",
        right_arm=q_right,
        left_arm=q_left,
        stiffness=80.0,
        damping_ratio=1.0,
        torque_limit=20.0,
        minimum_time=3.0,
        control_hold_time=3.0,
    )
    (ok if ok_flag else fail)(f"move(mode='impedance') -> {ok_flag}")


def test_cartesian_streaming(robot: RBY1, cfg) -> None:
    step(7, "Cartesian streaming — right arm Z 방향 사인파 (50 tick)")
    T_right = robot.get_ee_pose("right_arm")
    T_left = robot.get_ee_pose("left_arm")
    q_torso = robot.get_joint_positions("torso")
    q_head = robot.get_joint_positions("head")

    dt = cfg.stream.dt
    n_ticks = 50
    ok(f"stream.dt={dt}s  ticks={n_ticks}  duration={dt * n_ticks:.1f}s")

    stream = robot.open_stream(mode="cartesian")
    try:
        stream.send(torso=q_torso, right_arm=T_right, left_arm=T_left, head=q_head)
        ok("initial send (reset=True 자동)")

        for i in range(1, n_ticks):
            T_new = T_right.copy()
            T_new[2, 3] += 0.03 * np.sin(i * 0.3)  # Z +-3 cm 사인파
            stream.send(right_arm=T_new)
            time.sleep(dt)

        ok(f"streaming loop 완료 ({n_ticks} ticks)")
    finally:
        stream.close()
        ok("stream.close()")


def test_stream_pause_resume(robot: RBY1, cfg) -> None:
    step(8, "Stream + blocking interleave — pause / ready_pose / resume")

    dt = cfg.stream.dt
    stop_event = threading.Event()

    cart_targets = {
        "right_arm": robot.get_ee_pose("right_arm"),
        "left_arm": robot.get_ee_pose("left_arm"),
    }
    q_torso = robot.get_joint_positions("torso")
    q_head = robot.get_joint_positions("head")
    lock = threading.Lock()

    stream = robot.open_stream(mode="cartesian")

    def stream_loop() -> None:
        while not stop_event.is_set():
            with lock:
                rt = cart_targets["right_arm"].copy()
                lt = cart_targets["left_arm"].copy()
                qt = q_torso.copy()
                qh = q_head[:2].copy()
            stream.send(torso=qt, right_arm=rt, left_arm=lt, head=qh)
            time.sleep(dt)

    t = threading.Thread(target=stream_loop, daemon=True)
    t.start()
    ok("background stream thread 시작")
    time.sleep(0.5)

    # blocking command 전: 수동 pause -> blocking -> sync -> resume
    stream.pause()
    ok("stream.pause()")
    robot.ready(minimum_time=5.0)  # auto-pause: 이미 paused -> no-op
    with lock:
        cart_targets["right_arm"] = robot.get_ee_pose("right_arm")
        cart_targets["left_arm"] = robot.get_ee_pose("left_arm")
        q_torso[:] = robot.get_joint_positions("torso")
        q_head[:] = robot.get_joint_positions("head")
    stream.resume()
    ok("sync_targets + stream.resume()  (다음 send에서 reset=True 자동)")

    time.sleep(1.0)
    stop_event.set()
    t.join(timeout=2.0)
    stream.close()
    ok("stream thread 종료 + stream.close()")


# ── main ──────────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(description="rby1-workbench wrapper 기능 테스트")
    parser.add_argument("--address", default=None, help="로봇 주소 (기본값: rby1.yaml)")
    parser.add_argument("--model",   default=None, help="로봇 모델 (기본값: rby1.yaml)")
    parser.add_argument("--config",  default=None, help="사용자 rby1.yaml 경로")
    parser.add_argument(
        "--skip-streaming",
        action="store_true",
        help="streaming 단계 건너뜀 (7, 8)",
    )
    args = parser.parse_args()

    # ── STEP 1: 설정 & 연결 ────────────────────────────────────────────────
    step(1, "설정 & 연결 (initialize)")
    cfg = load_rby1_config(args.config)
    if args.address:
        cfg.address = args.address
    if args.model:
        cfg.model = args.model
    log.info("  address=%s  model=%s", cfg.address, cfg.model)

    robot = RBY1(cfg)
    try:
        robot.initialize()
    except Exception as e:
        fail(f"initialize 실패: {e}")
        sys.exit(1)
    ok(f"connected to {cfg.address}  model={robot.model.model_name}")

    # ── STEP 2-8 ──────────────────────────────────────────────────────────
    try:
        test_state_reading(robot)
        test_zero_pose(robot)
        test_ready_pose(robot)
        test_head(robot)
        test_joint_impedance(robot)

        if not args.skip_streaming:
            test_cartesian_streaming(robot, cfg)
            test_stream_pause_resume(robot, cfg)
        else:
            log.info("streaming 단계 건너뜀 (--skip-streaming)")

    except KeyboardInterrupt:
        log.info("사용자 중단 (Ctrl-C)")
    except Exception:
        log.exception("테스트 중 예외 발생")

    log.info("=" * 60)
    log.info("테스트 완료")
    log.info("=" * 60)


if __name__ == "__main__":
    main()
