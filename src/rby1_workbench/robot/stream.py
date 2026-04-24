"""RBY1 실시간 제어 스트림.

VR teleop main.py의 제어 루프를 일반화한 wrapper.
매 tick에 send()를 호출하면 모든 컴포넌트를 하나의 SDK command로 묶어 전송.
None인 컴포넌트는 마지막으로 보낸 값으로 자동 hold.
"""

from __future__ import annotations

import logging
import threading
from typing import Any

import numpy as np
import rby1_sdk as rby

from rby1_workbench.config.schema import RBY1Config

log = logging.getLogger(__name__)


class RBY1Stream:
    """실시간 whole-body 제어 스트림.

    사용 예::

        stream = robot.create_stream()

        # 첫 send는 reset=True 자동 적용 (CartesianImpedance 시작 시 튐 방지)
        stream.send(
            torso=q_torso,
            right_arm=T_right,
            left_arm=T_left,
            head=np.array([yaw, pitch]),
        )

        # 이후: 바뀐 것만 — 나머지는 마지막 값으로 자동 hold
        stream.send(right_arm=T_right_new)

        # blocking command 전후 일시정지 / 재개
        stream.pause()
        robot.movej(...)          # SDK 스트림과 충돌 없이 실행됨
        stream.resume()           # 다음 send부터 reset=True 자동 재적용

        stream.cancel()

    context manager::

        with robot.create_stream() as stream:
            stream.send(...)
    """

    def __init__(self, sdk_robot: Any, cfg: RBY1Config):
        self._robot = sdk_robot
        self._cfg = cfg
        self._stream = sdk_robot.create_command_stream()
        self._last: dict[str, Any] = {}
        self._paused = False
        self._first_send = True   # True → 다음 send에서 reset=True 자동 적용
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def send(
        self,
        torso: np.ndarray | None = None,
        right_arm: np.ndarray | None = None,
        left_arm: np.ndarray | None = None,
        head: np.ndarray | tuple[float, float] | None = None,
        reset: bool | None = None,
    ) -> None:
        """전체 컴포넌트를 하나의 command로 묶어 스트림 전송.

        Args:
            torso: 관절 위치 배열 또는 4×4 SE3 (torso_mode 설정에 따라).
            right_arm: 4×4 SE3 또는 관절 위치 배열.
            left_arm: 동상.
            head: [yaw_rad, pitch_rad] — ndarray 또는 (yaw, pitch) 튜플.
            reset: CartesianImpedance reset_reference 플래그.
                   None(기본)이면 스트림 시작/재개 후 첫 번째 send에서만 True 자동 적용.
        """
        with self._lock:
            if self._paused:
                return

            _reset = self._first_send if reset is None else reset

            # None이면 last로 채움
            torso     = torso     if torso     is not None else self._last.get("torso")
            right_arm = right_arm if right_arm is not None else self._last.get("right_arm")
            left_arm  = left_arm  if left_arm  is not None else self._last.get("left_arm")
            head      = head      if head      is not None else self._last.get("head")

            cmd = self._build(torso, right_arm, left_arm, head, _reset)
            if cmd is None:
                return

            self._stream.send_command(cmd)
            self._first_send = False

            # last 업데이트
            if torso is not None:
                self._last["torso"] = torso
            if right_arm is not None:
                self._last["right_arm"] = right_arm
            if left_arm is not None:
                self._last["left_arm"] = left_arm
            if head is not None:
                self._last["head"] = head

    def pause(self) -> None:
        """스트림 전송 일시정지.

        진행 중인 send()가 완료될 때까지 대기한 후 중단.
        이미 paused 상태라면 no-op.
        """
        with self._lock:
            self._paused = True

    def resume(self) -> None:
        """스트림 전송 재개.

        다음 send()부터 실제로 전송되며, reset=True가 자동 적용됨.
        이를 통해 blocking command 후 포즈가 달라진 상태에서도
        CartesianImpedance가 튀지 않고 부드럽게 재개됨.
        """
        with self._lock:
            self._paused = False
            self._first_send = True   # 재개 후 첫 send에서 reset_reference

    @property
    def paused(self) -> bool:
        """현재 일시정지 상태 여부."""
        return self._paused

    def cancel(self) -> None:
        """스트림 취소."""
        try:
            self._stream.cancel()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "RBY1Stream":
        return self

    def __exit__(self, *_: Any) -> None:
        self.cancel()

    # ------------------------------------------------------------------
    # Internal: command builder
    # ------------------------------------------------------------------

    def _build(
        self,
        torso: Any,
        right_arm: Any,
        left_arm: Any,
        head: Any,
        reset: bool,
    ) -> Any | None:
        cfg = self._cfg
        dt = cfg.stream.dt
        hold_time = dt * 10.0
        min_time = dt * 1.01

        has_body = any(v is not None for v in (torso, right_arm, left_arm))
        has_head = head is not None
        if not has_body and not has_head:
            return None

        cbc = rby.ComponentBasedCommandBuilder()

        # ---- head (항상 JointPosition) ----
        if has_head:
            h = np.asarray(head, dtype=float)
            cbc.set_head_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(hold_time))
                .set_position([float(h[0]), float(h[1])])
                .set_minimum_time(min_time)
            )

        if not has_body:
            return rby.RobotCommandBuilder().set_command(cbc)

        # ---- body ----
        body = rby.BodyComponentBasedCommandBuilder()

        if torso is not None:
            body.set_torso_command(
                self._build_torso(torso, hold_time, min_time, reset)
            )

        for component, T_or_q in (("right_arm", right_arm), ("left_arm", left_arm)):
            if T_or_q is None:
                continue
            mode = getattr(cfg.stream, f"{component}_mode")
            builder = self._build_arm(component, T_or_q, mode, hold_time, min_time, reset)
            getattr(body, f"set_{component}_command")(builder)

        cbc.set_body_command(body)
        return rby.RobotCommandBuilder().set_command(cbc)

    def _build_torso(
        self, q_or_T: Any, hold_time: float, min_time: float, reset: bool
    ) -> Any:
        mode = self._cfg.stream.torso_mode
        ci = self._cfg.cartesian_impedance
        header = rby.CommandHeaderBuilder().set_control_hold_time(hold_time)

        if mode == "joint_position":
            return (
                rby.JointPositionCommandBuilder()
                .set_command_header(header)
                .set_position(np.asarray(q_or_T, dtype=float).tolist())
                .set_minimum_time(min_time)
            )

        # cartesian_impedance (VR teleop torso 방식)
        T = np.asarray(q_or_T, dtype=float)
        builder = (
            rby.CartesianImpedanceControlCommandBuilder()
            .set_command_header(header)
            .set_minimum_time(min_time)
            .set_joint_stiffness(ci.torso_stiffness)
            .set_joint_torque_limit(ci.torso_torque_limit)
            .set_stop_joint_position_tracking_error(0)
            .set_stop_orientation_tracking_error(0)
            .set_reset_reference(reset)
        )
        for jname, (lo, hi) in ci.joint_limits.items():
            if jname.startswith("torso"):
                builder.add_joint_limit(jname, lo, hi)
        builder.add_target(
            "base", "link_torso_5", T,
            ci.torso_linear_velocity_limit,
            ci.torso_angular_velocity_limit,
            ci.torso_linear_accel_limit,
            ci.torso_angular_accel_limit,
        )
        return builder

    def _build_arm(
        self,
        component: str,
        T_or_q: Any,
        mode: str,
        hold_time: float,
        min_time: float,
        reset: bool,
    ) -> Any:
        ci = self._cfg.cartesian_impedance
        header = rby.CommandHeaderBuilder().set_control_hold_time(hold_time)
        frame = "link_right_arm_6" if component == "right_arm" else "link_left_arm_6"

        if mode == "joint_position":
            return (
                rby.JointPositionCommandBuilder()
                .set_command_header(header)
                .set_position(np.asarray(T_or_q, dtype=float).tolist())
                .set_minimum_time(min_time)
            )

        if mode == "cartesian_impedance":
            T = np.asarray(T_or_q, dtype=float)
            builder = (
                rby.CartesianImpedanceControlCommandBuilder()
                .set_command_header(header)
                .set_minimum_time(min_time)
                .set_joint_stiffness(ci.arm_stiffness)
                .set_joint_torque_limit(ci.arm_torque_limit)
                .set_stop_joint_position_tracking_error(0)
                .set_stop_orientation_tracking_error(0)
                .set_reset_reference(reset)
            )
            for jname, (lo, hi) in ci.joint_limits.items():
                if jname.startswith(component):
                    builder.add_joint_limit(jname, lo, hi)
            builder.add_target(
                "base", frame, T,
                ci.arm_linear_velocity_limit,
                ci.arm_angular_velocity_limit,
                ci.arm_linear_accel_limit,
                ci.arm_angular_accel_limit,
            )
            return builder

        raise ValueError(f"Unknown arm mode: '{mode}'")
