"""RBY1 실시간 제어 스트림.

VR teleop main.py의 제어 루프를 일반화한 wrapper.
매 tick에 send()를 호출하면 모든 컴포넌트를 하나의 SDK command로 묶어 전송.
None인 컴포넌트는 마지막으로 보낸 값으로 자동 hold.
"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np
import rby1_sdk as rby

from rby1_workbench.config.schema import RBY1Config

log = logging.getLogger(__name__)


class RBY1Stream:
    """실시간 whole-body 제어 스트림.

    사용 예::

        stream = robot.create_stream()

        # 첫 tick: 초기 포즈 전부 지정
        stream.send(
            torso=q_torso,
            right_arm=T_right,   # 4×4 SE3 (cartesian_impedance 모드)
            left_arm=T_left,
            head=(yaw, pitch),
            reset=True,          # CartesianImpedance reset_reference
        )

        # 이후: 바뀐 것만 — 나머지는 마지막 값으로 자동 hold
        stream.send(right_arm=T_right_new)

        stream.cancel()

    context manager::

        with robot.create_stream() as stream:
            stream.send(...)
    """

    def __init__(self, sdk_robot: Any, cfg: RBY1Config):
        self._robot = sdk_robot
        self._cfg = cfg
        self._stream = sdk_robot.create_command_stream()
        self._last: dict[str, Any] = {}   # component → 마지막 전송값

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def send(
        self,
        torso: np.ndarray | None = None,
        right_arm: np.ndarray | None = None,
        left_arm: np.ndarray | None = None,
        head: tuple[float, float] | None = None,
        reset: bool = False,
    ) -> None:
        """전체 컴포넌트를 하나의 command로 묶어 스트림 전송.

        Args:
            torso: 관절 위치 배열 (torso_mode=joint_position) 또는 4×4 SE3 (cartesian_impedance).
            right_arm: 4×4 SE3 행렬 (cartesian_impedance) 또는 관절 위치 배열.
            left_arm: 동상.
            head: (yaw_rad, pitch_rad) 튜플.
            reset: CartesianImpedanceControlCommandBuilder.set_reset_reference() 플래그.
                   팔이 새로 tracking 시작할 때 True로 설정하면 시작 시 튐 방지.
        """
        # None이면 last로 채움
        torso     = torso     if torso     is not None else self._last.get("torso")
        right_arm = right_arm if right_arm is not None else self._last.get("right_arm")
        left_arm  = left_arm  if left_arm  is not None else self._last.get("left_arm")
        head      = head      if head      is not None else self._last.get("head")

        cmd = self._build(torso, right_arm, left_arm, head, reset)
        if cmd is None:
            return

        self._stream.send_command(cmd)

        # last 업데이트
        if torso is not None:
            self._last["torso"] = torso
        if right_arm is not None:
            self._last["right_arm"] = right_arm
        if left_arm is not None:
            self._last["left_arm"] = left_arm
        if head is not None:
            self._last["head"] = head

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
            cbc.set_head_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(rby.CommandHeaderBuilder().set_control_hold_time(hold_time))
                .set_position([float(head[0]), float(head[1])])
                .set_minimum_time(min_time)
            )

        if not has_body:
            return rby.RobotCommandBuilder().set_command(cbc)

        # ---- body ----
        body = rby.BodyComponentBasedCommandBuilder()

        # torso
        if torso is not None:
            body.set_torso_command(
                self._build_torso(torso, hold_time, min_time, reset)
            )

        # arms
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
