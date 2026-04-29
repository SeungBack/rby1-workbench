"""Torso 제어 wrapper (robot.torso 프로퍼티로 접근)."""

from __future__ import annotations

import logging
from typing import Any, Literal

import numpy as np
import rby1_sdk as rby

log = logging.getLogger(__name__)


class TorsoController:
    """RBY1 토르소 관절 제어.

    사용 예::

        robot.torso.move_j(np.deg2rad([0, 45, -90, 45, 0, 0]), minimum_time=3.0)
        robot.torso.move_j(q, mode="position")
        robot.torso.zero()
    """

    def __init__(self, sdk_robot: Any, model: Any):
        self._robot = sdk_robot
        self._model = model

    def move_j(
        self,
        q: np.ndarray,
        minimum_time: float = 3.0,
        control_hold_time: float = 3.0,
        mode: Literal["impedance", "position"] = "impedance",
        stiffness: float = 100.0,
        damping_ratio: float = 1.0,
        torque_limit: float = 30.0,
    ) -> bool:
        """토르소를 지정 관절 위치로 이동 (blocking).

        Args:
            q: 관절 위치 배열 [rad], shape (6,), torso_0 ~ torso_5 순서
            minimum_time: 최소 이동 시간 [s]
            control_hold_time: target 도달 대기 시간 [s]
            mode: "impedance" (default) — JointImpedanceControl,
                  "position" — JointPosition
            stiffness: 임피던스 강성 [N·m/rad] (impedance 모드 전용)
            damping_ratio: 댐핑 비율 (impedance 모드 전용)
            torque_limit: 토크 한계 [N·m] (impedance 모드 전용)
        """
        q_list = np.asarray(q, dtype=float).tolist()
        dof = len(q_list)
        header = rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time)

        if mode == "impedance":
            torso_cmd = (
                rby.JointImpedanceControlCommandBuilder()
                .set_command_header(header)
                .set_minimum_time(minimum_time)
                .set_position(q_list)
                .set_stiffness([stiffness] * dof)
                .set_damping_ratio(damping_ratio)
                .set_torque_limit([torque_limit] * dof)
            )
        else:
            torso_cmd = (
                rby.JointPositionCommandBuilder()
                .set_command_header(header)
                .set_minimum_time(minimum_time)
                .set_position(q_list)
            )

        rv = self._robot.send_command(
            rby.RobotCommandBuilder().set_command(
                rby.ComponentBasedCommandBuilder().set_body_command(
                    rby.BodyComponentBasedCommandBuilder().set_torso_command(torso_cmd)
                )
            ),
            1,
        ).get()
        ok = rv.finish_code == rby.RobotCommandFeedback.FinishCode.Ok
        if not ok:
            log.warning("torso.move_j finished with code: %s", rv.finish_code)
        return ok

    def zero(self, minimum_time: float = 3.0, **kwargs) -> bool:
        """토르소를 zero pose로 이동."""
        dof = len(self._model.torso_idx)
        return self.move_j(np.zeros(dof), minimum_time=minimum_time, **kwargs)
