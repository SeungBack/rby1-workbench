"""Head 제어 wrapper (robot.head 프로퍼티로 접근)."""

from __future__ import annotations

import logging
from typing import Any

import numpy as np
import rby1_sdk as rby

log = logging.getLogger(__name__)


class HeadController:
    """RBY1 헤드 관절 제어.

    사용 예::

        robot.head.move_j(np.array([0.2, -0.1]), minimum_time=2.0)
        robot.head.zero()

        # EE 중점을 바라보는 yaw/pitch 계산
        yaw, pitch = HeadController.look_at_midpoint(T_right[:3,3], T_left[:3,3])
        robot.head.move_j(np.array([yaw, pitch]))
    """

    def __init__(self, sdk_robot: Any, model: Any):
        self._robot = sdk_robot
        self._model = model

    def move_j(
        self,
        q: np.ndarray,
        minimum_time: float = 1.0,
        control_hold_time: float = 1.0,
    ) -> bool:
        """헤드를 지정 관절 위치로 이동 (blocking, JointPosition).

        Args:
            q: [yaw, pitch] (rad)
            minimum_time: 최소 이동 시간 [s]
            control_hold_time: target 도달 대기 시간 [s]
        """
        q = np.asarray(q, dtype=float)
        dof = len(self._model.head_idx)
        positions = [0.0] * dof
        if dof >= 1:
            positions[0] = float(q[0])
        if dof >= 2:
            positions[1] = float(q[1])

        header = rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time)
        rv = self._robot.send_command(
            rby.RobotCommandBuilder().set_command(
                rby.ComponentBasedCommandBuilder().set_head_command(
                    rby.JointPositionCommandBuilder()
                    .set_command_header(header)
                    .set_minimum_time(minimum_time)
                    .set_position(positions)
                )
            ),
            1,
        ).get()
        ok = rv.finish_code == rby.RobotCommandFeedback.FinishCode.Ok
        if not ok:
            log.warning("head.move_j finished with code: %s", rv.finish_code)
        return ok

    def zero(self, minimum_time: float = 2.0) -> bool:
        """헤드를 zero pose로 이동."""
        return self.move_j(np.zeros(2), minimum_time=minimum_time)

    @staticmethod
    def look_at_midpoint(
        right_ee_pos: np.ndarray,
        left_ee_pos: np.ndarray,
    ) -> tuple[float, float]:
        """양 EE 중점을 바라보는 (yaw, pitch) 계산.

        Returns:
            (yaw_rad, pitch_rad) — 클리핑 포함.
        """
        center = (np.asarray(right_ee_pos) + np.asarray(left_ee_pos)) / 2.0
        yaw = float(np.arctan2(center[1], center[0]))
        pitch = float(np.arctan2(-center[2], center[0])) - np.deg2rad(10.0)
        yaw = float(np.clip(yaw, -np.deg2rad(29.0), np.deg2rad(29.0)))
        pitch = float(np.clip(pitch, -np.deg2rad(19.0), np.deg2rad(89.0)))
        return yaw, pitch
