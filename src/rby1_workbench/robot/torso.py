"""Torso 제어 wrapper (robot.torso 프로퍼티로 접근)."""

from __future__ import annotations

import logging
from typing import Any

import numpy as np
import rby1_sdk as rby

log = logging.getLogger(__name__)


class TorsoController:
    """RBY1 토르소 관절 제어.

    사용 예::

        robot.torso.move_j(np.deg2rad([0, 45, -90, 45, 0, 0]), minimum_time=3.0)
        robot.torso.zero()
    """

    def __init__(self, sdk_robot: Any, model: Any):
        self._robot = sdk_robot
        self._model = model

    def move_j(self, q: np.ndarray, minimum_time: float = 3.0) -> bool:
        """토르소를 지정 관절 위치로 이동 (blocking, JointPosition).

        Args:
            q: 관절 위치 배열 [rad], shape (6,)
            minimum_time: 최소 이동 시간 [s]
        """
        rv = self._robot.send_command(
            rby.RobotCommandBuilder().set_command(
                rby.ComponentBasedCommandBuilder().set_body_command(
                    rby.BodyComponentBasedCommandBuilder().set_torso_command(
                        rby.JointPositionCommandBuilder()
                        .set_minimum_time(minimum_time)
                        .set_position(np.asarray(q, dtype=float).tolist())
                    )
                )
            ),
            1,
        ).get()
        ok = rv.finish_code == rby.RobotCommandFeedback.FinishCode.Ok
        if not ok:
            log.warning("torso.move_j finished with code: %s", rv.finish_code)
        return ok

    def zero(self, minimum_time: float = 3.0) -> bool:
        """토르소를 zero pose로 이동."""
        dof = len(self._model.torso_idx)
        return self.move_j(np.zeros(dof), minimum_time=minimum_time)
