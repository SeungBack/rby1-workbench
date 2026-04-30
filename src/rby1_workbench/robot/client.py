"""Robot connection and state buffering helpers."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import logging
import threading
import time
from typing import Any

import numpy as np
import rby1_sdk as rby

from typing import Any


@dataclass(slots=True)
class StateSnapshot:
    timestamp: float
    position: np.ndarray


def connect_robot(cfg: Any) -> Any:
    """Connect to the robot, optionally performing power/servo/control setup."""
    robot = rby.create_robot(cfg.address, cfg.model)
    if not robot.connect():
        raise RuntimeError(f"Failed to connect to robot at {cfg.address}")

    if cfg.auto_power_on and not robot.is_power_on(cfg.power):
        if not robot.power_on(cfg.power):
            raise RuntimeError(f"Failed to power on devices matching '{cfg.power}'")

    if cfg.auto_servo_on and not robot.is_servo_on(cfg.servo):
        if not robot.servo_on(cfg.servo):
            raise RuntimeError(f"Failed to servo on devices matching '{cfg.servo}'")

    if cfg.auto_enable_control_manager:
        control_manager_state = robot.get_control_manager_state().state
        if control_manager_state in (
            rby.ControlManagerState.State.MajorFault,
            rby.ControlManagerState.State.MinorFault,
        ):
            if not robot.reset_fault_control_manager():
                raise RuntimeError("Failed to reset control manager fault state")

        if not robot.enable_control_manager(
            unlimited_mode_enabled=cfg.unlimited_mode_enabled
        ):
            raise RuntimeError("Failed to enable control manager")

    return robot


class RobotStateBuffer:
    """Thread-safe buffer for the latest robot joint state."""

    def __init__(self, robot: Any):
        self._robot = robot
        self._lock = threading.Lock()
        self._latest: StateSnapshot | None = None
        self._update_count = 0
        self._callback_error: Exception | None = None

    def start(self, rate_hz: float) -> None:
        # Seed one synchronous state so the viewer can render even if the
        # background callback is delayed or temporarily unavailable.
        try:
            state = self._robot.get_state()
            self._store_state(state)
        except Exception as exc:
            logging.warning("Failed to fetch initial robot state: %s", exc)

        self._robot.start_state_update(self._callback, rate_hz)

    def stop(self) -> None:
        self._robot.stop_state_update()

    def latest(self) -> StateSnapshot | None:
        with self._lock:
            if self._latest is None:
                return None
            return StateSnapshot(
                timestamp=self._latest.timestamp,
                position=self._latest.position.copy(),
            )

    def update_count(self) -> int:
        with self._lock:
            return self._update_count

    def callback_error(self) -> Exception | None:
        with self._lock:
            return self._callback_error

    def _callback(self, state: Any) -> None:
        try:
            self._store_state(state)
        except Exception as exc:
            with self._lock:
                self._callback_error = exc
            logging.exception("Robot state callback failed")

    def _store_state(self, state: Any) -> None:
        raw_timestamp = getattr(state, "timestamp", None)
        if raw_timestamp is None:
            timestamp = time.time()
        elif isinstance(raw_timestamp, datetime):
            timestamp = float(raw_timestamp.timestamp())
        else:
            try:
                timestamp = float(raw_timestamp)
            except (TypeError, ValueError):
                timestamp = time.time()

        position = np.asarray(state.position, dtype=np.float64).copy()
        with self._lock:
            self._latest = StateSnapshot(timestamp=timestamp, position=position)
            self._update_count += 1
