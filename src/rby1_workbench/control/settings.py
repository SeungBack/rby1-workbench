"""Command settings dataclasses for RB-Y1 control."""

from __future__ import annotations

from dataclasses import dataclass

from rby1_workbench.config.schema import JointControlConfig


@dataclass(slots=True)
class JointCommandSettings:
    body_mode: str = "impedance"
    body_minimum_time: float = 5.0
    head_minimum_time: float = 2.0
    control_hold_time: float = 1000000.0
    stiffness: float = 50.0
    damping_ratio: float = 1.0
    torque_limit: float = 30.0

    @classmethod
    def from_control_config(cls, cfg: JointControlConfig) -> "JointCommandSettings":
        return cls(
            body_mode=cfg.body_mode,
            body_minimum_time=cfg.body_minimum_time,
            head_minimum_time=cfg.head_minimum_time,
            control_hold_time=cfg.control_hold_time,
            stiffness=cfg.stiffness,
            damping_ratio=cfg.damping_ratio,
            torque_limit=cfg.torque_limit,
        )


@dataclass(slots=True)
class CartesianCommandSettings:
    linear_velocity_limit: float = 0.3
    angular_velocity_limit: float = 100.0
    acceleration_limit_scaling: float = 0.8
    minimum_time: float = 0.0
    control_hold_time: float = 1000000.0
    stop_position_tracking_error: float = 1e-3
    stop_orientation_tracking_error: float = 1e-4
