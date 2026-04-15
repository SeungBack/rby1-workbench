"""Structured configuration models for rby1-workbench apps."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

from omegaconf import DictConfig, OmegaConf


@dataclass(slots=True)
class RobotConfig:
    address: str
    model: str = "m"
    power: str = ".*"
    servo: str = ".*"
    auto_power_on: bool = False
    auto_servo_on: bool = False
    auto_enable_control_manager: bool = False
    unlimited_mode_enabled: bool = False
    state_update_hz: float = 10.0


@dataclass(slots=True)
class VizConfig:
    application_id: str = "rby1_visualize_robot"
    spawn_viewer: bool = True
    world_frame: str = "world"
    arrow_length_m: float = 0.08
    log_robot_frames: bool = True
    log_head_frames: bool = True
    log_joint_scalars: bool = True
    log_skeletons: bool = True
    log_meshes: bool = False
    mesh_dir: str | None = None


@dataclass(slots=True)
class VisualizeRobotConfig:
    robot: RobotConfig
    viz: VizConfig


@dataclass(slots=True)
class ViserConfig:
    host: str = "0.0.0.0"
    port: int = 8080
    title: str = "RB-Y1 Joint Control"


@dataclass(slots=True)
class JointControlConfig:
    enable_torso: bool = False
    enable_right_arm: bool = False
    enable_left_arm: bool = False
    enable_head: bool = True
    send_on_update: bool = True
    command_rate_hz: float = 10.0
    body_mode: str = "impedance"
    body_minimum_time: float = 1.0
    head_minimum_time: float = 2.0
    control_hold_time: float = 1000000.0
    stiffness: float = 50.0
    damping_ratio: float = 1.0
    torque_limit: float = 30.0
    jog_step_small_deg: float = 1.0
    jog_step_large_deg: float = 5.0
    cartesian_linear_velocity_limit: float = 0.3
    cartesian_angular_velocity_limit: float = 3.14159
    cartesian_acceleration_limit_scaling: float = 0.8
    cartesian_minimum_time: float = 1.0
    cartesian_position_step_m: float = 0.01
    cartesian_orientation_step_deg: float = 5.0


@dataclass(slots=True)
class ControlVizConfig:
    """Optional Rerun visualization companion for the joint control panel."""

    enable: bool = False
    application_id: str = "rby1_joint_control"
    spawn_viewer: bool = True
    world_frame: str = "world"
    arrow_length_m: float = 0.08
    log_skeletons: bool = True
    state_update_hz: float = 10.0


@dataclass(slots=True)
class ViserJointControlAppConfig:
    robot: RobotConfig
    viser: ViserConfig
    command: JointControlConfig
    viz: ControlVizConfig = field(default_factory=ControlVizConfig)


def load_visualize_robot_config(cfg: DictConfig | Mapping[str, Any]) -> VisualizeRobotConfig:
    """Convert a Hydra/OmegaConf config into dataclasses."""
    if isinstance(cfg, DictConfig):
        data = OmegaConf.to_container(cfg, resolve=True)
    else:
        data = dict(cfg)

    robot_cfg = RobotConfig(**data["robot"])
    viz_cfg = VizConfig(**data["viz"])
    return VisualizeRobotConfig(robot=robot_cfg, viz=viz_cfg)


def load_viser_joint_control_config(
    cfg: DictConfig | Mapping[str, Any],
) -> ViserJointControlAppConfig:
    """Convert a Hydra/OmegaConf config into dataclasses for the viser panel."""
    if isinstance(cfg, DictConfig):
        data = OmegaConf.to_container(cfg, resolve=True)
    else:
        data = dict(cfg)

    robot_cfg = RobotConfig(**data["robot"])
    viser_cfg = ViserConfig(**data["viser"])
    command_cfg = JointControlConfig(**data["command"])
    viz_cfg = ControlVizConfig(**data["viz"]) if "viz" in data else ControlVizConfig()
    return ViserJointControlAppConfig(
        robot=robot_cfg,
        viser=viser_cfg,
        command=command_cfg,
        viz=viz_cfg,
    )


def package_root() -> Path:
    return Path(__file__).resolve().parents[3]
