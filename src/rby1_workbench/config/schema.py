"""Structured configuration models for rby1-workbench apps."""

from __future__ import annotations

from dataclasses import dataclass
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


def load_visualize_robot_config(cfg: DictConfig | Mapping[str, Any]) -> VisualizeRobotConfig:
    """Convert a Hydra/OmegaConf config into dataclasses."""
    if isinstance(cfg, DictConfig):
        data = OmegaConf.to_container(cfg, resolve=True)
    else:
        data = dict(cfg)

    robot_cfg = RobotConfig(**data["robot"])
    viz_cfg = VizConfig(**data["viz"])
    return VisualizeRobotConfig(robot=robot_cfg, viz=viz_cfg)


def package_root() -> Path:
    return Path(__file__).resolve().parents[3]
