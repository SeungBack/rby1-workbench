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


@dataclass(slots=True)
class RealSenseConfig:
    serial_number: str | None = None
    color_width: int = 640
    color_height: int = 480
    depth_width: int = 640
    depth_height: int = 480
    fps: int = 30
    enable_depth: bool = True
    align_depth_to_color: bool = True
    warmup_frames: int = 15


@dataclass(slots=True)
class Sam3Config:
    checkpoint_path: str | None = None
    device: str = "auto"
    resolution: int = 1008
    confidence_threshold: float = 0.5
    interactive_multimask_output: bool = False
    enable_compile: bool = False
    enable_autocast: bool = True
    autocast_dtype: str = "bfloat16"


@dataclass(slots=True)
class OpenCVVisualizerConfig:
    window_name: str = "RealSense SAM3"
    mask_alpha: float = 0.45
    point_radius: int = 6
    box_thickness: int = 2
    show_help: bool = True
    show_depth: bool = True
    depth_hconcat: bool = True
    depth_preview_size: int = 180
    font_scale: float = 0.5
    line_height: int = 18
    wait_key_ms: int = 1
    initial_window_width: int = 1600
    initial_window_height: int = 900


@dataclass(slots=True)
class RealtimeSam3AppConfig:
    realsense: RealSenseConfig = field(default_factory=RealSenseConfig)
    sam3: Sam3Config = field(default_factory=Sam3Config)
    visualizer: OpenCVVisualizerConfig = field(default_factory=OpenCVVisualizerConfig)
    initial_text_prompt: str = ""


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


def load_realtime_sam3_config(
    cfg: DictConfig | Mapping[str, Any],
) -> RealtimeSam3AppConfig:
    """Convert a Hydra/OmegaConf config into dataclasses for the SAM3 app."""
    if isinstance(cfg, DictConfig):
        data = OmegaConf.to_container(cfg, resolve=True)
    else:
        data = dict(cfg)

    realsense_cfg = RealSenseConfig(**data.get("realsense", {}))
    sam3_cfg = Sam3Config(**data.get("sam3", {}))
    visualizer_cfg = OpenCVVisualizerConfig(**data.get("visualizer", {}))
    return RealtimeSam3AppConfig(
        realsense=realsense_cfg,
        sam3=sam3_cfg,
        visualizer=visualizer_cfg,
        initial_text_prompt=data.get("initial_text_prompt", ""),
    )


def package_root() -> Path:
    return Path(__file__).resolve().parents[3]


# ---------------------------------------------------------------------------
# RBY1 robot control config loader
# ---------------------------------------------------------------------------

def load_rby1_config(path: str | Path | None = None) -> DictConfig:
    """conf/rby1.yaml 기본값 로드. path가 주어지면 사용자 YAML을 merge (사용자 값 우선).

    Args:
        path: 사용자 정의 YAML 경로. None이면 패키지 기본값만 사용.

    Returns:
        OmegaConf DictConfig. 필드는 cfg.address, cfg.stream.dt 등으로 접근.
    """
    default = OmegaConf.load(Path(__file__).parent / "conf" / "rby1.yaml")
    if path is None:
        return default
    user = OmegaConf.load(path)
    return OmegaConf.merge(default, user)
