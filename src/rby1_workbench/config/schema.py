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


# ---------------------------------------------------------------------------
# RBY1 robot control config (새 wrapper용)
# ---------------------------------------------------------------------------

@dataclass
class CartesianImpedanceStreamConfig:
    """CartesianImpedanceControlCommandBuilder 파라미터. VR teleop 기본값 사용."""
    # 팔 (7-DOF)
    arm_stiffness: list = field(default_factory=lambda: [80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 40.0])
    arm_torque_limit: list = field(default_factory=lambda: [30.0] * 7)
    arm_linear_velocity_limit: float = 2.0
    arm_angular_velocity_limit: float = 6.2831853  # 2*pi
    arm_linear_accel_limit: float = 20.0
    arm_angular_accel_limit: float = 251.327412   # 80*pi
    # 토르소 (6-DOF)
    torso_stiffness: list = field(default_factory=lambda: [400.0] * 6)
    torso_torque_limit: list = field(default_factory=lambda: [500.0] * 6)
    torso_linear_velocity_limit: float = 1.0
    torso_angular_velocity_limit: float = 1.5707963  # pi/2
    torso_linear_accel_limit: float = 10.0
    torso_angular_accel_limit: float = 62.8318530   # 20*pi
    # 관절 리밋 (joint name → [lower, upper])
    joint_limits: dict = field(default_factory=lambda: {
        "right_arm_3": [-2.6, -0.5],
        "right_arm_5": [0.2, 1.9],
        "left_arm_3": [-2.6, -0.5],
        "left_arm_5": [0.2, 1.9],
        "torso_1": [-0.523598776, 1.3],
        "torso_2": [-2.617993878, -0.2],
    })


@dataclass
class StreamConfig:
    """create_stream() 동작 설정."""
    dt: float = 0.1  # 제어 주기 [s]; control_hold_time = dt*10, minimum_time = dt*1.01
    torso_mode: str = "joint_position"        # "joint_position" | "cartesian_impedance"
    right_arm_mode: str = "cartesian_impedance"
    left_arm_mode: str = "cartesian_impedance"


@dataclass
class GripperConfig:
    enabled: bool = False
    tool_flange_voltage: int = 12
    tcp_host: str = "0.0.0.0"   # TCPGripperServer bind 주소
    tcp_port: int = 5000


@dataclass
class RBY1Config:
    """RBY1 로봇 전체 설정."""
    address: str = "192.168.30.1:50051"
    model: str = "a"
    power_pattern: str = ".*"
    servo_pattern: str = ".*"
    unlimited_mode: bool = True
    state_update_hz: float = 100.0
    stream: StreamConfig = field(default_factory=StreamConfig)
    cartesian_impedance: CartesianImpedanceStreamConfig = field(
        default_factory=CartesianImpedanceStreamConfig
    )
    gripper: GripperConfig = field(default_factory=GripperConfig)

    @classmethod
    def from_dict(cls, d: dict) -> "RBY1Config":
        d = dict(d)
        stream = StreamConfig(**d.pop("stream", {}))
        ci_raw = d.pop("cartesian_impedance", {})
        ci = CartesianImpedanceStreamConfig(**ci_raw)
        gripper = GripperConfig(**d.pop("gripper", {}))
        return cls(stream=stream, cartesian_impedance=ci, gripper=gripper, **d)

    @classmethod
    def from_yaml(cls, path: str) -> "RBY1Config":
        import yaml
        with open(path) as f:
            data = yaml.safe_load(f)
        return cls.from_dict(data or {})

    @classmethod
    def from_hydra(cls, cfg: Any) -> "RBY1Config":
        """Hydra DictConfig → RBY1Config 변환."""
        if isinstance(cfg, DictConfig):
            data = OmegaConf.to_container(cfg, resolve=True)
        else:
            data = dict(cfg)
        return cls.from_dict(data)

    def __repr__(self) -> str:
        return (
            f"RBY1Config(address={self.address!r}, model={self.model!r}, "
            f"unlimited_mode={self.unlimited_mode}, "
            f"stream=StreamConfig(dt={self.stream.dt}, "
            f"torso={self.stream.torso_mode!r}, "
            f"right_arm={self.stream.right_arm_mode!r}, "
            f"left_arm={self.stream.left_arm_mode!r}), "
            f"gripper=GripperConfig(enabled={self.gripper.enabled}, "
            f"tcp={self.gripper.tcp_host}:{self.gripper.tcp_port}))"
        )
