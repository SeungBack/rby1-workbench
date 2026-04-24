"""Public library API for rby1-workbench."""

# Config
from rby1_workbench.config.schema import (
    # 새 wrapper용
    RBY1Config,
    StreamConfig,
    CartesianImpedanceStreamConfig,
    GripperConfig,
    # 기존 viz 앱용 (유지)
    JointControlConfig,
    RobotConfig,
    ViserConfig,
    ViserJointControlAppConfig,
    VisualizeRobotConfig,
    VizConfig,
)

# 새 robot wrapper
from rby1_workbench.robot.rby1 import RBY1
from rby1_workbench.robot.stream import RBY1Stream
from rby1_workbench.robot.head import HeadController
from rby1_workbench.robot.gripper import (
    GripperController,
    GripperTCPClient,
    InspireGripperController,
    TCPGripperServer,
)

# 기존 viz 시스템 (유지)
from rby1_workbench.geometry.transform_graph import TransformEdge, TransformGraph
from rby1_workbench.robot.client import RobotStateBuffer, StateSnapshot, connect_robot
from rby1_workbench.robot.kinematics import KinematicResult, RobotKinematics
from rby1_workbench.control.joint_commands import (
    JointCommandClient,
    JointCommandSettings,
    JointGroupSpec,
)
from rby1_workbench.control.presets import ready_pose_targets_for_model
from rby1_workbench.viz.live_robot_viewer import run_visualize_robot
from rby1_workbench.viz.rerun_session import RerunSession

__version__ = "0.1.0"

__all__ = [
    # 새 wrapper
    "RBY1",
    "RBY1Config",
    "RBY1Stream",
    "StreamConfig",
    "CartesianImpedanceStreamConfig",
    "GripperConfig",
    "GripperController",
    "GripperTCPClient",
    "HeadController",
    "InspireGripperController",
    "TCPGripperServer",
    # 기존
    "JointCommandClient",
    "JointCommandSettings",
    "JointControlConfig",
    "JointGroupSpec",
    "KinematicResult",
    "RobotConfig",
    "RobotKinematics",
    "RobotStateBuffer",
    "RerunSession",
    "StateSnapshot",
    "TransformEdge",
    "TransformGraph",
    "ViserConfig",
    "ViserJointControlAppConfig",
    "VisualizeRobotConfig",
    "VizConfig",
    "connect_robot",
    "ready_pose_targets_for_model",
    "run_visualize_robot",
]
