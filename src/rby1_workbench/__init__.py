"""Public library API for rby1-workbench."""

# Config
from rby1_workbench.config.schema import (
    # 새 wrapper용
    load_rby1_config,
    # 기존 viz 앱용 (유지)
    OpenCVVisualizerConfig,
    RealSenseConfig,
    RealtimeSam3AppConfig,
    Sam3Config,
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
from rby1_workbench.perception.realtime_segmentation import run_realtime_sam3_realsense
from rby1_workbench.perception.realsense import RealSenseFrame, RealSenseStream
from rby1_workbench.perception.sam3 import (
    PromptBox,
    PromptPoint,
    Sam3Prediction,
    Sam3PromptState,
    Sam3RealtimePredictor,
)
from rby1_workbench.perception.visualizer import OpenCVPromptVisualizer
from rby1_workbench.viz.live_robot_viewer import run_visualize_robot
from rby1_workbench.viz.rerun_session import RerunSession

__version__ = "0.1.0"

__all__ = [
    # 새 wrapper
    "RBY1",
    "RBY1Stream",
    "load_rby1_config",
    "OpenCVPromptVisualizer",
    "OpenCVVisualizerConfig",
    "PromptBox",
    "PromptPoint",
    "RealSenseConfig",
    "RealSenseFrame",
    "RealSenseStream",
    "RealtimeSam3AppConfig",
    "Sam3Config",
    "Sam3Prediction",
    "Sam3PromptState",
    "Sam3RealtimePredictor",
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
    "run_realtime_sam3_realsense",
    "run_visualize_robot",
]
