"""Public library API for rby1-workbench."""

# Config
from rby1_workbench.config.schema import load_rby1_config, load_sam3_config

# Robot wrapper
from rby1_workbench.robot.rby1 import RBY1
from rby1_workbench.robot.stream import RBY1Stream
from rby1_workbench.robot.head import HeadController
from rby1_workbench.robot.gripper import (
    GripperController,
    GripperTCPClient,
    InspireGripperController,
    TCPGripperServer,
)

# Viz / geometry
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
    # Config
    "load_rby1_config",
    "load_sam3_config",
    # Robot wrapper
    "RBY1",
    "RBY1Stream",
    "GripperController",
    "GripperTCPClient",
    "HeadController",
    "InspireGripperController",
    "TCPGripperServer",
    # Geometry / kinematics
    "KinematicResult",
    "RobotKinematics",
    "TransformEdge",
    "TransformGraph",
    # Control
    "JointCommandClient",
    "JointCommandSettings",
    "JointGroupSpec",
    "ready_pose_targets_for_model",
    # Robot state
    "RobotStateBuffer",
    "StateSnapshot",
    "connect_robot",
    # Perception
    "OpenCVPromptVisualizer",
    "PromptBox",
    "PromptPoint",
    "RealSenseFrame",
    "RealSenseStream",
    "Sam3Prediction",
    "Sam3PromptState",
    "Sam3RealtimePredictor",
    "run_realtime_sam3_realsense",
    # Viz
    "RerunSession",
    "run_visualize_robot",
]
