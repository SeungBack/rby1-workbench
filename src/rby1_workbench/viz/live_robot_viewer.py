"""Reusable live robot viewer built on top of the rby1_workbench library API."""

from __future__ import annotations

import logging
from pathlib import Path
import time

import rerun as rr

from rby1_workbench.config.schema import VisualizeRobotConfig
from rby1_workbench.robot.client import RobotStateBuffer, connect_robot
from rby1_workbench.robot.kinematics import RobotKinematics
from rby1_workbench.viz.mesh_assets import default_link_mesh_map, discover_default_mesh_dir
from rby1_workbench.viz.rerun_session import RerunSession


def run_visualize_robot(cfg: VisualizeRobotConfig) -> None:
    """Run the live robot frame viewer until interrupted."""
    logging.info("Connecting to RB-Y1 at %s", cfg.robot.address)
    robot = connect_robot(cfg.robot)
    state_buffer = RobotStateBuffer(robot)
    kinematics = RobotKinematics(robot)
    rerun_session = RerunSession(cfg.viz)
    mesh_paths = None

    if cfg.viz.log_meshes:
        mesh_dir = None if cfg.viz.mesh_dir is None else Path(cfg.viz.mesh_dir)
        if mesh_dir is None:
            mesh_dir = discover_default_mesh_dir(cfg.robot.model)

        if mesh_dir is None:
            logging.warning("Mesh logging requested, but no mesh directory could be found.")
        else:
            mesh_paths = default_link_mesh_map(mesh_dir)
            if mesh_paths:
                logging.info(
                    "Mesh logging enabled from %s with %d assets",
                    mesh_dir,
                    len(mesh_paths),
                )
            else:
                logging.warning("Mesh logging requested, but no GLB assets were resolved in %s", mesh_dir)

    rerun_session.init()
    state_buffer.start(cfg.robot.state_update_hz)

    logging.info("Streaming robot frames at %.1f Hz", cfg.robot.state_update_hz)
    step = 0
    no_state_warning_emitted = False
    last_update_count = -1
    try:
        while True:
            snapshot = state_buffer.latest()
            if snapshot is None:
                if not no_state_warning_emitted:
                    logging.warning(
                        "No robot state received yet. Waiting for state stream..."
                    )
                    callback_error = state_buffer.callback_error()
                    if callback_error is not None:
                        logging.warning("Latest state callback error: %s", callback_error)
                    no_state_warning_emitted = True
                time.sleep(0.05)
                continue

            if no_state_warning_emitted:
                logging.info("Robot state stream is now active.")
                no_state_warning_emitted = False

            result = kinematics.compute(snapshot.position)
            rr.set_time("step", sequence=step)
            step += 1

            current_update_count = state_buffer.update_count()
            if current_update_count == last_update_count and step == 1:
                logging.info(
                    "Rendering from the initial synchronous state snapshot while waiting for streamed updates."
                )
            last_update_count = current_update_count

            rerun_session.log_transform_graph(
                result.graph,
                namespace=f"{cfg.viz.world_frame}/robot",
                frame_labels=result.frame_labels,
            )

            if mesh_paths:
                rerun_session.log_meshes(
                    namespace=f"{cfg.viz.world_frame}/robot",
                    base_transforms=result.base_transforms,
                    mesh_paths=mesh_paths,
                )

            if cfg.viz.log_skeletons:
                rerun_session.log_line_strip(
                    f"{cfg.viz.world_frame}/robot/torso_chain",
                    result.skeletons["torso"],
                    [190, 190, 190],
                )
                rerun_session.log_line_strip(
                    f"{cfg.viz.world_frame}/robot/right_arm_chain",
                    result.skeletons["right_arm"],
                    [255, 120, 120],
                )
                rerun_session.log_line_strip(
                    f"{cfg.viz.world_frame}/robot/left_arm_chain",
                    result.skeletons["left_arm"],
                    [120, 160, 255],
                )
                if cfg.viz.log_head_frames:
                    rerun_session.log_line_strip(
                        f"{cfg.viz.world_frame}/robot/head_chain",
                        result.skeletons["head"],
                        [255, 220, 120],
                    )

            if cfg.viz.log_joint_scalars:
                for joint_name in kinematics.head_joint_names:
                    rerun_session.log_scalar(
                        f"plots/head/{joint_name}",
                        result.joint_positions_by_name[joint_name],
                    )

            time.sleep(1.0 / cfg.robot.state_update_hz)
    except KeyboardInterrupt:
        logging.info("Stopping robot visualization.")
    finally:
        state_buffer.stop()
