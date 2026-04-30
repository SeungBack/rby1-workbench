"""Reusable live robot viewer built on top of the rby1_workbench library API."""

from __future__ import annotations

import logging
from pathlib import Path
import time

import numpy as np
import rerun as rr

from omegaconf import DictConfig, OmegaConf
from rby1_workbench.perception.realsense import RealSenseFrame
from rby1_workbench.perception.shm_stream import create_camera_stream
from rby1_workbench.robot.client import RobotStateBuffer, connect_robot
from rby1_workbench.robot.kinematics import RobotKinematics
from rby1_workbench.viz.mesh_assets import default_link_mesh_map, discover_default_mesh_dir
from rby1_workbench.viz.rerun_session import RerunSession


def _log_pointcloud(
    frame: RealSenseFrame,
    K: np.ndarray,
    depth_scale: float,
    T_base_cam: np.ndarray,
    entity_path: str,
    stride: int,
    max_depth_m: float,
) -> None:
    depth_m = frame.depth.astype(np.float32) * depth_scale
    h, w = depth_m.shape
    ys = np.arange(0, h, stride)
    xs = np.arange(0, w, stride)
    yy, xx = np.meshgrid(ys, xs, indexing="ij")
    z = depth_m[yy, xx]
    mask = (z > 0.1) & (z < max_depth_m)
    z = z[mask]
    x = (xx[mask] - K[0, 2]) * z / K[0, 0]
    y = (yy[mask] - K[1, 2]) * z / K[1, 1]
    pts_cam = np.stack([x, y, z], axis=1)
    colors = frame.color_rgb[yy[mask], xx[mask]]
    pts_world = pts_cam @ T_base_cam[:3, :3].T + T_base_cam[:3, 3]
    rr.log(entity_path, rr.Points3D(pts_world, colors=colors, radii=0.005))


def run_visualize_robot(cfg: DictConfig) -> None:
    """Run the live robot frame viewer until interrupted."""
    logging.info("Connecting to RB-Y1 at %s", cfg.robot.address)
    robot = connect_robot(cfg.robot)
    state_buffer = RobotStateBuffer(robot)
    urdf_path = getattr(cfg.robot, "urdf_path", None)
    urdf_data = Path(urdf_path).read_text() if urdf_path else ""
    flip = list(OmegaConf.select(cfg.robot, "kinematics.flip_joints", default=[]))
    kinematics = RobotKinematics(robot, urdf_data=urdf_data, flip_joints=flip)
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

    calib_cfg = getattr(cfg, "calib", None)
    if calib_cfg is not None and calib_cfg.auto_load:
        from rby1_workbench.calibration.hand_eye_solver import (
            HandEyeSolver,
            resolve_calibration_output_dir,
        )
        output_dir = resolve_calibration_output_dir(calib_cfg.output_dir)
        result = HandEyeSolver.load_latest(output_dir)
        if result is not None:
            T, frame_from, frame_to = result
            kinematics.register_static_frame(frame_from, frame_to, T)
            logging.info("Calibration loaded: %s → %s", frame_from, frame_to)
        else:
            logging.info("Calibration auto-load: no file found in '%s'.", output_dir)

    cam = None
    cam_K: np.ndarray | None = None
    rs_cfg = getattr(cfg, "realsense", None)
    if getattr(cfg, "camera_source", None) is not None:
        try:
            cam = create_camera_stream(cfg)
            cam.start()
            cam_K, _ = cam.get_intrinsics()
            logging.info("Camera started (depth_scale: %.6f m/count)", cam.depth_scale)
        except Exception as e:
            logging.warning("Camera init failed, point cloud disabled: %s", e)
            cam = None

    rerun_session.init()

    if cam is not None and cam_K is not None:
        try:
            _first = cam.get_frame()
            H, W = _first.color_rgb.shape[:2]
            rr.log(
                f"{cfg.viz.world_frame}/camera",
                rr.Pinhole(image_from_camera=cam_K, width=W, height=H),
                static=True,
            )
        except Exception as e:
            logging.warning("Pinhole logging failed: %s", e)

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

            if cam is not None:
                try:
                    frame = cam.get_frame()
                    if "camera_optical" in result.base_transforms:
                        T = result.base_transforms["camera_optical"]
                        rr.log(
                            f"{cfg.viz.world_frame}/camera",
                            rr.Transform3D(translation=T[:3, 3], mat3x3=T[:3, :3]),
                        )
                    rr.log(f"{cfg.viz.world_frame}/camera/image", rr.Image(frame.color_rgb))
                    if (
                        cam_K is not None
                        and "camera_optical" in result.base_transforms
                        and frame.depth is not None
                    ):
                        _log_pointcloud(
                            frame,
                            cam_K,
                            cam.depth_scale,
                            result.base_transforms["camera_optical"],
                            f"{cfg.viz.world_frame}/points",
                            stride=int(getattr(rs_cfg, "pointcloud_stride", 4)),
                            max_depth_m=float(getattr(rs_cfg, "pointcloud_max_depth_m", 3.0)),
                        )
                except Exception as e:
                    logging.debug("Camera frame skipped: %s", e)

            time.sleep(1.0 / cfg.robot.state_update_hz)
    except KeyboardInterrupt:
        logging.info("Stopping robot visualization.")
    finally:
        state_buffer.stop()
        if cam is not None:
            cam.stop()
