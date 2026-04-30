"""Realtime FoundGrasp grasp inference on RealSense frames."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import os
import sys
import threading

import cv2
import numpy as np
import rerun as rr
import torch

from rby1_workbench import RBY1, load_rby1_config
from rby1_workbench.control.grasp_execution import (
    FoundGraspCandidate,
    LeftArmGraspExecutor,
)
from rby1_workbench.perception.shm_stream import create_camera_stream

log = logging.getLogger(__name__)


def _ensure_path(root: str) -> None:
    if root not in sys.path:
        sys.path.insert(0, root)


def _load_model(cfg):
    root = cfg.found_grasp.foundgrasp_root
    _ensure_path(root)

    from utils.config import load_config_with_base
    from model.found_grasp import FoundGrasp

    config_abs = os.path.join(root, "configs", cfg.found_grasp.config_path)
    if not config_abs.endswith(".yaml"):
        config_abs += ".yaml"
    model_cfg = load_config_with_base(config_abs)

    model = FoundGrasp(model_cfg.model)
    ckpt = torch.load(cfg.found_grasp.checkpoint_path, map_location=cfg.found_grasp.device)
    model.load_state_dict(ckpt.get("model_state_dict", ckpt))
    model = model.to(cfg.found_grasp.device)
    model.eval()
    return model, model_cfg


def _depth_to_xyz(depth_m: np.ndarray, K: np.ndarray) -> np.ndarray:
    H, W = depth_m.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - K[0, 2]) * depth_m / K[0, 0]
    Y = (v - K[1, 2]) * depth_m / K[1, 1]
    return np.stack([X, Y, depth_m], axis=-1)


def _preprocess_color(
    color_rgb: np.ndarray, target_size: tuple[int, int]
) -> tuple[torch.Tensor, torch.Tensor]:
    """Returns (pixel_values [1,3,H,W] normalized, og_color [1,3,H,W] 0-1)."""
    H, W = target_size
    resized = cv2.resize(color_rgb, (W, H), interpolation=cv2.INTER_LINEAR)
    t = torch.from_numpy(resized).permute(2, 0, 1).float()
    mean = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
    return (t / 255.0 - mean) / std, t / 255.0


@dataclass(slots=True)
class _RuntimeState:
    lock: threading.Lock
    candidates: list[FoundGraspCandidate]
    selected_index: int
    status_msg: str


def _grasp_to_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    T[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return T


def _extract_candidates(gg) -> list[FoundGraspCandidate]:
    scores = getattr(gg, "scores", None)
    widths = getattr(gg, "widths", None)
    heights = getattr(gg, "heights", None)
    depths = getattr(gg, "depths", None)
    fallback = getattr(gg, "grasp_group_array", None)

    candidates: list[FoundGraspCandidate] = []
    for index in range(len(gg)):
        score = float(scores[index]) if scores is not None else float(fallback[index, 0])
        width = float(widths[index]) if widths is not None else float(fallback[index, 1])
        height = float(heights[index]) if heights is not None else float(fallback[index, 2])
        depth = float(depths[index]) if depths is not None else float(fallback[index, 3])
        candidates.append(
            FoundGraspCandidate(
                index=index,
                score=score,
                width_m=width,
                height_m=height,
                depth_m=depth,
                camera_opticalTgrasp=_grasp_to_transform(
                    gg.rotation_matrices[index],
                    gg.translations[index],
                ),
            )
        )
    return candidates


def _build_executor(cfg) -> LeftArmGraspExecutor | None:
    mode = str(cfg.execution.mode).lower().strip()
    if mode == "off":
        return None

    robot_cfg = load_rby1_config()
    robot_cfg.address = cfg.robot.address
    robot_cfg.model = cfg.robot.model
    backend = str(cfg.robot.backend).lower().strip()
    endpoint = None if cfg.robot.endpoint is None else str(cfg.robot.endpoint)
    robot = RBY1(robot_cfg, backend=backend, endpoint=endpoint)
    needs_motion = bool(cfg.robot.move_to_startup_pose) or mode == "execute"
    if needs_motion:
        robot.initialize()
        if bool(cfg.robot.move_to_startup_pose):
            _move_robot_to_startup_pose(robot, cfg.robot.startup_pose)
    else:
        robot.connect()
    log.info("FoundGrasp execution robot ready (mode=%s, backend=%s)", mode, backend)
    return LeftArmGraspExecutor(robot, cfg.execution)


def _move_robot_to_startup_pose(robot: RBY1, startup_pose_cfg) -> None:
    if hasattr(startup_pose_cfg, "keys"):
        mode = str(startup_pose_cfg.mode).lower().strip()
        minimum_time = float(startup_pose_cfg.minimum_time)
    else:
        mode = str(startup_pose_cfg).lower().strip()
        minimum_time = 5.0

    if mode == "ready":
        ok = robot.ready(minimum_time=minimum_time)
    elif mode == "zero":
        ok = robot.zero(minimum_time=minimum_time)
    elif mode == "joint":
        kwargs = {"mode": "joint", "minimum_time": minimum_time}
        for component in ("torso", "right_arm", "left_arm", "head"):
            target = getattr(startup_pose_cfg, component, None)
            if target is not None:
                kwargs[component] = np.asarray(target, dtype=np.float64)
        if len(kwargs) == 2:
            raise ValueError("startup_pose.mode='joint' requires at least one component target")
        ok = robot.move(**kwargs)
    else:
        raise ValueError(f"Unsupported robot.startup_pose.mode: {mode}")

    if not ok:
        raise RuntimeError(f"Failed to move robot to startup pose '{mode}'")
    log.info("Robot moved to startup pose '%s' (minimum_time=%.2fs)", mode, minimum_time)


def _run_candidate_action(
    executor: LeftArmGraspExecutor,
    candidate: FoundGraspCandidate,
    state: _RuntimeState,
) -> None:
    try:
        plan = executor.plan(candidate)
        ok = executor.execute(plan)
        with state.lock:
            state.status_msg = (
                f"candidate {candidate.index}: {'done' if ok else 'failed'} "
                f"(mode={executor.mode}, score={candidate.score:.3f})"
            )
    except Exception as exc:
        log.exception("Candidate action failed")
        with state.lock:
            state.status_msg = f"candidate {candidate.index}: error: {exc}"


def _status_line(state: _RuntimeState, execution_mode: str) -> tuple[str, str]:
    with state.lock:
        count = len(state.candidates)
        selected_index = min(state.selected_index, max(count - 1, 0))
        state.selected_index = selected_index
        if count == 0:
            summary = f"mode={execution_mode} selected=0/0"
        else:
            candidate = state.candidates[selected_index]
            summary = (
                f"mode={execution_mode} score={candidate.score:.3f} "
                f"selected={selected_index + 1}/{count}"
            )
        return summary, state.status_msg


def _infer_and_log(
    model,
    model_cfg,
    color_rgb: np.ndarray,
    depth_m: np.ndarray,
    K: np.ndarray,
    cfg,
    frame_idx: int,
    state: _RuntimeState,
) -> None:
    _ensure_path(cfg.found_grasp.foundgrasp_root)
    from utils.visualization import log_to_rerun, visualize_and_save_opencv

    device = cfg.found_grasp.device
    target_size = tuple(model_cfg.dataset.image_size)  # (H, W)

    xyz_orig = _depth_to_xyz(depth_m, K)
    pixel_values, og_color = _preprocess_color(color_rgb, target_size)
    pixel_values = pixel_values.unsqueeze(0)
    og_color = og_color.unsqueeze(0)

    H, W = target_size
    depth_resized = cv2.resize(depth_m, (W, H), interpolation=cv2.INTER_NEAREST)
    xyz_resized = cv2.resize(xyz_orig, (W, H), interpolation=cv2.INTER_NEAREST)

    xyz_t = torch.from_numpy(xyz_resized).permute(2, 0, 1).float().unsqueeze(0)
    depth_t = torch.from_numpy(depth_resized).unsqueeze(0).unsqueeze(0).float()
    depth_mask = (depth_t > 0.1).float()

    data = {
        "pixel_values": pixel_values.to(device),
        "og_color": og_color.to(device),
        "xyz": xyz_t.to(device),
        "depth": depth_t.to(device),
        "depth_mask": depth_mask.to(device),
    }

    with torch.no_grad():
        predictions = model(data)

    inf_cfg = model_cfg.get("inference", {})
    legacy = inf_cfg.get("pose_confidence_exp", None)
    gg = model.to_grasp_group(
        predictions={k: v[0] for k, v in predictions.items()},
        xyz=data["xyz"][0],
        graspness_threshold=inf_cfg.get("graspness_threshold", 0.01),
        objectness_threshold=inf_cfg.get("objectness_threshold", 0.5),
        graspness_exp=inf_cfg.get("graspness_exp", 0.1),
        grasp_score_exp=inf_cfg.get("grasp_score_exp", 2.0),
        total_score_threshold=inf_cfg.get("total_score_threshold", None),
        approach_kappa_exp=inf_cfg.get("approach_kappa_exp", legacy if legacy is not None else 1.0),
        gripper_kappa_exp=inf_cfg.get("gripper_kappa_exp", legacy if legacy is not None else 0.5),
        depth_confidence_exp=inf_cfg.get("depth_confidence_exp", 0.0),
        width_multiplier=inf_cfg.get("width_multiplier", 1.2),
        z_max=inf_cfg.get("z_max", 1.5),
        empty_grasp_min_points=inf_cfg.get("empty_grasp_min_points", 0),
        segm_erosion_k=inf_cfg.get("segm_erosion_k", 0),
    )
    # sort and select top 50
    gg = gg.nms(translation_thresh=0.03, rotation_thresh=30.0 / 180.0 * np.pi)
    gg = gg.sort_by_score()[:50]
    log.info("frame %d: %d grasps", frame_idx, len(gg))
    candidates = _extract_candidates(gg)

    vis_sample = {
        "og_color": og_color[0],
        "xyz": xyz_t[0],
        "depth": depth_t[0],
    }
    for key, val in predictions.items():
        vis_sample[f"pred_{key}"] = val[0]

    vis_result = visualize_and_save_opencv(vis_sample, save_dir=None)
    log_to_rerun(vis_sample, vis_result, xyz_resized.reshape(-1, 3), gg, frame_idx)
    with state.lock:
        state.candidates = candidates
        state.selected_index = min(state.selected_index, max(len(candidates) - 1, 0))
        if candidates:
            top = candidates[state.selected_index]
            state.status_msg = (
                f"frame {frame_idx}: {len(candidates)} grasps, "
                f"selected={state.selected_index + 1} score={top.score:.3f}"
            )
        else:
            state.status_msg = f"frame {frame_idx}: no grasps found"


def run_found_grasp(cfg) -> None:
    log.info("Loading FoundGrasp model...")
    model, model_cfg = _load_model(cfg)
    log.info("Model loaded.")

    cam = create_camera_stream(cfg)
    cam.start()
    K, _ = cam.get_intrinsics()
    depth_scale = cam.depth_scale

    rr.init(cfg.viz.application_id, spawn=False)
    if cfg.viz.spawn_viewer:
        rr.spawn(port=int(getattr(cfg.viz, "port", 9877)))

    cv2.namedWindow("FoundGrasp", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("FoundGrasp", 1280, 720)

    setup_error: str | None = None
    try:
        executor = _build_executor(cfg)
        execution_mode = "off" if executor is None else executor.mode
    except Exception as exc:
        log.exception("FoundGrasp execution setup failed")
        executor = None
        execution_mode = "off"
        setup_error = str(exc)
    frame_idx = 0
    infer_thread: threading.Thread | None = None
    action_thread: threading.Thread | None = None
    state = _RuntimeState(
        lock=threading.Lock(),
        candidates=[],
        selected_index=0,
        status_msg="g: infer",
    )
    if setup_error is not None:
        with state.lock:
            state.status_msg = f"execution setup error: {setup_error}"

    log.info("g: infer  |  [: prev grasp  ]: next grasp  |  e: preview/execute  |  q: quit")
    try:
        while True:
            frame = cam.get_frame()

            if frame.depth is None:
                log.error("No depth — set enable_depth: true in camera_server.yaml")
                break

            display = cv2.cvtColor(frame.color_rgb, cv2.COLOR_RGB2BGR)
            infer_busy = infer_thread is not None and infer_thread.is_alive()
            action_busy = action_thread is not None and action_thread.is_alive()
            busy = infer_busy or action_busy
            summary, status_msg = _status_line(state, execution_mode)
            cv2.putText(
                display,
                "busy..." if busy else "g: infer  [ ]: select  e: act  q: quit",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2,
            )
            cv2.putText(
                display,
                summary,
                (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 220, 120), 2,
            )
            cv2.putText(
                display,
                status_msg,
                (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (120, 220, 255), 2,
            )
            cv2.imshow("FoundGrasp", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("[") and not busy:
                with state.lock:
                    if state.candidates:
                        state.selected_index = (state.selected_index - 1) % len(state.candidates)
                        candidate = state.candidates[state.selected_index]
                        state.status_msg = (
                            f"selected grasp {state.selected_index + 1}/{len(state.candidates)} "
                            f"score={candidate.score:.3f}"
                        )
                continue
            if key == ord("]") and not busy:
                with state.lock:
                    if state.candidates:
                        state.selected_index = (state.selected_index + 1) % len(state.candidates)
                        candidate = state.candidates[state.selected_index]
                        state.status_msg = (
                            f"selected grasp {state.selected_index + 1}/{len(state.candidates)} "
                            f"score={candidate.score:.3f}"
                        )
                continue
            if key == ord("g") and not busy:
                depth_m = frame.depth.astype(np.float32) * depth_scale
                infer_thread = threading.Thread(
                    target=_infer_and_log,
                    args=(model, model_cfg, frame.color_rgb, depth_m, K, cfg, frame_idx, state),
                    daemon=True,
                )
                infer_thread.start()
                frame_idx += 1
                with state.lock:
                    state.status_msg = f"frame {frame_idx - 1}: inferring..."
                continue
            if key == ord("e") and not busy:
                if executor is None:
                    with state.lock:
                        state.status_msg = "execution.mode=off"
                    continue
                with state.lock:
                    if not state.candidates:
                        state.status_msg = "no grasp candidates"
                        continue
                    candidate = state.candidates[state.selected_index]
                    state.status_msg = (
                        f"candidate {candidate.index}: action start "
                        f"(mode={executor.mode}, score={candidate.score:.3f})"
                    )
                action_thread = threading.Thread(
                    target=_run_candidate_action,
                    args=(executor, candidate, state),
                    daemon=True,
                )
                action_thread.start()
    finally:
        if executor is not None:
            executor.shutdown()
        cam.stop()
        cv2.destroyAllWindows()
