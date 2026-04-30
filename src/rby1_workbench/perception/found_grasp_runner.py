"""Realtime FoundGrasp grasp inference on RealSense frames."""

from __future__ import annotations

import logging
import os
import sys
import threading

import cv2
import numpy as np
import rerun as rr
import torch

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


def _infer_and_log(
    model,
    model_cfg,
    color_rgb: np.ndarray,
    depth_m: np.ndarray,
    K: np.ndarray,
    cfg,
    frame_idx: int,
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
    log.info("frame %d: %d grasps", frame_idx, len(gg))

    vis_sample = {
        "og_color": og_color[0],
        "xyz": xyz_t[0],
        "depth": depth_t[0],
    }
    for key, val in predictions.items():
        vis_sample[f"pred_{key}"] = val[0]

    vis_result = visualize_and_save_opencv(vis_sample, save_dir=None)
    log_to_rerun(vis_sample, vis_result, xyz_resized.reshape(-1, 3), gg, frame_idx)


def run_found_grasp(cfg) -> None:
    log.info("Loading FoundGrasp model...")
    model, model_cfg = _load_model(cfg)
    log.info("Model loaded.")

    cam = create_camera_stream(cfg)
    cam.start()
    K, _ = cam.get_intrinsics()
    depth_scale = cam.depth_scale

    rr.init(cfg.viz.application_id, spawn=cfg.viz.spawn_viewer)

    cv2.namedWindow("FoundGrasp", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("FoundGrasp", 1280, 720)

    frame_idx = 0
    infer_thread: threading.Thread | None = None

    log.info("g: run grasp inference  |  q: quit")
    try:
        while True:
            frame = cam.get_frame()

            if frame.depth is None:
                log.error("No depth — set enable_depth: true in camera_server.yaml")
                break

            display = cv2.cvtColor(frame.color_rgb, cv2.COLOR_RGB2BGR)
            busy = infer_thread is not None and infer_thread.is_alive()
            cv2.putText(
                display,
                "inferring..." if busy else "g: grasp  q: quit",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2,
            )
            cv2.imshow("FoundGrasp", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("g") and not busy:
                depth_m = frame.depth.astype(np.float32) * depth_scale
                infer_thread = threading.Thread(
                    target=_infer_and_log,
                    args=(model, model_cfg, frame.color_rgb, depth_m, K, cfg, frame_idx),
                    daemon=True,
                )
                infer_thread.start()
                frame_idx += 1
    finally:
        cam.stop()
        cv2.destroyAllWindows()
