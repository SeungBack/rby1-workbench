"""RealSense stream wrapper used by perception apps."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from omegaconf import DictConfig


@dataclass(slots=True)
class RealSenseFrame:
    """Single synchronized frame bundle from a RealSense camera."""

    color_bgr: np.ndarray
    color_rgb: np.ndarray
    depth: np.ndarray | None
    timestamp_ms: float
    frame_index: int


class RealSenseStream:
    """Thin wrapper around pyrealsense2 pipeline setup and frame retrieval."""

    def __init__(self, config: DictConfig):
        self.config = config
        self._rs = self._import_rs()
        self._pipeline = None
        self._align = None
        self._started = False

    @staticmethod
    def _import_rs():
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise ImportError(
                "pyrealsense2 is required for RealSense streaming. "
                "Install it in the active environment before running this app."
            ) from exc
        return rs

    def start(self) -> None:
        """Start the RealSense pipeline and warm up a few frames."""
        if self._started:
            return

        rs = self._rs
        cfg = rs.config()
        if self.config.serial_number:
            cfg.enable_device(self.config.serial_number)

        cfg.enable_stream(
            rs.stream.color,
            self.config.color_width,
            self.config.color_height,
            rs.format.bgr8,
            self.config.fps,
        )
        if self.config.enable_depth:
            cfg.enable_stream(
                rs.stream.depth,
                self.config.depth_width,
                self.config.depth_height,
                rs.format.z16,
                self.config.fps,
            )

        self._pipeline = rs.pipeline()
        self._pipeline.start(cfg)
        self._align = (
            rs.align(rs.stream.color)
            if self.config.enable_depth and self.config.align_depth_to_color
            else None
        )

        for _ in range(max(0, self.config.warmup_frames)):
            self._pipeline.wait_for_frames()

        self._started = True

    def stop(self) -> None:
        """Stop the RealSense pipeline."""
        if self._pipeline is not None:
            self._pipeline.stop()
        self._pipeline = None
        self._align = None
        self._started = False

    def get_intrinsics(self) -> tuple[np.ndarray, np.ndarray]:
        """Color stream intrinsics as (K 3×3, D). start() 이후에만 유효."""
        if not self._started or self._pipeline is None:
            raise RuntimeError("RealSenseStream.start() must be called first.")
        rs = self._rs
        profile = self._pipeline.get_active_profile()
        intr = (
            profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
            .get_intrinsics()
        )
        K = np.array(
            [[intr.fx, 0.0, intr.ppx],
             [0.0, intr.fy, intr.ppy],
             [0.0, 0.0,     1.0     ]],
            dtype=np.float64,
        )
        D = np.array(intr.coeffs, dtype=np.float64)
        return K, D

    def get_frame(self, timeout_ms: int = 5000) -> RealSenseFrame:
        """Return the next available frame bundle."""
        if not self._started or self._pipeline is None:
            raise RuntimeError("RealSenseStream.start() must be called first.")

        frames = self._pipeline.wait_for_frames(timeout_ms)
        if self._align is not None:
            frames = self._align.process(frames)

        color_frame = frames.get_color_frame()
        if not color_frame:
            raise RuntimeError("Failed to receive a color frame from RealSense.")

        depth_frame = frames.get_depth_frame() if self.config.enable_depth else None
        color_bgr = np.asanyarray(color_frame.get_data()).copy()
        color_rgb = color_bgr[:, :, ::-1].copy()
        depth = np.asanyarray(depth_frame.get_data()).copy() if depth_frame else None

        return RealSenseFrame(
            color_bgr=color_bgr,
            color_rgb=color_rgb,
            depth=depth,
            timestamp_ms=float(color_frame.get_timestamp()),
            frame_index=int(color_frame.get_frame_number()),
        )

    def __enter__(self) -> "RealSenseStream":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()
