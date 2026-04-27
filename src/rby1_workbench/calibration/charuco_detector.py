"""ChArUco board detection and pose estimation."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import cv2.aruco as aruco
import numpy as np
from omegaconf import DictConfig


@dataclass
class DetectionResult:
    rvec: np.ndarray       # (3,1) rotation of board in camera frame
    tvec: np.ndarray       # (3,1) translation of board in camera frame
    corners: np.ndarray    # detected ChArUco corners
    ids: np.ndarray        # corner ids
    n_corners: int


class CharucoDetector:
    """Detects a ChArUco board and estimates its pose in the camera frame."""

    def __init__(self, board_cfg: DictConfig, K: np.ndarray, D: np.ndarray):
        self.K = np.asarray(K, dtype=np.float64)
        self.D = np.asarray(D, dtype=np.float64)

        dictionary = aruco.getPredefinedDictionary(getattr(aruco, board_cfg.marker_dict))

        try:
            self._board = aruco.CharucoBoard(
                (board_cfg.cols, board_cfg.rows),
                float(board_cfg.square_len),
                float(board_cfg.marker_len),
                dictionary,
            )
        except TypeError:
            self._board = aruco.CharucoBoard_create(
                board_cfg.cols, board_cfg.rows,
                float(board_cfg.square_len), float(board_cfg.marker_len),
                dictionary,
            )

        self._detector_params = aruco.DetectorParameters()
        self._detector = aruco.ArucoDetector(dictionary, self._detector_params)

    def detect(self, frame_bgr: np.ndarray) -> DetectionResult | None:
        """Detect board and estimate pose. Returns None if detection fails."""
        corners, ids, rejected = self._detector.detectMarkers(frame_bgr)
        if ids is None:
            return None

        try:
            corners, ids, rejected, _ = aruco.refineDetectedMarkers(
                frame_bgr, self._board, corners, ids, rejected,
                self.K, self.D, errorCorrectionRate=-1,
                parameters=self._detector_params,
            )
        except Exception:
            pass

        retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            corners, ids, frame_bgr, self._board, self.K, self.D,
        )
        if retval < 4:
            return None

        ok, rvec, tvec = aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, self._board, self.K, self.D, None, None,
        )
        if not ok:
            return None

        return DetectionResult(
            rvec=rvec,
            tvec=tvec,
            corners=charuco_corners,
            ids=charuco_ids,
            n_corners=retval,
        )

    def draw(self, frame_bgr: np.ndarray, result: DetectionResult | None) -> np.ndarray:
        """Overlay detection result on a copy of the frame."""
        vis = frame_bgr.copy()
        if result is None:
            return vis
        cv2.drawFrameAxes(vis, self.K, self.D, result.rvec, result.tvec, 0.05)
        aruco.drawDetectedCornersCharuco(vis, result.corners, result.ids)
        return vis
