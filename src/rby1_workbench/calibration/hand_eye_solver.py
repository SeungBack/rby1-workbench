"""Hand-eye calibration sample collection and solver."""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from rby1_workbench.config.schema import package_root

log = logging.getLogger(__name__)

_METHODS: dict[str, int] = {
    "TSAI":       cv2.CALIB_HAND_EYE_TSAI,
    "PARK":       cv2.CALIB_HAND_EYE_PARK,
    "HORAUD":     cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF":    cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}

# https://github.com/RealManRobot/hand_eye_calibration

@dataclass
class CalibSample:
    baseTgripper: np.ndarray  # ^base T_gripper, e.g. base -> link_head_2
    rvec: np.ndarray          # ^camera R_board as Rodrigues vector
    tvec: np.ndarray          # ^camera t_board


@dataclass(frozen=True)
class BoardConsistency:
    """How well the solved eye-in-hand transform keeps the fixed board still."""

    translation_mean_m: list[float]
    translation_std_m: list[float]
    translation_max_error_m: float
    rotation_max_error_deg: float
    motion_max_rotation_deg: float
    motion_max_translation_m: float


def _rot_to_quat_wxyz(R: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation matrix → quaternion (w, x, y, z)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(w), float(x), float(y), float(z)


def _make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(R, dtype=np.float64)
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def _rotation_angle_deg(R: np.ndarray) -> float:
    cos_theta = (float(np.trace(R)) - 1.0) * 0.5
    return float(np.rad2deg(np.arccos(np.clip(cos_theta, -1.0, 1.0))))


def camera_opticalTforward() -> np.ndarray:
    """Return a fixed child transform that flips optical +Z to camera forward.

    For RB-Y1 visualization we want a derived camera frame whose forward axis
    points the same way as the physical camera front in the robot's +X-facing
    convention. This is a 180-degree rotation about the optical Y axis.
    """
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.diag([-1.0, 1.0, -1.0])
    return T


def resolve_calibration_output_dir(output_dir: str | Path) -> Path:
    """Resolve output_dir to an absolute path independent from cwd."""
    path = Path(output_dir).expanduser()
    if path.is_absolute():
        return path
    return package_root() / path


def _diagnostics_to_json(diagnostics: BoardConsistency) -> dict[str, object]:
    return {
        "board_translation_mean_m": diagnostics.translation_mean_m,
        "board_translation_std_m": diagnostics.translation_std_m,
        "board_translation_max_error_m": diagnostics.translation_max_error_m,
        "board_rotation_max_error_deg": diagnostics.rotation_max_error_deg,
        "motion_max_rotation_deg": diagnostics.motion_max_rotation_deg,
        "motion_max_translation_m": diagnostics.motion_max_translation_m,
    }


class HandEyeSolver:
    """Collects eye-in-hand samples and solves for gripperTcam.

    Usage::

        solver = HandEyeSolver()
        solver.add_sample(T_base_to_head2, rvec, tvec)   # repeat N times
        gripperTcam = solver.solve()
        solver.save(gripperTcam, "outputs", hand_link="link_head_2")

        # Load in a new session:
        T, frame_from, frame_to = HandEyeSolver.load_latest("outputs")
    """

    def __init__(self) -> None:
        self._samples: list[CalibSample] = []

    @property
    def n_samples(self) -> int:
        return len(self._samples)

    def add_sample(
        self,
        baseTgripper: np.ndarray,
        rvec: np.ndarray,
        tvec: np.ndarray,
    ) -> None:
        self._samples.append(CalibSample(
            baseTgripper=np.asarray(baseTgripper, dtype=np.float64).copy(),
            rvec=np.asarray(rvec, dtype=np.float64).copy(),
            tvec=np.asarray(tvec, dtype=np.float64).copy(),
        ))

    def clear(self) -> None:
        self._samples.clear()

    def solve(self, method: str = "DANIILIDIS") -> np.ndarray:
        """Solve and return gripperTcam (4x4), i.e. ^gripper T_camera.

        This is eye-in-hand for the RB-Y1 head camera:

        - base: RB-Y1 ``base``
        - gripper: the moving head link that rigidly carries the camera,
          normally ``link_head_2``
        - camera: the RealSense color optical frame used by ChArUco pose
          estimation
        - target: the stationary ChArUco board

        OpenCV names the output ``R_cam2gripper`` / ``t_cam2gripper``.
        In homogeneous-transform notation that is ^gripper T_camera, which
        is exactly the static child transform to register under ``gripper``.
        """
        if self.n_samples < 3:
            raise ValueError(f"Need at least 3 samples, have {self.n_samples}.")

        cv_method = _METHODS.get(method.upper())
        if cv_method is None:
            raise ValueError(f"Unknown method '{method}'. Valid: {list(_METHODS)}")

        R_g2b = [s.baseTgripper[:3, :3] for s in self._samples]
        t_g2b = [s.baseTgripper[:3, 3].reshape(3, 1) for s in self._samples] # 수정
        R_t2c = [cv2.Rodrigues(s.rvec)[0] for s in self._samples]
        t_t2c = [s.tvec.reshape(3, 1) for s in self._samples]                # 수정
        

        R, t = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=cv_method)

        gripperTcam = _make_transform(R, t)

        # ---------------------------------------------------------
        # [스윙 역전 보정]
        # 회전은 완벽하지만, 로봇과 OpenCV 간의 Active/Passive 회전 해석 차이로 인해
        # Lever-Arm Reversal Problem
        # Translation의 X축(전후 방향)이 뒤집힌 경우 수동으로 보정합니다.
        # ---------------------------------------------------------
        # gripperTcam[0, 3] = -gripperTcam[0, 3]  # X축 위치 반전 (-0.097 -> +0.097)
        # gripperTcam[1, 3] = -gripperTcam[1, 3]  # Y축 위치 반전]
        # 만약 Y나 Z도 물리적 위치(예: 카메라가 관절 위쪽(+Z)에 있음)와 다르다면 함께 반전해 줍니다.
        
        return gripperTcam

    def solve_best(self) -> tuple[np.ndarray, str, "BoardConsistency"]:
        """Try all methods, return (gripperTcam, method_name, diagnostics) for the best.

        Best = lowest translation_max_error_m from board_consistency.
        All results are logged so you can compare.
        """
        results: list[tuple[str, np.ndarray, BoardConsistency]] = []
        for name in _METHODS:
            try:
                T = self.solve(name)
                diag = self.board_consistency(T)
                results.append((name, T, diag))
            except Exception as e:
                log.debug("Method %s failed: %s", name, e)

        if not results:
            raise RuntimeError("All calibration methods failed.")

        results.sort(key=lambda x: x[2].translation_max_error_m)
        log.info("Board consistency per method:")
        for name, _, diag in results:
            marker = " ← best" if name == results[0][0] else ""
            log.info(
                "  %-12s  pos_max: %.4f m  rot_max: %.2f deg%s",
                name, diag.translation_max_error_m, diag.rotation_max_error_deg, marker,
            )
        best_name, best_T, best_diag = results[0]
        return best_T, best_name, best_diag

    def board_poses_in_base(self, gripperTcam: np.ndarray) -> list[np.ndarray]:
        """Return ^base T_board for each captured sample.

        In a valid eye-in-hand dataset with a stationary board, these poses
        should cluster tightly:

            ^base T_board = ^base T_gripper * ^gripper T_camera * ^camera T_board
        """
        gripperTcam = np.asarray(gripperTcam, dtype=np.float64)
        poses: list[np.ndarray] = []
        for sample in self._samples:
            camTboard = _make_transform(cv2.Rodrigues(sample.rvec)[0], sample.tvec)
            poses.append(sample.baseTgripper @ gripperTcam @ camTboard)
        return poses

    def board_consistency(self, gripperTcam: np.ndarray) -> BoardConsistency:
        """Summarize whether the fixed board remains fixed after calibration."""
        board_poses = self.board_poses_in_base(gripperTcam)
        translations = np.asarray([T[:3, 3] for T in board_poses], dtype=np.float64)
        translation_mean = translations.mean(axis=0)
        translation_errors = np.linalg.norm(translations - translation_mean, axis=1)

        R_ref = board_poses[0][:3, :3]
        rotation_errors = [
            _rotation_angle_deg(R_ref.T @ T[:3, :3])
            for T in board_poses
        ]

        motion_angles: list[float] = []
        motion_translations: list[float] = []
        for first, second in zip(self._samples, self._samples[1:]):
            T_rel = np.linalg.inv(first.baseTgripper) @ second.baseTgripper
            motion_angles.append(_rotation_angle_deg(T_rel[:3, :3]))
            motion_translations.append(float(np.linalg.norm(T_rel[:3, 3])))

        return BoardConsistency(
            translation_mean_m=translation_mean.tolist(),
            translation_std_m=translations.std(axis=0).tolist(),
            translation_max_error_m=float(translation_errors.max()),
            rotation_max_error_deg=float(max(rotation_errors, default=0.0)),
            motion_max_rotation_deg=float(max(motion_angles, default=0.0)),
            motion_max_translation_m=float(max(motion_translations, default=0.0)),
        )

    @staticmethod
    def load_result(path: str | Path) -> tuple[np.ndarray, str, str]:
        """Load a saved calibration JSON.

        Returns:
            (gripperTcam, frame_from, frame_to)
        """
        with Path(path).open() as f:
            data = json.load(f)
        T = np.array(data["matrix"], dtype=np.float64)
        return T, data["frame_from"], data["frame_to"]

    @staticmethod
    def load_latest(output_dir: str | Path) -> tuple[np.ndarray, str, str] | None:
        """Find and load the most recently saved calibration JSON.

        Returns (gripperTcam, frame_from, frame_to), or None if no file found.
        Files are sorted by filename (timestamp-based), so newest = last alphabetically.
        """
        out = resolve_calibration_output_dir(output_dir)
        if not out.exists():
            return None
        files = sorted(out.glob("head_camera_calib_*.json"))
        if not files:
            return None
        return HandEyeSolver.load_result(files[-1])

    def save(
        self,
        gripperTcam: np.ndarray,
        output_dir: str,
        hand_link: str = "link_head_2",
        diagnostics: BoardConsistency | None = None,
        method: str = "",
        inverse_candidate_diagnostics: BoardConsistency | None = None,
    ) -> Path:
        """Save result to a timestamped JSON file. Returns the saved path."""
        out = resolve_calibration_output_dir(output_dir)
        out.mkdir(parents=True, exist_ok=True)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path  = out / f"head_camera_calib_{stamp}.json"

        R = gripperTcam[:3, :3]
        t = gripperTcam[:3, 3]
        w, x, y, z = _rot_to_quat_wxyz(R)

        data = {
            "frame_from":      hand_link,
            "frame_to":        "camera_optical",
            "transform":       "gripperTcam",
            "transform_notation": "^gripper T_camera",
            "derived_frame_to": "camera_forward",
            "calibration_type": "eye-in-hand",
            "calib_method":    method,
            "n_samples":       self.n_samples,
            "timestamp":       stamp,
            "matrix":          gripperTcam.tolist(),
            "derived_matrix":  (gripperTcam @ camera_opticalTforward()).tolist(),
            "position":        t.tolist(),
            "quaternion_wxyz": [w, x, y, z],
        }
        if diagnostics is not None:
            data["diagnostics"] = _diagnostics_to_json(diagnostics)
        if inverse_candidate_diagnostics is not None:
            data["inverse_candidate_diagnostics"] = _diagnostics_to_json(
                inverse_candidate_diagnostics
            )
        data["samples"] = [
            {
                "baseTgripper": sample.baseTgripper.tolist(),
                "rvec": sample.rvec.tolist(),
                "tvec": sample.tvec.tolist(),
            }
            for sample in self._samples
        ]
        with path.open("w") as f:
            json.dump(data, f, indent=2)

        log.info("Saved → %s", path)
        return path
