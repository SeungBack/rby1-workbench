"""Head camera hand-eye calibration app.

Calibrates link_head_2Tcam (^link_head_2 T_camera_optical) using a stationary ChArUco board.

Usage:
    python -m rby1_workbench.apps.head_camera_calib
    python -m rby1_workbench.apps.head_camera_calib --config my_calib.yaml
    python -m rby1_workbench.apps.head_camera_calib --robot-config my_rby1.yaml

Keys (OpenCV window):
    s  — capture sample  (board must be detected)
    m  — move head to random pose within ±range of current position
         (first press records current pose as home)
    c  — compute calibration and save result
    d  — discard all samples
    q  — quit
"""

from __future__ import annotations

import argparse
import logging
import sys

import cv2
import numpy as np

from rby1_workbench import RBY1, load_rby1_config
from rby1_workbench.calibration import CharucoDetector, HandEyeSolver, camera_opticalTforward
from rby1_workbench.config.schema import load_calib_config
from rby1_workbench.perception.realsense import RealSenseStream

log = logging.getLogger(__name__)

_GREEN = (0, 255, 0)
_RED   = (0, 0, 255)
_WHITE = (255, 255, 255)
_FONT  = cv2.FONT_HERSHEY_SIMPLEX

# head joint limits (from HeadController)
_YAW_LIM   = np.deg2rad(29.0)
_PITCH_MIN = -np.deg2rad(19.0)
_PITCH_MAX = np.deg2rad(89.0)


def _draw_hud(frame, n_samples: int, detected: bool, home_set: bool) -> None:
    board_str = "DETECTED" if detected else "NOT DETECTED"
    color     = _GREEN if detected else _RED
    cv2.putText(frame, f"Samples: {n_samples}  |  Board: {board_str}",
                (10, 30), _FONT, 0.7, color, 2)
    home_str = "home set" if home_set else "no home (press m to set)"
    cv2.putText(frame, f"s:capture  m:random move ({home_str})  c:compute  d:discard  q:quit",
                (10, 60), _FONT, 0.50, _WHITE, 1)


def _random_torso_pose(
    home: np.ndarray,
    noise_ranges: dict[int, float],
    rng: np.random.Generator,
) -> np.ndarray:
    """Return torso joint positions with uniform noise on selected indices.

    Args:
        home: baseline torso joint positions (6-DOF).
        noise_ranges: mapping of joint index → noise half-range (radians).
        rng: numpy random generator.
    """
    q = home.copy()
    for idx, half_range in noise_ranges.items():
        q[int(idx)] += rng.uniform(-half_range, half_range)
    return q


def _random_head_pose(
    home: np.ndarray,
    yaw_range: float,
    pitch_range: float,
    rng: np.random.Generator,
) -> tuple[float, float]:
    """Return (yaw, pitch) clamped to head joint limits."""
    yaw   = float(np.clip(home[0] + rng.uniform(-yaw_range,   yaw_range),   -_YAW_LIM,   _YAW_LIM))
    pitch = float(np.clip(home[1] + rng.uniform(-pitch_range, pitch_range), _PITCH_MIN, _PITCH_MAX))
    return yaw, pitch

    


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-8s %(message)s",
    )

    parser = argparse.ArgumentParser(description="Head camera hand-eye calibration")
    parser.add_argument("--config",       default=None, help="calib YAML override")
    parser.add_argument("--robot-config", default=None, help="rby1 YAML override")
    args = parser.parse_args()

    calib_cfg = load_calib_config(args.config)
    robot_cfg  = load_rby1_config(args.robot_config)
    if "calib" in robot_cfg:
        robot_cfg.calib.auto_load = False
    head_link  = calib_cfg.robot.head_link

    # ── Robot (connect only — calibration works with servo off) ──────────
    robot = RBY1(robot_cfg)
    try:
        robot.connect()
    except Exception as e:
        log.error("Robot connection failed: %s", e)
        sys.exit(1)
    
    

    # ── Camera ───────────────────────────────────────────────────────────
    cam = RealSenseStream(calib_cfg.realsense)
    cam.start()
    K, D = cam.get_intrinsics()
    log.info("Camera intrinsics\nK:\n%s\nD: %s", K, D)

    # ── Calibration objects ──────────────────────────────────────────────
    detector  = CharucoDetector(calib_cfg.board, K, D)
    solver    = HandEyeSolver()
    rng       = np.random.default_rng()
    head_home: np.ndarray | None = None

    ac  = calib_cfg.auto_capture
    home_cfg = calib_cfg.home
    q_head_home  = np.asarray(home_cfg.head,  dtype=float)
    q_torso_home = np.asarray(home_cfg.torso, dtype=float)
    yaw, pitch = float(q_head_home[0]), float(q_head_home[1])
    q_torso    = q_torso_home.copy()
    
    print(robot.get_joint_positions('torso'))

    log.info("Moving to home position...")
    robot.ready_pose()
    print('1')
    robot.head.move_j(q_head_home)
    print('2')
    robot.torso.move_j(q_torso_home, mode='position')
    print('3')

    log.info("Ready. Keys: [s] capture  [m] random move  [c] compute  [d] discard  [q] quit")

    try:
        while True:
            frame  = cam.get_frame()
            result = detector.detect(frame.color_bgr)
            vis    = detector.draw(frame.color_bgr, result)
            _draw_hud(vis, solver.n_samples, result is not None, head_home is not None)

            cv2.imshow("Head Camera Calibration", vis)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            elif key == ord("s"):
                if result is None:
                    log.warning("Board not detected — move head until board is visible.")
                    continue
                try:
                    baseTgripper = robot.get_transform("base", head_link)
                except Exception as e:
                    log.warning("Could not get robot transform: %s", e)
                    continue
                q_actual_head  = robot.get_joint_positions("head")
                q_actual_torso = robot.get_joint_positions("torso")
                log.info(
                    "HEAD   cmd=[%.3f, %.3f]  actual=[%.3f, %.3f]  err=[%.3f, %.3f] rad",
                    yaw, pitch,
                    q_actual_head[0], q_actual_head[1],
                    yaw - q_actual_head[0], pitch - q_actual_head[1],
                )
                log.info(
                    "TORSO  cmd=%s\n       actual=%s\n       err=%s rad",
                    np.round(q_torso, 3), np.round(q_actual_torso, 3),
                    np.round(q_torso - q_actual_torso, 3),
                )
                print('baseTgripper:\n', baseTgripper.round(3))
                print(result.tvec)
                solver.add_sample(baseTgripper, result.rvec, result.tvec)
                log.info(
                    "Sample %d  |  ^base T_%s pos: %s",
                    solver.n_samples, head_link, baseTgripper[:3, 3].round(4),
                )

            elif key == ord("m"):

                if q_head_home is None:
                    log.info(
                        "Home pose recorded: yaw=%.1f° pitch=%.1f°",
                        np.rad2deg(q_head_home[0]), np.rad2deg(q_head_home[1]),
                    )

                yaw, pitch = _random_head_pose(q_head_home, ac.yaw_range, ac.pitch_range, rng)
                log.info("Moving head → yaw=%.1f° pitch=%.1f°", np.rad2deg(yaw), np.rad2deg(pitch))
                robot.head.move_j(np.array([yaw, pitch]))
                q_torso = _random_torso_pose(q_torso_home, ac.torso_noise, rng)
                robot.torso.move_j(q_torso, mode='position')

            elif key == ord("c"):
                if solver.n_samples < calib_cfg.min_samples:
                    log.warning(
                        "Need at least %d samples, have %d.",
                        calib_cfg.min_samples, solver.n_samples,
                    )
                    continue
                try:
                    gripperTcam = solver.solve(method=calib_cfg.calib_method)
                    diagnostics = solver.board_consistency(gripperTcam)
                    inverse_diagnostics = solver.board_consistency(np.linalg.inv(gripperTcam))
                except Exception as e:
                    log.error("Calibration solve failed: %s", e)
                    continue
                # gripperTcam = np.linalg.inv(gripperTcam)
                gripperTcam_forward = gripperTcam @ camera_opticalTforward()
                log.info("%sTcam (^%s T_camera_optical):\n%s", head_link, head_link, gripperTcam.round(6))
                log.info("Camera origin in %s (m): %s", head_link, gripperTcam[:3, 3].round(6))
                log.info(
                    "%sTcam_forward (^%s T_camera_forward):\n%s",
                    head_link,
                    head_link,
                    gripperTcam_forward.round(6),
                )
                log.info(
                    "Board consistency after solve | max pos err: %.4f m, max rot err: %.2f deg",
                    diagnostics.translation_max_error_m,
                    diagnostics.rotation_max_error_deg,
                )
                log.info(
                    "Captured motion span | max relative rot: %.1f deg, max relative trans: %.4f m",
                    diagnostics.motion_max_rotation_deg,
                    diagnostics.motion_max_translation_m,
                )
                log.info(
                    "Inverse-candidate board error | max pos err: %.4f m, max rot err: %.2f deg",
                    inverse_diagnostics.translation_max_error_m,
                    inverse_diagnostics.rotation_max_error_deg,
                )
                path = solver.save(
                    gripperTcam,
                    calib_cfg.output_dir,
                    hand_link=head_link,
                    diagnostics=diagnostics,
                    inverse_candidate_diagnostics=inverse_diagnostics,
                )
                log.info("Saved → %s", path)

            elif key == ord("d"):
                solver.clear()
                log.info("All samples discarded.")

    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
