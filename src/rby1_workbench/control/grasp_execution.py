"""Preview and execute FoundGrasp candidates with the RB-Y1 left arm."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import time

import numpy as np
import rerun as rr
from omegaconf import DictConfig

from rby1_workbench.robot.rby1 import RBY1


log = logging.getLogger(__name__)

_FRAME_COLORS = [[220, 60, 60], [60, 220, 60], [60, 60, 220]]
_CURRENT_EE_COLORS = [[90, 140, 255], [80, 200, 80], [255, 210, 70]]
_TARGET_EE_COLORS = [[255, 120, 60], [60, 220, 160], [60, 160, 255]]
_PREGRASP_EE_COLORS = [[255, 200, 60], [60, 220, 160], [60, 160, 255]]
_LIFT_EE_COLORS = [[255, 120, 200], [60, 220, 160], [60, 160, 255]]


def _as_transform(value: object, *, name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64)
    if arr.shape != (4, 4):
        raise ValueError(f"{name} must be a 4x4 transform, got shape {arr.shape}")
    return arr.copy()


def _translation_transform(offset_xyz: object) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    offset = np.asarray(offset_xyz, dtype=np.float64).reshape(3)
    T[:3, 3] = offset
    return T


def _log_frame(
    path: str,
    transform: np.ndarray,
    *,
    label: str,
    arrow_length_m: float,
    colors: list[list[int]] = _FRAME_COLORS,
    static: bool = False,
) -> None:
    position = transform[:3, 3]
    axes = transform[:3, :3] * float(arrow_length_m)
    rr.log(
        f"{path}/axes",
        rr.Arrows3D(
            origins=[position] * 3,
            vectors=[axes[:, 0], axes[:, 1], axes[:, 2]],
            colors=colors,
        ),
        static=static,
    )
    rr.log(
        f"{path}/origin",
        rr.Points3D([position], radii=[0.01], labels=[label]),
        static=static,
    )


@dataclass(slots=True)
class FoundGraspCandidate:
    index: int
    score: float
    width_m: float
    height_m: float
    depth_m: float
    camera_opticalTgrasp: np.ndarray


@dataclass(slots=True)
class LeftArmGraspPlan:
    candidate: FoundGraspCandidate
    baseTcamera_optical: np.ndarray
    baseTgrasp: np.ndarray
    baseTcurrent_ee: np.ndarray
    baseTpregrasp_ee: np.ndarray
    baseTtarget_ee: np.ndarray
    baseTlift_ee: np.ndarray


class LeftArmGraspExecutor:
    """Plan and optionally execute a left-arm grasp sequence."""

    def __init__(self, robot: RBY1, cfg: DictConfig):
        self._robot = robot
        self._cfg = cfg
        self._graspTee_left = _as_transform(cfg.graspTee_left, name="execution.graspTee_left")
        self._pregrasp_offset_grasp = np.asarray(
            cfg.pregrasp_offset_grasp, dtype=np.float64
        ).reshape(3)
        self._lift_offset_base = np.asarray(cfg.lift_offset_base, dtype=np.float64).reshape(3)
        self._arrow_length_m = float(cfg.frame_arrow_length_m)
        self._world_frame = str(cfg.world_frame)
        self._gripper_started = False
        self._uses_gripper = bool(cfg.use_gripper)

    @property
    def mode(self) -> str:
        return str(self._cfg.mode).lower().strip()

    def plan(self, candidate: FoundGraspCandidate) -> LeftArmGraspPlan:
        try:
            baseTcamera_optical = self._robot.get_transform("base", "camera_optical")
        except Exception as exc:
            raise RuntimeError(
                "camera_optical frame is unavailable. Check calibration auto-load first."
            ) from exc

        baseTgrasp = baseTcamera_optical @ candidate.camera_opticalTgrasp
        baseTcurrent_ee = self._robot.get_ee_pose("left_arm")
        baseTpregrasp_ee = (
            baseTgrasp
            @ _translation_transform(self._pregrasp_offset_grasp)
            @ self._graspTee_left
        )
        baseTtarget_ee = baseTgrasp @ self._graspTee_left
        baseTlift_ee = baseTtarget_ee.copy()
        baseTlift_ee[:3, 3] += self._lift_offset_base

        return LeftArmGraspPlan(
            candidate=candidate,
            baseTcamera_optical=baseTcamera_optical,
            baseTgrasp=baseTgrasp,
            baseTcurrent_ee=baseTcurrent_ee,
            baseTpregrasp_ee=baseTpregrasp_ee,
            baseTtarget_ee=baseTtarget_ee,
            baseTlift_ee=baseTlift_ee,
        )

    def log_plan(self, plan: LeftArmGraspPlan) -> None:
        rr.log(self._world_frame, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        _log_frame(
            f"{self._world_frame}/grasp_execution/camera_optical",
            plan.baseTcamera_optical,
            label="camera_optical",
            arrow_length_m=self._arrow_length_m,
            static=True,
        )
        _log_frame(
            f"{self._world_frame}/grasp_execution/grasp_candidate",
            plan.baseTgrasp,
            label=f"grasp[{plan.candidate.index}] score={plan.candidate.score:.3f}",
            arrow_length_m=self._arrow_length_m,
        )
        _log_frame(
            f"{self._world_frame}/grasp_execution/current_ee_left",
            plan.baseTcurrent_ee,
            label="current ee_left",
            arrow_length_m=self._arrow_length_m,
            colors=_CURRENT_EE_COLORS,
        )
        _log_frame(
            f"{self._world_frame}/grasp_execution/pregrasp_ee_left",
            plan.baseTpregrasp_ee,
            label="pregrasp ee_left",
            arrow_length_m=self._arrow_length_m,
            colors=_PREGRASP_EE_COLORS,
        )
        _log_frame(
            f"{self._world_frame}/grasp_execution/target_ee_left",
            plan.baseTtarget_ee,
            label="target ee_left",
            arrow_length_m=self._arrow_length_m,
            colors=_TARGET_EE_COLORS,
        )
        _log_frame(
            f"{self._world_frame}/grasp_execution/lift_ee_left",
            plan.baseTlift_ee,
            label="lift ee_left",
            arrow_length_m=self._arrow_length_m,
            colors=_LIFT_EE_COLORS,
        )

    def execute(self, plan: LeftArmGraspPlan) -> bool:
        self.log_plan(plan)
        if self.mode != "execute":
            log.info("Grasp execution mode=%s -> rerun preview only.", self.mode)
            return True

        self._ensure_execute_ready()

        q_torso = self._robot.get_joint_positions("torso")
        q_head = self._robot.get_joint_positions("head")
        Tright = self._robot.get_ee_pose("right_arm")

        if bool(self._cfg.open_gripper_before_approach) and self._uses_gripper:
            self._set_left_gripper(float(self._cfg.left_gripper_open_ratio))
            time.sleep(float(self._cfg.gripper_wait_s))

        ok = self._move_left(
            plan.baseTpregrasp_ee,
            torso=q_torso,
            head=q_head,
            right_arm=Tright,
            minimum_time=float(self._cfg.pregrasp_minimum_time),
        )
        ok = ok and self._move_left(
            plan.baseTtarget_ee,
            torso=q_torso,
            head=q_head,
            right_arm=Tright,
            minimum_time=float(self._cfg.grasp_minimum_time),
        )
        if not ok:
            return False

        if self._uses_gripper:
            self._set_left_gripper(float(self._cfg.left_gripper_close_ratio))
            time.sleep(float(self._cfg.gripper_wait_s))

        return self._move_left(
            plan.baseTlift_ee,
            torso=q_torso,
            head=q_head,
            right_arm=Tright,
            minimum_time=float(self._cfg.lift_minimum_time),
        )

    def shutdown(self) -> None:
        if self._gripper_started:
            try:
                self._robot.gripper.stop()
            except Exception:
                log.exception("Failed to stop left gripper control loop")
            self._gripper_started = False

    def _ensure_execute_ready(self) -> None:
        if not self._uses_gripper or self._gripper_started:
            return

        try:
            gripper = self._robot.gripper
        except Exception as exc:
            raise RuntimeError(
                "Gripper execution requires direct backend robot access."
            ) from exc

        if not gripper.setup():
            raise RuntimeError("Gripper setup failed")
        gripper.start()
        self._gripper_started = True

    def _set_left_gripper(self, value: float) -> None:
        self._robot.gripper.set_normalized(left=float(np.clip(value, 0.0, 1.0)))

    def _move_left(
        self,
        Tleft: np.ndarray,
        *,
        torso: np.ndarray,
        head: np.ndarray,
        right_arm: np.ndarray,
        minimum_time: float,
    ) -> bool:
        return self._robot.move(
            mode="cartesian",
            torso=torso,
            right_arm=right_arm,
            left_arm=Tleft,
            head=head,
            minimum_time=minimum_time,
            control_hold_time=float(self._cfg.control_hold_time),
            linear_velocity_limit=float(self._cfg.linear_velocity_limit),
            angular_velocity_limit=float(self._cfg.angular_velocity_limit),
            accel_limit_scaling=float(self._cfg.accel_limit_scaling),
            stop_position_error=float(self._cfg.stop_position_error),
            stop_orientation_error=float(self._cfg.stop_orientation_error),
        )
