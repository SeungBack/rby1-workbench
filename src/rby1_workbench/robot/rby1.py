"""RBY1 로봇 메인 wrapper 클래스."""

from __future__ import annotations

import logging
import threading
from contextlib import contextmanager
from typing import TYPE_CHECKING, Any, Iterator

import numpy as np
import rby1_sdk as rby

from rby1_workbench.config.schema import RBY1Config

if TYPE_CHECKING:
    from rby1_workbench.robot.gripper import GripperController
    from rby1_workbench.robot.head import HeadController
    from rby1_workbench.robot.stream import RBY1Stream

log = logging.getLogger(__name__)

# 공식 예제 기반 ready pose (22_joint_impedance_control.py)
_READY_POSE: dict[str, dict[str, np.ndarray]] = {
    "A": {
        "torso":     np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        "right_arm": np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        "left_arm":  np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    },
    "M": {
        "torso":     np.deg2rad([0.0, 45.0, -90.0, 45.0, 0.0, 0.0]),
        "right_arm": np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        "left_arm":  np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    },
    "UB": {
        "torso":     np.deg2rad([10.0, 0.0]),
        "right_arm": np.deg2rad([0.0, -5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
        "left_arm":  np.deg2rad([0.0, 5.0, 0.0, -120.0, 0.0, 70.0, 0.0]),
    },
}

# FK에 사용할 프레임 이름 (VR teleop 기준)
_FK_FRAMES = ["base", "link_torso_5", "link_right_arm_6", "link_left_arm_6"]
_FK_IDX = {"base": 0, "torso": 1, "right_arm": 2, "left_arm": 3}

_COMPONENT_ATTR = {
    "torso":     "torso_idx",
    "right_arm": "right_arm_idx",
    "left_arm":  "left_arm_idx",
    "head":      "head_idx",
}


class RBY1:
    """RBY1 로봇 wrapper.

    사용 예::

        cfg = RBY1Config.from_yaml("config/rby1.yaml")
        robot = RBY1(cfg)
        robot.initialize()                         # connect + power + servo + cm

        # 관절 위치 읽기
        q = robot.get_joint_positions("right_arm")  # np.ndarray [rad]
        n = robot.dof("right_arm")                  # int

        # blocking 이동 (활성 스트림 자동 pause/resume)
        robot.movej(torso=np.deg2rad([0,45,-90,45,0,0]), minimum_time=5.0)
        robot.ready_pose()
        robot.zero_pose()

        # 실시간 streaming
        stream = robot.create_stream()
        stream.send(right_arm=T_right, left_arm=T_left, torso=q_torso, head=q_head)
        stream.pause()
        robot.ready_pose()   # stream 이미 paused → 자동 pause 건너뜀
        stream.resume()      # 재개 후 첫 send에서 reset=True 자동 적용
        stream.cancel()
    """

    def __init__(self, cfg: RBY1Config):
        self._cfg = cfg
        self._robot: Any = None
        self._model: Any = None
        self._dyn_robot: Any = None
        self._dyn_state: Any = None
        self._dyn_lock = threading.Lock()
        self._head_ctrl: HeadController | None = None
        self._gripper_ctrl: GripperController | None = None
        self._active_stream: RBY1Stream | None = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> "RBY1":
        """gRPC 연결. 연결 실패 시 RuntimeError."""
        log.info("Connecting to RBY1 at %s (model=%s)", self._cfg.address, self._cfg.model)
        self._robot = rby.create_robot(self._cfg.address, self._cfg.model)
        if not self._robot.connect():
            raise RuntimeError(f"Failed to connect to {self._cfg.address}")
        self._model = self._robot.model()
        self._setup_fk()
        log.info("Connected. Model: %s", self._model.model_name)
        return self

    def power_on(self, pattern: str | None = None) -> "RBY1":
        p = pattern or self._cfg.power_pattern
        if not self._robot.is_power_on(p):
            if not self._robot.power_on(p):
                raise RuntimeError(f"power_on failed for pattern '{p}'")
            log.info("Power on: %s", p)
        else:
            log.info("Power already on: %s", p)
        return self

    def power_off(self, pattern: str | None = None) -> "RBY1":
        p = pattern or self._cfg.power_pattern
        if not self._robot.power_off(p):
            raise RuntimeError(f"power_off failed for pattern '{p}'")
        log.info("Power off: %s", p)
        return self

    def servo_on(self, pattern: str | None = None) -> "RBY1":
        p = pattern or self._cfg.servo_pattern
        if not self._robot.is_servo_on(p):
            if not self._robot.servo_on(p):
                raise RuntimeError(f"servo_on failed for pattern '{p}'")
            log.info("Servo on: %s", p)
        else:
            log.info("Servo already on: %s", p)
        return self

    def servo_off(self, pattern: str | None = None) -> "RBY1":
        p = pattern or self._cfg.servo_pattern
        if not self._robot.servo_off(p):
            raise RuntimeError(f"servo_off failed for pattern '{p}'")
        log.info("Servo off: %s", p)
        return self

    def enable_control_manager(self) -> "RBY1":
        cm = self._robot.get_control_manager_state()
        if cm.state in (
            rby.ControlManagerState.State.MajorFault,
            rby.ControlManagerState.State.MinorFault,
        ):
            log.warning("Control manager fault (%s). Resetting...", cm.state.name)
            if not self._robot.reset_fault_control_manager():
                raise RuntimeError("reset_fault_control_manager failed")
        if not self._robot.enable_control_manager(
            unlimited_mode_enabled=self._cfg.unlimited_mode
        ):
            raise RuntimeError("enable_control_manager failed")
        log.info("Control manager enabled (unlimited=%s)", self._cfg.unlimited_mode)
        return self

    def initialize(self) -> "RBY1":
        """connect → power_on → servo_on → enable_control_manager 한 번에."""
        return self.connect().power_on().servo_on().enable_control_manager()

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def get_state(self) -> Any:
        """현재 로봇 상태 (rby1_sdk RobotState)."""
        return self._robot.get_state()

    def get_joint_positions(self, component: str) -> np.ndarray:
        """지정 컴포넌트의 현재 관절 위치 [rad].

        Args:
            component: "torso" | "right_arm" | "left_arm" | "head"

        Returns:
            np.ndarray, shape (dof,)
        """
        q_all = np.asarray(self._robot.get_state().position, dtype=float)
        return q_all[self._component_indices(component)].copy()

    def dof(self, component: str) -> int:
        """컴포넌트의 자유도 (degrees of freedom).

        Args:
            component: "torso" | "right_arm" | "left_arm" | "head"
        """
        return len(self._component_indices(component))

    def start_state_stream(self, hz: float | None = None) -> None:
        self._robot.start_state_update(
            lambda s: None, hz or self._cfg.state_update_hz
        )

    def stop_state_stream(self) -> None:
        self._robot.stop_state_update()

    # ------------------------------------------------------------------
    # Blocking commands  (모두 활성 스트림을 자동 pause/resume)
    # ------------------------------------------------------------------

    def movej(
        self,
        torso: np.ndarray | None = None,
        right_arm: np.ndarray | None = None,
        left_arm: np.ndarray | None = None,
        head: np.ndarray | tuple[float, float] | None = None,
        minimum_time: float = 5.0,
    ) -> bool:
        """관절 위치 제어 (blocking). 활성 스트림은 자동 pause/resume."""
        with self._stream_paused():
            body = rby.BodyComponentBasedCommandBuilder()
            has_body = False
            if torso is not None:
                body.set_torso_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(minimum_time)
                    .set_position(np.asarray(torso, dtype=float).tolist())
                )
                has_body = True
            if right_arm is not None:
                body.set_right_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(minimum_time)
                    .set_position(np.asarray(right_arm, dtype=float).tolist())
                )
                has_body = True
            if left_arm is not None:
                body.set_left_arm_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(minimum_time)
                    .set_position(np.asarray(left_arm, dtype=float).tolist())
                )
                has_body = True

            cbc = rby.ComponentBasedCommandBuilder()
            if has_body:
                cbc.set_body_command(body)
            if head is not None:
                h = np.asarray(head, dtype=float)
                cbc.set_head_command(
                    rby.JointPositionCommandBuilder()
                    .set_minimum_time(minimum_time)
                    .set_position([float(h[0]), float(h[1])])
                )

            rv = self._robot.send_command(
                rby.RobotCommandBuilder().set_command(cbc), 1
            ).get()
            ok = rv.finish_code == rby.RobotCommandFeedback.FinishCode.Ok
            if not ok:
                log.warning("movej finished with code: %s", rv.finish_code)
            return ok

    def zero_pose(self, minimum_time: float = 5.0) -> bool:
        """모든 관절을 0으로 이동. 활성 스트림 자동 pause/resume."""
        m = self._model
        return self.movej(
            torso=np.zeros(len(m.torso_idx)),
            right_arm=np.zeros(len(m.right_arm_idx)),
            left_arm=np.zeros(len(m.left_arm_idx)),
            minimum_time=minimum_time,
        )

    def ready_pose(self, minimum_time: float = 5.0) -> bool:
        """SDK 표준 ready pose로 이동. 활성 스트림 자동 pause/resume."""
        model_name = self._model.model_name
        pose = _READY_POSE.get(model_name)
        if pose is None:
            log.warning("No ready pose defined for model '%s', using zeros.", model_name)
            return self.zero_pose(minimum_time)
        return self.movej(
            torso=pose["torso"],
            right_arm=pose["right_arm"],
            left_arm=pose["left_arm"],
            minimum_time=minimum_time,
        )

    def joint_impedance_control(
        self,
        right_arm: np.ndarray | None = None,
        left_arm: np.ndarray | None = None,
        torso: np.ndarray | None = None,
        stiffness: float = 100.0,
        damping_ratio: float = 1.0,
        torque_limit: float = 30.0,
        minimum_time: float = 5.0,
        control_hold_time: float = 10.0,
    ) -> bool:
        """관절 임피던스 제어 (blocking). torso=JointPosition, arm=JointImpedance.
        활성 스트림 자동 pause/resume."""
        with self._stream_paused():
            body = rby.BodyComponentBasedCommandBuilder()
            has_body = False
            header = rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time)

            if torso is not None:
                q = np.asarray(torso, dtype=float).tolist()
                body.set_torso_command(
                    rby.JointPositionCommandBuilder()
                    .set_command_header(header)
                    .set_position(q)
                    .set_minimum_time(minimum_time)
                )
                has_body = True

            for component, q_arr in (("right_arm", right_arm), ("left_arm", left_arm)):
                if q_arr is None:
                    continue
                q = np.asarray(q_arr, dtype=float)
                dof = len(q)
                builder = (
                    rby.JointImpedanceControlCommandBuilder()
                    .set_command_header(header)
                    .set_position(q.tolist())
                    .set_minimum_time(minimum_time)
                    .set_stiffness([stiffness] * dof)
                    .set_damping_ratio(damping_ratio)
                    .set_torque_limit([torque_limit] * dof)
                )
                getattr(body, f"set_{component}_command")(builder)
                has_body = True

            if not has_body:
                raise ValueError("joint_impedance_control: no component specified")

            rv = self._robot.send_command(
                rby.RobotCommandBuilder().set_command(
                    rby.ComponentBasedCommandBuilder().set_body_command(body)
                ),
                1,
            ).get()
            ok = rv.finish_code == rby.RobotCommandFeedback.FinishCode.Ok
            if not ok:
                log.warning("joint_impedance_control finished with code: %s", rv.finish_code)
            return ok

    def move_cartesian(
        self,
        right_arm: np.ndarray | None = None,
        left_arm: np.ndarray | None = None,
        linear_velocity_limit: float = 0.5,
        angular_velocity_limit: float = 1.5707963,
        accel_limit_scaling: float = 1.0,
        minimum_time: float = 5.0,
        stop_position_error: float = 1e-3,
        stop_orientation_error: float = 1e-3,
    ) -> bool:
        """Cartesian 위치 제어 (blocking, ee_right/ee_left 프레임).
        활성 스트림 자동 pause/resume."""
        with self._stream_paused():
            body = rby.BodyComponentBasedCommandBuilder()
            has_body = False
            header = rby.CommandHeaderBuilder().set_control_hold_time(1e6)

            frame_map = {"right_arm": "ee_right", "left_arm": "ee_left"}
            for component, T in (("right_arm", right_arm), ("left_arm", left_arm)):
                if T is None:
                    continue
                cart = (
                    rby.CartesianCommandBuilder()
                    .set_command_header(header)
                    .set_minimum_time(minimum_time)
                    .set_stop_position_tracking_error(stop_position_error)
                    .set_stop_orientation_tracking_error(stop_orientation_error)
                    .add_target(
                        "base",
                        frame_map[component],
                        np.asarray(T, dtype=float),
                        linear_velocity_limit,
                        angular_velocity_limit,
                        accel_limit_scaling,
                    )
                )
                getattr(body, f"set_{component}_command")(cart)
                has_body = True

            if not has_body:
                raise ValueError("move_cartesian: no component specified")

            rv = self._robot.send_command(
                rby.RobotCommandBuilder().set_command(
                    rby.ComponentBasedCommandBuilder().set_body_command(body)
                ),
                1,
            ).get()
            ok = rv.finish_code == rby.RobotCommandFeedback.FinishCode.Ok
            if not ok:
                log.warning("move_cartesian finished with code: %s", rv.finish_code)
            return ok

    # ------------------------------------------------------------------
    # Streaming
    # ------------------------------------------------------------------

    def create_stream(self) -> "RBY1Stream":
        """실시간 제어용 스트림 생성.

        이전에 생성한 스트림이 있으면 자동으로 cancel 후 새로 생성.
        생성된 스트림은 내부적으로 참조되어 blocking command 호출 시
        자동 pause/resume 대상이 됨.
        """
        from rby1_workbench.robot.stream import RBY1Stream
        if self._active_stream is not None:
            self._active_stream.cancel()
        stream = RBY1Stream(self._robot, self._cfg)
        self._active_stream = stream
        return stream

    # ------------------------------------------------------------------
    # FK helpers
    # ------------------------------------------------------------------

    def get_ee_pose(self, arm: str) -> np.ndarray:
        """현재 EE 포즈 (4×4, base 프레임).

        arm: "right_arm" | "left_arm"
        """
        if arm not in _FK_IDX:
            raise ValueError(f"arm must be 'right_arm' or 'left_arm', got '{arm}'")
        q = np.asarray(self._robot.get_state().position, dtype=float)
        with self._dyn_lock:
            self._dyn_state.set_q(q)
            self._dyn_robot.compute_forward_kinematics(self._dyn_state)
            T = self._dyn_robot.compute_transformation(
                self._dyn_state, _FK_IDX["base"], _FK_IDX[arm]
            )
        return np.asarray(T, dtype=float).copy()

    def get_torso_pose(self) -> np.ndarray:
        """현재 토르소 포즈 (4×4, base → link_torso_5)."""
        q = np.asarray(self._robot.get_state().position, dtype=float)
        with self._dyn_lock:
            self._dyn_state.set_q(q)
            self._dyn_robot.compute_forward_kinematics(self._dyn_state)
            T = self._dyn_robot.compute_transformation(
                self._dyn_state, _FK_IDX["base"], _FK_IDX["torso"]
            )
        return np.asarray(T, dtype=float).copy()

    # ------------------------------------------------------------------
    # Sub-controllers
    # ------------------------------------------------------------------

    @property
    def head(self) -> "HeadController":
        if self._head_ctrl is None:
            from rby1_workbench.robot.head import HeadController
            self._head_ctrl = HeadController(self._robot, self._model)
        return self._head_ctrl

    @property
    def gripper(self) -> "GripperController":
        if self._gripper_ctrl is None:
            from rby1_workbench.robot.gripper import GripperController
            self._gripper_ctrl = GripperController(self._robot, self._cfg.gripper)
        return self._gripper_ctrl

    # ------------------------------------------------------------------
    # Raw access
    # ------------------------------------------------------------------

    @property
    def model(self) -> Any:
        return self._model

    @property
    def sdk_robot(self) -> Any:
        """rby1_sdk 로봇 객체 직접 접근 (escape hatch)."""
        return self._robot

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _component_indices(self, component: str) -> list[int]:
        attr = _COMPONENT_ATTR.get(component)
        if attr is None:
            raise ValueError(
                f"Unknown component '{component}'. Valid: {list(_COMPONENT_ATTR)}"
            )
        return list(getattr(self._model, attr))

    @contextmanager
    def _stream_paused(self) -> Iterator[None]:
        """활성 스트림이 있고 아직 paused 상태가 아닐 때만 pause/resume."""
        stream = self._active_stream
        own_pause = stream is not None and not stream.paused
        if own_pause:
            stream.pause()
        try:
            yield
        finally:
            if own_pause and stream is not None:
                stream.resume()

    def _setup_fk(self) -> None:
        self._dyn_robot = self._robot.get_dynamics()
        self._dyn_state = self._dyn_robot.make_state(
            _FK_FRAMES, self._model.robot_joint_names
        )
