"""High-level RB-Y1 control facade.

Public control API:

```python
robot = RBY1(cfg, backend="direct")
robot.initialize()

robot.move(mode="joint", torso=q_torso, right_arm=q_right, left_arm=q_left)
robot.move(mode="cartesian", right_arm=T_right, left_arm=T_left, torso=q_torso)

stream = robot.open_stream(mode="cartesian")
stream.send(torso=q_torso, right_arm=T_right, left_arm=T_left, head=q_head)
stream.pause()
stream.resume()
stream.close()
```
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import logging
import threading
from contextlib import contextmanager
from typing import Any, Iterator, Mapping, Protocol

import numpy as np
import rby1_sdk as rby
from omegaconf import DictConfig, OmegaConf

from rby1_workbench.control.presets import ready_pose_targets_for_model
from rby1_workbench.robot.stream import (
    RBY1Stream,
    make_direct_stream,
    make_remote_stream,
)

log = logging.getLogger(__name__)

_COMPONENT_ATTR = {
    "torso": "torso_idx",
    "right_arm": "right_arm_idx",
    "left_arm": "left_arm_idx",
    "head": "head_idx",
}
_FK_FRAMES = ["base", "ee_right", "ee_left", "link_torso_5"]
_FK_IDX = {"base": 0, "right_arm": 1, "left_arm": 2, "torso": 3}


def _coerce_timestamp_seconds(value: Any) -> float | None:
    if value is None:
        return None
    if isinstance(value, datetime):
        return float(value.timestamp())
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


@dataclass(slots=True)
class RobotModelInfo:
    """Serializable subset of SDK model info used by public API."""

    model_name: str
    robot_joint_names: list[str]
    torso_idx: list[int]
    right_arm_idx: list[int]
    left_arm_idx: list[int]
    head_idx: list[int]

    @classmethod
    def from_sdk_model(cls, model: Any) -> "RobotModelInfo":
        return cls(
            model_name=str(model.model_name),
            robot_joint_names=list(model.robot_joint_names),
            torso_idx=list(model.torso_idx),
            right_arm_idx=list(model.right_arm_idx),
            left_arm_idx=list(model.left_arm_idx),
            head_idx=list(model.head_idx),
        )

    @classmethod
    def from_payload(cls, payload: Mapping[str, Any]) -> "RobotModelInfo":
        return cls(
            model_name=str(payload["model_name"]),
            robot_joint_names=list(payload["robot_joint_names"]),
            torso_idx=[int(v) for v in payload["torso_idx"]],
            right_arm_idx=[int(v) for v in payload["right_arm_idx"]],
            left_arm_idx=[int(v) for v in payload["left_arm_idx"]],
            head_idx=[int(v) for v in payload["head_idx"]],
        )

    def to_payload(self) -> dict[str, Any]:
        return {
            "model_name": self.model_name,
            "robot_joint_names": list(self.robot_joint_names),
            "torso_idx": list(self.torso_idx),
            "right_arm_idx": list(self.right_arm_idx),
            "left_arm_idx": list(self.left_arm_idx),
            "head_idx": list(self.head_idx),
        }


@dataclass(slots=True)
class RobotStateView:
    """Backend-agnostic robot state view."""

    position: np.ndarray
    timestamp: float | None = None


class _Backend(Protocol):
    model: RobotModelInfo

    def connect(self) -> None:
        ...

    def power_on(self, pattern: str | None = None) -> None:
        ...

    def power_off(self, pattern: str | None = None) -> None:
        ...

    def servo_on(self, pattern: str | None = None) -> None:
        ...

    def servo_off(self, pattern: str | None = None) -> None:
        ...

    def enable_control_manager(self) -> None:
        ...

    def initialize(self) -> None:
        ...

    def get_state(self) -> RobotStateView:
        ...

    def start_state_stream(self, hz: float | None = None) -> None:
        ...

    def stop_state_stream(self) -> None:
        ...

    def move(self, **kwargs: Any) -> bool:
        ...

    def ready(self, minimum_time: float = 5.0) -> bool:
        ...

    def zero(self, minimum_time: float = 5.0) -> bool:
        ...

    def open_stream(self, mode: str | Mapping[str, str] | None = None) -> RBY1Stream:
        ...

    def get_ee_pose(self, arm: str) -> np.ndarray:
        ...

    def get_torso_pose(self) -> np.ndarray:
        ...

    def register_static_frame(
        self,
        parent: str,
        child: str,
        transform: np.ndarray,
        label: str | None = None,
    ) -> None:
        ...

    def get_transform(self, parent: str, child: str) -> np.ndarray:
        ...

    @property
    def sdk_robot(self) -> Any:
        ...


class _DirectBackend:
    def __init__(self, cfg: DictConfig):
        self._cfg = cfg
        self._robot: Any = None
        self._sdk_model: Any = None
        self._model = RobotModelInfo("", [], [], [], [], [])
        self._dyn_robot: Any = None
        self._dyn_state: Any = None
        self._dyn_lock = threading.Lock()
        self._kin: Any = None
        self._gripper_ctrl: Any = None
        self._active_stream: RBY1Stream | None = None

    @property
    def model(self) -> RobotModelInfo:
        return self._model

    def connect(self) -> None:
        log.info("Connecting to RBY1 at %s (model=%s)", self._cfg.address, self._cfg.model)
        self._robot = rby.create_robot(self._cfg.address, self._cfg.model)
        if not self._robot.connect():
            raise RuntimeError(f"Failed to connect to {self._cfg.address}")
        self._sdk_model = self._robot.model()
        self._model = RobotModelInfo.from_sdk_model(self._sdk_model)
        self._setup_fk()
        self._auto_load_calibration()

    def _auto_load_calibration(self) -> None:
        calib_cfg = getattr(self._cfg, "calib", None)
        if calib_cfg is None or not calib_cfg.auto_load:
            return
        from rby1_workbench.calibration.hand_eye_solver import (
            HandEyeSolver,
            resolve_calibration_output_dir,
        )

        output_dir = resolve_calibration_output_dir(calib_cfg.output_dir)
        result = HandEyeSolver.load_latest(output_dir)
        if result is None:
            log.info("Calibration auto-load: no file found in '%s'.", output_dir)
            return
        transform, frame_from, frame_to = result
        self.register_static_frame(frame_from, frame_to, transform)
        log.info("Calibration auto-loaded: %s -> %s", frame_from, frame_to)

    def power_on(self, pattern: str | None = None) -> None:
        target = pattern or self._cfg.power_pattern
        if not self._robot.is_power_on(target):
            if not self._robot.power_on(target):
                raise RuntimeError(f"power_on failed for pattern '{target}'")

    def power_off(self, pattern: str | None = None) -> None:
        target = pattern or self._cfg.power_pattern
        if not self._robot.power_off(target):
            raise RuntimeError(f"power_off failed for pattern '{target}'")

    def servo_on(self, pattern: str | None = None) -> None:
        target = pattern or self._cfg.servo_pattern
        if not self._robot.is_servo_on(target):
            if not self._robot.servo_on(target):
                raise RuntimeError(f"servo_on failed for pattern '{target}'")

    def servo_off(self, pattern: str | None = None) -> None:
        target = pattern or self._cfg.servo_pattern
        if not self._robot.servo_off(target):
            raise RuntimeError(f"servo_off failed for pattern '{target}'")

    def enable_control_manager(self) -> None:
        cm = self._robot.get_control_manager_state()
        if cm.state in (
            rby.ControlManagerState.State.MajorFault,
            rby.ControlManagerState.State.MinorFault,
        ):
            if not self._robot.reset_fault_control_manager():
                raise RuntimeError("reset_fault_control_manager failed")
        if not self._robot.enable_control_manager(
            unlimited_mode_enabled=self._cfg.unlimited_mode
        ):
            raise RuntimeError("enable_control_manager failed")

    def initialize(self) -> None:
        self.connect()
        self.power_on()
        self.servo_on()
        self.enable_control_manager()

    def get_state(self) -> RobotStateView:
        state = self._robot.get_state()
        timestamp = getattr(state, "timestamp", None)
        return RobotStateView(
            position=np.asarray(state.position, dtype=float).copy(),
            timestamp=_coerce_timestamp_seconds(timestamp),
        )

    def start_state_stream(self, hz: float | None = None) -> None:
        self._robot.start_state_update(lambda state: None, hz or self._cfg.state_update_hz)

    def stop_state_stream(self) -> None:
        self._robot.stop_state_update()

    def move(
        self,
        *,
        mode: str = "joint",
        torso: np.ndarray | None = None,
        right_arm: np.ndarray | None = None,
        left_arm: np.ndarray | None = None,
        head: np.ndarray | tuple[float, float] | None = None,
        minimum_time: float = 5.0,
        control_hold_time: float | None = None,
        stiffness: float = 100.0,
        damping_ratio: float = 1.0,
        torque_limit: float = 30.0,
        linear_velocity_limit: float = 0.5,
        angular_velocity_limit: float = 1.5707963,
        accel_limit_scaling: float = 1.0,
        stop_position_error: float = 1e-3,
        stop_orientation_error: float = 1e-3,
    ) -> bool:
        mode_key = mode.lower().strip()
        with self._stream_paused():
            if mode_key == "joint":
                command = self._build_joint_move(
                    torso=torso,
                    right_arm=right_arm,
                    left_arm=left_arm,
                    head=head,
                    minimum_time=minimum_time,
                    control_hold_time=control_hold_time,
                )
            elif mode_key == "impedance":
                command = self._build_impedance_move(
                    torso=torso,
                    right_arm=right_arm,
                    left_arm=left_arm,
                    head=head,
                    minimum_time=minimum_time,
                    control_hold_time=control_hold_time,
                    stiffness=stiffness,
                    damping_ratio=damping_ratio,
                    torque_limit=torque_limit,
                )
            elif mode_key == "cartesian":
                command = self._build_cartesian_move(
                    torso=torso,
                    right_arm=right_arm,
                    left_arm=left_arm,
                    head=head,
                    minimum_time=minimum_time,
                    control_hold_time=control_hold_time,
                    linear_velocity_limit=linear_velocity_limit,
                    angular_velocity_limit=angular_velocity_limit,
                    accel_limit_scaling=accel_limit_scaling,
                    stop_position_error=stop_position_error,
                    stop_orientation_error=stop_orientation_error,
                )
            else:
                raise ValueError(f"Unsupported move mode: {mode}")
            return self._send_command(command)

    def ready(self, minimum_time: float = 5.0) -> bool:
        pose = ready_pose_targets_for_model(self._model)
        return self.move(minimum_time=minimum_time, **pose)

    def zero(self, minimum_time: float = 5.0) -> bool:
        return self.move(
            minimum_time=minimum_time,
            torso=np.zeros(len(self._model.torso_idx), dtype=float),
            right_arm=np.zeros(len(self._model.right_arm_idx), dtype=float),
            left_arm=np.zeros(len(self._model.left_arm_idx), dtype=float),
        )

    def open_stream(self, mode: str | Mapping[str, str] | None = None) -> RBY1Stream:
        if self._active_stream is not None and not self._active_stream.closed:
            self._active_stream.close()
        self._active_stream = make_direct_stream(
            self._robot,
            self._cfg,
            mode,
            on_close=self._clear_active_stream,
        )
        return self._active_stream

    def _clear_active_stream(self) -> None:
        self._active_stream = None

    def get_ee_pose(self, arm: str) -> np.ndarray:
        if arm not in {"right_arm", "left_arm"}:
            raise ValueError(f"arm must be 'right_arm' or 'left_arm', got '{arm}'")
        q_all = np.asarray(self._robot.get_state().position, dtype=float)
        with self._dyn_lock:
            self._dyn_state.set_q(q_all)
            self._dyn_robot.compute_forward_kinematics(self._dyn_state)
            transform = self._dyn_robot.compute_transformation(
                self._dyn_state,
                _FK_IDX["base"],
                _FK_IDX[arm],
            )
        return np.asarray(transform, dtype=float).copy()

    def get_torso_pose(self) -> np.ndarray:
        q_all = np.asarray(self._robot.get_state().position, dtype=float)
        with self._dyn_lock:
            self._dyn_state.set_q(q_all)
            self._dyn_robot.compute_forward_kinematics(self._dyn_state)
            transform = self._dyn_robot.compute_transformation(
                self._dyn_state,
                _FK_IDX["base"],
                _FK_IDX["torso"],
            )
        return np.asarray(transform, dtype=float).copy()

    def register_static_frame(
        self,
        parent: str,
        child: str,
        transform: np.ndarray,
        label: str | None = None,
    ) -> None:
        from rby1_workbench.robot.kinematics import RobotKinematics

        if self._kin is None:
            flip = list(
                OmegaConf.select(self._cfg, "kinematics.flip_joints", default=[])
            )
            self._kin = RobotKinematics(self._robot, flip_joints=flip)
        self._kin.register_static_frame(parent, child, transform, label=label)

    def get_transform(self, parent: str, child: str) -> np.ndarray:
        from rby1_workbench.robot.kinematics import RobotKinematics

        if self._kin is None:
            flip = list(
                OmegaConf.select(self._cfg, "kinematics.flip_joints", default=[])
            )
            self._kin = RobotKinematics(self._robot, flip_joints=flip)
        q_all = np.asarray(self._robot.get_state().position, dtype=float)
        result = self._kin.compute(q_all)
        base_t_parent = result.base_transforms[parent]
        base_t_child = result.base_transforms[child]
        return (np.linalg.inv(base_t_parent) @ base_t_child).copy()

    @property
    def sdk_robot(self) -> Any:
        return self._robot

    @property
    def gripper(self) -> Any:
        if self._gripper_ctrl is None:
            from rby1_workbench.robot.gripper import GripperController

            self._gripper_ctrl = GripperController(self._robot, self._cfg.gripper)
        return self._gripper_ctrl

    def _build_joint_move(
        self,
        *,
        torso: np.ndarray | None,
        right_arm: np.ndarray | None,
        left_arm: np.ndarray | None,
        head: np.ndarray | tuple[float, float] | None,
        minimum_time: float,
        control_hold_time: float | None,
    ) -> Any:
        header = (
            None
            if control_hold_time is None
            else rby.CommandHeaderBuilder().set_control_hold_time(control_hold_time)
        )
        body_builder = rby.BodyComponentBasedCommandBuilder()
        has_body = False

        for component, target in (
            ("torso", torso),
            ("right_arm", right_arm),
            ("left_arm", left_arm),
        ):
            if target is None:
                continue
            builder = rby.JointPositionCommandBuilder()
            if header is not None:
                builder.set_command_header(header)
            builder.set_minimum_time(minimum_time).set_position(
                np.asarray(target, dtype=float).tolist()
            )
            getattr(body_builder, f"set_{component}_command")(builder)
            has_body = True

        component_builder = rby.ComponentBasedCommandBuilder()
        if has_body:
            component_builder.set_body_command(body_builder)
        if head is not None:
            q_head = np.asarray(head, dtype=float)
            builder = rby.JointPositionCommandBuilder()
            if header is not None:
                builder.set_command_header(header)
            builder.set_minimum_time(minimum_time).set_position(
                [float(q_head[0]), float(q_head[1])]
            )
            component_builder.set_head_command(builder)

        if not has_body and head is None:
            raise ValueError("move(mode='joint'): no component specified")
        return rby.RobotCommandBuilder().set_command(component_builder)

    def _build_impedance_move(
        self,
        *,
        torso: np.ndarray | None,
        right_arm: np.ndarray | None,
        left_arm: np.ndarray | None,
        head: np.ndarray | tuple[float, float] | None,
        minimum_time: float,
        control_hold_time: float | None,
        stiffness: float,
        damping_ratio: float,
        torque_limit: float,
    ) -> Any:
        hold = 10.0 if control_hold_time is None else control_hold_time
        header = rby.CommandHeaderBuilder().set_control_hold_time(hold)
        body_builder = rby.BodyComponentBasedCommandBuilder()
        has_body = False

        if torso is not None:
            body_builder.set_torso_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(header)
                .set_position(np.asarray(torso, dtype=float).tolist())
                .set_minimum_time(minimum_time)
            )
            has_body = True

        for component, target in (("right_arm", right_arm), ("left_arm", left_arm)):
            if target is None:
                continue
            q_target = np.asarray(target, dtype=float)
            body_builder_method = getattr(body_builder, f"set_{component}_command")
            body_builder_method(
                rby.JointImpedanceControlCommandBuilder()
                .set_command_header(header)
                .set_position(q_target.tolist())
                .set_minimum_time(minimum_time)
                .set_stiffness([stiffness] * len(q_target))
                .set_damping_ratio(damping_ratio)
                .set_torque_limit([torque_limit] * len(q_target))
            )
            has_body = True

        component_builder = rby.ComponentBasedCommandBuilder()
        if has_body:
            component_builder.set_body_command(body_builder)
        if head is not None:
            q_head = np.asarray(head, dtype=float)
            component_builder.set_head_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(header)
                .set_position([float(q_head[0]), float(q_head[1])])
                .set_minimum_time(minimum_time)
            )

        if not has_body and head is None:
            raise ValueError("move(mode='impedance'): no component specified")
        return rby.RobotCommandBuilder().set_command(component_builder)

    def _build_cartesian_move(
        self,
        *,
        torso: np.ndarray | None,
        right_arm: np.ndarray | None,
        left_arm: np.ndarray | None,
        head: np.ndarray | tuple[float, float] | None,
        minimum_time: float,
        control_hold_time: float | None,
        linear_velocity_limit: float,
        angular_velocity_limit: float,
        accel_limit_scaling: float,
        stop_position_error: float,
        stop_orientation_error: float,
    ) -> Any:
        hold = 1e6 if control_hold_time is None else control_hold_time
        header = rby.CommandHeaderBuilder().set_control_hold_time(hold)
        body_builder = rby.BodyComponentBasedCommandBuilder()
        has_body = False

        if torso is not None:
            body_builder.set_torso_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(header)
                .set_position(np.asarray(torso, dtype=float).tolist())
                .set_minimum_time(minimum_time)
            )
            has_body = True

        frame_map = {"right_arm": "ee_right", "left_arm": "ee_left"}
        for component, target in (("right_arm", right_arm), ("left_arm", left_arm)):
            if target is None:
                continue
            body_builder_method = getattr(body_builder, f"set_{component}_command")
            body_builder_method(
                rby.CartesianCommandBuilder()
                .set_command_header(header)
                .set_minimum_time(minimum_time)
                .set_stop_position_tracking_error(stop_position_error)
                .set_stop_orientation_tracking_error(stop_orientation_error)
                .add_target(
                    "base",
                    frame_map[component],
                    np.asarray(target, dtype=float),
                    linear_velocity_limit,
                    angular_velocity_limit,
                    accel_limit_scaling,
                )
            )
            has_body = True

        component_builder = rby.ComponentBasedCommandBuilder()
        if has_body:
            component_builder.set_body_command(body_builder)
        if head is not None:
            q_head = np.asarray(head, dtype=float)
            component_builder.set_head_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(header)
                .set_position([float(q_head[0]), float(q_head[1])])
                .set_minimum_time(minimum_time)
            )

        if not has_body and head is None:
            raise ValueError("move(mode='cartesian'): no component specified")
        return rby.RobotCommandBuilder().set_command(component_builder)

    def _send_command(self, command: Any) -> bool:
        feedback = self._robot.send_command(command, 1).get()
        ok = feedback.finish_code == rby.RobotCommandFeedback.FinishCode.Ok
        if not ok:
            log.warning("Robot move finished with code: %s", feedback.finish_code)
        return ok

    def _component_indices(self, component: str) -> list[int]:
        attr = _COMPONENT_ATTR.get(component)
        if attr is None:
            raise ValueError(
                f"Unknown component '{component}'. Valid: {list(_COMPONENT_ATTR)}"
            )
        return list(getattr(self._model, attr))

    @contextmanager
    def _stream_paused(self) -> Iterator[None]:
        stream = self._active_stream
        own_pause = stream is not None and not stream.paused and not stream.closed
        if own_pause:
            stream.pause()
        try:
            yield
        finally:
            if own_pause and stream is not None and not stream.closed:
                stream.resume()

    def _setup_fk(self) -> None:
        self._dyn_robot = self._robot.get_dynamics()
        self._dyn_state = self._dyn_robot.make_state(
            _FK_FRAMES, self._sdk_model.robot_joint_names
        )


class _ClientBackend:
    def __init__(self, cfg: DictConfig, endpoint: str):
        from rby1_workbench.robot.rpc import RobotRpcClient

        self._cfg = cfg
        self._client = RobotRpcClient(endpoint)
        self._model: RobotModelInfo | None = None

    @property
    def model(self) -> RobotModelInfo:
        if self._model is None:
            payload = self._client.request("model_info")["model"]
            self._model = RobotModelInfo.from_payload(payload)
        return self._model

    def connect(self) -> None:
        response = self._client.request("connect")
        self._model = RobotModelInfo.from_payload(response["model"])

    def power_on(self, pattern: str | None = None) -> None:
        self._client.request("power_on", pattern=pattern)

    def power_off(self, pattern: str | None = None) -> None:
        self._client.request("power_off", pattern=pattern)

    def servo_on(self, pattern: str | None = None) -> None:
        self._client.request("servo_on", pattern=pattern)

    def servo_off(self, pattern: str | None = None) -> None:
        self._client.request("servo_off", pattern=pattern)

    def enable_control_manager(self) -> None:
        self._client.request("enable_control_manager")

    def initialize(self) -> None:
        response = self._client.request("initialize")
        self._model = RobotModelInfo.from_payload(response["model"])

    def get_state(self) -> RobotStateView:
        payload = self._client.request("get_state")
        return RobotStateView(
            position=np.asarray(payload["position"], dtype=float),
            timestamp=payload.get("timestamp"),
        )

    def start_state_stream(self, hz: float | None = None) -> None:
        raise NotImplementedError("Remote backend does not expose state callbacks")

    def stop_state_stream(self) -> None:
        raise NotImplementedError("Remote backend does not expose state callbacks")

    def move(self, **kwargs: Any) -> bool:
        payload = self._client.request("move", **_encode_move_kwargs(kwargs))
        return bool(payload["ok"])

    def ready(self, minimum_time: float = 5.0) -> bool:
        payload = self._client.request("ready", minimum_time=minimum_time)
        return bool(payload["ok"])

    def zero(self, minimum_time: float = 5.0) -> bool:
        payload = self._client.request("zero", minimum_time=minimum_time)
        return bool(payload["ok"])

    def open_stream(self, mode: str | Mapping[str, str] | None = None) -> RBY1Stream:
        payload = self._client.request("open_stream", mode=_encode_mode(mode))
        return make_remote_stream(self._client, payload["stream_id"])

    def get_ee_pose(self, arm: str) -> np.ndarray:
        payload = self._client.request("get_ee_pose", arm=arm)
        return np.asarray(payload["transform"], dtype=float)

    def get_torso_pose(self) -> np.ndarray:
        payload = self._client.request("get_torso_pose")
        return np.asarray(payload["transform"], dtype=float)

    def register_static_frame(
        self,
        parent: str,
        child: str,
        transform: np.ndarray,
        label: str | None = None,
    ) -> None:
        self._client.request(
            "register_static_frame",
            parent=parent,
            child=child,
            transform=np.asarray(transform, dtype=float).tolist(),
            label=label,
        )

    def get_transform(self, parent: str, child: str) -> np.ndarray:
        payload = self._client.request("get_transform", parent=parent, child=child)
        return np.asarray(payload["transform"], dtype=float)

    @property
    def sdk_robot(self) -> Any:
        raise RuntimeError("sdk_robot is only available for direct backend")


class RBY1:
    """Unified RB-Y1 control facade.

    `backend="direct"` talks to the SDK locally.
    `backend="client"` forwards the same API to an `RBY1Server`.

    Joint target contract:

    - `torso=q_torso`:
      array-like of shape `(len(model.torso_idx),)` in radians.
      Joint order follows `model.torso_idx`.
    - `right_arm=q_right`:
      array-like of shape `(len(model.right_arm_idx),)` in radians.
      Joint order follows `model.right_arm_idx`, typically
      `right_arm_0 ... right_arm_6`.
    - `left_arm=q_left`:
      array-like of shape `(len(model.left_arm_idx),)` in radians.
      Joint order follows `model.left_arm_idx`, typically
      `left_arm_0 ... left_arm_6`.
    - `head=q_head`:
      array-like of shape `(len(model.head_idx),)` in radians.
      Joint order follows `model.head_idx`, typically `[head_0, head_1]`.

    Cartesian target contract:

    - `right_arm=T_right`:
      4x4 homogeneous transform `base -> ee_right`
    - `left_arm=T_left`:
      4x4 homogeneous transform `base -> ee_left`

    In `move(mode="cartesian", ...)` and `open_stream(mode="cartesian")`,
    torso and head remain joint targets while arm targets are Cartesian.
    """

    def __init__(
        self,
        cfg: DictConfig,
        *,
        backend: str = "direct",
        endpoint: str | None = None,
    ):
        self._cfg = cfg
        backend_key = backend.lower().strip()
        if backend_key == "direct":
            self._backend: _Backend = _DirectBackend(cfg)
        elif backend_key == "client":
            if not endpoint:
                raise ValueError("endpoint is required for backend='client'")
            self._backend = _ClientBackend(cfg, endpoint)
        else:
            raise ValueError(f"Unsupported backend: {backend}")

    def connect(self) -> "RBY1":
        self._backend.connect()
        return self

    def power_on(self, pattern: str | None = None) -> "RBY1":
        self._backend.power_on(pattern)
        return self

    def power_off(self, pattern: str | None = None) -> "RBY1":
        self._backend.power_off(pattern)
        return self

    def servo_on(self, pattern: str | None = None) -> "RBY1":
        self._backend.servo_on(pattern)
        return self

    def servo_off(self, pattern: str | None = None) -> "RBY1":
        self._backend.servo_off(pattern)
        return self

    def enable_control_manager(self) -> "RBY1":
        self._backend.enable_control_manager()
        return self

    def initialize(self) -> "RBY1":
        self._backend.initialize()
        return self

    @property
    def model(self) -> RobotModelInfo:
        return self._backend.model

    def get_state(self) -> RobotStateView:
        return self._backend.get_state()

    def get_joint_positions(self, component: str) -> np.ndarray:
        q_all = self.get_state().position
        return q_all[self._component_indices(component)].copy()

    def dof(self, component: str) -> int:
        return len(self._component_indices(component))

    def start_state_stream(self, hz: float | None = None) -> None:
        self._backend.start_state_stream(hz)

    def stop_state_stream(self) -> None:
        self._backend.stop_state_stream()

    def move(
        self,
        *,
        mode: str = "impedance",
        torso: np.ndarray | None = None,
        right_arm: np.ndarray | None = None,
        left_arm: np.ndarray | None = None,
        head: np.ndarray | tuple[float, float] | None = None,
        minimum_time: float = 5.0,
        control_hold_time: float | None = None,
        stiffness: float = 100.0,
        damping_ratio: float = 1.0,
        torque_limit: float = 30.0,
        linear_velocity_limit: float = 0.5,
        angular_velocity_limit: float = 1.5707963,
        accel_limit_scaling: float = 1.0,
        stop_position_error: float = 1e-3,
        stop_orientation_error: float = 1e-3,
    ) -> bool:
        """Execute one blocking robot move.

        `mode="joint"`:
        `torso`, `right_arm`, `left_arm`, `head` are joint targets.

        `mode="impedance"`:
        torso is held with joint-position control, arms use joint impedance
        control, and head remains a joint target.

        `mode="cartesian"`:
        `right_arm=T_right` and `left_arm=T_left` are 4x4 `base -> ee_*`
        transforms, while `torso=q_torso` and `head=q_head` remain joint
        targets.
        """
        return self._backend.move(
            mode=mode,
            torso=torso,
            right_arm=right_arm,
            left_arm=left_arm,
            head=head,
            minimum_time=minimum_time,
            control_hold_time=control_hold_time,
            stiffness=stiffness,
            damping_ratio=damping_ratio,
            torque_limit=torque_limit,
            linear_velocity_limit=linear_velocity_limit,
            angular_velocity_limit=angular_velocity_limit,
            accel_limit_scaling=accel_limit_scaling,
            stop_position_error=stop_position_error,
            stop_orientation_error=stop_orientation_error,
        )

    def ready(self, minimum_time: float = 5.0) -> bool:
        return self._backend.ready(minimum_time)

    def zero(self, minimum_time: float = 5.0) -> bool:
        return self._backend.zero(minimum_time)

    def open_stream(
        self,
        mode: str | Mapping[str, str] | None = None,
    ) -> RBY1Stream:
        return self._backend.open_stream(mode)

    def get_ee_pose(self, arm: str) -> np.ndarray:
        return self._backend.get_ee_pose(arm)

    def get_torso_pose(self) -> np.ndarray:
        return self._backend.get_torso_pose()

    def register_static_frame(
        self,
        parent: str,
        child: str,
        transform: np.ndarray,
        label: str | None = None,
    ) -> None:
        self._backend.register_static_frame(parent, child, transform, label=label)

    def get_transform(self, parent: str, child: str) -> np.ndarray:
        return self._backend.get_transform(parent, child)

    @property
    def sdk_robot(self) -> Any:
        return self._backend.sdk_robot

    @property
    def gripper(self) -> Any:
        backend = self._backend
        if not isinstance(backend, _DirectBackend):
            raise RuntimeError("gripper is only available for direct backend")
        return backend.gripper

    def _component_indices(self, component: str) -> list[int]:
        attr = _COMPONENT_ATTR.get(component)
        if attr is None:
            raise ValueError(
                f"Unknown component '{component}'. Valid: {list(_COMPONENT_ATTR)}"
            )
        return list(getattr(self.model, attr))


def _encode_mode(mode: str | Mapping[str, str] | None) -> Any:
    if mode is None or isinstance(mode, str):
        return mode
    return dict(mode)


def _encode_move_kwargs(kwargs: Mapping[str, Any]) -> dict[str, Any]:
    encoded = dict(kwargs)
    for key in ("torso", "right_arm", "left_arm", "head"):
        value = encoded.get(key)
        if value is not None:
            encoded[key] = np.asarray(value, dtype=float).tolist()
    return encoded
