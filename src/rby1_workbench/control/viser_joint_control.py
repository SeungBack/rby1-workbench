"""Viser control panel modeled after examples/keyboard_control.py."""

from __future__ import annotations

import importlib
import logging
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np
from omegaconf import DictConfig, OmegaConf

from rby1_workbench import RBY1, load_rby1_config
from rby1_workbench.control.joint_commands import JointGroupSpec
from rby1_workbench.control.presets import ready_pose_targets_for_model
from rby1_workbench.robot.joints import component_joint_names


LOGGER = logging.getLogger(__name__)

_DISPLAY_NAMES = {
    "torso": "Torso",
    "right_arm": "Right Arm",
    "left_arm": "Left Arm",
    "head": "Head",
}
_BODY_COMPONENTS = ("torso", "right_arm", "left_arm")
_ARM_COMPONENTS = ("right_arm", "left_arm")


def _rotation_to_rpy_deg(rot: np.ndarray) -> np.ndarray:
    """ZYX Euler angles (roll, pitch, yaw) in degrees from a 3x3 matrix."""
    sy = float(np.sqrt(rot[0, 0] ** 2 + rot[1, 0] ** 2))
    if sy > 1e-6:
        roll = np.arctan2(rot[2, 1], rot[2, 2])
        pitch = np.arctan2(-rot[2, 0], sy)
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
    else:
        roll = np.arctan2(-rot[1, 2], rot[1, 1])
        pitch = np.arctan2(-rot[2, 0], sy)
        yaw = 0.0
    return np.rad2deg(np.array([roll, pitch, yaw], dtype=np.float64))


def _axis_angle_rot3(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """3x3 rotation matrix via Rodrigues formula for rotation around a unit axis."""
    k = axis / np.linalg.norm(axis)
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    K = np.array(
        [
            [0.0, -k[2], k[1]],
            [k[2], 0.0, -k[0]],
            [-k[1], k[0], 0.0],
        ],
        dtype=np.float64,
    )
    return c * np.eye(3, dtype=np.float64) + s * K + (1 - c) * np.outer(k, k)


def _load_viser() -> Any:
    try:
        return importlib.import_module("viser")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "viser is not installed. Run `pip install -e .` or install `viser` in your env."
        ) from exc


def _component_indices(model: Any, component: str) -> np.ndarray:
    indices_by_component = {
        "torso": np.asarray(list(model.torso_idx), dtype=np.int32),
        "right_arm": np.asarray(list(model.right_arm_idx), dtype=np.int32),
        "left_arm": np.asarray(list(model.left_arm_idx), dtype=np.int32),
        "head": np.asarray(list(model.head_idx), dtype=np.int32),
    }
    return indices_by_component[component]


def _build_joint_specs(robot: RBY1) -> dict[str, JointGroupSpec]:
    model = robot.model
    dyn_robot = robot.sdk_robot.get_dynamics()
    dyn_state = dyn_robot.make_state([], model.robot_joint_names)
    lower_limits = np.asarray(dyn_robot.get_limit_q_lower(dyn_state), dtype=np.float64)
    upper_limits = np.asarray(dyn_robot.get_limit_q_upper(dyn_state), dtype=np.float64)
    velocity_limits = np.asarray(dyn_robot.get_limit_qdot_upper(dyn_state), dtype=np.float64)
    acceleration_limits = np.asarray(
        dyn_robot.get_limit_qddot_upper(dyn_state), dtype=np.float64
    )

    specs: dict[str, JointGroupSpec] = {}
    for component in ("torso", "right_arm", "left_arm", "head"):
        indices = _component_indices(model, component)
        if indices.size == 0:
            continue
        specs[component] = JointGroupSpec(
            component=component,
            display_name=_DISPLAY_NAMES[component],
            joint_names=component_joint_names(model, component),
            indices=indices,
            lower_rad=lower_limits[indices].copy(),
            upper_rad=upper_limits[indices].copy(),
            velocity_limit=velocity_limits[indices].copy(),
            acceleration_limit=acceleration_limits[indices].copy(),
        )
    return specs


@dataclass(slots=True)
class JointUiState:
    component: str
    joint_index: int
    status_handle: Any
    number_handle: Any
    lower_rad: float
    upper_rad: float


@dataclass(slots=True)
class CartesianUiState:
    current_pos_handle: Any
    current_rpy_handle: Any
    target_pos_handle: Any
    target_rpy_handle: Any


class ViserKeyboardStylePanel:
    """Viser panel that mirrors the state model from keyboard_control.py."""

    def __init__(self, cfg: DictConfig):
        self._cfg = cfg
        self._robot = RBY1(self._build_robot_config(cfg))
        self._connect_robot()

        all_specs = _build_joint_specs(self._robot)
        enabled = {
            "torso": bool(cfg.command.enable_torso),
            "right_arm": bool(cfg.command.enable_right_arm),
            "left_arm": bool(cfg.command.enable_left_arm),
            "head": bool(cfg.command.enable_head),
        }
        self._group_specs = {
            component: spec
            for component, spec in all_specs.items()
            if enabled.get(component, False)
        }
        if not self._group_specs:
            raise ValueError(
                "No control groups are enabled. Set at least one of "
                "`command.enable_head`, `command.enable_torso`, "
                "`command.enable_right_arm`, or `command.enable_left_arm`."
            )

        self._targets_lock = threading.Lock()
        self._stream_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._status_msg = "Ready"
        self._stop_event = threading.Event()
        self._syncing_gui = False
        self._streaming_enabled = bool(cfg.command.send_on_update)
        self._command_rate_hz = max(float(cfg.command.command_rate_hz), 0.1)

        self._q_targets = {
            component: self._robot.get_joint_positions(component)
            for component in self._group_specs
        }
        self._cartesian_targets: dict[str, np.ndarray] = {
            arm: self._robot.get_ee_pose(arm)
            for arm in _ARM_COMPONENTS
            if arm in self._group_specs
        }

        self._joint_ui: dict[tuple[str, int], JointUiState] = {}
        self._cartesian_ui: dict[str, CartesianUiState] = {}

        self._stream = self._robot.open_stream(mode="cartesian") if self._cartesian_targets else None
        self._stream_thread: threading.Thread | None = None
        if self._stream is not None:
            self._stream_thread = threading.Thread(
                target=self._stream_loop,
                name="viser-keyboard-style-stream",
                daemon=True,
            )
            self._stream_thread.start()

        self._rerun_viz: Any = None
        if bool(cfg.viz.enable):
            from rby1_workbench.control.rerun_control_viz import ControlRerunViz

            self._rerun_viz = ControlRerunViz(cfg.viz, self._robot.sdk_robot)
            self._rerun_viz.start()

        viser = _load_viser()
        self._server = viser.ViserServer(host=cfg.viser.host, port=cfg.viser.port)
        self._build_gui()
        self._sync_targets_from_robot()

    @staticmethod
    def _build_robot_config(cfg: DictConfig) -> DictConfig:
        robot_cfg = OmegaConf.create(
            OmegaConf.to_container(load_rby1_config(), resolve=False)
        )
        robot_cfg.address = cfg.robot.address
        robot_cfg.model = cfg.robot.model
        robot_cfg.power_pattern = cfg.robot.power
        robot_cfg.servo_pattern = cfg.robot.servo
        robot_cfg.unlimited_mode = cfg.robot.unlimited_mode_enabled
        robot_cfg.state_update_hz = cfg.robot.state_update_hz

        robot_cfg.stream.dt = 1.0 / max(float(cfg.command.command_rate_hz), 0.1)
        robot_cfg.stream.torso_mode = "joint_position"
        robot_cfg.stream.right_arm_mode = "cartesian_impedance"
        robot_cfg.stream.left_arm_mode = "cartesian_impedance"

        robot_cfg.cartesian_impedance.arm_linear_velocity_limit = (
            cfg.command.cartesian_linear_velocity_limit
        )
        robot_cfg.cartesian_impedance.arm_angular_velocity_limit = (
            cfg.command.cartesian_angular_velocity_limit
        )
        accel_scale = max(float(cfg.command.cartesian_acceleration_limit_scaling), 0.0)
        robot_cfg.cartesian_impedance.arm_linear_accel_limit = (
            float(robot_cfg.cartesian_impedance.arm_linear_accel_limit) * accel_scale
        )
        robot_cfg.cartesian_impedance.arm_angular_accel_limit = (
            float(robot_cfg.cartesian_impedance.arm_angular_accel_limit) * accel_scale
        )
        return robot_cfg

    def _connect_robot(self) -> None:
        self._robot.connect()
        if bool(self._cfg.robot.auto_power_on):
            self._robot.power_on(self._cfg.robot.power)
        if bool(self._cfg.robot.auto_servo_on):
            self._robot.servo_on(self._cfg.robot.servo)
        if bool(self._cfg.robot.auto_enable_control_manager):
            self._robot.enable_control_manager()

    def close(self) -> None:
        self._stop_event.set()
        if self._rerun_viz is not None:
            self._rerun_viz.stop()
        if self._stream_thread is not None:
            self._stream_thread.join(timeout=2.0)
        with self._stream_lock:
            if self._stream is not None:
                self._stream.close()

    def spin(self) -> None:
        LOGGER.info(
            "Viser keyboard-style control panel ready at http://localhost:%s",
            self._cfg.viser.port,
        )
        try:
            while not self._stop_event.is_set():
                self._refresh_live_statuses()
                self._refresh_status_label()
                time.sleep(0.2)
        except KeyboardInterrupt:
            LOGGER.info("Stopping viser keyboard-style control panel")
        finally:
            self.close()

    def _build_gui(self) -> None:
        self._server.gui.add_markdown(f"## {self._cfg.viser.title}")
        self._server.gui.add_markdown(
            "This panel follows `examples/keyboard_control.py`: joint tabs edit local "
            "targets and send blocking commands, while Cartesian tabs drive a long-lived "
            "stream with explicit target sync from FK."
        )

        with self._server.gui.add_folder("Session"):
            self._streaming_handle = self._server.gui.add_checkbox(
                "Cartesian streaming",
                initial_value=self._streaming_enabled,
            )
            self._status_handle = self._server.gui.add_markdown("*Status: Ready*")
            self._server.gui.add_markdown(
                f"*Command rate: {self._command_rate_hz:.1f} Hz*"
            )
            self._streaming_handle.on_update(self._on_streaming_changed)

            apply_joint_button = self._server.gui.add_button("Apply joint targets")
            apply_joint_button.on_click(self._on_apply_joint_targets_clicked)

            if any(component in self._group_specs for component in _BODY_COMPONENTS):
                ready_button = self._server.gui.add_button("Move enabled body to ready pose")
                zero_button = self._server.gui.add_button("Zero enabled body")
                ready_button.on_click(self._on_ready_pose_clicked)
                zero_button.on_click(self._on_zero_pose_clicked)

        tab_group = self._server.gui.add_tab_group()
        for component, spec in self._group_specs.items():
            self._build_joint_tab(tab_group, component, spec)
        for arm in _ARM_COMPONENTS:
            if arm in self._cartesian_targets:
                self._build_cartesian_tab(tab_group, arm)

    def _build_joint_tab(self, tab_group: Any, component: str, spec: JointGroupSpec) -> None:
        with tab_group.add_tab(f"{spec.display_name} Joint"):
            self._server.gui.add_markdown(
                f"### {spec.display_name} Joint\n"
                "Adjust local joint targets here. Use the session-level `Apply joint targets` "
                "button to send all edited joint targets together."
            )

            for joint_index, joint_name in enumerate(spec.joint_names):
                self._server.gui.add_markdown(f"**{joint_name}**")
                status = self._server.gui.add_markdown("")
                number_handle = self._server.gui.add_number(
                    "Target [deg]",
                    initial_value=0.0,
                    min=float(np.rad2deg(spec.lower_rad[joint_index])),
                    max=float(np.rad2deg(spec.upper_rad[joint_index])),
                    step=float(self._cfg.command.jog_step_small_deg),
                )
                number_handle.on_update(
                    self._make_joint_input_callback(component, joint_index)
                )
                self._joint_ui[(component, joint_index)] = JointUiState(
                    component=component,
                    joint_index=joint_index,
                    status_handle=status,
                    number_handle=number_handle,
                    lower_rad=float(spec.lower_rad[joint_index]),
                    upper_rad=float(spec.upper_rad[joint_index]),
                )

                button_specs = [
                    (
                        f"-{float(self._cfg.command.jog_step_large_deg):.0f} deg",
                        -float(self._cfg.command.jog_step_large_deg),
                    ),
                    (
                        f"-{float(self._cfg.command.jog_step_small_deg):.0f} deg",
                        -float(self._cfg.command.jog_step_small_deg),
                    ),
                    ("Zero", None),
                    (
                        f"+{float(self._cfg.command.jog_step_small_deg):.0f} deg",
                        float(self._cfg.command.jog_step_small_deg),
                    ),
                    (
                        f"+{float(self._cfg.command.jog_step_large_deg):.0f} deg",
                        float(self._cfg.command.jog_step_large_deg),
                    ),
                ]
                for label, delta in button_specs:
                    button = self._server.gui.add_button(label)
                    if delta is None:
                        button.on_click(
                            self._make_joint_zero_callback(component, joint_index)
                        )
                    else:
                        button.on_click(
                            self._make_joint_jog_callback(component, joint_index, delta)
                        )

    def _build_cartesian_tab(self, tab_group: Any, arm: str) -> None:
        display_name = _DISPLAY_NAMES[arm]
        with tab_group.add_tab(f"{display_name} Cartesian"):
            self._server.gui.add_markdown(
                f"### {display_name} Cartesian\n"
                "This mirrors the keyboard example: jog the target, keep streaming on "
                "to move, and use Sync from FK after other blocking commands."
            )
            sync_button = self._server.gui.add_button("Sync from FK")
            sync_button.on_click(self._make_sync_fk_callback(arm))

            current_pos = self._server.gui.add_markdown("*Current pos (m): -*")
            current_rpy = self._server.gui.add_markdown("*Current RPY (deg): -*")
            target_pos = self._server.gui.add_markdown("*Target pos (m): -*")
            target_rpy = self._server.gui.add_markdown("*Target RPY (deg): -*")
            self._cartesian_ui[arm] = CartesianUiState(
                current_pos_handle=current_pos,
                current_rpy_handle=current_rpy,
                target_pos_handle=target_pos,
                target_rpy_handle=target_rpy,
            )

            axes = [
                ("X", np.array([1.0, 0.0, 0.0], dtype=np.float64)),
                ("Y", np.array([0.0, 1.0, 0.0], dtype=np.float64)),
                ("Z", np.array([0.0, 0.0, 1.0], dtype=np.float64)),
            ]
            pos_small = float(self._cfg.command.cartesian_position_step_m)
            pos_large = pos_small * 5.0
            rot_small = float(self._cfg.command.cartesian_orientation_step_deg)
            rot_large = rot_small * 2.0

            self._server.gui.add_markdown("### Position")
            for axis_label, axis_vec in axes:
                self._server.gui.add_markdown(f"**{axis_label}**")
                for sign, step in [(-1, pos_large), (-1, pos_small), (1, pos_small), (1, pos_large)]:
                    label = f"{'+' if sign > 0 else '-'}{step * 100:.0f} cm"
                    button = self._server.gui.add_button(label)
                    button.on_click(
                        self._make_cartesian_jog_callback(
                            arm=arm,
                            axis=axis_vec.copy(),
                            is_rotation=False,
                            delta=sign * step,
                        )
                    )

            self._server.gui.add_markdown("### Orientation")
            for axis_label, axis_vec in axes:
                self._server.gui.add_markdown(f"**R{axis_label.lower()}**")
                for sign, step in [(-1, rot_large), (-1, rot_small), (1, rot_small), (1, rot_large)]:
                    label = f"{'+' if sign > 0 else '-'}{step:.0f} deg"
                    button = self._server.gui.add_button(label)
                    button.on_click(
                        self._make_cartesian_jog_callback(
                            arm=arm,
                            axis=axis_vec.copy(),
                            is_rotation=True,
                            delta=sign * step,
                        )
                    )

    def _set_status(self, message: str) -> None:
        with self._status_lock:
            self._status_msg = message

    def _refresh_status_label(self) -> None:
        with self._status_lock:
            message = self._status_msg
        self._status_handle.content = f"*Status: {message}*"

    def _refresh_live_statuses(self) -> None:
        try:
            q_all = np.asarray(self._robot.get_state().position, dtype=np.float64)
        except Exception as exc:
            self._set_status(f"State read failed: {exc}")
            return

        self._refresh_joint_statuses(q_all)
        for arm in self._cartesian_targets:
            self._refresh_cartesian_status(arm)

    def _refresh_joint_statuses(self, q_all: np.ndarray) -> None:
        with self._targets_lock:
            target_snapshot = {
                component: target.copy()
                for component, target in self._q_targets.items()
            }

        self._syncing_gui = True
        try:
            for (component, joint_index), ui_state in self._joint_ui.items():
                spec = self._group_specs[component]
                current_rad = float(q_all[spec.indices[joint_index]])
                target_rad = float(target_snapshot[component][joint_index])
                ui_state.status_handle.content = (
                    f"Current: **{np.rad2deg(current_rad):.2f} deg**  "
                    f"Target: **{np.rad2deg(target_rad):.2f} deg**  "
                    f"(range {np.rad2deg(ui_state.lower_rad):.1f} to "
                    f"{np.rad2deg(ui_state.upper_rad):.1f})"
                )
                ui_state.number_handle.value = float(np.rad2deg(target_rad))
        finally:
            self._syncing_gui = False

    def _refresh_cartesian_status(self, arm: str) -> None:
        ui_state = self._cartesian_ui.get(arm)
        if ui_state is None:
            return

        with self._targets_lock:
            target = self._cartesian_targets[arm].copy()

        try:
            current = self._robot.get_ee_pose(arm)
        except Exception as exc:
            ui_state.current_pos_handle.content = f"*Current pos (m): error: {exc}*"
            ui_state.current_rpy_handle.content = "*Current RPY (deg): -*"
            current = None

        if current is not None:
            current_pos = current[:3, 3]
            current_rpy = _rotation_to_rpy_deg(current[:3, :3])
            ui_state.current_pos_handle.content = (
                "Current pos (m): "
                f"**x={current_pos[0]:.3f}  y={current_pos[1]:.3f}  z={current_pos[2]:.3f}**"
            )
            ui_state.current_rpy_handle.content = (
                "Current RPY (deg): "
                f"**rx={current_rpy[0]:.1f}  ry={current_rpy[1]:.1f}  rz={current_rpy[2]:.1f}**"
            )

        target_pos = target[:3, 3]
        target_rpy = _rotation_to_rpy_deg(target[:3, :3])
        ui_state.target_pos_handle.content = (
            "Target pos (m): "
            f"**x={target_pos[0]:.3f}  y={target_pos[1]:.3f}  z={target_pos[2]:.3f}**"
        )
        ui_state.target_rpy_handle.content = (
            "Target RPY (deg): "
            f"**rx={target_rpy[0]:.1f}  ry={target_rpy[1]:.1f}  rz={target_rpy[2]:.1f}**"
        )

    def _sync_targets_from_robot(self) -> None:
        q_targets: dict[str, np.ndarray] = {}
        for component in self._group_specs:
            q_targets[component] = self._robot.get_joint_positions(component)

        cart_targets: dict[str, np.ndarray] = {}
        for arm in self._cartesian_targets:
            cart_targets[arm] = self._robot.get_ee_pose(arm)

        with self._targets_lock:
            self._q_targets.update(q_targets)
            self._cartesian_targets.update(cart_targets)

        if self._rerun_viz is not None:
            for arm, target in cart_targets.items():
                self._rerun_viz.log_cartesian_target(arm, target)

        self._refresh_live_statuses()

    def _ensure_stream_ready(self) -> None:
        if not self._cartesian_targets:
            return
        with self._stream_lock:
            if self._stream is None or self._stream.closed:
                self._stream = self._robot.open_stream(mode="cartesian")

    def _stream_loop(self) -> None:
        interval = 1.0 / self._command_rate_hz
        while not self._stop_event.is_set():
            if self._streaming_enabled and self._cartesian_targets:
                try:
                    self._ensure_stream_ready()
                    with self._targets_lock:
                        kwargs: dict[str, np.ndarray] = {
                            arm: target.copy()
                            for arm, target in self._cartesian_targets.items()
                        }
                        if "torso" in self._q_targets:
                            kwargs["torso"] = self._q_targets["torso"].copy()
                        if "head" in self._q_targets:
                            kwargs["head"] = self._q_targets["head"][:2].copy()
                    with self._stream_lock:
                        stream = self._stream
                    if stream is not None:
                        stream.send(**kwargs)
                        if stream.closed:
                            self._set_status("Cartesian stream expired; recreating")
                            self._ensure_stream_ready()
                except Exception as exc:
                    LOGGER.exception("Cartesian stream loop failed")
                    self._set_status(f"Stream error: {exc}")
            time.sleep(interval)

    def _run_blocking(self, fn: Any) -> bool:
        with self._stream_lock:
            stream = self._stream
            if stream is not None:
                stream.pause()
        try:
            return bool(fn())
        finally:
            try:
                self._sync_targets_from_robot()
            finally:
                if self._cartesian_targets:
                    self._ensure_stream_ready()
                    with self._stream_lock:
                        if self._stream is not None:
                            self._stream.resume()

    def _move_enabled_body_targets(self, targets: dict[str, np.ndarray], status_label: str) -> None:
        body_targets = {
            component: values.copy()
            for component, values in targets.items()
            if component in self._group_specs and component in _BODY_COMPONENTS
        }
        if not body_targets:
            self._set_status(f"{status_label}: no enabled body components")
            return

        ok = self._run_blocking(
            lambda: self._robot.move(
                mode="joint",
                minimum_time=float(self._cfg.command.body_minimum_time),
                **body_targets,
            )
        )
        self._set_status(f"{status_label}: {'OK' if ok else 'FAILED'}")

    def _apply_head_target(self, target: np.ndarray) -> bool:
        target = np.asarray(target, dtype=np.float64)
        return self._robot.move(
            mode="joint",
            head=np.asarray(target, dtype=float),
            minimum_time=float(self._cfg.command.head_minimum_time),
        )

    def _apply_single_joint_component(self, component: str) -> None:
        with self._targets_lock:
            target = self._q_targets[component].copy()

        if component == "head":
            ok = self._run_blocking(lambda: self._apply_head_target(target))
        else:
            ok = self._run_blocking(
                lambda: self._robot.move(
                    mode="joint",
                    minimum_time=float(self._cfg.command.body_minimum_time),
                    **{component: target},
                )
            )
        self._set_status(
            f"Apply {_DISPLAY_NAMES[component]}: {'OK' if ok else 'FAILED'}"
        )

    def _make_joint_input_callback(self, component: str, joint_index: int):
        def _callback(_: Any) -> None:
            if self._syncing_gui:
                return
            ui_state = self._joint_ui[(component, joint_index)]
            value_deg = float(ui_state.number_handle.value)
            value_rad = float(np.deg2rad(value_deg))
            clipped = float(np.clip(value_rad, ui_state.lower_rad, ui_state.upper_rad))
            with self._targets_lock:
                self._q_targets[component][joint_index] = clipped
            self._set_status(
                f"{_DISPLAY_NAMES[component]} joint {joint_index} target updated"
            )
            self._refresh_live_statuses()

        return _callback

    def _make_joint_jog_callback(self, component: str, joint_index: int, delta_deg: float):
        def _callback(_: Any) -> None:
            ui_state = self._joint_ui[(component, joint_index)]
            delta_rad = float(np.deg2rad(delta_deg))
            with self._targets_lock:
                current = float(self._q_targets[component][joint_index])
                clipped = float(np.clip(current + delta_rad, ui_state.lower_rad, ui_state.upper_rad))
                self._q_targets[component][joint_index] = clipped
            self._apply_single_joint_component(component)

        return _callback

    def _make_joint_zero_callback(self, component: str, joint_index: int):
        def _callback(_: Any) -> None:
            ui_state = self._joint_ui[(component, joint_index)]
            clipped = float(np.clip(0.0, ui_state.lower_rad, ui_state.upper_rad))
            with self._targets_lock:
                self._q_targets[component][joint_index] = clipped
            self._apply_single_joint_component(component)

        return _callback

    def _make_sync_fk_callback(self, arm: str):
        def _callback(_: Any) -> None:
            target = self._robot.get_ee_pose(arm)
            with self._targets_lock:
                self._cartesian_targets[arm] = target
            if self._rerun_viz is not None:
                self._rerun_viz.log_cartesian_target(arm, target)
            self._set_status(f"{_DISPLAY_NAMES[arm]} Cartesian target synced from FK")
            self._refresh_live_statuses()

        return _callback

    def _make_cartesian_jog_callback(
        self,
        arm: str,
        axis: np.ndarray,
        is_rotation: bool,
        delta: float,
    ):
        def _callback(_: Any) -> None:
            with self._targets_lock:
                target = self._cartesian_targets[arm].copy()
                if is_rotation:
                    target[:3, :3] = _axis_angle_rot3(axis, np.deg2rad(delta)) @ target[:3, :3]
                else:
                    target[:3, 3] += axis * delta
                self._cartesian_targets[arm] = target

            if self._rerun_viz is not None:
                self._rerun_viz.log_cartesian_target(arm, target)

            unit = "deg" if is_rotation else "m"
            self._set_status(
                f"{_DISPLAY_NAMES[arm]} Cartesian {'rot' if is_rotation else 'trans'} "
                f"{delta:+.3f} {unit}"
            )
            self._refresh_live_statuses()

        return _callback

    def _on_streaming_changed(self, _: Any) -> None:
        self._streaming_enabled = bool(self._streaming_handle.value)
        self._set_status(
            "Cartesian streaming: ON" if self._streaming_enabled else "Cartesian streaming: PAUSED"
        )

    def _on_apply_joint_targets_clicked(self, _: Any) -> None:
        try:
            with self._targets_lock:
                targets = {
                    component: target.copy()
                    for component, target in self._q_targets.items()
                }

            kwargs: dict[str, np.ndarray] = {}
            for component in _BODY_COMPONENTS:
                if component in targets and component in self._group_specs:
                    kwargs[component] = targets[component]
            head_target = None
            if "head" in targets and "head" in self._group_specs:
                head_target = targets["head"].copy()

            if not kwargs and head_target is None:
                self._set_status("Apply joint targets: no enabled targets")
                return

            ok = self._run_blocking(
                lambda: self._apply_joint_targets(kwargs, head_target)
            )
            self._set_status(f"Apply joint targets: {'OK' if ok else 'FAILED'}")
        except Exception as exc:
            LOGGER.exception("Failed to apply joint targets")
            self._set_status(f"Apply joint targets failed: {exc}")

    def _apply_joint_targets(
        self,
        body_targets: dict[str, np.ndarray],
        head_target: np.ndarray | None,
    ) -> bool:
        ok = True
        if body_targets:
            ok = ok and self._robot.move(
                mode="joint",
                minimum_time=float(self._cfg.command.body_minimum_time),
                **body_targets,
            )
        if head_target is not None:
            ok = self._apply_head_target(head_target) and ok
        return ok

    def _on_ready_pose_clicked(self, _: Any) -> None:
        try:
            ready_targets = ready_pose_targets_for_model(self._robot.model)
            self._move_enabled_body_targets(ready_targets, "Ready pose")
        except Exception as exc:
            LOGGER.exception("Failed to move to ready pose")
            self._set_status(f"Ready pose failed: {exc}")

    def _on_zero_pose_clicked(self, _: Any) -> None:
        try:
            zero_targets = {
                component: np.zeros_like(self._q_targets[component])
                for component in _BODY_COMPONENTS
                if component in self._group_specs
            }
            self._move_enabled_body_targets(zero_targets, "Zero pose")
        except Exception as exc:
            LOGGER.exception("Failed to move to zero pose")
            self._set_status(f"Zero pose failed: {exc}")


def run_viser_joint_control_panel(cfg: DictConfig) -> None:
    panel = ViserKeyboardStylePanel(cfg)
    panel.spin()
