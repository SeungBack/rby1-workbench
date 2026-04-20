"""Viser-based RB-Y1 joint control panel."""

from __future__ import annotations

import importlib
import logging
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from rby1_workbench.config.schema import ViserJointControlAppConfig
from rby1_workbench.control.joint_commands import (
    CartesianCommandSettings,
    JointCommandClient,
    JointCommandSettings,
)
from rby1_workbench.control.presets import ready_pose_targets_for_model
from rby1_workbench.robot.client import connect_robot


def _rotation_to_rpy_deg(rot: np.ndarray) -> np.ndarray:
    """ZYX Euler angles (roll, pitch, yaw) in degrees from a 3×3 rotation matrix.

    Decomposes R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    """
    sy = float(np.sqrt(rot[0, 0] ** 2 + rot[1, 0] ** 2))
    if sy > 1e-6:
        roll  = np.arctan2( rot[2, 1],  rot[2, 2])
        pitch = np.arctan2(-rot[2, 0],  sy)
        yaw   = np.arctan2( rot[1, 0],  rot[0, 0])
    else:  # gimbal lock
        roll  = np.arctan2(-rot[1, 2],  rot[1, 1])
        pitch = np.arctan2(-rot[2, 0],  sy)
        yaw   = 0.0
    return np.rad2deg(np.array([roll, pitch, yaw], dtype=np.float64))


def _axis_angle_rot3(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """3×3 rotation matrix via Rodrigues formula for rotation around a unit axis."""
    k = axis / np.linalg.norm(axis)
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    K = np.array([
        [    0, -k[2],  k[1]],
        [ k[2],     0, -k[0]],
        [-k[1],  k[0],     0],
    ], dtype=np.float64)
    return c * np.eye(3, dtype=np.float64) + s * K + (1 - c) * np.outer(k, k)


def _load_viser() -> Any:
    try:
        return importlib.import_module("viser")
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "viser is not installed. Run `pip install -e .` or install `viser` in your env."
        ) from exc


@dataclass(slots=True)
class JointUiState:
    component: str
    joint_name: str
    joint_index: int
    status_handle: Any
    number_handle: Any
    lower_rad: float
    upper_rad: float


class ViserJointControlPanel:
    """Small joint control panel that sends RB-Y1 commands from jog-style GUI inputs."""

    def __init__(self, cfg: ViserJointControlAppConfig):
        self._cfg = cfg
        self._robot = connect_robot(cfg.robot)
        self._command_client = JointCommandClient(self._robot)
        self._group_specs = self._enabled_group_specs()
        if not self._group_specs:
            raise ValueError(
                "No joint groups are enabled. Set at least one of "
                "`command.enable_head`, `command.enable_torso`, "
                "`command.enable_right_arm`, or `command.enable_left_arm` to true."
            )

        self._stop_event = threading.Event()
        self._stream_on = bool(cfg.command.send_on_update)
        self._body_ctrl_mode: str = "joint"  # "joint" | "cartesian"
        self._syncing_gui = False
        self._target_by_component = self._command_client.current_targets()
        self._joint_ui: dict[tuple[str, int], JointUiState] = {}

        # Cartesian EE control state — one SE3 target per enabled arm
        self._cartesian_targets: dict[str, np.ndarray] = {}
        self._cartesian_status_handles: dict[str, tuple[Any, Any]] = {}
        for _arm in ("right_arm", "left_arm"):
            if _arm in self._group_specs:
                _T = self._command_client.compute_fk(_arm)
                self._cartesian_targets[_arm] = _T if _T is not None else np.eye(4, dtype=np.float64)

        # Optional Rerun viz companion
        self._rerun_viz: Any = None
        if cfg.viz.enable:
            from rby1_workbench.control.rerun_control_viz import ControlRerunViz
            self._rerun_viz = ControlRerunViz(cfg.viz, self._robot)
            self._rerun_viz.start()

        viser = _load_viser()
        self._server = viser.ViserServer(host=cfg.viser.host, port=cfg.viser.port)
        self._build_gui()
        self._refresh_joint_statuses()
        for _arm in self._cartesian_targets:
            self._refresh_cartesian_status(_arm)
        if self._rerun_viz is not None:
            for _arm, _T in self._cartesian_targets.items():
                self._rerun_viz.log_cartesian_target(_arm, _T)

    def close(self) -> None:
        if self._rerun_viz is not None:
            self._rerun_viz.stop()
        self._stop_event.set()

    def spin(self) -> None:
        logging.info(
            "Viser joint control panel ready at http://localhost:%s",
            self._cfg.viser.port,
        )
        try:
            while not self._stop_event.is_set():
                time.sleep(0.5)
        except KeyboardInterrupt:
            logging.info("Stopping viser joint control panel")
        finally:
            self.close()

    def _build_gui(self) -> None:
        self._server.gui.add_markdown(f"## {self._cfg.viser.title}")
        self._server.gui.add_markdown(
            "Use compact jog controls or direct target numbers. Body uses stream-based position or impedance commands. Head uses stream-based joint position commands."
        )

        with self._server.gui.add_folder("Command"):
            self._send_on_update_handle = self._server.gui.add_checkbox(
                "Send on jog",
                initial_value=self._cfg.command.send_on_update,
            )
            self._body_ctrl_mode_handle = self._server.gui.add_dropdown(
                "Body control",
                options=("joint", "cartesian"),
                initial_value="joint",
            )
            self._mode_handle = self._server.gui.add_dropdown(
                "Body mode",
                options=("impedance", "position"),
                initial_value=self._cfg.command.body_mode,
            )
            self._body_minimum_time_handle = self._server.gui.add_slider(
                "Body min time [s]",
                min=0.1,
                max=10.0,
                step=0.1,
                initial_value=self._cfg.command.body_minimum_time,
            )
            self._head_minimum_time_handle = self._server.gui.add_slider(
                "Head min time [s]",
                min=0.1,
                max=10.0,
                step=0.1,
                initial_value=self._cfg.command.head_minimum_time,
            )
            self._control_hold_time_handle = self._server.gui.add_slider(
                "Control hold [s]",
                min=0.0,
                max=1000000.0,
                step=1.0,
                initial_value=self._cfg.command.control_hold_time,
            )
            self._stiffness_handle = self._server.gui.add_slider(
                "Stiffness",
                min=1.0,
                max=200.0,
                step=1.0,
                initial_value=self._cfg.command.stiffness,
            )
            self._damping_ratio_handle = self._server.gui.add_slider(
                "Damping ratio",
                min=0.1,
                max=2.0,
                step=0.05,
                initial_value=self._cfg.command.damping_ratio,
            )
            self._torque_limit_handle = self._server.gui.add_slider(
                "Torque limit",
                min=1.0,
                max=100.0,
                step=1.0,
                initial_value=self._cfg.command.torque_limit,
            )
            self._send_button = self._server.gui.add_button("Send target pose")

            if any(component in self._group_specs for component in ("torso", "right_arm", "left_arm")):
                self._ready_pose_button = self._server.gui.add_button("Move to ready pose")
            else:
                self._ready_pose_button = None

            if "head" in self._group_specs:
                self._head_home_button = self._server.gui.add_button("Head to zero")
            else:
                self._head_home_button = None

        self._send_on_update_handle.on_update(self._on_send_on_update_changed)
        self._body_ctrl_mode_handle.on_update(self._on_body_ctrl_mode_changed)
        self._send_button.on_click(self._on_send_clicked)
        if self._ready_pose_button is not None:
            self._ready_pose_button.on_click(self._on_ready_pose_clicked)
        if self._head_home_button is not None:
            self._head_home_button.on_click(self._on_head_home_clicked)

        tab_group = self._server.gui.add_tab_group()
        for component, spec in self._group_specs.items():
            with tab_group.add_tab(spec.display_name):
                self._server.gui.add_markdown(
                    f"### {spec.display_name}\nCurrent component target values and compact jog controls."
                )
                apply_component_button = self._server.gui.add_button(
                    f"Apply {spec.display_name}"
                )
                apply_component_button.on_click(
                    self._make_apply_component_callback(component)
                )
                for joint_index, (joint_name, lower_rad, upper_rad) in enumerate(
                    zip(spec.joint_names, spec.lower_rad, spec.upper_rad)
                ):
                    self._server.gui.add_markdown(f"**{joint_name}**")
                    status = self._server.gui.add_markdown("")
                    number_handle = self._server.gui.add_number(
                        "Target [deg]",
                        initial_value=0.0,
                        min=float(np.rad2deg(lower_rad)),
                        max=float(np.rad2deg(upper_rad)),
                        step=self._cfg.command.jog_step_small_deg,
                    )
                    ui_state = JointUiState(
                        component=component,
                        joint_name=joint_name,
                        joint_index=joint_index,
                        status_handle=status,
                        number_handle=number_handle,
                        lower_rad=float(lower_rad),
                        upper_rad=float(upper_rad),
                    )
                    self._joint_ui[(component, joint_index)] = ui_state

                    minus_large_button = self._server.gui.add_button(
                        f"-{self._cfg.command.jog_step_large_deg:.0f} deg"
                    )
                    minus_small_button = self._server.gui.add_button(
                        f"-{self._cfg.command.jog_step_small_deg:.0f} deg"
                    )
                    plus_small_button = self._server.gui.add_button(
                        f"+{self._cfg.command.jog_step_small_deg:.0f} deg"
                    )
                    plus_large_button = self._server.gui.add_button(
                        f"+{self._cfg.command.jog_step_large_deg:.0f} deg"
                    )

                    minus_large_button.on_click(
                        self._make_joint_jog_callback(
                            component,
                            joint_index,
                            -self._cfg.command.jog_step_large_deg,
                        )
                    )
                    minus_small_button.on_click(
                        self._make_joint_jog_callback(
                            component,
                            joint_index,
                            -self._cfg.command.jog_step_small_deg,
                        )
                    )
                    plus_small_button.on_click(
                        self._make_joint_jog_callback(
                            component,
                            joint_index,
                            self._cfg.command.jog_step_small_deg,
                        )
                    )
                    plus_large_button.on_click(
                        self._make_joint_jog_callback(
                            component,
                            joint_index,
                            self._cfg.command.jog_step_large_deg,
                        )
                    )

        # Cartesian EE tabs — one per enabled arm
        _CART_DISPLAY = {"right_arm": "Cartesian Right", "left_arm": "Cartesian Left"}
        _AXES = [
            ("X",  np.array([1.0, 0.0, 0.0])),
            ("Y",  np.array([0.0, 1.0, 0.0])),
            ("Z",  np.array([0.0, 0.0, 1.0])),
        ]
        pos_s = self._cfg.command.cartesian_position_step_m
        pos_l = pos_s * 5.0
        rot_s = self._cfg.command.cartesian_orientation_step_deg
        rot_l = rot_s * 2.0

        for arm, tab_name in _CART_DISPLAY.items():
            if arm not in self._cartesian_targets:
                continue
            with tab_group.add_tab(tab_name):
                self._server.gui.add_markdown(
                    f"### {tab_name}\n"
                    "Cartesian EE jog in base frame. "
                    "Click **Sync from FK** before jogging to align target with robot."
                )
                sync_btn = self._server.gui.add_button("Sync from FK")
                sync_btn.on_click(self._make_sync_fk_callback(arm))

                pos_status = self._server.gui.add_markdown("*Target pos (m): —*")
                rpy_status = self._server.gui.add_markdown("*Target RPY (deg): —*")
                self._cartesian_status_handles[arm] = (pos_status, rpy_status)

                self._server.gui.add_markdown("### Position")
                for axis_label, axis_vec in _AXES:
                    self._server.gui.add_markdown(f"**{axis_label}**")
                    for sign, step in [(-1, pos_l), (-1, pos_s), (1, pos_s), (1, pos_l)]:
                        label = f"{'+' if sign > 0 else '-'}{step * 100:.0f} cm"
                        btn = self._server.gui.add_button(label)
                        btn.on_click(
                            self._make_cartesian_jog_callback(
                                arm, axis_vec.copy(), is_rotation=False, delta=sign * step
                            )
                        )

                self._server.gui.add_markdown("### Orientation")
                for axis_label, axis_vec in _AXES:
                    self._server.gui.add_markdown(f"**R{axis_label.lower()}**")
                    for sign, step in [(-1, rot_l), (-1, rot_s), (1, rot_s), (1, rot_l)]:
                        label = f"{'+' if sign > 0 else '-'}{step:.0f} deg"
                        btn = self._server.gui.add_button(label)
                        btn.on_click(
                            self._make_cartesian_jog_callback(
                                arm, axis_vec.copy(), is_rotation=True, delta=sign * step
                            )
                        )

                apply_cart_btn = self._server.gui.add_button(f"Apply {tab_name}")
                apply_cart_btn.on_click(self._make_apply_cartesian_callback(arm))

    def _enabled_group_specs(self) -> dict[str, Any]:
        all_specs = self._command_client.group_specs()
        enabled = {
            "torso": self._cfg.command.enable_torso,
            "right_arm": self._cfg.command.enable_right_arm,
            "left_arm": self._cfg.command.enable_left_arm,
            "head": self._cfg.command.enable_head,
        }
        return {
            component: spec
            for component, spec in all_specs.items()
            if enabled.get(component, False)
        }

    def _current_settings(self) -> JointCommandSettings:
        return JointCommandSettings(
            body_mode=str(self._mode_handle.value),
            body_minimum_time=float(self._body_minimum_time_handle.value),
            head_minimum_time=float(self._head_minimum_time_handle.value),
            control_hold_time=float(self._control_hold_time_handle.value),
            stiffness=float(self._stiffness_handle.value),
            damping_ratio=float(self._damping_ratio_handle.value),
            torque_limit=float(self._torque_limit_handle.value),
        )

    def _visible_targets(self, components: set[str] | None = None) -> dict[str, np.ndarray]:
        selected = self._group_specs.keys() if components is None else components
        return {
            component: self._target_by_component[component].copy()
            for component in selected
            if component in self._group_specs
        }

    def _send_current_targets(self, components: set[str] | None = None) -> None:
        settings = self._current_settings()
        target_components = set(self._group_specs) if components is None else set(components)
        self._command_client.apply_targets(
            self._visible_targets(target_components),
            settings,
        )
        logging.info(
            "Streamed joint target pose for %s (%s mode, body %.2fs, head %.2fs)",
            sorted(target_components),
            settings.body_mode,
            settings.body_minimum_time,
            settings.head_minimum_time,
        )

    def _pull_targets_from_gui(self, components: set[str]) -> None:
        for component in components:
            if component not in self._group_specs:
                continue

            spec = self._group_specs[component]
            values_rad = np.zeros(len(spec.joint_names), dtype=np.float64)
            for joint_index in range(len(spec.joint_names)):
                ui_state = self._joint_ui[(component, joint_index)]
                value_deg = float(ui_state.number_handle.value)
                value_rad = float(np.deg2rad(value_deg))
                values_rad[joint_index] = float(
                    np.clip(value_rad, ui_state.lower_rad, ui_state.upper_rad)
                )

            self._target_by_component[component] = values_rad
        self._refresh_joint_statuses()

    def _refresh_joint_statuses(self) -> None:
        self._syncing_gui = True
        try:
            for (component, joint_index), ui_state in self._joint_ui.items():
                spec = self._group_specs[component]
                target_rad = float(self._target_by_component[component][joint_index])
                lower_rad = float(spec.lower_rad[joint_index])
                upper_rad = float(spec.upper_rad[joint_index])
                ui_state.status_handle.content = (
                    f"Target: **{np.rad2deg(target_rad):.2f} deg**  "
                    f"(range {np.rad2deg(lower_rad):.1f} to {np.rad2deg(upper_rad):.1f})"
                )
                ui_state.number_handle.value = float(np.rad2deg(target_rad))
        finally:
            self._syncing_gui = False

    def _send_if_enabled(self, components: set[str]) -> None:
        if not self._stream_on or self._body_ctrl_mode != "joint":
            return
        try:
            self._send_current_targets(components)
        except Exception:
            logging.exception("Failed to stream command from viser panel")

    def _set_component_targets(self, component: str, values_rad: np.ndarray) -> None:
        self._target_by_component[component] = np.asarray(values_rad, dtype=np.float64).copy()
        self._refresh_joint_statuses()

    _BODY_COMPONENTS = frozenset({"torso", "right_arm", "left_arm"})

    def _body_send_components(self, component: str) -> set[str]:
        """Return the set of components to send for a given component.

        Stream commands fully replace the previous command.  Omitting a body
        component drops its stream control, causing it to go limp.  Therefore
        ALL enabled body components must be included in every body stream
        command.  Only the *target* for the triggering component changes;
        every other component sends its current target unchanged (hold).

        Head is independent via ComponentBasedCommandBuilder and is not
        included in body grouping.
        """
        if component in self._BODY_COMPONENTS:
            return set(self._group_specs) & self._BODY_COMPONENTS
        return {component}

    def _make_joint_jog_callback(
        self,
        component: str,
        joint_index: int,
        delta_deg: float,
    ):
        def _callback(_: Any) -> None:
            if self._syncing_gui:
                return
            ui_state = self._joint_ui[(component, joint_index)]
            delta_rad = np.deg2rad(delta_deg)
            current = float(self._target_by_component[component][joint_index])
            clipped = float(np.clip(current + delta_rad, ui_state.lower_rad, ui_state.upper_rad))
            self._target_by_component[component][joint_index] = clipped
            self._refresh_joint_statuses()
            self._send_if_enabled(self._body_send_components(component))

        return _callback

    def _make_apply_component_callback(self, component: str):
        def _callback(_: Any) -> None:
            try:
                self._pull_targets_from_gui({component})
                self._send_current_targets(self._body_send_components(component))
            except Exception:
                logging.exception("Failed to apply component target for %s", component)

        return _callback

    # ------------------------------------------------------------------
    # Cartesian EE control helpers
    # ------------------------------------------------------------------

    def _current_cartesian_settings(self) -> CartesianCommandSettings:
        return CartesianCommandSettings(
            linear_velocity_limit=self._cfg.command.cartesian_linear_velocity_limit,
            angular_velocity_limit=self._cfg.command.cartesian_angular_velocity_limit,
            acceleration_limit_scaling=self._cfg.command.cartesian_acceleration_limit_scaling,
            minimum_time=self._cfg.command.cartesian_minimum_time,
            control_hold_time=float(self._control_hold_time_handle.value),
        )

    def _refresh_cartesian_status(self, arm: str) -> None:
        if arm not in self._cartesian_targets or arm not in self._cartesian_status_handles:
            return
        T = self._cartesian_targets[arm]
        p = T[:3, 3]
        rpy = _rotation_to_rpy_deg(T[:3, :3])
        pos_handle, rpy_handle = self._cartesian_status_handles[arm]
        pos_handle.content = (
            f"Target pos (m): **x={p[0]:.3f}  y={p[1]:.3f}  z={p[2]:.3f}**"
        )
        rpy_handle.content = (
            f"Target RPY (deg): **rx={rpy[0]:.1f}  ry={rpy[1]:.1f}  rz={rpy[2]:.1f}**"
        )

    def _send_cartesian_targets(self) -> None:
        """Send ALL enabled Cartesian arm targets + torso joint hold in one body command.

        In Cartesian mode every arm must be sent as CartesianCommandBuilder
        simultaneously.  Mixing Cartesian and joint builders for different arms
        in successive stream commands causes Cartesian↔joint transitions on the
        non-targeted arm, which destabilises it.  Sending all arms Cartesian
        together avoids this entirely.

        Torso is always joint (SDK requirement) and is included as a position
        hold so it is not dropped from the stream.
        """
        cart_targets = {a: T.copy() for a, T in self._cartesian_targets.items()}
        if not cart_targets:
            return
        cart_settings = self._current_cartesian_settings()
        joint_settings = self._current_settings()
        torso_hold: dict[str, np.ndarray] = {}
        if "torso" in self._group_specs:
            torso_hold["torso"] = self._target_by_component["torso"].copy()
        self._command_client.apply_cartesian_targets(
            cart_targets, cart_settings, torso_hold, joint_settings
        )
        logging.info("Streamed Cartesian targets for %s", sorted(cart_targets))

    def _send_cartesian_if_enabled(self) -> None:
        if not self._stream_on or self._body_ctrl_mode != "cartesian":
            return
        try:
            self._send_cartesian_targets()
        except Exception:
            logging.exception("Failed to stream Cartesian command from viser panel")

    def _make_cartesian_jog_callback(
        self,
        arm: str,
        axis: np.ndarray,
        is_rotation: bool,
        delta: float,
    ):
        def _callback(_: Any) -> None:
            T = self._cartesian_targets[arm].copy()
            if is_rotation:
                dR = _axis_angle_rot3(axis, np.deg2rad(delta))
                T[:3, :3] = dR @ T[:3, :3]
            else:
                T[:3, 3] += axis * delta
            self._cartesian_targets[arm] = T
            self._refresh_cartesian_status(arm)
            if self._rerun_viz is not None:
                self._rerun_viz.log_cartesian_target(arm, T)
            self._send_cartesian_if_enabled({arm})

        return _callback

    def _make_sync_fk_callback(self, arm: str):
        def _callback(_: Any) -> None:
            try:
                T = self._command_client.compute_fk(arm)
                if T is None:
                    logging.warning("FK not available for %s — sync skipped", arm)
                    return
                self._cartesian_targets[arm] = T
                self._refresh_cartesian_status(arm)
                if self._rerun_viz is not None:
                    self._rerun_viz.log_cartesian_target(arm, T)
                logging.info("Synced Cartesian target from FK for %s", arm)
            except Exception:
                logging.exception("Failed to sync FK for %s", arm)

        return _callback

    def _make_apply_cartesian_callback(self, arm: str):
        def _callback(_: Any) -> None:
            try:
                self._send_cartesian_targets({arm})
            except Exception:
                logging.exception("Failed to apply Cartesian target for %s", arm)

        return _callback

    # ------------------------------------------------------------------
    # Joint control event handlers
    # ------------------------------------------------------------------

    def _on_send_on_update_changed(self, _: Any) -> None:
        self._stream_on = bool(self._send_on_update_handle.value)
        logging.info("Send on jog: %s", self._stream_on)

    def _on_ready_pose_clicked(self, _: Any) -> None:
        try:
            ready_targets = ready_pose_targets_for_model(self._command_client.model)
            changed_components: set[str] = set()
            for component, values in ready_targets.items():
                if component in self._group_specs:
                    self._set_component_targets(component, values)
                    changed_components.add(component)
            self._send_current_targets(changed_components)
            logging.info(
                "Moved enabled body joints to SDK ready pose for model %s",
                self._command_client.model.model_name,
            )
        except Exception:
            logging.exception("Failed to move to SDK ready pose")

    def _on_send_clicked(self, _: Any) -> None:
        try:
            self._pull_targets_from_gui(set(self._group_specs))
            self._send_current_targets()
        except Exception:
            logging.exception("Failed to send target pose")

    def _on_head_home_clicked(self, _: Any) -> None:
        if "head" not in self._group_specs:
            return

        try:
            self._set_component_targets("head", np.zeros_like(self._target_by_component["head"]))
            self._send_current_targets({"head"})
            logging.info("Moved head target to zero")
        except Exception:
            logging.exception("Failed to move head to zero")


def run_viser_joint_control_panel(cfg: ViserJointControlAppConfig) -> None:
    panel = ViserJointControlPanel(cfg)
    panel.spin()
