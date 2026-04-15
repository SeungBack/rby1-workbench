"""Joint-level command helpers shared by control apps."""

from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from typing import Any

import numpy as np
import rby1_sdk as rby

from rby1_workbench.config.schema import JointControlConfig
from rby1_workbench.robot.joints import component_joint_names


@dataclass(slots=True)
class JointGroupSpec:
    component: str
    display_name: str
    joint_names: list[str]
    indices: np.ndarray
    lower_rad: np.ndarray
    upper_rad: np.ndarray
    velocity_limit: np.ndarray
    acceleration_limit: np.ndarray


@dataclass(slots=True)
class JointCommandSettings:
    body_mode: str = "impedance"
    body_minimum_time: float = 5.0
    head_minimum_time: float = 2.0
    control_hold_time: float = 1000000.0
    stiffness: float = 50.0
    damping_ratio: float = 1.0
    torque_limit: float = 30.0

    @classmethod
    def from_control_config(cls, cfg: JointControlConfig) -> "JointCommandSettings":
        return cls(
            body_mode=cfg.body_mode,
            body_minimum_time=cfg.body_minimum_time,
            head_minimum_time=cfg.head_minimum_time,
            control_hold_time=cfg.control_hold_time,
            stiffness=cfg.stiffness,
            damping_ratio=cfg.damping_ratio,
            torque_limit=cfg.torque_limit,
        )


@dataclass(slots=True)
class CartesianCommandSettings:
    linear_velocity_limit: float = 0.3
    angular_velocity_limit: float = 3.14159
    acceleration_limit_scaling: float = 0.8
    minimum_time: float = 1.0
    control_hold_time: float = 1000000.0


class JointCommandClient:
    """Build and send RB-Y1 joint commands from named component targets."""

    _COMPONENT_ORDER = ("torso", "right_arm", "left_arm", "head")
    _DISPLAY_NAMES = {
        "torso": "Torso",
        "right_arm": "Right Arm",
        "left_arm": "Left Arm",
        "head": "Head",
    }
    # EE frame names used in CartesianCommandBuilder.add_target()
    _ARM_EE_FRAMES: dict[str, tuple[str, str]] = {
        "right_arm": ("base", "ee_right"),
        "left_arm": ("base", "ee_left"),
    }

    def __init__(self, robot: Any):
        self._robot = robot
        self._model = robot.model()
        self._stream = self._create_stream()
        self._dyn_robot = robot.get_dynamics()
        dyn_state = self._dyn_robot.make_state([], self._model.robot_joint_names)
        lower_limits = np.asarray(self._dyn_robot.get_limit_q_lower(dyn_state), dtype=np.float64)
        upper_limits = np.asarray(self._dyn_robot.get_limit_q_upper(dyn_state), dtype=np.float64)
        qdot_limits = np.asarray(self._dyn_robot.get_limit_qdot_upper(dyn_state), dtype=np.float64)
        qddot_limits = np.asarray(self._dyn_robot.get_limit_qddot_upper(dyn_state), dtype=np.float64)

        self._group_specs: dict[str, JointGroupSpec] = {}
        for component in self._COMPONENT_ORDER:
            indices = self._component_indices(component)
            if indices.size == 0:
                continue

            velocity_limit = qdot_limits[indices].copy()
            acceleration_limit = qddot_limits[indices].copy()
            if component in ("right_arm", "left_arm") and len(velocity_limit) > 0:
                velocity_limit[-1] *= 10.0
                acceleration_limit *= 30.0

            self._group_specs[component] = JointGroupSpec(
                component=component,
                display_name=self._DISPLAY_NAMES[component],
                joint_names=component_joint_names(self._model, component),
                indices=indices,
                lower_rad=lower_limits[indices].copy(),
                upper_rad=upper_limits[indices].copy(),
                velocity_limit=velocity_limit,
                acceleration_limit=acceleration_limit,
            )

        # FK dyn_states for Cartesian EE control (one per arm)
        self._fk_lock = threading.Lock()
        self._fk_dyn_states: dict[str, Any] = {}
        for arm, (from_frame, to_frame) in self._ARM_EE_FRAMES.items():
            try:
                self._fk_dyn_states[arm] = self._dyn_robot.make_state(
                    [from_frame, to_frame], self._model.robot_joint_names
                )
            except Exception as exc:
                logging.warning("FK dyn_state setup failed for %s: %s", arm, exc)

    @property
    def model(self) -> Any:
        return self._model

    def group_specs(self) -> dict[str, JointGroupSpec]:
        return {
            name: JointGroupSpec(
                component=spec.component,
                display_name=spec.display_name,
                joint_names=list(spec.joint_names),
                indices=spec.indices.copy(),
                lower_rad=spec.lower_rad.copy(),
                upper_rad=spec.upper_rad.copy(),
                velocity_limit=spec.velocity_limit.copy(),
                acceleration_limit=spec.acceleration_limit.copy(),
            )
            for name, spec in self._group_specs.items()
        }

    def targets_from_joint_vector(self, joint_positions: np.ndarray) -> dict[str, np.ndarray]:
        q = np.asarray(joint_positions, dtype=np.float64)
        return {
            component: q[spec.indices].copy()
            for component, spec in self._group_specs.items()
        }

    def current_targets(self) -> dict[str, np.ndarray]:
        state = self._robot.get_state()
        return self.targets_from_joint_vector(np.asarray(state.position, dtype=np.float64))

    def apply_targets(
        self,
        target_by_component: dict[str, np.ndarray],
        settings: JointCommandSettings,
    ) -> Any:
        command = self.build_command(target_by_component, settings)
        try:
            return self._stream.send_command(command)
        except RuntimeError as exc:
            if "expired" not in str(exc).lower():
                raise

            self._stream = self._create_stream()
            return self._stream.send_command(command)

    def build_command(
        self,
        target_by_component: dict[str, np.ndarray],
        settings: JointCommandSettings,
    ) -> Any:
        body_mode = settings.body_mode.lower().strip()
        if body_mode not in {"position", "impedance"}:
            raise ValueError(f"Unsupported body_mode: {settings.body_mode}")

        component_builder = rby.ComponentBasedCommandBuilder()
        body_builder = rby.BodyComponentBasedCommandBuilder()
        has_body_command = False

        for component in ("torso", "right_arm", "left_arm"):
            target = target_by_component.get(component)
            if target is None:
                continue

            joint_builder = self._make_body_joint_builder(
                component,
                np.asarray(target, dtype=np.float64),
                settings,
                body_mode=body_mode,
            )
            setter_name = f"set_{component}_command"
            getattr(body_builder, setter_name)(joint_builder)
            has_body_command = True

        head_target = target_by_component.get("head")
        if head_target is not None:
            component_builder.set_head_command(
                self._make_head_joint_builder(np.asarray(head_target, dtype=np.float64), settings)
            )

        if not has_body_command and head_target is None:
            raise ValueError("No joint targets were provided for any enabled component")

        if has_body_command:
            component_builder.set_body_command(body_builder)

        return rby.RobotCommandBuilder().set_command(component_builder)

    # ------------------------------------------------------------------
    # Cartesian EE control
    # ------------------------------------------------------------------

    def compute_fk(self, component: str) -> np.ndarray | None:
        """Return the current EE pose as a 4×4 matrix in the base frame.

        Uses the robot's live joint state.  Returns None if the FK dyn_state
        for this component is not available.
        """
        if component not in self._fk_dyn_states:
            return None
        try:
            q = np.asarray(self._robot.get_state().position, dtype=np.float64)
            fk_state = self._fk_dyn_states[component]
            with self._fk_lock:
                fk_state.set_q(q)
                self._dyn_robot.compute_forward_kinematics(fk_state)
                T = self._dyn_robot.compute_transformation(fk_state, 0, 1)
            return np.asarray(T, dtype=np.float64).copy()
        except Exception as exc:
            logging.warning("compute_fk failed for %s: %s", component, exc)
            return None

    def build_cartesian_command(
        self,
        targets: dict[str, np.ndarray],
        settings: CartesianCommandSettings,
        joint_holds: dict[str, np.ndarray] | None = None,
        joint_settings: JointCommandSettings | None = None,
    ) -> Any:
        """Build a stream command that moves each arm's EE to the given SE3 target.

        targets maps component name ("right_arm" / "left_arm") to a 4×4 target
        transform expressed in the base frame.

        joint_holds maps body component names to joint position arrays for
        components that are not being Cartesian-controlled.  These are sent as
        joint position hold commands so that omitting them from the stream does
        not drop their control.  joint_settings must be provided when
        joint_holds is non-empty.
        """
        body_builder = rby.BodyComponentBasedCommandBuilder()
        has_any = False
        for component, T_target in targets.items():
            if component not in self._ARM_EE_FRAMES:
                continue
            _, to_frame = self._ARM_EE_FRAMES[component]
            cart_builder = (
                rby.CartesianCommandBuilder()
                .set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(settings.control_hold_time)
                )
                .set_minimum_time(settings.minimum_time)
                .add_target(
                    "base",
                    to_frame,
                    T_target,
                    settings.linear_velocity_limit,
                    settings.angular_velocity_limit,
                    settings.acceleration_limit_scaling,
                )
            )
            setter = f"set_{component}_command"
            getattr(body_builder, setter)(cart_builder)
            has_any = True

        if not has_any:
            raise ValueError("No valid Cartesian arm targets provided")

        # Include joint-position holds for body components not in Cartesian targets
        # so their stream control is not dropped by this command.
        if joint_holds and joint_settings:
            for component, position in joint_holds.items():
                if component not in targets and component in self._group_specs:
                    hold_builder = self._make_body_joint_builder(
                        component,
                        np.asarray(position, dtype=np.float64),
                        joint_settings,
                        body_mode=joint_settings.body_mode,
                    )
                    setter_name = f"set_{component}_command"
                    getattr(body_builder, setter_name)(hold_builder)

        return rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(body_builder)
        )

    def apply_cartesian_targets(
        self,
        targets: dict[str, np.ndarray],
        settings: CartesianCommandSettings,
        joint_holds: dict[str, np.ndarray] | None = None,
        joint_settings: JointCommandSettings | None = None,
    ) -> Any:
        """Send a Cartesian EE command through the shared command stream."""
        command = self.build_cartesian_command(targets, settings, joint_holds, joint_settings)
        try:
            return self._stream.send_command(command)
        except RuntimeError as exc:
            if "expired" not in str(exc).lower():
                raise
            self._stream = self._create_stream()
            return self._stream.send_command(command)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _component_indices(self, component: str) -> np.ndarray:
        indices_by_component = {
            "torso": np.asarray(list(self._model.torso_idx), dtype=np.int32),
            "right_arm": np.asarray(list(self._model.right_arm_idx), dtype=np.int32),
            "left_arm": np.asarray(list(self._model.left_arm_idx), dtype=np.int32),
            "head": np.asarray(list(self._model.head_idx), dtype=np.int32),
        }
        return indices_by_component[component]

    def _create_stream(self) -> Any:
        return self._robot.create_command_stream(priority=1)

    @staticmethod
    def _make_header(settings: JointCommandSettings) -> Any:
        return rby.CommandHeaderBuilder().set_control_hold_time(settings.control_hold_time)

    def _make_body_joint_builder(
        self,
        component: str,
        position: np.ndarray,
        settings: JointCommandSettings,
        body_mode: str,
    ) -> Any:
        spec = self._group_specs[component]
        clipped_position = np.clip(position, spec.lower_rad, spec.upper_rad)

        # Match the SDK teleoperation example: torso stays on joint position
        # control, while arm commands can switch between position/impedance.
        if component == "torso" or body_mode == "position":
            builder = rby.JointPositionCommandBuilder()
        else:
            builder = rby.JointImpedanceControlCommandBuilder()

        (
            builder.set_command_header(self._make_header(settings))
            .set_position(clipped_position.tolist())
            .set_minimum_time(settings.body_minimum_time)
        )

        if component in ("right_arm", "left_arm"):
            (
                builder.set_velocity_limit(spec.velocity_limit.tolist())
                .set_acceleration_limit(spec.acceleration_limit.tolist())
            )

        if component != "torso" and body_mode == "impedance":
            (
                builder.set_stiffness([settings.stiffness] * len(clipped_position))
                .set_damping_ratio(settings.damping_ratio)
                .set_torque_limit([settings.torque_limit] * len(clipped_position))
            )

        return builder

    def _make_head_joint_builder(
        self,
        position: np.ndarray,
        settings: JointCommandSettings,
    ) -> Any:
        return (
            rby.JointPositionCommandBuilder()
            .set_command_header(self._make_header(settings))
            .set_position(position.tolist())
            .set_minimum_time(settings.head_minimum_time)
        )
