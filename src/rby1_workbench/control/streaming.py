"""Periodic streaming controllers aligned with the RB-Y1 SDK examples."""

from __future__ import annotations

import logging
import threading
from typing import Any

import numpy as np

from rby1_workbench.control.joint_commands import JointCommandClient, JointGroupSpec
from rby1_workbench.control.settings import CartesianCommandSettings, JointCommandSettings
from rby1_workbench.control.target_state import CartesianTargetState, JointTargetState


def _rodrigues(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    k = axis / np.linalg.norm(axis)
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]], dtype=np.float64)
    return c * np.eye(3, dtype=np.float64) + s * K + (1 - c) * np.outer(k, k)


class JointStreamingController:
    """Continuously re-sends joint targets like the official streaming examples.

    The core idea is to separate:
    - target state mutation
    - periodic stream sending

    This avoids event-driven partial sends fighting shared stream state.
    """

    def __init__(
        self,
        robot: Any | None,
        enabled_components: set[str],
        settings: JointCommandSettings | None = None,
        *,
        client: JointCommandClient | None = None,
        period_s: float = 0.02,
    ) -> None:
        if client is None:
            if robot is None:
                raise ValueError("Either robot or client must be provided")
            self._client = JointCommandClient(robot)
        else:
            self._client = client
        available = set(self._client.group_specs().keys())
        self._enabled = frozenset(enabled_components & available)
        self._specs: dict[str, JointGroupSpec] = {
            name: spec
            for name, spec in self._client.group_specs().items()
            if name in self._enabled
        }
        self._period_s = float(period_s)
        self._settings = settings or JointCommandSettings()
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

        current = self._client.current_targets()
        self._targets = JointTargetState.from_component_targets(
            {
                component: current[component]
                for component in self._enabled
                if component in current
            }
        )

    @property
    def enabled_components(self) -> frozenset[str]:
        return self._enabled

    def specs(self) -> dict[str, JointGroupSpec]:
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
            for name, spec in self._specs.items()
        }

    def sync_from_robot(self) -> None:
        current = self._client.current_targets()
        with self._lock:
            for component in self._enabled:
                if component in current:
                    self._targets.set(component, current[component])

    def get_target(self, component: str) -> np.ndarray | None:
        with self._lock:
            return self._targets.get(component)

    def get_targets(self) -> JointTargetState:
        with self._lock:
            return self._targets.copy()

    def set_target(self, component: str, target_rad: np.ndarray) -> None:
        spec = self._specs[component]
        clipped = np.clip(
            np.asarray(target_rad, dtype=np.float64),
            spec.lower_rad,
            spec.upper_rad,
        )
        with self._lock:
            self._targets.set(component, clipped)

    def set_target_rad(self, component: str, joint_index: int, value_rad: float) -> None:
        with self._lock:
            current = self._targets.get(component)
            if current is None:
                current = np.zeros(len(self._specs[component].joint_names), dtype=np.float64)
            spec = self._specs[component]
            current[joint_index] = float(
                np.clip(value_rad, spec.lower_rad[joint_index], spec.upper_rad[joint_index])
            )
            self._targets.set(component, current)

    def jog_target(self, component: str, joint_index: int, delta_rad: float) -> None:
        with self._lock:
            current = self._targets.get(component)
            if current is None:
                current = np.zeros(len(self._specs[component].joint_names), dtype=np.float64)
            spec = self._specs[component]
            current[joint_index] = float(
                np.clip(
                    current[joint_index] + delta_rad,
                    spec.lower_rad[joint_index],
                    spec.upper_rad[joint_index],
                )
            )
            self._targets.set(component, current)

    def update_settings(self, settings: JointCommandSettings) -> None:
        with self._lock:
            self._settings = settings

    def send_once(self) -> None:
        with self._lock:
            targets = self._targets.as_dict()
            settings = self._settings
        if not targets:
            return
        self._client.apply_targets(targets, settings)

    def _stream_settings(self) -> JointCommandSettings:
        with self._lock:
            return self._settings

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=max(self._period_s * 4.0, 1.0))
            if self._thread.is_alive():
                logging.warning("JointStreamingController thread did not stop cleanly")
            else:
                self._thread = None

    def _run_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                with self._lock:
                    targets = self._targets.as_dict()
                if targets:
                    self._client.apply_targets(targets, self._stream_settings())
            except Exception:
                logging.exception("JointStreamingController loop failed")
            if self._stop_event.wait(self._period_s):
                break


class CartesianStreamingController:
    """Example-aligned Cartesian target controller.

    Unlike the joint teleoperation loop, the official Cartesian examples send a
    target once through the command stream and then monitor feedback rather than
    continuously re-sending the same target in a tight loop.  This controller
    therefore stores target state but performs one-shot sends on demand.
    """

    _ARM_COMPONENTS = frozenset({"right_arm", "left_arm"})

    def __init__(
        self,
        robot: Any | None,
        enabled_arms: set[str],
        *,
        client: JointCommandClient | None = None,
        cart_settings: CartesianCommandSettings | None = None,
        period_s: float = 0.02,
    ) -> None:
        if client is None:
            if robot is None:
                raise ValueError("Either robot or client must be provided")
            self._client = JointCommandClient(robot)
        else:
            self._client = client
        self._enabled = frozenset(enabled_arms & self._ARM_COMPONENTS)
        self._cart_settings = cart_settings or CartesianCommandSettings()
        self._lock = threading.RLock()
        self._active_arm: str | None = next(iter(sorted(self._enabled)), None)

        targets: dict[str, np.ndarray] = {}
        for arm in self._enabled:
            T = self._client.compute_fk(arm)
            if T is not None:
                targets[arm] = T
        self._targets = CartesianTargetState.from_arm_targets(targets)

    @property
    def enabled_arms(self) -> frozenset[str]:
        return self._enabled

    @property
    def active_arm(self) -> str | None:
        with self._lock:
            return self._active_arm

    def set_active_arm(self, arm: str) -> None:
        if arm not in self._enabled:
            raise ValueError(f"Arm '{arm}' is not enabled for Cartesian control")
        with self._lock:
            self._active_arm = arm

    def sync_from_robot(self, arms: set[str] | None = None) -> None:
        names = self._enabled if arms is None else (arms & self._enabled)
        with self._lock:
            for arm in names:
                T = self._client.compute_fk(arm)
                if T is not None:
                    self._targets.set(arm, T)

    def get_target(self, arm: str) -> np.ndarray | None:
        with self._lock:
            return self._targets.get(arm)

    def get_targets(self) -> CartesianTargetState:
        with self._lock:
            return self._targets.copy()

    def set_target(self, arm: str, target: np.ndarray) -> None:
        if arm not in self._enabled:
            return
        with self._lock:
            self._targets.set(arm, target)

    def jog_position(self, arm: str, axis: np.ndarray, delta_m: float) -> None:
        if arm not in self._enabled:
            return
        with self._lock:
            current = self._targets.get(arm)
            if current is None:
                current = np.eye(4, dtype=np.float64)
            current[:3, 3] += axis * delta_m
            self._targets.set(arm, current)

    def jog_orientation(self, arm: str, axis: np.ndarray, delta_rad: float) -> None:
        if arm not in self._enabled:
            return
        with self._lock:
            current = self._targets.get(arm)
            if current is None:
                current = np.eye(4, dtype=np.float64)
            current[:3, :3] = _rodrigues(axis, delta_rad) @ current[:3, :3]
            self._targets.set(arm, current)

    def update_settings(
        self,
        cart_settings: CartesianCommandSettings,
    ) -> None:
        with self._lock:
            self._cart_settings = cart_settings

    def _active_targets(self) -> dict[str, np.ndarray]:
        with self._lock:
            if self._active_arm is None:
                return {}
            target = self._targets.get(self._active_arm)
            if target is None:
                return {}
            return {self._active_arm: target}

    def send_once(self) -> None:
        with self._lock:
            active_arm = self._active_arm
            cart_settings = self._cart_settings
            target = None if active_arm is None else self._targets.get(active_arm)
        if active_arm is None or target is None:
            return
        self._client.apply_cartesian_stream_target(
            active_arm,
            target,
            cart_settings,
        )

    def start(self) -> None:
        return

    def stop(self) -> None:
        return
