"""Legacy stateful robot controller for joint and Cartesian body+head control.

Prefer `target_state.py` + `streaming.py` for new work.
"""

from __future__ import annotations

import logging
from typing import Any

import numpy as np

from rby1_workbench.control.joint_commands import JointCommandClient, JointGroupSpec
from rby1_workbench.control.settings import CartesianCommandSettings, JointCommandSettings


def _rodrigues(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    """3×3 rotation matrix via Rodrigues formula."""
    k = axis / np.linalg.norm(axis)
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]], dtype=np.float64)
    return c * np.eye(3, dtype=np.float64) + s * K + (1 - c) * np.outer(k, k)


class RobotController:
    """Stateful controller wrapping JointCommandClient.

    Responsibilities:
    - Joint and Cartesian target state per component
    - Grouped body commands (all enabled body components always sent together
      so no component loses stream control)
    - Automatic joint target resync when sending joint commands after Cartesian,
      preventing stale-target snapping when arms were moved via Cartesian control
    """

    _BODY_COMPONENTS = frozenset({"torso", "right_arm", "left_arm"})
    _ARM_COMPONENTS = frozenset({"right_arm", "left_arm"})

    def __init__(self, robot: Any, enabled_components: set[str]) -> None:
        self._client = JointCommandClient(robot)
        available = set(self._client.group_specs().keys())
        self._enabled = frozenset(enabled_components & available)
        self._enabled_body = self._enabled & self._BODY_COMPONENTS
        self._enabled_arms = self._enabled & self._ARM_COMPONENTS
        self._specs: dict[str, JointGroupSpec] = {
            k: v for k, v in self._client.group_specs().items() if k in self._enabled
        }

        all_targets = self._client.current_targets()
        self._joint_targets: dict[str, np.ndarray] = {
            c: v.copy() for c, v in all_targets.items() if c in self._enabled
        }
        self._cart_targets: dict[str, np.ndarray] = {}
        for arm in self._enabled_arms:
            T = self._client.compute_fk(arm)
            self._cart_targets[arm] = T if T is not None else np.eye(4, dtype=np.float64)
        self._joint_targets_stale: dict[str, bool] = {component: False for component in self._enabled}
        self._joint_targets_user_modified: dict[str, bool] = {
            component: False for component in self._enabled
        }
        self._stale_cartesian_arms: set[str] = set()

    # ------------------------------------------------------------------
    # Read-only properties
    # ------------------------------------------------------------------

    @property
    def enabled_components(self) -> frozenset[str]:
        return self._enabled

    @property
    def model(self) -> Any:
        return self._client.model

    def specs(self) -> dict[str, JointGroupSpec]:
        return dict(self._specs)

    def get_joint_target_rad(self, component: str, joint_index: int) -> float:
        return float(self._joint_targets[component][joint_index])

    def get_joint_targets(self, component: str) -> np.ndarray:
        return self._joint_targets[component].copy()

    def get_cartesian_target(self, arm: str) -> np.ndarray | None:
        T = self._cart_targets.get(arm)
        return T.copy() if T is not None else None

    # ------------------------------------------------------------------
    # Joint target manipulation
    # ------------------------------------------------------------------

    def jog_joint(self, component: str, joint_index: int, delta_rad: float) -> None:
        spec = self._specs[component]
        # Always resync from live robot state so jog delta is relative to the
        # actual joint position, not a stale target left over from Cartesian control.
        self._resync_joint_targets({component})
        current = float(self._joint_targets[component][joint_index])
        self._joint_targets[component][joint_index] = float(
            np.clip(current + delta_rad, spec.lower_rad[joint_index], spec.upper_rad[joint_index])
        )
        self._joint_targets_user_modified[component] = True

    def set_joint_targets(self, component: str, values_rad: np.ndarray) -> None:
        spec = self._specs[component]
        self._joint_targets[component] = np.clip(
            np.asarray(values_rad, dtype=np.float64), spec.lower_rad, spec.upper_rad
        ).copy()
        self._joint_targets_stale[component] = False
        self._joint_targets_user_modified[component] = True

    def set_joint_target_rad(self, component: str, joint_index: int, value_rad: float) -> None:
        spec = self._specs[component]
        self._joint_targets[component][joint_index] = float(
            np.clip(value_rad, spec.lower_rad[joint_index], spec.upper_rad[joint_index])
        )
        self._joint_targets_stale[component] = False
        self._joint_targets_user_modified[component] = True

    # ------------------------------------------------------------------
    # Cartesian target manipulation
    # ------------------------------------------------------------------

    def jog_cartesian_position(self, arm: str, axis: np.ndarray, delta_m: float) -> None:
        # Rebase each jog on live FK so Cartesian motion still works right
        # after joint commands and while switching control modes.
        self.sync_cartesian_from_fk(arm)
        self._cart_targets[arm][:3, 3] += axis * delta_m

    def jog_cartesian_orientation(self, arm: str, axis: np.ndarray, delta_rad: float) -> None:
        self.sync_cartesian_from_fk(arm)
        T = self._cart_targets[arm]
        T[:3, :3] = _rodrigues(axis, delta_rad) @ T[:3, :3]

    def sync_cartesian_from_fk(self, arm: str) -> bool:
        """Update Cartesian target from live FK. Returns True on success."""
        T = self._client.compute_fk(arm)
        if T is None:
            return False
        self._cart_targets[arm] = T.copy()
        self._stale_cartesian_arms.discard(arm)
        return True

    # ------------------------------------------------------------------
    # Send
    # ------------------------------------------------------------------

    def _resync_joint_targets(self, components: set[str]) -> None:
        current = self._client.current_targets()
        for c in components:
            if c in current:
                self._joint_targets[c] = current[c].copy()
                self._joint_targets_stale[c] = False
                self._joint_targets_user_modified[c] = False

    def _mark_joint_targets_stale(self, components: set[str]) -> None:
        for component in components:
            if component in self._joint_targets_stale:
                self._joint_targets_stale[component] = True
                self._joint_targets_user_modified[component] = False

    def _mark_cartesian_targets_stale(self) -> None:
        self._stale_cartesian_arms = set(self._enabled_arms)

    def _sync_stale_cartesian_targets(self) -> None:
        for arm in list(self._stale_cartesian_arms):
            self.sync_cartesian_from_fk(arm)

    def resync_cartesian_targets(self) -> None:
        """Update all Cartesian arm targets from live FK."""
        for arm in self._enabled_arms:
            self.sync_cartesian_from_fk(arm)

    def send_joint_for_component(
        self,
        active_component: str,
        settings: JointCommandSettings,
        *,
        resync_others: bool = True,
    ) -> Any:
        """Send joint command for active_component.

        Sends only the active component, matching the SDK examples where
        component-level commands are sent independently.  If that component's
        joint target became stale due to prior Cartesian control and the user
        has not edited it since, it is first resynced from the live robot state.
        """
        send_set = {active_component}
        if resync_others:
            if (
                self._joint_targets_stale.get(active_component, False)
                and not self._joint_targets_user_modified.get(active_component, False)
            ):
                self._resync_joint_targets({active_component})

        targets = {c: self._joint_targets[c].copy() for c in send_set if c in self._joint_targets}
        if not targets:
            return None
        result = self._client.apply_targets(targets, settings)
        for component in targets:
            self._joint_targets_stale[component] = False
            self._joint_targets_user_modified[component] = False
        if active_component in self._BODY_COMPONENTS:
            self._mark_cartesian_targets_stale()
        logging.debug("Sent joint for %s", active_component)
        return result

    def send_all_joint(self, settings: JointCommandSettings) -> Any:
        """Send joint targets for ALL enabled components (explicit global send)."""
        targets = {
            c: self._joint_targets[c].copy() for c in self._enabled if c in self._joint_targets
        }
        if not targets:
            return None
        result = self._client.apply_targets(targets, settings)
        for component in targets:
            self._joint_targets_stale[component] = False
            self._joint_targets_user_modified[component] = False
        if self._enabled_body:
            self._mark_cartesian_targets_stale()
        return result

    def send_cartesian(
        self,
        cart_settings: CartesianCommandSettings,
        joint_settings: JointCommandSettings,
        active_arm: str | None = None,
    ) -> Any:
        """Send Cartesian command for one arm or all enabled arms.

        Per-arm sends match the SDK examples more closely and avoid unintended
        re-commanding of the other arm while the user is manipulating only one side.
        """
        if active_arm is None:
            self._sync_stale_cartesian_targets()
            arm_names = list(self._enabled_arms)
        else:
            if active_arm not in self._enabled_arms:
                return None
            if active_arm in self._stale_cartesian_arms:
                self.sync_cartesian_from_fk(active_arm)
            arm_names = [active_arm]

        cart_targets = {
            arm: self._cart_targets[arm].copy()
            for arm in arm_names
            if arm in self._cart_targets
        }
        if not cart_targets:
            return None
        result = self._client.apply_cartesian_targets(
            cart_targets,
            cart_settings,
        )
        self._mark_joint_targets_stale(set(cart_targets))
        logging.debug("Sent Cartesian for %s", sorted(cart_targets))
        return result
