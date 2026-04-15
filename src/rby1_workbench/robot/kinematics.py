"""Forward kinematics utilities for live RB-Y1 visualization."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from rby1_workbench.geometry.transform_graph import TransformGraph
from rby1_workbench.geometry.se3 import identity, relative_transform
from rby1_workbench.robot.joints import (
    HEAD_LINK_CHAIN,
    LEFT_ARM_LINK_CHAIN,
    RIGHT_ARM_LINK_CHAIN,
    TORSO_LINK_CHAIN,
    component_joint_names,
    default_frame_label_map,
    default_parent_map,
    unique_link_order,
)


@dataclass(slots=True)
class KinematicResult:
    graph: TransformGraph
    base_transforms: dict[str, np.ndarray]
    frame_labels: dict[str, str]
    skeletons: dict[str, np.ndarray]
    joint_positions_by_name: dict[str, float]


class RobotKinematics:
    """Compute named link transforms and frame graph from the latest joint state."""

    def __init__(self, robot: Any):
        self._robot = robot
        self._model = robot.model()
        self._dyn_robot = robot.get_dynamics()
        self._link_names = unique_link_order()
        self._link_index = {
            name: index for index, name in enumerate(self._link_names)
        }
        self._frame_labels = default_frame_label_map()
        self._parent_map = default_parent_map()
        self._dyn_state = self._dyn_robot.make_state(
            self._link_names, self._model.robot_joint_names
        )

    @property
    def model(self) -> Any:
        return self._model

    @property
    def head_joint_names(self) -> list[str]:
        return component_joint_names(self._model, "head")

    def compute(self, joint_positions: np.ndarray) -> KinematicResult:
        q = np.asarray(joint_positions, dtype=np.float64).copy()

        self._dyn_state.set_q(q)
        self._dyn_robot.compute_forward_kinematics(self._dyn_state)

        base_transforms: dict[str, np.ndarray] = {"base": identity()}
        base_index = self._link_index["base"]
        for link_name in self._link_names[1:]:
            link_index = self._link_index[link_name]
            base_transforms[link_name] = np.asarray(
                self._dyn_robot.compute_transformation(
                    self._dyn_state, base_index, link_index
                ),
                dtype=np.float64,
            )

        graph = TransformGraph(root="base")
        for child, parent in self._parent_map.items():
            if parent is None:
                continue

            graph.add_transform(
                parent,
                child,
                relative_transform(base_transforms[parent], base_transforms[child]),
            )

        skeletons = {
            "torso": self._chain_positions(base_transforms, TORSO_LINK_CHAIN),
            "right_arm": self._chain_positions(base_transforms, RIGHT_ARM_LINK_CHAIN),
            "left_arm": self._chain_positions(base_transforms, LEFT_ARM_LINK_CHAIN),
            "head": self._chain_positions(base_transforms, HEAD_LINK_CHAIN),
        }
        joint_positions_by_name = {
            joint_name: float(q[index])
            for index, joint_name in enumerate(self._model.robot_joint_names)
        }

        return KinematicResult(
            graph=graph,
            base_transforms=base_transforms,
            frame_labels=dict(self._frame_labels),
            skeletons=skeletons,
            joint_positions_by_name=joint_positions_by_name,
        )

    @staticmethod
    def _chain_positions(
        base_transforms: dict[str, np.ndarray], link_names: list[str]
    ) -> np.ndarray:
        return np.asarray(
            [base_transforms[link_name][:3, 3] for link_name in link_names],
            dtype=np.float64,
        )
