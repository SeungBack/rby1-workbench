"""Forward kinematics utilities for live RB-Y1 visualization."""

from __future__ import annotations

from dataclasses import dataclass, field
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


@dataclass(slots=True)
class _StaticFrame:
    parent: str
    child: str
    T_parent_to_child: np.ndarray
    label: str


class RobotKinematics:
    """Compute named link transforms and frame graph from the latest joint state."""

    def __init__(self, robot: Any, urdf_data: str = "", flip_joints: list[str] | None = None):
        self._robot = robot
        self._model = robot.model()
        self._dyn_robot = robot.get_dynamics(urdf_data)
        flip_set = set(flip_joints or [])
        self._flip_indices: list[int] = [
            i for i, name in enumerate(self._model.robot_joint_names)
            if name in flip_set
        ]
        self._link_names = unique_link_order()
        self._link_index = {
            name: index for index, name in enumerate(self._link_names)
        }
        self._frame_labels = default_frame_label_map()
        self._parent_map = default_parent_map()
        self._dyn_state = self._dyn_robot.make_state(
            self._link_names, self._model.robot_joint_names
        )
        self._static_frames: list[_StaticFrame] = []

    def register_static_frame(
        self,
        parent: str,
        child: str,
        T_parent_to_child: np.ndarray,
        label: str | None = None,
    ) -> None:
        """Register a static child frame (e.g., camera calibration result).

        After registration, every compute() call includes this frame in
        base_transforms, graph, and frame_labels automatically.

        parent must be a robot link name or a previously registered static frame.
        Frames are processed in registration order, so chained static frames
        must be registered parent-first.
        """
        self._static_frames.append(_StaticFrame(
            parent=parent,
            child=child,
            T_parent_to_child=np.asarray(T_parent_to_child, dtype=np.float64).copy(),
            label=label if label is not None else child,
        ))

    @property
    def model(self) -> Any:
        return self._model

    @property
    def head_joint_names(self) -> list[str]:
        return component_joint_names(self._model, "head")

    def compute(self, joint_positions: np.ndarray) -> KinematicResult:
        q = np.asarray(joint_positions, dtype=np.float64).copy()

        q_fk = q.copy()
        for idx in self._flip_indices:
            q_fk[idx] = -q_fk[idx]

        self._dyn_state.set_q(q_fk)
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

        frame_labels = dict(self._frame_labels)

        for sf in self._static_frames:
            if sf.parent not in base_transforms:
                raise KeyError(
                    f"Static frame parent '{sf.parent}' not found. "
                    "Register parent frames before child frames."
                )
            base_transforms[sf.child] = base_transforms[sf.parent] @ sf.T_parent_to_child
            graph.add_transform(sf.parent, sf.child, sf.T_parent_to_child)
            frame_labels[sf.child] = sf.label

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
            frame_labels=frame_labels,
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
