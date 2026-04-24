"""Rerun session wrapper for frame and skeleton logging."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

from omegaconf import DictConfig
from rby1_workbench.geometry.transform_graph import TransformGraph


FRAME_COLORS = [[220, 60, 60], [60, 220, 60], [60, 60, 220]]


class RerunSession:
    """Manage a simple 3D Rerun scene for robot frames."""

    def __init__(self, cfg: DictConfig):
        self.cfg = cfg
        self._static_mesh_entities: set[str] = set()

    def init(self) -> None:
        blueprint = rrb.Blueprint(
            rrb.Spatial3DView(
                origin="/",
                eye_controls=rrb.EyeControls3D(
                    position=(2.7, -1.8, 1.6),
                    look_target=(0.0, 0.0, 0.8),
                    eye_up=(0.0, 0.0, 1.0),
                ),
            )
        )

        rr.init(self.cfg.application_id, spawn=self.cfg.spawn_viewer)
        rr.send_blueprint(blueprint)
        rr.log(self.cfg.world_frame, rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        self.log_frame(
            f"{self.cfg.world_frame}/origin",
            np.eye(4, dtype=np.float64),
            arrow_length=self.cfg.arrow_length_m,
            static=True,
        )

    def log_frame(
        self,
        path: str,
        transform: np.ndarray,
        label: str | None = None,
        arrow_length: float | None = None,
        colors: Iterable[Iterable[int]] = FRAME_COLORS,
        static: bool = False,
    ) -> None:
        length = self.cfg.arrow_length_m if arrow_length is None else arrow_length
        position = transform[:3, 3]
        axes = transform[:3, :3] * length

        rr.log(
            f"{path}/axes",
            rr.Arrows3D(
                origins=[position] * 3,
                vectors=[axes[:, 0], axes[:, 1], axes[:, 2]],
                colors=list(colors),
            ),
            static=static,
        )
        rr.log(
            f"{path}/origin",
            rr.Points3D(
                [position],
                radii=[0.01],
                labels=[label] if label is not None else None,
            ),
            static=static,
        )

    def log_transform_graph(
        self,
        graph: TransformGraph,
        namespace: str,
        frame_labels: dict[str, str] | None = None,
    ) -> None:
        absolute_transforms = graph.resolve()
        for frame_name, transform in absolute_transforms.items():
            label = None if frame_labels is None else frame_labels.get(frame_name, frame_name)
            self.log_frame(f"{namespace}/{frame_name}", transform, label=label)

    @staticmethod
    def log_line_strip(path: str, points: np.ndarray, color: list[int]) -> None:
        rr.log(path, rr.LineStrips3D([points], colors=[color]))

    @staticmethod
    def log_scalar(path: str, value: float) -> None:
        rr.log(path, rr.Scalars(float(value)))

    def log_meshes(
        self,
        namespace: str,
        base_transforms: dict[str, np.ndarray],
        mesh_paths: dict[str, Path],
    ) -> None:
        for frame_name, asset_path in mesh_paths.items():
            transform = base_transforms.get(frame_name)
            if transform is None:
                continue

            entity_path = f"{namespace}/{frame_name}/mesh"
            if entity_path not in self._static_mesh_entities:
                rr.log(entity_path, rr.Asset3D(path=asset_path), static=True)
                self._static_mesh_entities.add(entity_path)

            rr.log(
                entity_path,
                rr.Transform3D(
                    translation=transform[:3, 3],
                    mat3x3=transform[:3, :3],
                ),
            )
