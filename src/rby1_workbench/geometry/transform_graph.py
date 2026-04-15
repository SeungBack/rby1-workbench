"""Lightweight transform graph for frame tracking and visualization."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from rby1_workbench.geometry.se3 import compose, identity


@dataclass(frozen=True, slots=True)
class TransformEdge:
    parent: str
    child: str
    transform: np.ndarray


class TransformGraph:
    """Directed tree of parent->child transforms."""

    def __init__(self, root: str):
        self.root = root
        self._edges: list[TransformEdge] = []
        self._children_by_parent: dict[str, list[str]] = {root: []}
        self._edge_by_child: dict[str, TransformEdge] = {}

    def add_transform(self, parent: str, child: str, transform: np.ndarray) -> None:
        if child == self.root:
            raise ValueError("Root frame cannot be added as a child.")
        if child in self._edge_by_child:
            raise ValueError(f"Frame '{child}' already exists in the graph.")

        edge = TransformEdge(
            parent=parent,
            child=child,
            transform=np.asarray(transform, dtype=np.float64),
        )
        self._edges.append(edge)
        self._edge_by_child[child] = edge
        self._children_by_parent.setdefault(parent, []).append(child)
        self._children_by_parent.setdefault(child, [])

    def edges(self) -> list[TransformEdge]:
        return list(self._edges)

    def frames(self) -> list[str]:
        names = [self.root]
        names.extend(edge.child for edge in self._edges)
        return names

    def resolve(self, root_transform: np.ndarray | None = None) -> dict[str, np.ndarray]:
        absolute: dict[str, np.ndarray] = {
            self.root: identity() if root_transform is None else np.asarray(root_transform, dtype=np.float64)
        }

        for edge in self._edges:
            parent_transform = absolute[edge.parent]
            absolute[edge.child] = compose(parent_transform, edge.transform)

        return absolute
