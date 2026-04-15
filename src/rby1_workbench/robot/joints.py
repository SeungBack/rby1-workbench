"""Joint/link naming helpers for RB-Y1 kinematic visualization."""

from __future__ import annotations

from typing import Any


TORSO_LINK_CHAIN = ["base"] + [f"link_torso_{index}" for index in range(6)]
RIGHT_ARM_LINK_CHAIN = TORSO_LINK_CHAIN + [f"link_right_arm_{index}" for index in range(7)] + ["ee_right"]
LEFT_ARM_LINK_CHAIN = TORSO_LINK_CHAIN + [f"link_left_arm_{index}" for index in range(7)] + ["ee_left"]
HEAD_LINK_CHAIN = TORSO_LINK_CHAIN + [f"link_head_{index}" for index in range(3)]


def unique_link_order() -> list[str]:
    ordered_links: list[str] = []
    seen: set[str] = set()

    for chain in (TORSO_LINK_CHAIN, RIGHT_ARM_LINK_CHAIN, LEFT_ARM_LINK_CHAIN, HEAD_LINK_CHAIN):
        for link_name in chain:
            if link_name not in seen:
                seen.add(link_name)
                ordered_links.append(link_name)

    return ordered_links


def default_parent_map() -> dict[str, str | None]:
    parent_map: dict[str, str | None] = {"base": None}

    for chain in (TORSO_LINK_CHAIN, RIGHT_ARM_LINK_CHAIN, LEFT_ARM_LINK_CHAIN, HEAD_LINK_CHAIN):
        for parent, child in zip(chain, chain[1:]):
            parent_map.setdefault(child, parent)

    return parent_map


def default_frame_label_map() -> dict[str, str]:
    labels: dict[str, str] = {"base": "base", "ee_right": "ee_right", "ee_left": "ee_left"}

    for index in range(6):
        labels[f"link_torso_{index}"] = f"torso_{index}"

    for index in range(7):
        labels[f"link_right_arm_{index}"] = f"right_arm_{index}"
        labels[f"link_left_arm_{index}"] = f"left_arm_{index}"

    labels["link_head_0"] = "head_base"
    labels["link_head_1"] = "head_0"
    labels["link_head_2"] = "head_1"
    return labels


def joint_index_map(model: Any) -> dict[str, int]:
    return {name: index for index, name in enumerate(model.robot_joint_names)}


def component_joint_names(model: Any, component_name: str) -> list[str]:
    indices_by_component = {
        "torso": list(model.torso_idx),
        "right_arm": list(model.right_arm_idx),
        "left_arm": list(model.left_arm_idx),
        "head": list(model.head_idx),
    }
    indices = indices_by_component[component_name]
    return [model.robot_joint_names[index] for index in indices]
