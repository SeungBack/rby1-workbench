"""Viser-based RB-Y1 joint control panel app."""

from __future__ import annotations

import logging

import hydra
from omegaconf import DictConfig

from rby1_workbench.control.viser_joint_control import run_viser_joint_control_panel


logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


@hydra.main(
    config_path="../conf",
    config_name="viser_joint_control_panel",
    version_base=None,
)
def cli(cfg: DictConfig) -> None:
    run_viser_joint_control_panel(cfg)


if __name__ == "__main__":
    cli()
