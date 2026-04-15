"""Live RB-Y1 robot frame visualization using Rerun."""

from __future__ import annotations

import logging

import hydra
from omegaconf import DictConfig

from rby1_workbench.config.schema import load_visualize_robot_config
from rby1_workbench.viz.live_robot_viewer import run_visualize_robot


logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


@hydra.main(config_path="../conf", config_name="visualize_robot", version_base=None)
def cli(cfg: DictConfig) -> None:
    app_cfg = load_visualize_robot_config(cfg)
    run_visualize_robot(app_cfg)


if __name__ == "__main__":
    cli()
