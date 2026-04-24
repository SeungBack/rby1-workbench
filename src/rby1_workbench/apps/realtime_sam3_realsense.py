"""Realtime RealSense + SAM3 segmentation app."""

from __future__ import annotations

import logging

import hydra
from omegaconf import DictConfig

from rby1_workbench.perception.realtime_segmentation import run_realtime_sam3_realsense


logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


@hydra.main(
    config_path="../conf",
    config_name="realtime_sam3_realsense",
    version_base=None,
)
def cli(cfg: DictConfig) -> None:
    run_realtime_sam3_realsense(cfg)


if __name__ == "__main__":
    cli()
