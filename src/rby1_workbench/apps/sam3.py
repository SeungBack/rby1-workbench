"""Realtime RealSense + SAM3 segmentation app."""

from __future__ import annotations

import logging

import hydra
from omegaconf import DictConfig

from rby1_workbench.perception.sam3_runner import run_sam3


logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)


@hydra.main(
    config_path="../conf",
    config_name="sam3",
    version_base=None,
)
def cli(cfg: DictConfig) -> None:
    run_sam3(cfg)


if __name__ == "__main__":
    cli()
