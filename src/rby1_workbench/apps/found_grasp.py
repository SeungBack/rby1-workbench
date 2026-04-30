"""Realtime FoundGrasp grasp inference app."""

from __future__ import annotations

import logging

import hydra
from omegaconf import DictConfig

from rby1_workbench.perception.found_grasp_runner import run_found_grasp

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")


@hydra.main(
    config_path="../conf",
    config_name="found_grasp",
    version_base=None,
)
def cli(cfg: DictConfig) -> None:
    run_found_grasp(cfg)


if __name__ == "__main__":
    cli()
