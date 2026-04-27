"""Configuration loaders for rby1-workbench."""

from __future__ import annotations

from pathlib import Path

from omegaconf import DictConfig, OmegaConf


def package_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _package_conf_path(name: str) -> Path:
    return Path(__file__).resolve().parents[1] / "conf" / name


def load_rby1_config(path: str | Path | None = None) -> DictConfig:
    """packaged `src/rby1_workbench/conf/rby1.yaml` 기본값을 로드한다."""
    default = OmegaConf.load(_package_conf_path("rby1.yaml"))
    if path is None:
        return default
    return OmegaConf.merge(default, OmegaConf.load(path))


def load_sam3_config(path: str | Path | None = None) -> DictConfig:
    """packaged `src/rby1_workbench/conf/realtime_sam3_realsense.yaml`을 로드한다."""
    default = OmegaConf.load(_package_conf_path("realtime_sam3_realsense.yaml"))
    if path is None:
        return default
    return OmegaConf.merge(default, OmegaConf.load(path))


def load_calib_config(path: str | Path | None = None) -> DictConfig:
    """packaged `src/rby1_workbench/conf/head_camera_calib.yaml`을 로드한다."""
    default = OmegaConf.load(_package_conf_path("head_camera_calib.yaml"))
    if path is None:
        return default
    return OmegaConf.merge(default, OmegaConf.load(path))
