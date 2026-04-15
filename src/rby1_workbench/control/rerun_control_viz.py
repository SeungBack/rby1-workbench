"""Rerun visualization companion for the viser joint control panel."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

import numpy as np
import rerun as rr

from rby1_workbench.config.schema import ControlVizConfig, VizConfig
from rby1_workbench.robot.kinematics import RobotKinematics
from rby1_workbench.viz.rerun_session import RerunSession


# RGB axis colors for current EE frames (live FK)
_EE_CURRENT_COLORS: dict[str, list[list[int]]] = {
    "right_arm": [[220, 80,  80],  [80, 200, 80],  [80,  80, 220]],
    "left_arm":  [[80, 120, 220],  [80, 200, 80],  [220, 180, 80]],
}
# RGB axis colors for Cartesian target frames (from jog)
_EE_TARGET_COLORS: dict[str, list[list[int]]] = {
    "right_arm": [[255, 140,  40], [40, 220, 140], [40, 140, 255]],
    "left_arm":  [[40, 200, 255],  [40, 220, 140], [255, 220,  40]],
}
_EE_FRAME_NAMES: dict[str, str] = {
    "right_arm": "ee_right",
    "left_arm": "ee_left",
}


class ControlRerunViz:
    """Optional Rerun viewer that runs alongside the viser joint control panel.

    Background thread logs the live robot skeleton and current EE frames from
    FK at ``state_update_hz``.  Call :meth:`log_cartesian_target` immediately
    after each jog step to keep the target frame up-to-date in the viewer.
    """

    def __init__(self, cfg: ControlVizConfig, robot: Any) -> None:
        self._cfg = cfg
        self._robot = robot
        # Own kinematics instance — only accessed from the background thread.
        self._kinematics = RobotKinematics(robot)

        # Build a VizConfig compatible with RerunSession from our slimmer config.
        viz_cfg = VizConfig(
            application_id=cfg.application_id,
            spawn_viewer=cfg.spawn_viewer,
            world_frame=cfg.world_frame,
            arrow_length_m=cfg.arrow_length_m,
            log_robot_frames=True,
            log_head_frames=True,
            log_joint_scalars=False,
            log_skeletons=cfg.log_skeletons,
            log_meshes=False,
        )
        self._session = RerunSession(viz_cfg)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        """Initialise the Rerun session and start the background viz thread."""
        self._session.init()
        self._thread = threading.Thread(
            target=self._viz_loop, name="control-rerun-viz", daemon=True
        )
        self._thread.start()
        logging.info("Rerun control viz started (app_id=%s)", self._cfg.application_id)

    def stop(self) -> None:
        """Signal the background thread to stop and wait for it."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # ------------------------------------------------------------------
    # Called from the main viser thread after each jog / sync
    # ------------------------------------------------------------------

    def log_cartesian_target(self, arm: str, T: np.ndarray) -> None:
        """Log a Cartesian target frame immediately (main thread, non-blocking)."""
        try:
            ee_frame = _EE_FRAME_NAMES.get(arm, arm)
            colors = _EE_TARGET_COLORS.get(arm, [[255, 200, 40], [40, 220, 140], [40, 140, 255]])
            path = f"{self._cfg.world_frame}/robot/control/target_{ee_frame}"
            self._session.log_frame(path, T, label=f"target {ee_frame}", colors=colors)
        except Exception:
            logging.exception("log_cartesian_target failed for %s", arm)

    # ------------------------------------------------------------------
    # Background thread
    # ------------------------------------------------------------------

    def _viz_loop(self) -> None:
        interval = 1.0 / max(self._cfg.state_update_hz, 0.1)
        step = 0
        while not self._stop_event.is_set():
            try:
                state = self._robot.get_state()
                q = np.asarray(state.position, dtype=np.float64)
                result = self._kinematics.compute(q)

                rr.set_time("step", sequence=step)
                step += 1

                # Robot frame graph
                self._session.log_transform_graph(
                    result.graph,
                    namespace=f"{self._cfg.world_frame}/robot",
                    frame_labels=result.frame_labels,
                )

                # Skeletons
                if self._cfg.log_skeletons:
                    self._session.log_line_strip(
                        f"{self._cfg.world_frame}/robot/torso_chain",
                        result.skeletons["torso"],
                        [190, 190, 190],
                    )
                    self._session.log_line_strip(
                        f"{self._cfg.world_frame}/robot/right_arm_chain",
                        result.skeletons["right_arm"],
                        [255, 120, 120],
                    )
                    self._session.log_line_strip(
                        f"{self._cfg.world_frame}/robot/left_arm_chain",
                        result.skeletons["left_arm"],
                        [120, 160, 255],
                    )
                    self._session.log_line_strip(
                        f"{self._cfg.world_frame}/robot/head_chain",
                        result.skeletons["head"],
                        [255, 220, 120],
                    )

                # Current EE frames (live FK)
                for arm, ee_frame in _EE_FRAME_NAMES.items():
                    T_ee = result.base_transforms.get(ee_frame)
                    if T_ee is not None:
                        colors = _EE_CURRENT_COLORS.get(
                            arm, [[220, 60, 60], [60, 220, 60], [60, 60, 220]]
                        )
                        path = f"{self._cfg.world_frame}/robot/control/current_{ee_frame}"
                        self._session.log_frame(path, T_ee, label=f"current {ee_frame}", colors=colors)

            except Exception:
                logging.exception("Rerun viz loop error")

            time.sleep(interval)
