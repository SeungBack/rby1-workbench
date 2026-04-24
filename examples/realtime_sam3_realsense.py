"""Programmatic example for launching realtime RealSense + SAM3 segmentation."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rby1_workbench import (
    OpenCVVisualizerConfig,
    RealSenseConfig,
    RealtimeSam3AppConfig,
    Sam3Config,
    run_realtime_sam3_realsense,
)


def main() -> None:
    cfg = RealtimeSam3AppConfig(
        realsense=RealSenseConfig(
            color_width=640,
            color_height=480,
            depth_width=640,
            depth_height=480,
            fps=30,
        ),
        sam3=Sam3Config(
            checkpoint_path=None,
            device="auto",
            confidence_threshold=0.5,
        ),
        visualizer=OpenCVVisualizerConfig(
            window_name="RealSense SAM3 Example",
            show_depth=True,
        ),
        initial_text_prompt="bottle",
    )
    run_realtime_sam3_realsense(cfg)


if __name__ == "__main__":
    main()
