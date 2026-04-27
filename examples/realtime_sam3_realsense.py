"""Programmatic example for launching realtime RealSense + SAM3 segmentation."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from rby1_workbench.config import load_sam3_config
from rby1_workbench.perception.realtime_segmentation import run_realtime_sam3_realsense


def main() -> None:
    cfg = load_sam3_config()
    cfg.sam3.enable_compile = False
    cfg.sam3.resolution = 1008
    cfg.sam3.checkpoint_path = None  # set your model path here
    cfg.visualizer.window_name = "RealSense SAM3 Example"
    cfg.visualizer.show_depth = True
    cfg.initial_text_prompt = "bottle"
    run_realtime_sam3_realsense(cfg)


if __name__ == "__main__":
    main()
