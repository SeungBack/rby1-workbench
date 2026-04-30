"""Reusable runtime loop for RealSense + SAM3 live segmentation."""

from __future__ import annotations

from collections import deque
import logging
import time

from omegaconf import DictConfig
from rby1_workbench.perception.realsense import RealSenseStream
from rby1_workbench.perception.sam3 import Sam3RealtimePredictor
from rby1_workbench.perception.visualizer import OpenCVPromptVisualizer


LOGGER = logging.getLogger(__name__)


def run_sam3(cfg: DictConfig) -> None:
    """Launch the realtime RealSense + SAM3 prompt-and-segment loop."""
    stream = RealSenseStream(cfg.realsense)
    predictor = Sam3RealtimePredictor(cfg.sam3)
    visualizer = OpenCVPromptVisualizer(
        cfg.visualizer,
        initial_text_prompt=cfg.initial_text_prompt,
    )

    _print_usage_banner(cfg)

    frame_times: deque[float] = deque(maxlen=30)
    stream.start()
    try:
        while True:
            frame = stream.get_frame()
            now = time.perf_counter()
            frame_times.append(now)
            stream_fps = _estimate_fps(frame_times)

            predictions = []
            errors = []
            active_prompts = visualizer.iter_active_prompts()
            shared_state = None
            if active_prompts:
                try:
                    shared_state = predictor.prepare_frame_state(frame.color_rgb)
                except Exception as exc:  # pragma: no cover - runtime feedback path
                    LOGGER.exception("SAM3 image encoding failed")
                    errors.append(f"image-encode: {exc}")

            for instance_index, prompt_state in visualizer.iter_active_prompts():
                try:
                    predictions.append(
                        predictor.predict(
                            frame.color_rgb,
                            prompt_state,
                            instance_index=instance_index,
                            base_state=shared_state,
                        )
                    )
                except Exception as exc:  # pragma: no cover - runtime feedback path
                    LOGGER.exception("SAM3 inference failed for %s", prompt_state.name)
                    errors.append(f"{prompt_state.name}: {exc}")

            visualizer.set_status_message(" | ".join(errors) if errors else None)
            image = visualizer.render(frame, predictions, stream_fps)
            should_quit = visualizer.handle_key(visualizer.show(image))
            if should_quit:
                break
    except KeyboardInterrupt:  # pragma: no cover - interactive shutdown path
        LOGGER.info("Stopping realtime SAM3 RealSense app")
    finally:
        stream.stop()
        visualizer.close()


def _estimate_fps(frame_times: deque[float]) -> float:
    if len(frame_times) < 2:
        return 0.0
    elapsed = frame_times[-1] - frame_times[0]
    if elapsed <= 0.0:
        return 0.0
    return (len(frame_times) - 1) / elapsed


def _print_usage_banner(cfg: DictConfig) -> None:
    amp_status = "ON" if cfg.sam3.enable_autocast and cfg.sam3.device != "cpu" else "OFF/CPU"
    print(
        "\n"
        "=== RealSense + SAM3 사용법 ===\n"
        f"- Depth 표시: {'좌우 hconcat' if cfg.visualizer.depth_hconcat else '오버레이/미리보기'}\n"
        f"- 초기 창 크기: {cfg.visualizer.initial_window_width} x {cfg.visualizer.initial_window_height}\n"
        f"- AMP autocast: {amp_status} (dtype={cfg.sam3.autocast_dtype})\n"
        "- 마우스:\n"
        "  좌클릭: point 추가\n"
        "  드래그: box 추가 (box 모드일 때)\n"
        "- 키보드:\n"
        "  p / n / b : positive point / negative point / box 모드\n"
        "  t         : active instance의 text prompt 편집\n"
        "  i         : 새 instance 추가\n"
        "  d         : active instance 삭제\n"
        "  [ / ]     : active instance 전환\n"
        "  c         : active instance의 point/box만 초기화\n"
        "  x         : active instance의 모든 prompt 초기화\n"
        "  q         : 종료\n"
        "- 참고:\n"
        "  프레임당 image encoding은 1회만 수행하고, 여러 instance가 공유합니다.\n"
        "  여러 instance를 추가해도 예전보다 추론 중복이 줄어듭니다.\n"
    )
