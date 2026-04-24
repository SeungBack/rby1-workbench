"""OpenCV visualizer for live SAM3 prompting and overlays."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from omegaconf import DictConfig
from rby1_workbench.perception.realsense import RealSenseFrame
from rby1_workbench.perception.sam3 import (
    PromptBox,
    PromptPoint,
    Sam3Prediction,
    Sam3PromptState,
)


@dataclass(slots=True)
class _OverlayColors:
    positive_point: tuple[int, int, int] = (0, 220, 0)
    negative_point: tuple[int, int, int] = (0, 0, 220)
    preview_box: tuple[int, int, int] = (220, 220, 0)
    instance_palette: tuple[tuple[int, int, int], ...] = (
        (255, 140, 0),
        (30, 144, 255),
        (50, 205, 50),
        (199, 21, 133),
        (255, 215, 0),
        (0, 206, 209),
        (255, 99, 71),
        (154, 205, 50),
    )


class OpenCVPromptVisualizer:
    """Owns the OpenCV window, multi-instance prompt state, and overlay rendering."""

    def __init__(
        self,
        config: DictConfig,
        initial_text_prompt: str = "",
    ):
        self.config = config
        self._colors = _OverlayColors()
        self._cv2 = self._import_cv2()
        self._mode = "positive_point"
        self._editing_text = False
        self._text_buffer = initial_text_prompt.strip()
        self._drag_start: tuple[int, int] | None = None
        self._preview_box: PromptBox | None = None
        self._status_message: str | None = None
        self._window_initialized = False

        self._instances: list[Sam3PromptState] = [
            Sam3PromptState(name="instance-1", text=initial_text_prompt.strip())
        ]
        self._active_index = 0

        self._cv2.namedWindow(self.config.window_name, self._cv2.WINDOW_NORMAL)
        self._cv2.setMouseCallback(self.config.window_name, self._on_mouse)
        self._text_line_type = self._cv2.LINE_8

    @staticmethod
    def _import_cv2():
        try:
            import cv2
        except ImportError as exc:
            raise ImportError(
                "opencv-python is required for visualization. "
                "Install it in the active environment before running this app."
            ) from exc
        return cv2

    @property
    def prompt_state(self) -> Sam3PromptState:
        return self._instances[self._active_index]

    @property
    def instances(self) -> list[Sam3PromptState]:
        return self._instances

    def iter_active_prompts(self) -> list[tuple[int, Sam3PromptState]]:
        return [
            (index, instance)
            for index, instance in enumerate(self._instances)
            if instance.has_any_prompt()
        ]

    def close(self) -> None:
        self._cv2.destroyWindow(self.config.window_name)

    def set_status_message(self, message: str | None) -> None:
        self._status_message = message

    def show(self, image: np.ndarray) -> int:
        if not self._window_initialized:
            self._cv2.resizeWindow(
                self.config.window_name,
                self.config.initial_window_width,
                self.config.initial_window_height,
            )
            self._window_initialized = True
        self._cv2.imshow(self.config.window_name, image)
        return self._cv2.waitKey(self.config.wait_key_ms) & 0xFF

    def handle_key(self, key: int) -> bool:
        """Handle an OpenCV key code. Returns True when the app should exit."""
        if key == 255:
            return False

        if self._editing_text:
            return self._handle_text_edit_key(key)

        if key == ord("q"):
            return True
        if key == ord("p"):
            self._mode = "positive_point"
        elif key == ord("n"):
            self._mode = "negative_point"
        elif key == ord("b"):
            self._mode = "box"
        elif key == ord("t"):
            self._editing_text = True
            self._text_buffer = self.prompt_state.text
        elif key == ord("c"):
            self.prompt_state.clear_geometry()
            self._preview_box = None
        elif key == ord("x"):
            self.prompt_state.clear_all()
            self._preview_box = None
        elif key in (ord("i"), ord("="), ord("+")):
            self._add_instance()
        elif key in (ord("d"), ord("-"), 8, 127):
            self._delete_active_instance()
        elif key in (ord("["), ord(",")):
            self._active_index = (self._active_index - 1) % len(self._instances)
            self._preview_box = None
        elif key in (ord("]"), ord(".")):
            self._active_index = (self._active_index + 1) % len(self._instances)
            self._preview_box = None
        return False

    def render(
        self,
        frame: RealSenseFrame,
        predictions: list[Sam3Prediction],
        stream_fps: float,
    ) -> np.ndarray:
        color_canvas = frame.color_bgr.copy()

        if predictions:
            color_canvas = self._draw_predictions(color_canvas, predictions)

        self._draw_prompt_overlays(color_canvas)

        if self.config.show_depth and frame.depth is not None and self.config.depth_hconcat:
            depth_canvas = self._build_depth_canvas(frame.depth, color_canvas.shape[0], color_canvas.shape[1])
            canvas = np.hstack([color_canvas, depth_canvas])
        else:
            canvas = color_canvas
            if self.config.show_depth and frame.depth is not None:
                self._draw_depth_preview(canvas, frame.depth)

        self._draw_hud(canvas, predictions, stream_fps)
        return canvas

    def _draw_predictions(
        self,
        canvas: np.ndarray,
        predictions: list[Sam3Prediction],
    ) -> np.ndarray:
        overlay = canvas.copy()
        alpha = float(np.clip(self.config.mask_alpha, 0.0, 1.0))
        has_mask = False

        for prediction in predictions:
            color = np.asarray(self._instance_color(prediction.instance_index), dtype=np.uint8)
            for mask in prediction.masks:
                overlay[mask] = color
                has_mask = True

        result = (
            self._cv2.addWeighted(overlay, alpha, canvas, 1.0 - alpha, 0)
            if has_mask
            else canvas
        )
        for prediction in predictions:
            color = self._instance_color(prediction.instance_index)
            for box, score in zip(prediction.boxes_xyxy, prediction.scores):
                x0, y0, x1, y1 = [int(round(value)) for value in box]
                self._cv2.rectangle(result, (x0, y0), (x1, y1), color, self.config.box_thickness)
                self._cv2.putText(
                    result,
                    f"{prediction.instance_name} {score:.2f}",
                    (x0, max(16, y0 - 6)),
                    self._cv2.FONT_HERSHEY_SIMPLEX,
                    self.config.font_scale,
                    color,
                    1,
                    self._text_line_type,
                )
        return result

    def _draw_prompt_overlays(self, canvas: np.ndarray) -> None:
        for index, instance in enumerate(self._instances):
            color = self._instance_color(index)
            thickness = self.config.box_thickness + 1 if index == self._active_index else 1

            for point in instance.points:
                center = (point.x, point.y)
                fill_color = (
                    self._colors.positive_point if point.label > 0 else self._colors.negative_point
                )
                self._cv2.circle(canvas, center, self.config.point_radius, fill_color, -1)
                self._cv2.circle(canvas, center, self.config.point_radius + 2, color, thickness)

            if instance.box is not None:
                box = instance.box
                self._cv2.rectangle(
                    canvas,
                    (box.x0, box.y0),
                    (box.x1, box.y1),
                    color,
                    thickness,
                )
                self._cv2.putText(
                    canvas,
                    instance.name,
                    (box.x0, max(16, box.y0 - 6)),
                    self._cv2.FONT_HERSHEY_SIMPLEX,
                    self.config.font_scale,
                    color,
                    1,
                    self._text_line_type,
                )

        if self._preview_box is not None:
            box = self._preview_box
            self._cv2.rectangle(
                canvas,
                (box.x0, box.y0),
                (box.x1, box.y1),
                self._colors.preview_box,
                1,
            )

    def _build_depth_canvas(self, depth: np.ndarray, target_height: int, target_width: int) -> np.ndarray:
        depth_vis = depth.astype(np.float32)
        if np.max(depth_vis) > 0:
            depth_vis = depth_vis / np.max(depth_vis)
        depth_vis = (depth_vis * 255).astype(np.uint8)
        depth_vis = self._cv2.applyColorMap(depth_vis, self._cv2.COLORMAP_JET)
        depth_vis = self._cv2.resize(depth_vis, (target_width, target_height))
        self._cv2.putText(
            depth_vis,
            "depth",
            (12, 24),
            self._cv2.FONT_HERSHEY_SIMPLEX,
            self.config.font_scale,
            (255, 255, 255),
            1,
            self._text_line_type,
        )
        return depth_vis

    def _draw_depth_preview(self, canvas: np.ndarray, depth: np.ndarray) -> None:
        preview_size = self.config.depth_preview_size
        if preview_size <= 0:
            return

        depth_vis = self._build_depth_canvas(depth, preview_size, preview_size)
        h, w = canvas.shape[:2]
        x0 = max(0, w - preview_size - 12)
        y0 = 12
        x1 = x0 + preview_size
        y1 = y0 + preview_size
        canvas[y0:y1, x0:x1] = depth_vis
        self._cv2.rectangle(canvas, (x0, y0), (x1, y1), (255, 255, 255), 1)

    def _draw_hud(
        self,
        canvas: np.ndarray,
        predictions: list[Sam3Prediction],
        stream_fps: float,
    ) -> None:
        if not self.config.show_help:
            return

        active = self.prompt_state
        if self._editing_text:
            text_line = f"Text edit: {self._text_buffer}_"
        elif active.text:
            text_line = f"Text: {active.text}"
        else:
            text_line = "Text: <empty>"

        prediction_summary = (
            " | ".join(
                f"{prediction.instance_name}:{len(prediction.masks)}"
                for prediction in predictions
            )
            if predictions
            else "waiting for prompt"
        )

        lines = [
            f"Mode: {self._mode}",
            f"Active: {active.name} ({self._active_index + 1}/{len(self._instances)})",
            text_line,
            f"Routing: {self._resolve_routing(active)}",
            f"Points: {len(active.points)} | Box: {'yes' if active.box else 'no'}",
            f"Instances: {', '.join(instance.name for instance in self._instances)}",
            f"Predictions: {prediction_summary}",
            f"FPS: {stream_fps:.1f}",
            "Keys: p/n/b prompt mode | t text | i add inst | d delete inst | [/] switch inst",
            "Keys: c clear geom | x clear active | q quit",
        ]
        if self._status_message:
            lines.append(f"Status: {self._status_message}")

        top = 24
        left = 12
        panel_height = max(36, self.config.line_height * len(lines) + 10)
        panel_width = max(640, min(canvas.shape[1] - 24, int(canvas.shape[1] * 0.6)))
        self._cv2.rectangle(
            canvas,
            (left - 8, top - 18),
            (left + panel_width, top - 18 + panel_height),
            (0, 0, 0),
            -1,
        )

        for index, line in enumerate(lines):
            y = top + index * self.config.line_height
            self._cv2.putText(
                canvas,
                line,
                (left, y),
                self._cv2.FONT_HERSHEY_SIMPLEX,
                self.config.font_scale,
                (255, 255, 255),
                1,
                self._text_line_type,
            )

    def _handle_text_edit_key(self, key: int) -> bool:
        if key in (13, 10):
            self.prompt_state.text = self._text_buffer.strip()
            self._editing_text = False
            return False
        if key == 27:
            self._editing_text = False
            self._text_buffer = self.prompt_state.text
            return False
        if key in (8, 127):
            self._text_buffer = self._text_buffer[:-1]
            return False
        if key == ord("q"):
            return True
        if key == ord(" "):
            self._text_buffer += " "
            return False
        if 32 <= key <= 126:
            self._text_buffer += chr(key)
        return False

    def _resolve_routing(self, prompt_state: Sam3PromptState) -> str:
        if prompt_state.has_geometry():
            if prompt_state.has_text():
                return "geometry + text label"
            return "geometry"
        if prompt_state.has_text():
            return "text"
        return "none"

    def _add_instance(self) -> None:
        next_index = len(self._instances) + 1
        self._instances.append(Sam3PromptState(name=f"instance-{next_index}"))
        self._active_index = len(self._instances) - 1
        self._text_buffer = self.prompt_state.text
        self._preview_box = None

    def _delete_active_instance(self) -> None:
        if len(self._instances) == 1:
            self.prompt_state.clear_all()
            self._preview_box = None
            return
        self._instances.pop(self._active_index)
        self._active_index = min(self._active_index, len(self._instances) - 1)
        self._text_buffer = self.prompt_state.text
        self._preview_box = None

    def _instance_color(self, index: int) -> tuple[int, int, int]:
        return self._colors.instance_palette[index % len(self._colors.instance_palette)]

    def _on_mouse(self, event: int, x: int, y: int, _flags: int, _userdata=None) -> None:
        if self._editing_text:
            return

        if self._mode == "box":
            self._handle_box_mouse(event, x, y)
            return

        if event == self._cv2.EVENT_LBUTTONDOWN:
            label = 1 if self._mode == "positive_point" else 0
            self.prompt_state.points.append(PromptPoint(x=x, y=y, label=label))

    def _handle_box_mouse(self, event: int, x: int, y: int) -> None:
        if event == self._cv2.EVENT_LBUTTONDOWN:
            self._drag_start = (x, y)
            self._preview_box = PromptBox(x, y, x, y)
        elif event == self._cv2.EVENT_MOUSEMOVE and self._drag_start is not None:
            x0, y0 = self._drag_start
            self._preview_box = PromptBox(x0, y0, x, y)
        elif event == self._cv2.EVENT_LBUTTONUP and self._drag_start is not None:
            x0, y0 = self._drag_start
            x_min, x_max = sorted((x0, x))
            y_min, y_max = sorted((y0, y))
            if x_max > x_min and y_max > y_min:
                self.prompt_state.box = PromptBox(x_min, y_min, x_max, y_max)
            self._drag_start = None
            self._preview_box = None
