"""SAM3 wrapper for realtime text and interactive segmentation.

This wrapper follows the official split between:
- image grounding via ``build_sam3_image_model`` + ``Sam3Processor``
- SAM1-style interactive prompting via ``predict_inst``

Official SAM 3.1 release materials add a multiplex *video* predictor path.
This realtime app is built on the image predictor path, so it expects the
standard image checkpoint flow (`sam3.pt` / default HF download), not the
`sam3.1_multiplex.pt` video checkpoint.
"""

from __future__ import annotations

from contextlib import nullcontext
from dataclasses import dataclass, field
import logging
from pathlib import Path
import time
import warnings

import numpy as np
from PIL import Image

from omegaconf import DictConfig


LOGGER = logging.getLogger(__name__)


@dataclass(slots=True)
class PromptPoint:
    x: int
    y: int
    label: int = 1


@dataclass(slots=True)
class PromptBox:
    x0: int
    y0: int
    x1: int
    y1: int

    def as_xyxy(self) -> np.ndarray:
        x0, x1 = sorted((self.x0, self.x1))
        y0, y1 = sorted((self.y0, self.y1))
        return np.asarray([x0, y0, x1, y1], dtype=np.float32)


@dataclass(slots=True)
class Sam3PromptState:
    """Runtime prompt state shared between the visualizer and predictor."""

    name: str = "instance-1"
    text: str = ""
    points: list[PromptPoint] = field(default_factory=list)
    box: PromptBox | None = None

    def has_text(self) -> bool:
        return bool(self.text.strip())

    def has_geometry(self) -> bool:
        return bool(self.points) or self.box is not None

    def has_any_prompt(self) -> bool:
        return self.has_text() or self.has_geometry()

    def clear_geometry(self) -> None:
        self.points.clear()
        self.box = None

    def clear_all(self) -> None:
        self.text = ""
        self.clear_geometry()


@dataclass(slots=True)
class Sam3Prediction:
    """Normalized prediction payload for rendering."""

    instance_name: str
    instance_index: int
    source: str
    masks: np.ndarray
    scores: np.ndarray
    boxes_xyxy: np.ndarray
    latency_ms: float
    text: str | None = None


class Sam3RealtimePredictor:
    """Wrap SAM3 grounding and interactive segmentation for live images."""

    def __init__(self, config: DictConfig):
        self.config = config
        self._torch = self._import_torch()
        build_sam3_image_model, Sam3Processor = self._import_sam3()

        self.device = self._select_device(config.device)
        self._configure_cuda()
        checkpoint_path = self._resolve_checkpoint_path(config.checkpoint_path)

        self.model = self._build_model(
            build_sam3_image_model=build_sam3_image_model,
            checkpoint_path=checkpoint_path,
        )
        self.processor = Sam3Processor(
            self.model,
            resolution=config.resolution,
            device=self.device,
            confidence_threshold=config.confidence_threshold,
        )

    @staticmethod
    def _import_torch():
        try:
            import torch
        except ImportError as exc:
            raise ImportError(
                "PyTorch is required for SAM3 inference. "
                "Install torch in the active environment before running this app."
            ) from exc
        return torch

    @staticmethod
    def _import_sam3():
        try:
            with warnings.catch_warnings():
                warnings.filterwarnings(
                    "ignore",
                    message="Importing from timm.models.layers is deprecated, please import via timm.layers",
                    category=FutureWarning,
                )
                from sam3.model_builder import build_sam3_image_model
                from sam3.model.sam3_image_processor import Sam3Processor
        except ImportError as exc:
            raise ImportError(
                "The local 'sam3' package is required. "
                "Install the sibling sam3 repo with `pip install -e /home/kimm/Workspaces/sam3`."
            ) from exc
        return build_sam3_image_model, Sam3Processor

    def _build_model(self, build_sam3_image_model, checkpoint_path: str):
        with warnings.catch_warnings():
            warnings.filterwarnings(
                "ignore",
                message="Importing from timm.models.layers is deprecated, please import via timm.layers",
                category=FutureWarning,
            )
            model = build_sam3_image_model(
                checkpoint_path=checkpoint_path,
                device=self.device,
                eval_mode=True,
                load_from_HF=checkpoint_path is None,
                enable_inst_interactivity=True,
                compile=self.config.enable_compile,
            )
        return model

    def _select_device(self, configured_device: str) -> str:
        if configured_device != "auto":
            return configured_device
        return "cuda" if self._torch.cuda.is_available() else "cpu"

    def _configure_cuda(self) -> None:
        if self.device != "cuda":
            return
        device_props = self._torch.cuda.get_device_properties(0)
        self._torch.backends.cudnn.benchmark = True
        if device_props.major >= 8:
            self._torch.backends.cuda.matmul.allow_tf32 = True
            self._torch.backends.cudnn.allow_tf32 = True

    def _resolve_checkpoint_path(self, checkpoint_path: str | None) -> str | None:
        if checkpoint_path:
            path = Path(checkpoint_path).expanduser()
            if not path.exists():
                raise FileNotFoundError(f"SAM3 checkpoint not found: {path}")
            if path.name == "sam3.1_multiplex.pt":
                raise ValueError(
                    "This realtime app uses the official SAM3 image predictor path. "
                    "`sam3.1_multiplex.pt` is the SAM 3.1 multiplex video checkpoint, "
                    "not the supported image-interactive checkpoint for this app. "
                    "Use `sam3.pt` or leave `checkpoint_path` unset to use the default "
                    "official image checkpoint flow."
                )
            return str(path)

        workspace_default = Path(__file__).resolve().parents[4] / "sam3" / "sam3.pt"
        if workspace_default.exists():
            return str(workspace_default)

        LOGGER.info(
            "No local `sam3.pt` found at %s. Falling back to the official default HF image checkpoint flow.",
            workspace_default,
        )
        return None

    def _autocast_context(self):
        if self.device != "cuda" or not self.config.enable_autocast:
            return nullcontext()

        dtype_name = self.config.autocast_dtype.lower()
        dtype = self._torch.bfloat16 if dtype_name == "bfloat16" else self._torch.float16
        return self._torch.autocast(device_type="cuda", dtype=dtype)

    def _inference_context(self):
        return self._torch.inference_mode()

    def predict(
        self,
        image_rgb: np.ndarray,
        prompt_state: Sam3PromptState,
        *,
        instance_index: int = 0,
        base_state: dict | None = None,
    ) -> Sam3Prediction:
        """Run SAM3 on a single RGB frame."""
        if not prompt_state.has_any_prompt():
            raise ValueError("At least one prompt is required for SAM3 inference.")

        started_at = time.perf_counter()
        state = self._copy_state(base_state) if base_state is not None else self._prepare_state(image_rgb)

        if prompt_state.has_geometry():
            masks, scores, boxes = self._predict_interactive(state, prompt_state)
            source = "geometry"
        else:
            masks, scores, boxes = self._predict_text(state, prompt_state.text.strip())
            source = "text"

        latency_ms = (time.perf_counter() - started_at) * 1000.0
        return Sam3Prediction(
            instance_name=prompt_state.name,
            instance_index=instance_index,
            source=source,
            masks=masks,
            scores=scores,
            boxes_xyxy=boxes,
            latency_ms=latency_ms,
            text=prompt_state.text.strip() or None,
        )

    def _prepare_state(self, image_rgb: np.ndarray) -> dict:
        pil_image = Image.fromarray(image_rgb)
        with self._inference_context(), self._autocast_context():
            return self.processor.set_image(pil_image, state={})

    def prepare_frame_state(self, image_rgb: np.ndarray) -> dict:
        """Compute image features once for the current frame and reuse across instances."""
        return self._prepare_state(image_rgb)

    @staticmethod
    def _copy_state(base_state: dict) -> dict:
        state = dict(base_state)
        if "backbone_out" in base_state:
            state["backbone_out"] = dict(base_state["backbone_out"])
        return state

    def _to_numpy(self, value) -> np.ndarray:
        """Convert tensors or array-likes to NumPy, normalizing float dtypes first."""
        if isinstance(value, np.ndarray):
            return value

        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "is_floating_point") and value.is_floating_point():
            value = value.float()
        if hasattr(value, "cpu"):
            value = value.cpu()
        if hasattr(value, "numpy"):
            return value.numpy()
        return np.asarray(value)

    def _predict_text(self, state: dict, text_prompt: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        with self._inference_context(), self._autocast_context():
            output = self.processor.set_text_prompt(prompt=text_prompt, state=state)

        masks = self._to_numpy(output["masks"].squeeze(1)).astype(bool)
        scores = self._to_numpy(output["scores"]).astype(np.float32)
        boxes = self._to_numpy(output["boxes"]).astype(np.float32)
        return masks, scores, boxes

    def _predict_interactive(
        self,
        state: dict,
        prompt_state: Sam3PromptState,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        point_coords = None
        point_labels = None
        box = None

        if prompt_state.points:
            point_coords = np.asarray(
                [[point.x, point.y] for point in prompt_state.points],
                dtype=np.float32,
            )
            point_labels = np.asarray(
                [point.label for point in prompt_state.points],
                dtype=np.int32,
            )

        if prompt_state.box is not None:
            box = prompt_state.box.as_xyxy()

        with self._inference_context(), self._autocast_context():
            masks, scores, _ = self.model.predict_inst(
                state,
                point_coords=point_coords,
                point_labels=point_labels,
                box=box,
                multimask_output=self.config.interactive_multimask_output,
                return_logits=False,
                normalize_coords=True,
            )

        masks = np.asarray(masks).astype(bool)
        scores = np.asarray(scores, dtype=np.float32)
        keep = scores >= self.config.confidence_threshold
        if scores.size > 0 and not np.any(keep):
            keep[np.argmax(scores)] = True
        if scores.size > 0:
            masks = masks[keep]
            scores = scores[keep]

        boxes = self._boxes_from_masks(masks)
        return masks, scores, boxes

    @staticmethod
    def _boxes_from_masks(masks: np.ndarray) -> np.ndarray:
        if masks.size == 0:
            return np.zeros((0, 4), dtype=np.float32)

        boxes: list[list[float]] = []
        for mask in masks:
            ys, xs = np.where(mask)
            if xs.size == 0 or ys.size == 0:
                boxes.append([0.0, 0.0, 0.0, 0.0])
                continue
            boxes.append(
                [
                    float(xs.min()),
                    float(ys.min()),
                    float(xs.max()),
                    float(ys.max()),
                ]
            )
        return np.asarray(boxes, dtype=np.float32)
