"""SharedMemoryCameraStream — RealSenseStream과 동일한 API.

rby1-camera-server (apps/camera_server.py) 가 실행 중인 상태에서 사용한다.
카메라 이름(name)으로 접근하며 serial number는 몰라도 된다.

Usage:
    cam = SharedMemoryCameraStream("head")
    cam.start()
    K, D = cam.get_intrinsics()
    frame = cam.get_frame()   # RealSenseFrame 반환
    cam.stop()
"""

from __future__ import annotations

import json
import logging
import struct
import threading
from multiprocessing import resource_tracker
from multiprocessing.shared_memory import SharedMemory


def _open_shm(name: str) -> SharedMemory:
    """기존 SHM에 연결하되, resource tracker에서 제외해 프로세스 종료 시 unlink 방지."""
    shm = SharedMemory(name=name, create=False)
    resource_tracker.unregister(shm._name, "shared_memory")
    return shm

import numpy as np

from rby1_workbench.perception.realsense import RealSenseFrame

log = logging.getLogger(__name__)

_META_FMT  = "=QIIdddddddddd?"
_META_SIZE = 128


class SharedMemoryCameraStream:
    """camera_server.py shared memory에서 프레임을 읽는 스트림 클라이언트."""

    def __init__(
        self,
        name: str,
        address: str = "tcp://127.0.0.1:5550",
        prefix: str  = "rby1",
    ):
        self._name    = name
        self._address = address
        self._prefix  = prefix

        self._frame:    RealSenseFrame | None = None
        self._lock      = threading.Lock()
        self._new_frame = threading.Event()
        self._running   = False
        self._thread:   threading.Thread | None = None

        self._shm_color: SharedMemory | None = None
        self._shm_depth: SharedMemory | None = None
        self._shm_meta:  SharedMemory | None = None
        self._w = 0
        self._h = 0

    # ------------------------------------------------------------------
    # Public API (RealSenseStream 호환)
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Shared memory에 연결하고 수신 스레드를 시작한다.

        camera_server.py가 먼저 실행돼 있어야 한다.
        """
        name, prefix = self._name, self._prefix
        try:
            self._shm_color = _open_shm(f"{prefix}_{name}_color")
            self._shm_meta  = _open_shm(f"{prefix}_{name}_meta")
        except FileNotFoundError:
            raise RuntimeError(
                f"Camera '{name}' shared memory not found. "
                "Is camera_server.py running?"
            )
        try:
            self._shm_depth = _open_shm(f"{prefix}_{name}_depth")
        except FileNotFoundError:
            self._shm_depth = None

        # meta에서 해상도 캐싱 (서버가 이미 기록함)
        _, w, h, *_ = struct.unpack_from(_META_FMT, bytes(self._shm_meta.buf))
        self._w, self._h = int(w), int(h)

        self._running = True
        self._thread  = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        if not self._new_frame.wait(timeout=5.0):
            raise RuntimeError(
                f"Camera server did not send frames for '{name}' within 5 s."
            )

    def stop(self) -> None:
        self._running = False
        for shm in (self._shm_color, self._shm_depth, self._shm_meta):
            if shm:
                shm.close()

    def get_intrinsics(self) -> tuple[np.ndarray, np.ndarray]:
        """(K 3×3, D 5) — start() 이후에만 유효."""
        if self._shm_meta is None:
            raise RuntimeError("start() must be called first.")
        _, _, _, fx, fy, cx, cy, _, k1, k2, p1, p2, k3, _ = struct.unpack_from(
            _META_FMT, bytes(self._shm_meta.buf)
        )
        K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        D = np.array([k1, k2, p1, p2, k3], dtype=np.float64)
        return K, D

    @property
    def depth_scale(self) -> float:
        if self._shm_meta is None:
            return 1e-3
        _, _, _, _, _, _, _, depth_scale, *_ = struct.unpack_from(
            _META_FMT, bytes(self._shm_meta.buf)
        )
        return float(depth_scale)

    def get_frame(self) -> RealSenseFrame:
        """가장 최신 프레임을 반환한다. 첫 프레임 도착까지 블록."""
        self._new_frame.wait()
        with self._lock:
            assert self._frame is not None
            return self._frame

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _recv_loop(self) -> None:
        import zmq
        ctx  = zmq.Context.instance()
        sock = ctx.socket(zmq.SUB)
        sock.connect(self._address)
        sock.setsockopt_string(zmq.SUBSCRIBE, self._name)

        while self._running:
            if not sock.poll(200):   # 200 ms timeout → running 체크
                continue
            _, data = sock.recv_multipart()
            msg     = json.loads(data)
            frame   = self._read_frame(msg)
            with self._lock:
                self._frame = frame
            self._new_frame.set()

        sock.close()

    # context manager support
    def __enter__(self) -> "SharedMemoryCameraStream":
        self.start()
        return self

    def __exit__(self, *_) -> None:
        self.stop()

    def _read_frame(self, msg: dict) -> RealSenseFrame:
        w, h = self._w, self._h
        color_bgr = np.ndarray((h, w, 3), dtype=np.uint8, buffer=self._shm_color.buf).copy()
        color_rgb = color_bgr[:, :, ::-1].copy()

        depth = None
        if self._shm_depth and msg.get("has_depth"):
            depth = np.ndarray((h, w), dtype=np.uint16, buffer=self._shm_depth.buf).copy()

        return RealSenseFrame(
            color_bgr    = color_bgr,
            color_rgb    = color_rgb,
            depth        = depth,
            timestamp_ms = float(msg.get("timestamp_ms", 0.0)),
            frame_index  = int(msg.get("seq", 0)),
        )


def create_camera_stream(cfg):
    """cfg.camera_source 설정에 따라 스트림을 생성한다.

    camera_source.mode == "direct" → RealSenseStream(cfg.realsense)
    camera_source.mode == "server" → SharedMemoryCameraStream(name, ...)

    Args:
        cfg: camera_source 섹션을 포함하는 DictConfig.
    """
    source = cfg.camera_source
    if source.mode == "direct":
        from rby1_workbench.perception.realsense import RealSenseStream
        return RealSenseStream(cfg.realsense)
    return SharedMemoryCameraStream(
        name    = source.name,
        address = getattr(source, "address", "tcp://127.0.0.1:5550"),
        prefix  = getattr(source, "prefix",  "rby1"),
    )
