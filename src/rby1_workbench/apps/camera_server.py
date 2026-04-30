"""Standalone RealSense camera server.

카메라를 단독으로 점유하고, 프레임을 Shared Memory에 기록한 뒤
ZMQ PUB으로 메타데이터(알림)를 발송한다.

Usage:
    python -m rby1_workbench.perception.camera_server
    python -m rby1_workbench.perception.camera_server --config my_server.yaml

Shared memory 이름 규칙:
    {prefix}_{name}_color   — uint8  (H, W, 3)  BGR
    {prefix}_{name}_depth   — uint16 (H, W)      [enable_depth=true 시]
    {prefix}_{name}_meta    — 고정 크기 struct (intrinsics, seq, ...)

ZMQ 메시지 형식 (multipart):
    frame 0: camera name (bytes)  ← topic prefix, SUB 필터에 사용
    frame 1: JSON  {"name", "seq", "timestamp_ms", "has_depth"}
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import struct
import threading
from multiprocessing.shared_memory import SharedMemory
from pathlib import Path

import numpy as np
from omegaconf import DictConfig, OmegaConf

from rby1_workbench.perception.realsense import RealSenseStream

log = logging.getLogger(__name__)

# seq(Q) w(I) h(I) fx fy cx cy depth_scale k1 k2 p1 p2 k3(10d) has_depth(?)
_META_FMT  = "=QIIdddddddddd?"
_META_SIZE = 128   # 97 bytes packed → pad to 128


def _shm_cleanup(name: str) -> None:
    """잔존 shared memory 세그먼트를 조용히 제거."""
    try:
        shm = SharedMemory(name=name, create=False, size=1)
        shm.close()
        shm.unlink()
    except FileNotFoundError:
        pass


class _CameraWorker:
    """카메라 1개를 담당하는 캡처 스레드."""

    def __init__(self, cam_cfg: DictConfig, prefix: str, sock, zmq_lock: threading.Lock):
        self._cfg      = cam_cfg
        self._name     = cam_cfg.name
        self._prefix   = prefix
        self._sock     = sock
        self._zmq_lock = zmq_lock
        self._seq      = 0
        self._running  = False
        self._thread: threading.Thread | None = None
        self._shm_color: SharedMemory | None = None
        self._shm_depth: SharedMemory | None = None
        self._shm_meta:  SharedMemory | None = None
        self._cam: RealSenseStream | None = None

    def start(self) -> None:
        name, prefix = self._name, self._prefix
        w, h = self._cfg.color_width, self._cfg.color_height

        # 이전 실행 잔존 세그먼트 제거
        for suffix in ("color", "depth", "meta"):
            _shm_cleanup(f"{prefix}_{name}_{suffix}")

        # 카메라 초기화
        self._cam = RealSenseStream(self._cfg)
        self._cam.start()
        K, D = self._cam.get_intrinsics()
        depth_scale = self._cam.depth_scale if self._cfg.enable_depth else 0.0

        # Shared memory 생성
        self._shm_color = SharedMemory(name=f"{prefix}_{name}_color", create=True, size=h * w * 3)
        if self._cfg.enable_depth:
            self._shm_depth = SharedMemory(name=f"{prefix}_{name}_depth", create=True, size=h * w * 2)
        self._shm_meta = SharedMemory(name=f"{prefix}_{name}_meta", create=True, size=_META_SIZE)

        # intrinsics를 meta에 기록 (seq=0, 캡처 전)
        d = np.zeros(5, dtype=np.float64)
        d[:len(D)] = D[:5]
        meta = struct.pack(_META_FMT, 0, w, h,
                           K[0, 0], K[1, 1], K[0, 2], K[1, 2],
                           depth_scale, d[0], d[1], d[2], d[3], d[4],
                           self._cfg.enable_depth)
        self._shm_meta.buf[:len(meta)] = meta

        self._running = True
        self._thread = threading.Thread(target=self._loop, name=f"cam-{name}", daemon=True)
        self._thread.start()
        log.info("Camera '%s' started  (SHM prefix: %s_%s_*)", name, prefix, name)

    def _loop(self) -> None:
        name = self._name
        w, h = self._cfg.color_width, self._cfg.color_height
        color_arr = np.ndarray((h, w, 3), dtype=np.uint8,  buffer=self._shm_color.buf)
        depth_arr = (
            np.ndarray((h, w), dtype=np.uint16, buffer=self._shm_depth.buf)
            if self._shm_depth else None
        )

        while self._running:
            try:
                frame = self._cam.get_frame()
            except Exception as e:
                log.error("Camera '%s' frame error: %s", name, e)
                continue

            self._seq += 1
            color_arr[:] = frame.color_bgr
            has_depth = depth_arr is not None and frame.depth is not None
            if has_depth:
                depth_arr[:] = frame.depth

            # meta의 seq(첫 8바이트)만 갱신
            struct.pack_into("=Q", self._shm_meta.buf, 0, self._seq)

            msg = json.dumps({
                "name":         name,
                "seq":          self._seq,
                "timestamp_ms": frame.timestamp_ms,
                "has_depth":    has_depth,
            }).encode()

            with self._zmq_lock:
                self._sock.send_multipart([name.encode(), msg])

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cam:
            self._cam.stop()
        for shm in (self._shm_color, self._shm_depth, self._shm_meta):
            if shm:
                try:
                    shm.close()
                    shm.unlink()
                except Exception:
                    pass


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-8s %(message)s")

    parser = argparse.ArgumentParser(description="RealSense camera server")
    parser.add_argument("--config", default=None, help="YAML override")
    args = parser.parse_args()

    conf_path = Path(__file__).parents[1] / "conf" / "camera_server.yaml"
    cfg = OmegaConf.load(conf_path)
    if args.config:
        cfg = OmegaConf.merge(cfg, OmegaConf.load(args.config))

    import zmq
    ctx  = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind(cfg.zmq_pub_address)
    zmq_lock = threading.Lock()

    workers = [
        _CameraWorker(cam_cfg, cfg.shm_prefix, sock, zmq_lock)
        for cam_cfg in cfg.cameras
    ]

    stop_event = threading.Event()

    def _handle_signal(sig, frame):
        log.info("Shutting down...")
        stop_event.set()

    signal.signal(signal.SIGINT,  _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    for w in workers:
        w.start()

    log.info("Camera server ready on %s  (cameras: %s)",
             cfg.zmq_pub_address, [c.name for c in cfg.cameras])

    stop_event.wait()

    for w in workers:
        w.stop()
    sock.close()
    ctx.term()


cli = main

if __name__ == "__main__":
    main()
