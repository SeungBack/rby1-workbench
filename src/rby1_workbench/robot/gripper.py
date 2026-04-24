"""Gripper wrapper (robot.gripper 프로퍼티로 접근).

rby1-sdk DynamixelBus 기반 그리퍼. vr-teleop/python/gripper.py 로직을 통합.
TCP 통신 지원:
  - TCPGripperServer: 로봇 쪽에서 실행, TCP 수신 → 로컬 그리퍼 구동
  - GripperTCPClient: 원격 쪽에서 실행, (right, left) float TCP 전송
Inspire Hand(InspireHandCAN)는 InspireGripperController로 별도 제공.
"""

from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from typing import Any

import numpy as np
import rby1_sdk as rby

from rby1_workbench.config.schema import GripperConfig

log = logging.getLogger(__name__)


class GripperController:
    """RBY1 기본 그리퍼 (Dynamixel-based, vr-teleop gripper.py 통합).

    사용 예::

        robot.gripper.setup()      # tool flange 전원 인가 + initialize + homing
        robot.gripper.start()      # 제어 루프 시작 (background thread)

        robot.gripper.open()
        robot.gripper.close()
        robot.gripper.set_normalized(right=0.5, left=0.0)  # 0=열림, 1=닫힘

        robot.gripper.stop()
    """

    _DEVICE_IDS = [0, 1]   # [right, left]
    _GRIPPER_DIRECTION = False

    def __init__(self, sdk_robot: Any, cfg: GripperConfig):
        self._robot = sdk_robot
        self._cfg = cfg
        self._bus: Any = None
        self._min_q = np.array([np.inf, np.inf])
        self._max_q = np.array([-np.inf, -np.inf])
        self._target_q: np.ndarray | None = None
        self._running = False
        self._thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def setup(self, verbose: bool = False) -> bool:
        """tool flange 전원 인가 → DynamixelBus 초기화 → homing.

        vr-teleop main.py L257-266 + gripper.py 패턴.
        """
        for arm in ["right", "left"]:
            if not self._robot.set_tool_flange_output_voltage(
                arm, self._cfg.tool_flange_voltage
            ):
                log.error("Failed to supply %dV to tool flange (%s)", self._cfg.tool_flange_voltage, arm)
                return False
        log.info("Tool flange voltage: %dV", self._cfg.tool_flange_voltage)
        time.sleep(0.5)

        self._bus = rby.DynamixelBus(rby.upc.GripperDeviceName)
        self._bus.open_port()
        self._bus.set_baud_rate(2_000_000)
        self._bus.set_torque_constant([1, 1])

        # ping check
        for dev_id in self._DEVICE_IDS:
            if not self._bus.ping(dev_id):
                log.error("Gripper Dynamixel ID %d not active", dev_id)
                return False
            if verbose:
                log.info("Gripper Dynamixel ID %d active", dev_id)

        self._bus.group_sync_write_torque_enable(
            [(dev_id, 1) for dev_id in self._DEVICE_IDS]
        )
        log.info("Gripper initialized")

        log.info("Gripper homing...")
        self._homing()
        return True

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def start(self) -> None:
        """그리퍼 제어 루프 시작 (background thread)."""
        if self._bus is None:
            raise RuntimeError("Call setup() before start()")
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        # 초기 target: 열린 상태
        self._target_q = self._min_q.copy()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        log.info("Gripper control loop started")

    def stop(self) -> None:
        """그리퍼 제어 루프 정지."""
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def set_normalized(
        self,
        right: float | None = None,
        left: float | None = None,
    ) -> None:
        """normalized [0=열림, 1=닫힘] 값으로 그리퍼 제어.

        None인 쪽은 현재 target 유지.
        """
        if self._target_q is None:
            self._target_q = self._min_q.copy()

        current = self._get_normalized_target()
        if right is not None:
            current[0] = float(np.clip(right, 0.0, 1.0))
        if left is not None:
            current[1] = float(np.clip(left, 0.0, 1.0))
        self._set_normalized_target(current)

    def open(self) -> None:
        """양 그리퍼 열기."""
        self.set_normalized(right=0.0, left=0.0)

    def close(self) -> None:
        """양 그리퍼 닫기."""
        self.set_normalized(right=1.0, left=1.0)

    def get_normalized(self) -> np.ndarray:
        """현재 target [right, left] normalized 값."""
        return self._get_normalized_target()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _homing(self) -> None:
        """min/max 엔코더 범위 탐색 (vr-teleop gripper.py homing 로직)."""
        self._bus.group_sync_write_torque_enable(
            [(dev_id, 0) for dev_id in self._DEVICE_IDS]
        )
        self._bus.group_sync_write_operating_mode(
            [(dev_id, rby.DynamixelBus.CurrentControlMode) for dev_id in self._DEVICE_IDS]
        )
        self._bus.group_sync_write_torque_enable(
            [(dev_id, 1) for dev_id in self._DEVICE_IDS]
        )

        direction = 0
        q = np.zeros(2)
        prev_q = np.zeros(2)
        counter = 0
        while direction < 2:
            torque_dir = 0.5 if direction == 0 else -0.5
            self._bus.group_sync_write_send_torque(
                [(dev_id, torque_dir) for dev_id in self._DEVICE_IDS]
            )
            rv = self._bus.group_fast_sync_read_encoder(self._DEVICE_IDS)
            if rv is not None:
                for dev_id, enc in rv:
                    q[dev_id] = enc
            self._min_q = np.minimum(self._min_q, q)
            self._max_q = np.maximum(self._max_q, q)
            if np.array_equal(prev_q, q):
                counter += 1
            prev_q = q.copy()
            if counter >= 30:
                direction += 1
                counter = 0
            time.sleep(0.1)

        log.info("Gripper homing done. min=%s, max=%s", self._min_q, self._max_q)

    def _loop(self) -> None:
        """Position control loop (vr-teleop gripper.py loop 로직)."""
        self._bus.group_sync_write_torque_enable(
            [(dev_id, 0) for dev_id in self._DEVICE_IDS]
        )
        self._bus.group_sync_write_operating_mode(
            [(dev_id, rby.DynamixelBus.CurrentBasedPositionControlMode) for dev_id in self._DEVICE_IDS]
        )
        self._bus.group_sync_write_torque_enable(
            [(dev_id, 1) for dev_id in self._DEVICE_IDS]
        )
        self._bus.group_sync_write_send_torque(
            [(dev_id, 5) for dev_id in self._DEVICE_IDS]
        )
        while self._running:
            if self._target_q is not None:
                self._bus.group_sync_write_send_position(
                    [(dev_id, float(q)) for dev_id, q in enumerate(self._target_q.tolist())]
                )
            time.sleep(0.1)

    def _get_normalized_target(self) -> np.ndarray:
        if self._target_q is None:
            return np.zeros(2)
        rng = self._max_q - self._min_q
        if not np.isfinite(rng).all() or np.any(rng == 0):
            return np.zeros(2)
        norm = (self._target_q - self._min_q) / rng
        if not self._GRIPPER_DIRECTION:
            norm = 1.0 - norm
        return norm.copy()

    def _set_normalized_target(self, normalized: np.ndarray) -> None:
        if not np.isfinite(self._min_q).all() or not np.isfinite(self._max_q).all():
            log.warning("Gripper min/max not valid yet. Run homing first.")
            return
        if self._GRIPPER_DIRECTION:
            self._target_q = normalized * (self._max_q - self._min_q) + self._min_q
        else:
            self._target_q = (1.0 - normalized) * (self._max_q - self._min_q) + self._min_q


# ---------------------------------------------------------------------------
# Inspire Hand wrapper
# ---------------------------------------------------------------------------

class InspireGripperController:
    """Inspire Dexterous Hand CAN wrapper.

    inspire_wrapper/hand_wrapper.py의 InspireHandCAN을 RBY1 패턴으로 감싼 클래스.
    GripperController와 인터페이스 일치.

    사용 예::

        from rby1_workbench.robot.gripper import InspireGripperController
        hand = InspireGripperController(port="/dev/ttyUSB0", hand_id="01")
        hand.setup()
        hand.open()
        hand.close()
        hand.set_normalized(right=0.5)
    """

    def __init__(self, port: str = "/dev/ttyUSB0", hand_id: str = "01"):
        try:
            import sys
            from rby1_workbench.config.schema import package_root
            wrapper_path = str(package_root().parent / "inspire_wrapper")
            if wrapper_path not in sys.path:
                sys.path.insert(0, wrapper_path)
            from hand_wrapper import InspireHandCAN
        except ImportError as e:
            raise ImportError(
                "InspireHandCAN을 import할 수 없습니다. "
                "inspire_wrapper/hand_wrapper.py 경로를 확인하세요."
            ) from e

        self._hand = InspireHandCAN(port=port, hand_id=hand_id, use_normalized=True)

    def setup(self, verbose: bool = False) -> bool:
        return self._hand.connect()

    def stop(self) -> None:
        self._hand.disconnect()

    def start(self) -> None:
        pass  # Inspire hand는 별도 루프 없음

    def open(self) -> None:
        self._hand.open_hand()

    def close(self) -> None:
        self._hand.close_hand()

    def set_normalized(
        self,
        right: float | None = None,
        left: float | None = None,
    ) -> None:
        """0=열림, 1=닫힘. 현재는 단일 hand 기준."""
        if right is not None:
            val = float(np.clip(right, 0.0, 1.0))
            self._hand.set_angle([val] * 6)

    def get_normalized(self) -> np.ndarray:
        result = self._hand.get_angle_setting()
        if result:
            return np.array([float(result[0]), 0.0])
        return np.zeros(2)


# ---------------------------------------------------------------------------
# TCP 통신 (OmniTeleop/utils/gripper_server.py + gripper_client.py 패턴)
# ---------------------------------------------------------------------------

class TCPGripperServer:
    """로봇 쪽에서 실행. TCP로 (right, left) float 수신 → GripperController 구동.

    OmniTeleop gripper_server.py의 TCPGripperServer 패턴 통합.

    사용 예::

        gripper = robot.gripper
        gripper.setup()
        gripper.start()

        server = TCPGripperServer(gripper, host="0.0.0.0", port=5000)
        server.start()
        # ... 원격 GripperTCPClient가 연결하면 자동으로 그리퍼 구동
        server.stop()

    또는 GripperConfig를 넘기면 host/port 자동 적용::

        server = TCPGripperServer(gripper, cfg=robot._cfg.gripper)
        server.start()
    """

    _PACKET_FMT = "ff"
    _PACKET_SIZE = struct.calcsize(_PACKET_FMT)  # 8 bytes

    def __init__(
        self,
        gripper: GripperController,
        host: str = "0.0.0.0",
        port: int = 5000,
        cfg: GripperConfig | None = None,
    ):
        self._gripper = gripper
        self._host = cfg.tcp_host if cfg else host
        self._port = cfg.tcp_port if cfg else port
        self._server_sock: socket.socket | None = None
        self._client_sock: socket.socket | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._latest: np.ndarray | None = None
        self._lock = threading.Lock()
        self._poll_thread: threading.Thread | None = None

    def start(self) -> None:
        """TCP 서버 + 그리퍼 폴링 루프 시작."""
        self._running = True
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        self._poll_thread = threading.Thread(target=self._poll, daemon=True)
        self._poll_thread.start()
        log.info("TCPGripperServer listening on %s:%d", self._host, self._port)

    def stop(self) -> None:
        """서버 정지."""
        self._running = False
        if self._client_sock:
            try:
                self._client_sock.close()
            except Exception:
                pass
        if self._server_sock:
            try:
                self._server_sock.close()
            except Exception:
                pass
        for t in (self._thread, self._poll_thread):
            if t is not None:
                t.join()
        self._thread = None
        self._poll_thread = None

    def _serve(self) -> None:
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((self._host, self._port))
        self._server_sock.listen(1)
        self._server_sock.settimeout(1.0)
        log.info("TCPGripperServer waiting for client...")

        while self._running:
            try:
                self._client_sock, addr = self._server_sock.accept()
                log.info("Gripper client connected from %s", addr)
                self._client_sock.settimeout(0.1)
                self._recv_loop()
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    log.error("TCPGripperServer error: %s", e)

    def _recv_loop(self) -> None:
        while self._running:
            try:
                data = self._client_sock.recv(self._PACKET_SIZE)
                if not data:
                    log.warning("Gripper client disconnected")
                    break
                if len(data) == self._PACKET_SIZE:
                    right, left = struct.unpack(self._PACKET_FMT, data)
                    with self._lock:
                        self._latest = np.array(
                            [np.clip(right, 0.0, 1.0), np.clip(left, 0.0, 1.0)]
                        )
            except socket.timeout:
                continue
            except Exception as e:
                log.error("TCPGripperServer recv error: %s", e)
                break
        if self._client_sock:
            self._client_sock.close()
            self._client_sock = None

    def _poll(self) -> None:
        """수신된 최신 값을 그리퍼에 10ms 주기로 적용."""
        while self._running:
            with self._lock:
                pos = self._latest
            if pos is not None:
                self._gripper.set_normalized(right=float(pos[0]), left=float(pos[1]))
            time.sleep(0.01)


class GripperTCPClient:
    """원격 쪽에서 실행. TCPGripperServer에 (right, left) float 전송.

    OmniTeleop gripper_client.py의 GripperClient 패턴 통합.

    사용 예::

        client = GripperTCPClient("192.168.30.1", port=5000)
        client.connect()

        client.send_normalized(right=0.5, left=0.0)
        client.open()
        client.close()

        client.disconnect()

    context manager::

        with GripperTCPClient("192.168.30.1") as client:
            client.send_normalized(0.8, 0.8)
    """

    _PACKET_FMT = "ff"

    def __init__(self, host: str, port: int = 5000):
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> bool:
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.connect((self._host, self._port))
            log.info("GripperTCPClient connected to %s:%d", self._host, self._port)
            return True
        except Exception as e:
            log.error("GripperTCPClient connect failed: %s", e)
            self._sock = None
            return False

    def disconnect(self) -> None:
        if self._sock:
            self._sock.close()
            self._sock = None
        log.info("GripperTCPClient disconnected")

    def send_normalized(self, right: float, left: float) -> bool:
        """(right, left) normalized [0=열림, 1=닫힘] 전송."""
        if not self.connected:
            log.error("GripperTCPClient not connected")
            return False
        try:
            data = struct.pack(
                self._PACKET_FMT,
                float(np.clip(right, 0.0, 1.0)),
                float(np.clip(left, 0.0, 1.0)),
            )
            self._sock.sendall(data)
            return True
        except Exception as e:
            log.error("GripperTCPClient send failed: %s", e)
            self._sock = None
            return False

    def open(self) -> bool:
        return self.send_normalized(0.0, 0.0)

    def close(self) -> bool:
        return self.send_normalized(1.0, 1.0)

    def __enter__(self) -> "GripperTCPClient":
        self.connect()
        return self

    def __exit__(self, *_: Any) -> None:
        self.disconnect()
