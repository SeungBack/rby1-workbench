"""Minimal RPC transport for remote RBY1 control."""

from __future__ import annotations

import json
import logging
import socket
import socketserver
import threading
from typing import Any
from uuid import uuid4

import numpy as np
from omegaconf import DictConfig

from rby1_workbench.robot.rby1 import RBY1

log = logging.getLogger(__name__)


class RobotRpcClient:
    """Stateless JSON-over-TCP requester used by client backend."""

    def __init__(self, endpoint: str):
        self._host, self._port = _parse_endpoint(endpoint)

    def request(self, method: str, **params: Any) -> dict[str, Any]:
        payload = {"method": method, "params": params}
        with socket.create_connection((self._host, self._port)) as sock:
            handle = sock.makefile("rwb")
            handle.write(json.dumps(payload).encode("utf-8") + b"\n")
            handle.flush()
            line = handle.readline()
        if not line:
            raise RuntimeError(f"RPC server closed connection for method '{method}'")
        response = json.loads(line.decode("utf-8"))
        if not response.get("ok", False):
            raise RuntimeError(response.get("error", "Unknown RPC error"))
        return dict(response.get("result", {}))


class RBY1Server:
    """Serve `RBY1` over a lightweight JSON-over-TCP protocol."""

    def __init__(
        self,
        cfg: DictConfig,
        *,
        host: str = "0.0.0.0",
        port: int = 5555,
        auto_initialize: bool = True,
    ) -> None:
        self._cfg = cfg
        self._host = host
        self._port = int(port)
        self._auto_initialize = auto_initialize
        self._robot = RBY1(cfg, backend="direct")
        self._sessions: dict[str, Any] = {}
        self._sessions_lock = threading.Lock()
        self._server: socketserver.ThreadingTCPServer | None = None
        self._started = False

    @property
    def robot(self) -> RBY1:
        return self._robot

    def serve_forever(self) -> None:
        if not self._started and self._auto_initialize:
            self._robot.initialize()
            self._started = True

        owner = self

        class _Handler(socketserver.StreamRequestHandler):
            def handle(self) -> None:
                line = self.rfile.readline()
                if not line:
                    return
                try:
                    request = json.loads(line.decode("utf-8"))
                    method = str(request["method"])
                    params = dict(request.get("params", {}))
                    result = owner._dispatch(method, params)
                    response = {"ok": True, "result": result}
                except Exception as exc:
                    log.exception("RBY1 RPC request failed")
                    response = {"ok": False, "error": str(exc)}
                self.wfile.write(json.dumps(response).encode("utf-8") + b"\n")

        class _ThreadingServer(socketserver.ThreadingTCPServer):
            allow_reuse_address = True

        with _ThreadingServer((self._host, self._port), _Handler) as server:
            self._server = server
            log.info("RBY1Server listening on %s:%d", self._host, self._port)
            server.serve_forever()

    def shutdown(self) -> None:
        if self._server is not None:
            self._server.shutdown()

    def _dispatch(self, method: str, params: dict[str, Any]) -> dict[str, Any]:
        if method == "connect":
            self._robot.connect()
            self._started = True
            return {"model": self._robot.model.to_payload()}
        if method == "initialize":
            self._robot.initialize()
            self._started = True
            return {"model": self._robot.model.to_payload()}
        if method == "model_info":
            return {"model": self._robot.model.to_payload()}
        if method == "power_on":
            self._robot.power_on(params.get("pattern"))
            return {}
        if method == "power_off":
            self._robot.power_off(params.get("pattern"))
            return {}
        if method == "servo_on":
            self._robot.servo_on(params.get("pattern"))
            return {}
        if method == "servo_off":
            self._robot.servo_off(params.get("pattern"))
            return {}
        if method == "enable_control_manager":
            self._robot.enable_control_manager()
            return {}
        if method == "get_state":
            state = self._robot.get_state()
            return {
                "position": state.position.tolist(),
                "timestamp": state.timestamp,
            }
        if method == "get_ee_pose":
            transform = self._robot.get_ee_pose(str(params["arm"]))
            return {"transform": transform.tolist()}
        if method == "get_torso_pose":
            transform = self._robot.get_torso_pose()
            return {"transform": transform.tolist()}
        if method == "get_transform":
            transform = self._robot.get_transform(
                str(params["parent"]),
                str(params["child"]),
            )
            return {"transform": transform.tolist()}
        if method == "register_static_frame":
            self._robot.register_static_frame(
                str(params["parent"]),
                str(params["child"]),
                np.asarray(params["transform"], dtype=float),
                label=params.get("label"),
            )
            return {}
        if method == "move":
            ok = self._robot.move(**_decode_move_kwargs(params))
            return {"ok": bool(ok)}
        if method == "ready":
            ok = self._robot.ready(float(params.get("minimum_time", 5.0)))
            return {"ok": bool(ok)}
        if method == "zero":
            ok = self._robot.zero(float(params.get("minimum_time", 5.0)))
            return {"ok": bool(ok)}
        if method == "open_stream":
            stream = self._robot.open_stream(mode=params.get("mode"))
            stream_id = uuid4().hex
            with self._sessions_lock:
                self._sessions[stream_id] = stream
            return {"stream_id": stream_id}
        if method == "stream_send":
            stream = self._get_stream(str(params["stream_id"]))
            stream.send(
                torso=_decode_array(params.get("torso")),
                right_arm=_decode_array(params.get("right_arm")),
                left_arm=_decode_array(params.get("left_arm")),
                head=_decode_array(params.get("head")),
                reset=params.get("reset"),
            )
            return {}
        if method == "stream_pause":
            self._get_stream(str(params["stream_id"])).pause()
            return {}
        if method == "stream_resume":
            self._get_stream(str(params["stream_id"])).resume()
            return {}
        if method == "stream_close":
            stream_id = str(params["stream_id"])
            with self._sessions_lock:
                stream = self._sessions.pop(stream_id, None)
            if stream is not None:
                stream.close()
            return {}
        raise ValueError(f"Unknown RPC method: {method}")

    def _get_stream(self, stream_id: str) -> Any:
        with self._sessions_lock:
            stream = self._sessions.get(stream_id)
        if stream is None:
            raise ValueError(f"Unknown stream id: {stream_id}")
        return stream


def _parse_endpoint(endpoint: str) -> tuple[str, int]:
    value = endpoint.strip()
    if value.startswith("tcp://"):
        value = value[len("tcp://") :]
    host, port_text = value.rsplit(":", 1)
    return host, int(port_text)


def _decode_array(value: Any) -> np.ndarray | None:
    if value is None:
        return None
    return np.asarray(value, dtype=float)


def _decode_move_kwargs(params: dict[str, Any]) -> dict[str, Any]:
    decoded = dict(params)
    for key in ("torso", "right_arm", "left_arm", "head"):
        decoded[key] = _decode_array(decoded.get(key))
    return decoded
