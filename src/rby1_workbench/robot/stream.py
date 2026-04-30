"""RBY1 continuous control stream helpers.

Public API:

```python
stream = robot.open_stream(mode="cartesian")
stream.send(torso=q_torso, right_arm=T_right, left_arm=T_left, head=q_head)
stream.pause()
stream.resume()
stream.close()
```
"""

from __future__ import annotations

import logging
import threading
from typing import Any, Callable, Mapping, Protocol

import numpy as np
import rby1_sdk as rby
from omegaconf import DictConfig

log = logging.getLogger(__name__)

_COMPONENT_ORDER = ("torso", "right_arm", "left_arm", "head")
_ARM_COMPONENTS = frozenset({"right_arm", "left_arm"})


def normalize_stream_mode(mode: str | Mapping[str, str] | None) -> dict[str, str]:
    """Return per-component stream modes.

    Supported public values:
    - `"joint"`: every component is interpreted as a joint target
    - `"cartesian"`: `torso`/`head` stay joint, arms use Cartesian targets
    - mapping: component -> `"joint"` or `"cartesian"`
    """
    if mode is None:
        return {
            "torso": "joint",
            "right_arm": "cartesian",
            "left_arm": "cartesian",
            "head": "joint",
        }

    if isinstance(mode, str):
        key = mode.lower().strip()
        if key == "joint":
            return {component: "joint" for component in _COMPONENT_ORDER}
        if key == "cartesian":
            return {
                "torso": "joint",
                "right_arm": "cartesian",
                "left_arm": "cartesian",
                "head": "joint",
            }
        raise ValueError(f"Unsupported stream mode: {mode}")

    normalized = {component: "joint" for component in _COMPONENT_ORDER}
    for component, value in mode.items():
        if component not in normalized:
            raise ValueError(f"Unknown stream component: {component}")
        component_mode = str(value).lower().strip()
        if component_mode not in {"joint", "cartesian"}:
            raise ValueError(
                f"Unsupported stream mode for {component}: {value}"
            )
        if component not in _ARM_COMPONENTS and component_mode == "cartesian":
            raise ValueError(
                f"Cartesian stream mode is only supported for arms, got {component}"
            )
        normalized[component] = component_mode
    return normalized


class _RemoteRequester(Protocol):
    def request(self, method: str, **params: Any) -> dict[str, Any]:
        ...


class _DirectStreamSession:
    """Direct SDK-backed streaming session."""

    def __init__(
        self,
        sdk_robot: Any,
        cfg: DictConfig,
        mode: dict[str, str],
        *,
        on_close: Callable[[], None] | None = None,
    ) -> None:
        self._robot = sdk_robot
        self._cfg = cfg
        self._mode = mode
        self._stream = sdk_robot.create_command_stream()
        self._last: dict[str, Any] = {}
        self._paused = False
        self._closed = False
        self._first_send = True
        self._lock = threading.Lock()
        self._on_close = on_close

    def send(
        self,
        *,
        torso: np.ndarray | None = None,
        right_arm: np.ndarray | None = None,
        left_arm: np.ndarray | None = None,
        head: np.ndarray | tuple[float, float] | None = None,
        reset: bool | None = None,
    ) -> None:
        with self._lock:
            if self._paused or self._closed:
                return

            effective_reset = self._first_send if reset is None else reset

            torso = torso if torso is not None else self._last.get("torso")
            right_arm = right_arm if right_arm is not None else self._last.get("right_arm")
            left_arm = left_arm if left_arm is not None else self._last.get("left_arm")
            head = head if head is not None else self._last.get("head")

            command = self._build(
                torso=torso,
                right_arm=right_arm,
                left_arm=left_arm,
                head=head,
                reset=effective_reset,
            )
            if command is None:
                return

            try:
                self._stream.send_command(command)
            except RuntimeError as exc:
                if "expired" in str(exc).lower():
                    log.warning("Command stream expired; closing stream")
                    self._closed = True
                    self._notify_closed()
                    return
                raise

            self._first_send = False
            if torso is not None:
                self._last["torso"] = np.asarray(torso, dtype=float).copy()
            if right_arm is not None:
                self._last["right_arm"] = np.asarray(right_arm, dtype=float).copy()
            if left_arm is not None:
                self._last["left_arm"] = np.asarray(left_arm, dtype=float).copy()
            if head is not None:
                self._last["head"] = np.asarray(head, dtype=float).copy()

    def pause(self) -> None:
        with self._lock:
            self._paused = True

    def resume(self) -> None:
        with self._lock:
            self._paused = False
            self._first_send = True

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def closed(self) -> bool:
        return self._closed

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        try:
            self._stream.cancel()
        except Exception:
            pass
        self._notify_closed()

    def _notify_closed(self) -> None:
        if self._on_close is not None:
            self._on_close()
            self._on_close = None

    def _build(
        self,
        *,
        torso: Any,
        right_arm: Any,
        left_arm: Any,
        head: Any,
        reset: bool,
    ) -> Any | None:
        dt = float(self._cfg.stream.dt)
        hold_time = dt * 10.0
        min_time = dt * 1.01

        has_body = any(value is not None for value in (torso, right_arm, left_arm))
        has_head = head is not None
        if not has_body and not has_head:
            return None

        component_builder = rby.ComponentBasedCommandBuilder()
        if has_head:
            q_head = np.asarray(head, dtype=float)
            component_builder.set_head_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(hold_time)
                )
                .set_position([float(q_head[0]), float(q_head[1])])
                .set_minimum_time(min_time)
            )

        if not has_body:
            return rby.RobotCommandBuilder().set_command(component_builder)

        body_builder = rby.BodyComponentBasedCommandBuilder()
        if torso is not None:
            body_builder.set_torso_command(
                rby.JointPositionCommandBuilder()
                .set_command_header(
                    rby.CommandHeaderBuilder().set_control_hold_time(hold_time)
                )
                .set_position(np.asarray(torso, dtype=float).tolist())
                .set_minimum_time(min_time)
            )

        for component, target in (("right_arm", right_arm), ("left_arm", left_arm)):
            if target is None:
                continue
            if self._mode[component] == "joint":
                builder = (
                    rby.JointPositionCommandBuilder()
                    .set_command_header(
                        rby.CommandHeaderBuilder().set_control_hold_time(hold_time)
                    )
                    .set_position(np.asarray(target, dtype=float).tolist())
                    .set_minimum_time(min_time)
                )
            else:
                ci = self._cfg.cartesian_impedance
                frame = "ee_right" if component == "right_arm" else "ee_left"
                builder = (
                    rby.CartesianImpedanceControlCommandBuilder()
                    .set_command_header(
                        rby.CommandHeaderBuilder().set_control_hold_time(hold_time)
                    )
                    .set_minimum_time(min_time)
                    .set_joint_stiffness(list(ci.arm_stiffness))
                    .set_joint_torque_limit(list(ci.arm_torque_limit))
                    .set_stop_joint_position_tracking_error(0)
                    .set_stop_orientation_tracking_error(0)
                    .set_reset_reference(reset)
                )
                for joint_name, (lower, upper) in ci.joint_limits.items():
                    if joint_name.startswith(component):
                        builder.add_joint_limit(joint_name, lower, upper)
                builder.add_target(
                    "base",
                    frame,
                    np.asarray(target, dtype=float),
                    ci.arm_linear_velocity_limit,
                    ci.arm_angular_velocity_limit,
                    ci.arm_linear_accel_limit,
                    ci.arm_angular_accel_limit,
                )
            getattr(body_builder, f"set_{component}_command")(builder)

        component_builder.set_body_command(body_builder)
        return rby.RobotCommandBuilder().set_command(component_builder)


class _RemoteStreamSession:
    """RPC-backed streaming session."""

    def __init__(self, requester: _RemoteRequester, stream_id: str) -> None:
        self._requester = requester
        self._stream_id = stream_id
        self._paused = False
        self._closed = False

    def send(
        self,
        *,
        torso: np.ndarray | None = None,
        right_arm: np.ndarray | None = None,
        left_arm: np.ndarray | None = None,
        head: np.ndarray | tuple[float, float] | None = None,
        reset: bool | None = None,
    ) -> None:
        if self._paused or self._closed:
            return
        self._requester.request(
            "stream_send",
            stream_id=self._stream_id,
            torso=_encode_array(torso),
            right_arm=_encode_array(right_arm),
            left_arm=_encode_array(left_arm),
            head=_encode_array(head),
            reset=reset,
        )

    def pause(self) -> None:
        if self._closed:
            return
        self._requester.request("stream_pause", stream_id=self._stream_id)
        self._paused = True

    def resume(self) -> None:
        if self._closed:
            return
        self._requester.request("stream_resume", stream_id=self._stream_id)
        self._paused = False

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def closed(self) -> bool:
        return self._closed

    def close(self) -> None:
        if self._closed:
            return
        self._requester.request("stream_close", stream_id=self._stream_id)
        self._closed = True


class RBY1Stream:
    """User-facing continuous control stream.

    `send(...)` accepts per-component targets.  Unless otherwise documented by
    the stream mode, target arrays are interpreted as follows:

    - `torso=q_torso`:
      joint target in radians, shape `(len(model.torso_idx),)`
    - `right_arm=q_right` or `right_arm=T_right`:
      joint target in radians for joint mode, or 4x4 `base -> ee_right`
      transform for Cartesian mode
    - `left_arm=q_left` or `left_arm=T_left`:
      joint target in radians for joint mode, or 4x4 `base -> ee_left`
      transform for Cartesian mode
    - `head=q_head`:
      joint target `[head_0, head_1]` in radians, shape `(2,)`

    Joint order always follows the model indices exposed by RB-Y1 SDK:
    `model.torso_idx`, `model.right_arm_idx`, `model.left_arm_idx`,
    `model.head_idx`.
    """

    def __init__(self, session: _DirectStreamSession | _RemoteStreamSession):
        self._session = session

    def send(
        self,
        *,
        torso: np.ndarray | None = None,
        right_arm: np.ndarray | None = None,
        left_arm: np.ndarray | None = None,
        head: np.ndarray | tuple[float, float] | None = None,
        reset: bool | None = None,
    ) -> None:
        self._session.send(
            torso=torso,
            right_arm=right_arm,
            left_arm=left_arm,
            head=head,
            reset=reset,
        )

    def pause(self) -> None:
        self._session.pause()

    def resume(self) -> None:
        self._session.resume()

    @property
    def paused(self) -> bool:
        return self._session.paused

    @property
    def closed(self) -> bool:
        return self._session.closed

    def close(self) -> None:
        self._session.close()

    def __enter__(self) -> "RBY1Stream":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()


def make_direct_stream(
    sdk_robot: Any,
    cfg: DictConfig,
    mode: str | Mapping[str, str] | None,
    *,
    on_close: Callable[[], None] | None = None,
) -> RBY1Stream:
    return RBY1Stream(
        _DirectStreamSession(
            sdk_robot=sdk_robot,
            cfg=cfg,
            mode=normalize_stream_mode(mode),
            on_close=on_close,
        )
    )


def make_remote_stream(
    requester: _RemoteRequester,
    stream_id: str,
) -> RBY1Stream:
    return RBY1Stream(_RemoteStreamSession(requester, stream_id))


def _encode_array(value: Any) -> Any:
    if value is None:
        return None
    return np.asarray(value, dtype=float).tolist()
