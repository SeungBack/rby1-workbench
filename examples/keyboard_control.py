"""RBY1 키보드 제어 예제.

터미널 UI (curses) 기반. 추가 패키지 불필요.

── 키 맵 ────────────────────────────────────────────────────────────────
 컴포넌트 선택      1=torso  2=right_arm  3=left_arm  4=head
 모드 선택          j=joint  c=cartesian  (head는 항상 joint)

 Joint 모드:
   ← / →           이전 / 다음 관절 선택
   ↑ / ↓           +step_s / -step_s (기본 5deg)
   Page Up/Down    +step_l / -step_l (기본 15deg)
   Enter           현재 targets 로봇에 전송 (blocking movej)
   0               선택한 관절을 0으로 초기화

 Cartesian 모드 (실시간 스트리밍):
   w / s           base X 방향 +/-
   d / a           base Y 방향 +/-
   e / q           base Z 방향 +/-
   i / k           X축 회전 +/-
   l / j           Y축 회전 +/-
   o / u           Z축 회전 +/-
   r               현재 FK 위치로 target 동기화

 공통:
   p               ready pose
   z               zero pose
   Space           cartesian streaming 일시정지 / 재개
   Esc / ctrl+c    종료
────────────────────────────────────────────────────────────────────────

실행:
    python examples/keyboard_control.py --address 192.168.30.1:50051
"""

from __future__ import annotations

import argparse
import curses
import logging
import threading
import time
from typing import Any

import numpy as np

from rby1_workbench import RBY1, RBY1Config
from rby1_workbench.robot.stream import RBY1Stream

# ── 제어 파라미터 ──────────────────────────────────────────────────────
STEP_TRANS   = 0.01          # 1 cm
STEP_TRANS_L = 0.05          # 5 cm
STEP_ROT     = np.deg2rad(5)   # 5 deg
STEP_ROT_L   = np.deg2rad(15)
STEP_JOINT   = np.deg2rad(5)
STEP_JOINT_L = np.deg2rad(15)
STREAM_HZ    = 10.0          # cartesian streaming 주기

COMPONENTS = ["torso", "right_arm", "left_arm", "head"]
COMPONENT_LABELS = {"torso": "Torso", "right_arm": "Right Arm",
                    "left_arm": "Left Arm", "head": "Head"}
MODES = ["joint", "cartesian"]


# ── 축 회전 행렬 (Rodrigues) ───────────────────────────────────────────
def _rot(axis: np.ndarray, angle: float) -> np.ndarray:
    k = axis / np.linalg.norm(axis)
    c, s = np.cos(angle), np.sin(angle)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return c * np.eye(3) + s * K + (1 - c) * np.outer(k, k)


# ── Controller ────────────────────────────────────────────────────────
class KeyboardController:
    def __init__(self, robot: RBY1):
        self.robot = robot
        self.model = robot.model

        # 상태
        self.component: str = "right_arm"
        self.mode: str = "cartesian"
        self.joint_idx: int = 0
        self.streaming: bool = True
        self.status_msg: str = "Ready"

        # joint targets (컴포넌트별 현재 목표 관절각 [rad])
        state = robot.get_state()
        q_all = np.asarray(state.position, dtype=float)
        self.q_targets: dict[str, np.ndarray] = {
            "torso":     q_all[list(self.model.torso_idx)].copy(),
            "right_arm": q_all[list(self.model.right_arm_idx)].copy(),
            "left_arm":  q_all[list(self.model.left_arm_idx)].copy(),
            "head":      q_all[list(self.model.head_idx)].copy(),
        }

        # cartesian targets (base 프레임 SE3)
        self.cart_targets: dict[str, np.ndarray] = {
            "right_arm": robot.get_ee_pose("right_arm"),
            "left_arm":  robot.get_ee_pose("left_arm"),
        }

        # streaming
        self._stream: RBY1Stream | None = None
        self._stream_lock = threading.Lock()
        self._stream_thread: threading.Thread | None = None
        self._stop_stream = threading.Event()

    # ── 스트림 관리 ───────────────────────────────────────────────────

    def start_stream(self) -> None:
        if self._stream_thread is not None and self._stream_thread.is_alive():
            return
        self._stop_stream.clear()
        self._stream_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self._stream_thread.start()

    def stop_stream(self) -> None:
        self._stop_stream.set()
        with self._stream_lock:
            if self._stream is not None:
                self._stream.cancel()
                self._stream = None

    def _stream_loop(self) -> None:
        with self._stream_lock:
            self._stream = self.robot.create_stream()
        reset = True
        while not self._stop_stream.is_set():
            if self.streaming and self.mode == "cartesian":
                try:
                    with self._stream_lock:
                        if self._stream is None:
                            self._stream = self.robot.create_stream()
                        q_torso = self.q_targets["torso"].copy()
                        rt = self.cart_targets.get("right_arm")
                        lt = self.cart_targets.get("left_arm")
                        self._stream.send(
                            torso=q_torso,
                            right_arm=rt,
                            left_arm=lt,
                            head=tuple(self.q_targets["head"][:2]),
                            reset=reset,
                        )
                        reset = False
                except Exception as e:
                    self.status_msg = f"Stream error: {e}"
                    reset = True
            time.sleep(1.0 / STREAM_HZ)

    # ── 키 처리 ───────────────────────────────────────────────────────

    def handle_key(self, key: int) -> bool:
        """True 반환 시 종료."""

        # 종료
        if key in (27, ord('q') - 96):  # Esc or ctrl+q
            return True

        # 컴포넌트 선택
        comp_map = {ord('1'): "torso", ord('2'): "right_arm",
                    ord('3'): "left_arm", ord('4'): "head"}
        if key in comp_map:
            self.component = comp_map[key]
            self.joint_idx = 0
            if self.component == "head":
                self.mode = "joint"
            self.status_msg = f"Selected: {COMPONENT_LABELS[self.component]}"
            return False

        # 모드 선택
        if key == ord('j'):
            self.mode = "joint"
            self.status_msg = "Mode: Joint"
            return False
        if key == ord('c') and self.component != "head":
            if self.component not in ("right_arm", "left_arm"):
                self.status_msg = "Cartesian mode: right_arm / left_arm만 지원"
                return False
            self.mode = "cartesian"
            self.status_msg = "Mode: Cartesian"
            return False

        # 공통 커맨드
        if key == ord('p'):
            self.stop_stream()
            ok = self.robot.ready_pose(minimum_time=5.0)
            self._sync_targets_from_robot()
            self.start_stream()
            self.status_msg = "Ready pose" + (" OK" if ok else " FAILED")
            return False
        if key == ord('z'):
            self.stop_stream()
            ok = self.robot.zero_pose(minimum_time=5.0)
            self._sync_targets_from_robot()
            self.start_stream()
            self.status_msg = "Zero pose" + (" OK" if ok else " FAILED")
            return False
        if key == ord(' '):
            self.streaming = not self.streaming
            self.status_msg = "Streaming: " + ("ON" if self.streaming else "PAUSED")
            return False

        # ── Joint 모드 키 ─────────────────────────────────────────────
        if self.mode == "joint":
            q = self.q_targets[self.component]
            dof = len(q)
            if dof == 0:
                return False

            if key == curses.KEY_LEFT:
                self.joint_idx = max(0, self.joint_idx - 1)
            elif key == curses.KEY_RIGHT:
                self.joint_idx = min(dof - 1, self.joint_idx + 1)
            elif key == curses.KEY_UP:
                q[self.joint_idx] += STEP_JOINT
                self.status_msg = f"j{self.joint_idx} +{np.rad2deg(STEP_JOINT):.0f}°"
            elif key == curses.KEY_DOWN:
                q[self.joint_idx] -= STEP_JOINT
                self.status_msg = f"j{self.joint_idx} -{np.rad2deg(STEP_JOINT):.0f}°"
            elif key == curses.KEY_PPAGE:  # Page Up
                q[self.joint_idx] += STEP_JOINT_L
                self.status_msg = f"j{self.joint_idx} +{np.rad2deg(STEP_JOINT_L):.0f}°"
            elif key == curses.KEY_NPAGE:  # Page Down
                q[self.joint_idx] -= STEP_JOINT_L
                self.status_msg = f"j{self.joint_idx} -{np.rad2deg(STEP_JOINT_L):.0f}°"
            elif key == ord('0'):
                q[self.joint_idx] = 0.0
                self.status_msg = f"j{self.joint_idx} → 0"
            elif key in (curses.KEY_ENTER, 10, 13):
                self._send_joint_blocking()
            return False

        # ── Cartesian 모드 키 ─────────────────────────────────────────
        if self.mode == "cartesian" and self.component in self.cart_targets:
            T = self.cart_targets[self.component]

            trans_map = {
                ord('w'): (np.array([1,0,0]), STEP_TRANS),
                ord('s'): (np.array([1,0,0]), -STEP_TRANS),
                ord('d'): (np.array([0,1,0]), STEP_TRANS),
                ord('a'): (np.array([0,1,0]), -STEP_TRANS),
                ord('e'): (np.array([0,0,1]), STEP_TRANS),
                ord('q'): (np.array([0,0,1]), -STEP_TRANS),
            }
            rot_map = {
                ord('i'): (np.array([1,0,0]), STEP_ROT),
                ord('k'): (np.array([1,0,0]), -STEP_ROT),
                ord('l'): (np.array([0,1,0]), STEP_ROT),
                ord('j'): (np.array([0,1,0]), -STEP_ROT),
                ord('o'): (np.array([0,0,1]), STEP_ROT),
                ord('u'): (np.array([0,0,1]), -STEP_ROT),
            }

            if key in trans_map:
                axis, delta = trans_map[key]
                T[:3, 3] += axis * delta
                self.status_msg = f"Trans {axis} {delta*100:+.0f}cm"
            elif key in rot_map:
                axis, delta = rot_map[key]
                dR = _rot(axis, delta)
                T[:3, :3] = dR @ T[:3, :3]
                self.status_msg = f"Rot {axis} {np.rad2deg(delta):+.0f}°"
            elif key == ord('r'):
                if self.component in ("right_arm", "left_arm"):
                    self.cart_targets[self.component] = self.robot.get_ee_pose(self.component)
                    self.status_msg = "Target synced from FK"

        return False

    # ── 내부 헬퍼 ─────────────────────────────────────────────────────

    def _send_joint_blocking(self) -> None:
        """현재 joint targets를 blocking movej로 전송."""
        self.stop_stream()
        comp = self.component
        kwargs: dict[str, Any] = {}
        if comp == "head":
            kwargs["head"] = tuple(self.q_targets["head"][:2])
        else:
            kwargs[comp] = self.q_targets[comp].copy()
        ok = self.robot.movej(minimum_time=3.0, **kwargs)
        self._sync_cart_from_fk()
        self.start_stream()
        self.status_msg = f"movej {COMPONENT_LABELS[comp]}" + (" OK" if ok else " FAILED")

    def _sync_targets_from_robot(self) -> None:
        state = self.robot.get_state()
        q_all = np.asarray(state.position, dtype=float)
        self.q_targets["torso"]     = q_all[list(self.model.torso_idx)].copy()
        self.q_targets["right_arm"] = q_all[list(self.model.right_arm_idx)].copy()
        self.q_targets["left_arm"]  = q_all[list(self.model.left_arm_idx)].copy()
        self.q_targets["head"]      = q_all[list(self.model.head_idx)].copy()
        self._sync_cart_from_fk()

    def _sync_cart_from_fk(self) -> None:
        for arm in ("right_arm", "left_arm"):
            try:
                self.cart_targets[arm] = self.robot.get_ee_pose(arm)
            except Exception:
                pass


# ── curses UI ─────────────────────────────────────────────────────────
def _run_ui(stdscr: Any, ctrl: KeyboardController) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_CYAN,   curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_GREEN,  curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED,    curses.COLOR_BLACK)

    ctrl.start_stream()

    def rpy_deg(R: np.ndarray) -> np.ndarray:
        sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
        if sy > 1e-6:
            roll  = np.arctan2(R[2,1], R[2,2])
            pitch = np.arctan2(-R[2,0], sy)
            yaw   = np.arctan2(R[1,0], R[0,0])
        else:
            roll  = np.arctan2(-R[1,2], R[1,1])
            pitch = np.arctan2(-R[2,0], sy)
            yaw   = 0.0
        return np.rad2deg([roll, pitch, yaw])

    while True:
        key = stdscr.getch()
        if key != -1:
            if ctrl.handle_key(key):
                break

        # ── 화면 그리기 ───────────────────────────────────────────────
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        row = 0

        def put(r: int, c: int, text: str, attr: int = 0) -> None:
            try:
                stdscr.addstr(r, c, text[:w-c-1], attr)
            except curses.error:
                pass

        put(row, 0, "═" * min(w-1, 64), curses.color_pair(1)); row += 1
        put(row, 0, "  RBY1 Keyboard Control", curses.color_pair(1) | curses.A_BOLD); row += 1
        put(row, 0, "═" * min(w-1, 64), curses.color_pair(1)); row += 1
        row += 1

        # 선택 상태
        comp_str = " | ".join(
            f"[{COMPONENT_LABELS[c]}]" if c == ctrl.component else COMPONENT_LABELS[c]
            for c in COMPONENTS
        )
        put(row, 0, f" Component : {comp_str}  (1/2/3/4)"); row += 1
        mode_str = "[joint]  cartesian" if ctrl.mode == "joint" else " joint  [cartesian]"
        stream_str = "▶ ON" if ctrl.streaming else "⏸ PAUSED"
        put(row, 0, f" Mode      : {mode_str}  (j/c)   Stream: {stream_str}  (Space)"); row += 1
        row += 1

        # Joint targets
        put(row, 0, " Joint Targets:", curses.color_pair(2)); row += 1
        q = ctrl.q_targets[ctrl.component]
        for i, v in enumerate(q):
            marker = "►" if (ctrl.mode == "joint" and i == ctrl.joint_idx) else " "
            attr = curses.color_pair(3) | curses.A_BOLD if i == ctrl.joint_idx else 0
            put(row, 2, f"{marker} j{i}: {np.rad2deg(v):7.2f} °", attr)
            row += 1
            if row >= h - 8:
                break
        row += 1

        # Cartesian EE (right/left arm)
        if ctrl.component in ("right_arm", "left_arm"):
            T_t = ctrl.cart_targets[ctrl.component]
            try:
                T_c = ctrl.robot.get_ee_pose(ctrl.component)
                p_c = T_c[:3, 3]
                p_t = T_t[:3, 3]
                rpy_c = rpy_deg(T_c[:3, :3])
                rpy_t = rpy_deg(T_t[:3, :3])
                put(row, 0, f" EE ({COMPONENT_LABELS[ctrl.component]}):", curses.color_pair(2)); row += 1
                put(row, 2, f"  Current pos (m): x={p_c[0]:6.3f}  y={p_c[1]:6.3f}  z={p_c[2]:6.3f}"); row += 1
                put(row, 2, f"  Target  pos (m): x={p_t[0]:6.3f}  y={p_t[1]:6.3f}  z={p_t[2]:6.3f}"); row += 1
                put(row, 2, f"  Current RPY(°) : r={rpy_c[0]:6.1f}  p={rpy_c[1]:6.1f}  y={rpy_c[2]:6.1f}"); row += 1
                put(row, 2, f"  Target  RPY(°) : r={rpy_t[0]:6.1f}  p={rpy_t[1]:6.1f}  y={rpy_t[2]:6.1f}"); row += 1
            except Exception:
                pass
            row += 1

        # 키 맵 요약
        if ctrl.mode == "joint":
            put(row, 0, " ←/→:관절선택  ↑/↓:±5°  PgUp/Dn:±15°  0:초기화  Enter:전송  p:ready  z:zero", curses.color_pair(1))
        else:
            put(row, 0, " w/s:±X  d/a:±Y  e/q:±Z  i/k:rotX  l/j:rotY  o/u:rotZ  r:FK동기화  p:ready", curses.color_pair(1))
        row += 1

        # 상태 메시지
        put(h-2, 0, f" Status: {ctrl.status_msg}", curses.color_pair(4) if "FAILED" in ctrl.status_msg else curses.color_pair(3))
        put(h-1, 0, " Esc: 종료", curses.color_pair(1))

        stdscr.refresh()
        time.sleep(0.05)

    ctrl.stop_stream()


# ── Entry ─────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="RBY1 키보드 제어")
    parser.add_argument("--address", default="192.168.30.1:50051")
    parser.add_argument("--model",   default="a")
    parser.add_argument("--config",  default=None, help="rby1.yaml 경로 (없으면 기본값)")
    args = parser.parse_args()

    logging.basicConfig(
        filename="/tmp/rby1_keyboard.log",
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    if args.config:
        cfg = RBY1Config.from_yaml(args.config)
    else:
        cfg = RBY1Config(address=args.address, model=args.model)

    print(f"Connecting to {cfg.address}...")
    robot = RBY1(cfg)
    robot.initialize()
    print("Connected. Starting keyboard control UI...")

    ctrl = KeyboardController(robot)
    curses.wrapper(_run_ui, ctrl)
    print("Exited.")


if __name__ == "__main__":
    main()
