# rby1-workbench — Agent Context

## 프로젝트 개요

`rby1-workbench`는 공식 `rby1-sdk`와 VR teleop 예제를 래핑하는 재사용 가능한 로봇 제어 라이브러리다.
시각화 시스템(Rerun, Viser)은 기존 코드를 유지하고, 로봇 제어 레이어는 SDK 예제 패턴을 그대로 따르도록 새로 작성됐다.

## 레포지토리 구조

```
rby1-workbench/
  src/rby1_workbench/
    config/schema.py        ← RBY1Config, StreamConfig, GripperConfig 등 dataclass 설정
    conf/rby1.yaml          ← 기본 설정값 (VR teleop 기준)
    robot/
      rby1.py               ← RBY1 메인 wrapper (핵심)
      stream.py             ← RBY1Stream 실시간 제어 스트림 (핵심)
      head.py               ← HeadController
      gripper.py            ← GripperController, TCPGripperServer, GripperTCPClient, InspireGripperController
      client.py             ← 기존 viz 시스템용 (수정 금지)
      kinematics.py         ← 기존 viz 시스템용 (수정 금지)
    control/                ← 기존 viz 시스템용 (수정 금지)
    viz/                    ← 기존 viz 시스템용 (수정 금지)
    apps/                   ← 기존 CLI 앱 (수정 금지)
  examples/
    robot_control_basic.py  ← 기본 제어 예제 (blocking + streaming)
    keyboard_control.py     ← curses 기반 키보드 제어 예제
```

관련 참고 레포:
- `rby1-sdk/examples/python/` — 공식 SDK Python 예제 (제어 패턴 기준)
- `OmniTeleop/utils/` — gripper TCP 서버/클라이언트 패턴
- `vr-teleop/python/` — VR teleop (streaming 파라미터 기준)

## 핵심 설계 원칙

1. **SDK 예제와 동일한 제어 패턴** — 공식 예제(`22_joint_impedance_control.py`, `cartesian_command_stream.py`)를 그대로 따름. 임의로 변경하면 실제 로봇에서 오작동 가능
2. **기존 viz 시스템 불변** — `client.py`, `kinematics.py`, `control/`, `viz/`, `apps/` 는 건드리지 않음
3. **설정 기반** — `RBY1Config.from_yaml()` 또는 직접 생성으로 모든 파라미터 조절 가능

## 주요 클래스 API

### RBY1 (`robot/rby1.py`)

```python
robot = RBY1(cfg)
robot.initialize()        # connect + power_on + servo_on + enable_control_manager

# 상태 읽기
robot.get_joint_positions("right_arm")  # → np.ndarray [rad], 컴포넌트: torso/right_arm/left_arm/head
robot.dof("right_arm")                  # → int
robot.get_ee_pose("right_arm")          # → 4×4 SE3, base 프레임
robot.get_torso_pose()                  # → 4×4 SE3

# blocking 이동 (내부적으로 활성 스트림 자동 pause/resume)
robot.movej(torso=q, right_arm=q, left_arm=q, head=q, minimum_time=5.0)
robot.ready_pose(minimum_time=5.0)
robot.zero_pose(minimum_time=5.0)
robot.joint_impedance_control(right_arm=q, stiffness=100.0, ...)
robot.move_cartesian(right_arm=T, ...)   # blocking, ee_right/ee_left 프레임

# 스트리밍
stream = robot.create_stream()   # 이전 스트림 자동 cancel, 내부 참조 유지

# 서브 컨트롤러
robot.head.move(yaw, pitch)
robot.gripper.set_normalized(right, left)
```

**자동 pause/resume**: `movej()`, `ready_pose()`, `zero_pose()`, `joint_impedance_control()`, `move_cartesian()`은 활성 스트림이 있을 때 자동으로 `stream.pause()` → 실행 → `stream.resume()` 처리.
단, 스트림이 이미 paused 상태면 건드리지 않음 (`own_pause = not stream.paused`).

### RBY1Stream (`robot/stream.py`)

```python
stream = robot.create_stream()

# 전송 (첫 번째 send에서 reset=True 자동 적용)
stream.send(
    torso=q_torso,          # np.ndarray (joint_position 모드) 또는 4×4 SE3 (cartesian_impedance 모드)
    right_arm=T_right,      # 4×4 SE3 (기본값: cartesian_impedance)
    left_arm=T_left,
    head=np.array([yaw, pitch]),  # ndarray 또는 (yaw, pitch) 튜플
)
# None인 컴포넌트는 마지막 전송값으로 자동 hold

# pause/resume (blocking command 전후 안전 전환)
stream.pause()        # 진행 중인 send() 완료 대기 후 중단
stream.resume()       # 재개, 다음 send에서 reset=True 자동 적용
stream.paused         # bool property

stream.cancel()       # 스트림 종료
```

**스레드 안전**: `send()`, `pause()`, `resume()` 모두 내부 `threading.Lock`으로 보호. 백그라운드 스레드에서 안전하게 호출 가능.

**auto-reset 동작**:
- 스트림 생성 후 첫 `send()` → `reset=True` 자동
- `stream.resume()` 후 첫 `send()` → `reset=True` 자동
- 이후 `send()`들 → `reset=False`

### RBY1Config (`config/schema.py`)

```python
# 생성 방법
cfg = RBY1Config()                          # 기본값
cfg = RBY1Config(address="...", model="a")
cfg = RBY1Config.from_yaml("conf/rby1.yaml")
cfg = RBY1Config.from_hydra(hydra_cfg)

print(cfg)  # 간결한 repr: RBY1Config(address=..., model=..., stream=..., gripper=...)
```

주요 필드: `address`, `model`, `stream: StreamConfig`, `cartesian_impedance: CartesianImpedanceStreamConfig`, `gripper: GripperConfig`

## SDK 제어 패턴 주의사항

### Cartesian 빌더 두 종류 — 혼용 금지

| 용도 | 빌더 | `add_target` 인자 수 | 참고 예제 |
|------|------|---------------------|-----------|
| **Streaming** | `CartesianImpedanceControlCommandBuilder` | 7개 (from, to, T, lin_vel, ang_vel, lin_accel, ang_accel) | VR teleop |
| **Blocking** | `CartesianCommandBuilder` | 6개 (from, to, T, lin_vel, ang_vel, accel_scaling) | `cartesian_command_stream.py` |

### 프레임 이름

- **스트리밍 Cartesian** (CartesianImpedance): `link_right_arm_6`, `link_left_arm_6`, `link_torso_5`
- **Blocking Cartesian** (CartesianCommand): `ee_right`, `ee_left`
- **FK 계산**: `["base", "link_torso_5", "link_right_arm_6", "link_left_arm_6"]`

### Streaming 타이밍 (VR teleop 기준)

```python
dt = cfg.stream.dt          # 기본 0.1s
hold_time = dt * 10.0       # 1.0s
min_time  = dt * 1.01       # 0.101s
```

### Ready Pose (22_joint_impedance_control.py 기준)

| 모델 | torso | right_arm | left_arm |
|------|-------|-----------|----------|
| A, M | `[0,45,-90,45,0,0]°` | `[0,-5,0,-120,0,70,0]°` | `[0,5,0,-120,0,70,0]°` |
| UB | `[10,0]°` | 동일 | 동일 |

## 예제 코드

### 기본 패턴

```python
from rby1_workbench import RBY1, RBY1Config

cfg = RBY1Config(address="192.168.30.1:50051", model="a")
robot = RBY1(cfg)
robot.initialize()

# 관절 이동 (blocking, 스트림 자동 pause/resume)
robot.ready_pose()
q = robot.get_joint_positions("right_arm")   # 현재 관절값

# 실시간 스트리밍
T_right = robot.get_ee_pose("right_arm")
T_left  = robot.get_ee_pose("left_arm")
q_torso = robot.get_joint_positions("torso")
q_head  = robot.get_joint_positions("head")

stream = robot.create_stream()
stream.send(torso=q_torso, right_arm=T_right, left_arm=T_left, head=q_head)
# 이후 변경분만 send, 나머지는 자동 hold
stream.send(right_arm=T_right_new)
stream.cancel()
```

### 백그라운드 스레드 + pause/resume 패턴

```python
# keyboard_control.py 참조
stream = robot.create_stream()

def stream_loop():
    while not stop_event.is_set():
        stream.send(torso=..., right_arm=..., left_arm=..., head=...)
        time.sleep(dt)

t = threading.Thread(target=stream_loop, daemon=True)
t.start()

# blocking command 시 (수동 pause - 타겟 sync 후 resume 보장)
stream.pause()
robot.ready_pose()          # auto-pause: 이미 paused → no-op
sync_targets_from_robot()   # 로컬 타겟 동기화
stream.resume()             # 이후 첫 send에서 reset=True 자동
```

## Gripper 사용

```python
# 로컬 DynamixelBus (로봇에 직접 연결된 경우)
robot.gripper.setup()
robot.gripper.start()
robot.gripper.set_normalized(right=0.5, left=0.5)  # 0.0=open, 1.0=close

# TCP 서버 (원격 컨트롤러가 OmniTeleop GripperClient로 접속)
from rby1_workbench import TCPGripperServer
server = TCPGripperServer(robot.gripper, host="0.0.0.0", port=5000)
server.start()

# TCP 클라이언트
from rby1_workbench import GripperTCPClient
client = GripperTCPClient()
client.connect("192.168.30.1", 5000)
client.send_normalized(right=0.0, left=0.0)   # open
```

TCP 프로토콜: `struct.pack('ff', right, left)` — 8바이트, OmniTeleop 호환.

## 자주 실수하는 것들

1. **스트림 생성 후 첫 send에 `reset=True` 생략** — 자동 처리되므로 명시적으로 안 넘겨도 됨
2. **blocking command와 streaming 동시 사용** — `robot.movej()` 등이 자동 pause/resume 하지만, 로컬 타겟 동기화(`get_joint_positions`, `get_ee_pose`)는 사용자가 해야 함
3. **`CartesianImpedanceControlCommandBuilder` vs `CartesianCommandBuilder`** — 스트리밍/blocking 각각 다른 빌더 사용, `add_target` 인자 수 다름
4. **프레임 이름** — 스트리밍은 `link_*_arm_6`, blocking은 `ee_right`/`ee_left`
5. **기존 viz 코드 건드리지 않기** — `client.py`, `kinematics.py`, `control/`, `viz/`, `apps/`는 별도 시스템

## 현재 상태 (2026-04-24 기준)

완료된 것:
- `RBY1` wrapper: lifecycle, blocking cmds, FK, sub-controllers
- `RBY1Stream`: pause/resume, auto-reset, thread-safe, head ndarray
- `HeadController`: blocking joint position + look_at_midpoint
- `GripperController` (local Dynamixel), `TCPGripperServer`, `GripperTCPClient`, `InspireGripperController`
- `RBY1Config`: dataclass + YAML/Hydra 로딩 + `__repr__`
- 예제: `robot_control_basic.py`, `keyboard_control.py`

아직 없는 것 (필요 시 추가):
- 상태 콜백 기반 streaming (현재는 polling)
- Viser 연동 제어 UI (기존 viser 코드는 viz 전용)
- 그리퍼 힘 피드백 / 토크 제어
- 멀티 로봇 지원
