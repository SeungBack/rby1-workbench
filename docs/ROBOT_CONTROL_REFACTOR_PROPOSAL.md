# Robot Control Refactor Proposal

## Goal

`src/rby1_workbench/robot`의 제어 API를 더 쉽고 일관되게 정리한다.

이번 refactor의 최우선 원칙은 다음과 같다.

1. 내부 SDK command 조립 방식은 `rby1-sdk/examples/python` 예제 패턴을 최대한 그대로 따른다.
2. 사용자에게 노출되는 제어 표면은 적고 명확해야 한다.
3. blocking command와 streaming command의 역할을 분리하되, 호출 형태는 최대한 비슷하게 만든다.

## Final Review Result

현재까지의 검토를 반영한 최종 채택안은 아래와 같다.

1. public API는 `move + stream`으로 간다.
2. `RBY1`는 backend 선택형 facade로 둔다.
3. `direct`와 `client`는 같은 호출 표면을 사용한다.
4. `server`는 별도 `RBY1Server` 프로세스 역할로 분리한다.
5. 기존 `movej`, `move_cartesian`, `joint_impedance_control`, `create_stream().send()`, `head.move_j()`, `torso.move_j()`는 새 API로 직접 리팩터링한다.

예상 최종 사용 형태:

```python
robot = RBY1(cfg, backend="direct")
robot = RBY1(cfg, backend="client", endpoint="tcp://127.0.0.1:5555")

robot.move(
    mode="joint",
    torso=q_torso,
    right_arm=q_right,
    left_arm=q_left,
    head=q_head,
    minimum_time=5.0,
)

stream = robot.open_stream(mode="cartesian")
stream.send(right_arm=T_right, left_arm=T_left, torso=q_torso, head=q_head)
stream.pause()
stream.resume()
stream.close()
```

주의:

- 아래 문서의 `command / execute / session` 아이디어는 내부 구조 참고용으로만 남긴다.
- 최종 public API naming은 `move / open_stream / stream.send`가 우선이다.

## Public API Contract

### 1. `RBY1`

- `RBY1(cfg, backend="direct")`
- `RBY1(cfg, backend="client", endpoint=...)`
- `move(...)`
- `open_stream(...)`
- `get_joint_positions(component)`
- `get_transform(parent, child)`

### 2. `RBY1Server`

- 별도 프로세스로 실행
- 내부적으로 direct backend `RBY1`를 하나 소유
- 외부 `client` 요청을 받아 `move(...)` / `stream.send(...)`를 수행

### 3. `RBY1Stream`

- `stream = robot.open_stream(...)`
- `stream.send(...)`
- `stream.pause()`
- `stream.resume()`
- `stream.close()`

## Data Contract For Docstrings

리팩터링 후에는 모든 public method docstring에 아래 계약을 명시해야 한다.

### Joint target arrays

모든 joint target은 `np.ndarray` 또는 array-like이며 단위는 `rad`다.

- `q_torso`
  - shape: `(dof_torso,)`
  - order: `model.torso_idx` 순서
  - canonical names:
    - A/M: `torso_0, torso_1, torso_2, torso_3, torso_4, torso_5`
    - UB: `torso_0, torso_1`
- `q_right`
  - shape: `(7,)`
  - order: `model.right_arm_idx` 순서
  - canonical names: `right_arm_0 ... right_arm_6`
- `q_left`
  - shape: `(7,)`
  - order: `model.left_arm_idx` 순서
  - canonical names: `left_arm_0 ... left_arm_6`
- `q_head`
  - shape: `(2,)`
  - order: `model.head_idx` 순서
  - canonical names: `head_0, head_1`

public docstring에는 "joint order is `model.<component>_idx` order"를 먼저 적고, 그 다음 canonical joint names를 예시로 적는 것이 좋다.

### Cartesian target matrices

모든 cartesian target은 다음 형태를 따른다.

- `T_right`
  - shape: `(4, 4)`
  - meaning: homogeneous transform `base -> target_frame_of_right_arm`
- `T_left`
  - shape: `(4, 4)`
  - meaning: homogeneous transform `base -> target_frame_of_left_arm`
- `T_torso`
  - shape: `(4, 4)`
  - meaning: homogeneous transform `base -> target_frame_of_torso`

그리고 가장 중요하게, refactor 전에 target frame을 하나로 고정해야 한다.

현재 코드는 frame 계약이 일관적이지 않다.

- 기존 `RBY1.get_ee_pose()`는 내부적으로 `link_right_arm_6` / `link_left_arm_6`를 사용한다.
- `JointCommandClient.compute_fk()`는 `ee_right` / `ee_left`를 사용한다.
- 기존 stream cartesian impedance도 `link_right_arm_6` / `link_left_arm_6`를 사용한다.

따라서 이번 refactor에서 반드시 아래 둘 중 하나를 선택해야 한다.

옵션 A:

- public cartesian frame을 `ee_right` / `ee_left`로 통일
- stream 내부에서 필요하면 adapter를 둔다

옵션 B:

- public cartesian frame을 `link_right_arm_6` / `link_left_arm_6`로 통일
- 메서드 이름도 `get_arm_tip_pose()`처럼 더 정확하게 바꾼다

현재는 옵션 A를 추천한다. 이유는:

1. `ee_right` / `ee_left`가 SDK 예제 naming과 더 잘 맞는다.
2. public API에서 "EE pose"라는 이름과 의미가 일치한다.
3. 내부 stream builder의 특수한 frame choice는 adapter로 숨길 수 있다.

### Control mode terms

docstring에서는 아래 용어를 분리해서 써야 한다.

- `mode="joint"`: target representation이 joint array
- `mode="cartesian"`: target representation이 4x4 transform
- `control="position"`: joint position command
- `control="impedance"`: joint impedance or cartesian impedance style control

즉 `mode`와 `control`을 혼용하지 않도록 한다.

## Required Naming Cleanup

이번 refactor에서 아래 이름 충돌은 제거하는 것이 맞다.

- `movej`
- `move_j`
- `send`
- `update`

최종 public naming:

- one-shot: `move(...)`
- stream factory: `open_stream(...)`
- continuous send: `stream.send(...)`

이 naming은 사용자가 기억하기 가장 쉽고, direct / client backend 모두에 동일하게 적용하기 쉽다.

## Current Pain Points

현재는 같은 로봇 제어를 여러 방식으로 호출할 수 있다.

- `robot.movej(...)`
- `robot.joint_impedance_control(...)`
- `robot.move_cartesian(...)`
- `robot.create_stream().send(...)`
- `robot.head.move_j(...)`
- `robot.torso.move_j(...)`

이 상태의 문제는 다음과 같다.

1. 어디까지가 "주 API"인지 모호하다.
2. head / torso만 별도 controller로 빠져 있어 whole-body 사용 흐름과 분리된다.
3. `movej`, `move_j`, `send`가 섞여 naming이 일관되지 않다.
4. 호출 전에는 어떤 함수가 joint position인지, cartesian인지, impedance인지 바로 드러나지 않는다.

## What Must Stay Close To SDK Examples

SDK 예제를 보면 실제 command 조립은 크게 아래 패턴으로 정리된다.

### 1. Blocking joint position

- `helper.py`의 `movej(...)`
- `send_robot_command.py`
- `09_demo_motion.py`

공통점:

- `RobotCommandBuilder`
- `ComponentBasedCommandBuilder`
- `BodyComponentBasedCommandBuilder`
- 각 부위에 `JointPositionCommandBuilder`

### 2. Blocking cartesian

- `cartesian_command.py`
- `09_demo_motion.py`

공통점:

- body component별 `CartesianCommandBuilder`
- `add_target("base", target_frame, T, ...)`

### 3. Blocking joint impedance

- `22_joint_impedance_control.py`

공통점:

- arm은 `JointImpedanceControlCommandBuilder`
- torso는 필요 시 `JointPosition` 또는 `JointImpedance`를 별도로 선택

### 4. Streaming

- `command_stream.py`
- `cartesian_command_stream.py`

공통점:

- `create_command_stream()`
- 매 tick마다 완성된 SDK command를 다시 조립해 `send_command(...)`

즉 refactor의 핵심은 "새 명령 방식을 발명"하는 것이 아니라, 위 네 패턴을 더 적은 public API로 다시 포장하는 것이다.

## Recommended API Direction

핵심은 `move`와 `stream`을 "입력 형식"에서 나누지 않는 것이다.

- command payload shape는 하나로 통일
- one-shot blocking 실행과 continuous streaming 실행만 분리

즉, 사용자가 만드는 target 자체는 같고, 그것을 어떻게 실행하느냐만 달라진다.

## Unified Command Shape

권장 command shape:

```python
command = MotionCommand(
    torso=q_torso,         # joint target
    right_arm=q_right,     # joint target or SE3 target
    left_arm=T_left,       # joint target or SE3 target
    head=q_head,           # joint target
    mode={
        "torso": "joint",
        "right_arm": "cartesian",
        "left_arm": "cartesian",
        "head": "joint",
    },
)
```

실제 Python API는 dataclass든 builder든 더 간단하게 감쌀 수 있지만, 중요한 점은:

1. target payload는 하나다.
2. 각 component가 joint target인지 cartesian target인지 명시된다.
3. 이 payload는 local call에도, external process IPC에도 똑같이 쓸 수 있다.

## Execution Modes

같은 command payload를 두 방식으로 실행한다.

### 1. One-shot execution

현재의 `movej`, `move_cartesian`, `joint_impedance_control`이 여기에 해당한다.

- 한 번 command를 조립해 전송
- finish code를 기다림
- blocking call

예시:

```python
robot.execute(command, minimum_time=5.0)
robot.execute_ready(minimum_time=5.0)
robot.execute_zero(minimum_time=5.0)
```

### 2. Streaming execution

현재의 `create_stream().send(...)`가 여기에 해당한다.

- resident stream/session이 살아 있음
- 새 target payload를 받으면 그것으로 다음 command를 전송
- sender는 target만 갱신하고, 실제 resend loop는 stream/session이 담당할 수 있음

예시:

```python
session = robot.session()
session.update(command)
session.pause()
session.resume()
session.close()
```

이 구조에서는 `stream.send(...)`보다 `session.update(...)`가 의미상 더 정확하다.

## Why Move And Stream Should Not Be Fully Collapsed

`move`와 `stream`은 입력 데이터가 달라서가 아니라 실행 의미가 다르다.

### one-shot / blocking

- 목표 자세로 한 번 이동
- 완료 대기
- ready, zero, head move 같은 작업에 적합

### streaming / session

- target이 계속 바뀜
- 실제 전송 주기, last-value hold, pause/resume, expired stream 재생성 같은 lifecycle이 필요
- teleop, external tracker, keyboard/viser control에 적합

즉 둘을 완전히 하나의 메서드로 합치면 오히려 API는 짧아져도 의미가 흐려질 수 있다.

대신 아래처럼 "command는 통합, executor는 분리"가 더 자연스럽다.

- `MotionCommand`
- `robot.execute(...)`
- `robot.session().update(...)`

## External Process Streaming

사용자 요구:

- 로봇을 붙잡고 있는 프로세스는 따로 상주
- 다른 Python 코드가 target stream만 보냄
- 상주 프로세스가 그것을 받아 실제 SDK command로 송신

이 요구에는 local object API만으로는 부족하고, process boundary가 있는 control service가 필요하다.

권장 구조:

### On robot side

- `RobotControlServer`
- 내부에 `RBY1`와 active streaming session 보유
- IPC/TCP/ZMQ 등으로 `MotionCommand` 메시지를 수신
- 수신된 command를 one-shot 실행하거나 session target으로 반영

### On external side

- `RobotControlClient`
- `execute(command)`
- `update(command)`
- `pause_session()`
- `resume_session()`

이 구조는 이미 gripper 쪽의 `TCPGripperServer` / `GripperTCPClient` 패턴과도 잘 맞는다.

## Recommended Public API

권장 방향은 public API를 아래 두 축으로 정리하는 것이다.

### A. Local direct control

```python
robot.command.joint(...)
robot.command.cartesian(...)
robot.command.ready(...)
robot.command.zero(...)

robot.execute(...)
```

예시:

```python
command = robot.command.joint(
    torso=q_torso,
    right_arm=q_right,
    left_arm=q_left,
    head=q_head,
)
robot.execute(command, minimum_time=5.0)

command = robot.command.cartesian(
    right_arm=T_right,
    left_arm=T_left,
)
robot.execute(command, minimum_time=3.0)

command = robot.command.impedance(
    right_arm=q_right,
    stiffness=100.0,
    damping_ratio=1.0,
    torque_limit=10.0,
)
robot.execute(command, minimum_time=5.0)

robot.execute_ready()
robot.execute_zero()
```

### B. Local streaming session

```python
session = robot.session()
session.update(...)
session.pause()
session.resume()
session.close()
```

예시:

```python
session = robot.session()
command = robot.command.cartesian(
    torso=q_torso,
    right_arm=T_right,
    left_arm=T_left,
    head=q_head,
)
session.update(command)
```

## Why This Shape

이 구조의 장점은 다음과 같다.

1. 사용자는 command payload와 executor를 분리해서 이해할 수 있다.
2. local direct control과 external streaming control이 같은 payload 형식을 공유할 수 있다.
3. head도 whole-body API에 자연스럽게 포함된다.
4. 내부 구현은 기존 SDK-aligned builder를 그대로 재사용할 수 있다.

## Compatibility Direction

리팩터링 시에는 아래 방향을 권장한다.

### Keep as primary

- `robot.command.joint(...)`
- `robot.command.cartesian(...)`
- `robot.command.impedance(...)`
- `robot.command.ready(...)`
- `robot.command.zero(...)`
- `robot.execute(...)`
- `robot.session()`

### Remove during refactor

- `robot.movej(...)`
- `robot.move_cartesian(...)`
- `robot.joint_impedance_control(...)`
- `robot.create_stream()`
- `robot.head.move_j(...)`
- `robot.torso.move_j(...)`

기존 코드는 새 API에 맞게 함께 정리한다.

## Suggested Internal Structure

구조적으로는 아래가 가장 자연스럽다.

- `RBY1`
  - lifecycle / state / kinematics / raw sdk access 담당
  - `command` property 제공
  - `execute(...)`
  - `session()` factory 제공
- `MotionCommand`
  - component target payload
- `CommandFactory`
  - `joint`
  - `cartesian`
  - `impedance`
  - `ready`
  - `zero`
- `RobotCommandSession`
  - resident command stream lifecycle
  - `update`, `pause`, `resume`, `close`
- `RobotControlServer` / `RobotControlClient`
  - external process 연동용 선택 계층

즉 부위별 controller보다 command payload와 executor를 중심으로 두는 편이 더 명확하다.

## Naming Recommendation

권장 naming은 다음과 같다.

- `movej` 대신 `command.joint + execute`
- `move_cartesian` 대신 `command.cartesian + execute`
- `joint_impedance_control` 대신 `command.impedance + execute`
- `create_stream().send` 대신 `session().update`
- `move_j` 대신 `command.joint + execute`

특히 `move_j`와 `movej`가 같이 존재하는 상태는 이번 refactor에서 제거하는 것이 좋다.

## Open Design Questions For Review

아래 두 가지는 구현 전 사용자와 합의가 필요하다.

### 1. Public API 이름

옵션 A:

- `command + execute + session`

옵션 B:

- `move + stream`

현재 요구사항에는 옵션 A가 더 맞다. external client/server까지 확장할 때도 의미가 덜 흔들린다.

### 2. External transport

옵션 A:

- in-process refactor만 먼저 진행

옵션 B:

- 이번에 control server/client 뼈대까지 같이 만든다

현재 목표가 "다른 Python 코드가 stream을 보낸다"까지 포함이면 옵션 B가 더 자연스럽다.

## Recommended Decision

현재 기준으로는 아래 합의를 추천한다.

1. 공식 public control API는 `command + execute + session`으로 재구성한다.
2. 내부 SDK command 조립은 기존 `rby1.py`, `stream.py`, `joint_commands.py`의 SDK-aligned 패턴을 유지한다.
3. `head` / `torso` 개별 controller는 제거하거나 내부 전용으로 낮추고, example에서는 쓰지 않는다.
4. 필요 시 `RobotControlServer` / `RobotControlClient`를 추가해 외부 프로세스 제어를 지원한다.
5. 새 example은 다음 네 갈래로 만든다.
   - blocking joint execute
   - blocking cartesian / impedance execute
   - local session update
   - external client -> robot server streaming

## Non-Goals

이번 refactor에서 바로 하지 않아도 되는 것:

- 새 motion primitive 추가
- SDK 예제에 없는 command 조합 발명
- low-level command builder의 동작 변경
- gripper API 재설계
