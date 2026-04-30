# rby1-workbench — Agent Context

## 프로젝트 개요

`rby1-workbench`는 RB-Y1 제어, 시각화, perception 실험을 위한 재사용 가능한 라이브러리다.
현재 설정 계층은 custom dataclass가 아니라 `OmegaConf DictConfig + packaged YAML` 기반으로 정리되어 있다.

- 로봇 제어 라이브러리: `RBY1`, `RBY1Stream`, head/gripper wrapper
- 시각화: Rerun live viewer, Viser joint control panel
- perception: RealSense + SAM3 realtime segmentation
- 설정 로딩: `load_rby1_config()`, `load_sam3_config()`

## 레포지토리 구조

```text
rby1-workbench/
  src/rby1_workbench/
    config/
      schema.py        ← load_rby1_config(), load_sam3_config(), package_root()
    conf/
      rby1.yaml
      visualize_robot.yaml
      viser_joint_control_panel.yaml
      realtime_sam3_realsense.yaml
    robot/
      rby1.py
      stream.py
      head.py
      gripper.py
      client.py
      joints.py
      kinematics.py
    control/
      joint_commands.py
      presets.py
      settings.py
      viser_joint_control.py
      rerun_control_viz.py
    viz/
      live_robot_viewer.py
      rerun_session.py
    perception/
      realsense.py
      sam3.py
      visualizer.py
      realtime_segmentation.py
    apps/
      visualize_robot.py
      viser_joint_control_panel.py
      realtime_sam3_realsense.py
  examples/
    robot_quickstart.py
    robot_control_basic.py
    keyboard_control.py
  docs/
    IMPLEMENTATION.md
    PROGRESS.md
    TROUBLE_SHOOTINGS.md
```

관련 참고 레포:
- `rby1-sdk/examples/python/` — 공식 SDK Python 예제
- `OmniTeleop/utils/` — gripper TCP 패턴
- `vr-teleop/python/` — streaming 파라미터 참고

## 핵심 설계 원칙

1. **SDK 예제와 동일한 제어 패턴 유지**
   공식 예제(`22_joint_impedance_control.py`, `cartesian_command_stream.py`)와 최대한 같은 제어 흐름을 따른다.
2. **라이브러리 우선, 앱은 thin wrapper**
   `apps/`는 Hydra CLI entrypoint이고, 실제 동작은 `robot/`, `control/`, `viz/`, `perception/` 라이브러리 계층에 둔다.
3. **설정은 DictConfig + YAML**
   public API는 `load_rby1_config()` / `load_sam3_config()`를 사용한다.
4. **안전한 기본값**
   viewer/control 앱은 기본적으로 보수적 설정을 유지하고, 자동 power/servo/control-manager enable은 명시적으로 켜야 한다.

## 주요 API

### Config Loader (`config/schema.py`)

```python
from rby1_workbench import load_rby1_config, load_sam3_config

cfg = load_rby1_config()                 # packaged conf/rby1.yaml
cfg = load_rby1_config("/tmp/my.yaml")   # 기본값 + 사용자 YAML merge

sam_cfg = load_sam3_config()             # packaged conf/realtime_sam3_realsense.yaml
sam_cfg = load_sam3_config("/tmp/sam3.yaml")
```

- 반환 타입은 `DictConfig`
- `path`를 주면 packaged default YAML 위에 사용자 YAML을 merge한다
- CLI/Hydra app은 별도 YAML(`visualize_robot.yaml`, `viser_joint_control_panel.yaml`, `realtime_sam3_realsense.yaml`)을 직접 사용한다

### RBY1 (`robot/rby1.py`)

```python
robot = RBY1(cfg)
robot.initialize()        # connect + optional power/servo/control-manager enable

robot.get_joint_positions("right_arm")
robot.dof("right_arm")
robot.get_ee_pose("right_arm")
robot.get_torso_pose()

robot.move(mode="joint", torso=q, right_arm=q, left_arm=q, head=q, minimum_time=5.0)
robot.move(mode="impedance", right_arm=q, stiffness=100.0, ...)
robot.move(mode="cartesian", right_arm=T, left_arm=T, torso=q)
robot.ready(minimum_time=5.0)
robot.zero(minimum_time=5.0)

stream = robot.open_stream(mode="cartesian")
stream.send(right_arm=T, left_arm=T, torso=q)
robot.move(mode="joint", head=np.array([yaw, pitch]))
robot.gripper.set_normalized(right, left)
```

자동 pause/resume:
- `move()`, `ready()`, `zero()`는 활성 스트림이 있으면 자동으로 pause/resume 처리한다.
- 사용자가 먼저 `stream.pause()`한 상태면 내부 auto-pause는 no-op로 동작한다.

### RBY1Stream (`robot/stream.py`)

```python
stream = robot.open_stream(mode="cartesian")

stream.send(
    torso=q_torso,
    right_arm=T_right,
    left_arm=T_left,
    head=np.array([yaw, pitch]),
)

stream.pause()
stream.resume()
stream.close()
```

현재 보장하는 동작:
- 첫 `send()` 및 `resume()` 이후 첫 `send()`에서 `reset=True` 자동 적용
- `pause()`, `resume()`, `send()`는 내부 lock으로 보호
- `cancel()`은 `_cancelled` 플래그로 idempotent
- expired stream / double-cancel 계열 오류를 방어
- `ListConfig` 입력은 내부에서 `list()`로 변환해 SDK가 기대하는 Python list로 맞춤

## 설정 사용 패턴

### 기본 로봇 예제

```python
from rby1_workbench import RBY1, load_rby1_config

cfg = load_rby1_config()
cfg.address = "192.168.30.1:50051"
cfg.model = "a"

robot = RBY1(cfg)
robot.initialize()
robot.ready()
```

### 사용자 YAML override

```python
cfg = load_rby1_config("my_rby1.yaml")
sam_cfg = load_sam3_config("my_sam3.yaml")
```

### 앱별 설정 파일

- `src/rby1_workbench/conf/rby1.yaml`:
  robot wrapper / streaming 기본값
- `src/rby1_workbench/conf/visualize_robot.yaml`:
  live viewer app 설정
- `src/rby1_workbench/conf/viser_joint_control_panel.yaml`:
  viser joint control app 설정
- `src/rby1_workbench/conf/realtime_sam3_realsense.yaml`:
  RealSense + SAM3 app 설정

## SDK 제어 패턴 주의사항

### Cartesian 빌더 두 종류

| 용도 | 빌더 | 참고 |
|------|------|------|
| streaming | `CartesianImpedanceControlCommandBuilder` | VR teleop 계열 |
| blocking | `CartesianCommandBuilder` | `cartesian_command_stream.py` |

### 프레임 이름

- public Cartesian target: `base -> ee_right`, `base -> ee_left`
- torso pose: `base -> link_torso_5`
- FK 계산: `["base", "ee_right", "ee_left", "link_torso_5"]`

### streaming 타이밍

```python
dt = cfg.stream.dt
hold_time = dt * 10.0
min_time = dt * 1.01
```

## 예제 코드

### `examples/robot_quickstart.py`

- `RBY1` 입문 예제
- `status`, `move`, `stream` subcommand로 핵심 흐름만 보여줌
- `--backend client --endpoint ...` 로 `RBY1(..., backend="client")` 사용 흐름도 함께 보여줌

### `examples/robot_control_basic.py`

- 입문 예제보다는 comprehensive smoke/integration test
- 상태 읽기, zero/ready pose, head, joint impedance, cartesian streaming, pause/resume interleave까지 순차 테스트

### `examples/keyboard_control.py`

- curses 기반 터미널 UI
- joint mode + cartesian streaming mode 모두 포함
- background streaming thread와 blocking command interleave 패턴 예제

## 자주 실수하는 것들

1. `DictConfig`를 dataclass처럼 가정하는 것
   현재 public config API는 dataclass가 아니라 `OmegaConf DictConfig`다.
2. blocking command와 streaming 상태 동기화를 분리하지 않는 것
   auto-pause/resume는 처리되지만, 로컬 target sync는 호출자가 책임져야 한다.
3. Cartesian builder를 streaming/blocking 사이에서 혼용하는 것
4. streaming frame 이름과 blocking frame 이름을 혼동하는 것
5. Cartesian panel 이슈를 low-level builder 문제로만 보는 것
   먼저 [TROUBLE_SHOOTINGS.md](./TROUBLE_SHOOTINGS.md)를 확인한다.

## 현재 상태 (2026-04-24 기준)

완료된 것:
- config dataclass 제거 및 DictConfig loader 기반으로 전면 전환
- `RBY1`, `RBY1Stream`, head/gripper wrapper
- live robot viewer / viser joint control panel / realtime SAM3 app
- 예제: `robot_quickstart.py`, `robot_control_basic.py`, `keyboard_control.py`
- stream 안정성 수정: cancel idempotent, expired send 방어, ListConfig 정규화

아직 unresolved 또는 후속 검토가 필요한 것:
- `viser_joint_control_panel`의 Cartesian 두 번 클릭 이슈
- tool / camera mount frame registration
- calibration / replay / dataset pipeline
