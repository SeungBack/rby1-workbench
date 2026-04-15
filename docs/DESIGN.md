# Design Notes

Last updated: 2026-04-15

## Intent

`rby1-workbench`는 단순 예제 모음이 아니라, 다음 작업을 공통 구조 위에서 수행하기 위한 작업 공간입니다.

- 좌표계 / frame 추적
- 로봇 시각화
- head mounted camera calibration
- control app 구성
- config 기반 재사용

배포 목표는 현재 외부 공개 패키지보다, 로컬 워크스페이스에서 `pip install -e .`로 가져다 쓰는 내부 라이브러리에 가깝습니다.

패키지 import 이름은 일반적인 `rby1` 대신 `rby1_workbench`를 사용합니다.
이유는 `rby1-sdk`와의 혼동을 줄이고, 레포 이름과 더 직접적으로 대응시키기 위해서입니다.

## Current Design Direction

### 1. Frame-First

핵심은 viewer가 아니라 frame vocabulary입니다.

- `base`
- `link_torso_*`
- `link_right_arm_*`
- `link_left_arm_*`
- `link_head_*`
- `ee_right`
- `ee_left`

앞으로 calibration과 control도 같은 frame 이름을 공유하도록 유지합니다.

### 2. Thin Apps, Thick Library

앱은 얇게 두고, 재사용 가능한 코어는 라이브러리 내부에 둡니다.

- `apps/`
  실행 진입점
- `robot/`
  SDK 연결, state, FK
- `geometry/`
  SE3, transform graph
- `viz/`
  Rerun integration
- `config/`
  structured config

현재 `visualize_robot`는 CLI 앱이지만, 실제 동작 로직은
`rby1_workbench.viz.live_robot_viewer.run_visualize_robot(...)`에 두어
import 기반 재사용이 가능하도록 유지합니다.

### 3. Safe Defaults

읽기 전용 viewer는 하드웨어 상태를 바꾸지 않아야 합니다.

- auto power off by default
- auto servo off by default
- auto control manager enable off by default

실제 제어 앱에서만 명시적으로 enable 하도록 설계합니다.

### 4. Named Interfaces Over Raw Indices

가능하면 raw joint slice보다 이름 기반 접근을 우선합니다.

- `model.robot_joint_names`
- `model.head_idx`
- `model.right_arm_idx`
- `model.left_arm_idx`
- `model.torso_idx`

이 원칙은 head control, calibration, logging까지 동일하게 적용합니다.

### 5. Split Input and Visualization

인터랙션 입력과 로봇 시각화는 같은 도구에 억지로 넣지 않습니다.

- `Rerun`
  관찰, frame 확인, mesh, target/error overlay
- `Viser`
  slider, button, 이후의 EE gizmo 같은 입력 UI
- `control/`
  실제 SDK command builder와 streaming 로직

이렇게 나누면 제어 루프와 viewer 책임이 분명해지고, 나중에 cartesian target gizmo를 추가할 때도 구조가 덜 흔들립니다.

## Planned Extensions

### Calibration

추가 예정 frame chain:

- `base -> torso -> head -> camera_mount -> camera_optical`
- `camera_optical -> target`

필요 컴포넌트:

- image source abstraction
- target detection
- dataset capture
- solve / validate / export

### Control

추가 예정 모듈:

- joint command presets
- joint impedance wrappers
- cartesian impedance wrappers
- head command utilities
- teleop app migration

현재 첫 단계는 `viser joint slider panel -> control command builder` 경로를 먼저 안정화하는 것입니다.
그 다음 단계에서 `EE target gizmo -> cartesian command stream`으로 확장합니다.

초기 joint 제어 UX는 아래를 기본으로 둡니다.
- SDK ready pose로 즉시 이동
- 최소한의 head zero 기능
- joint jog 버튼 기반의 작은 증분 조작

즉, 사용자가 큰 slider를 끌다가 값이 튀는 패널이 아니라, 자주 쓰는 preset과 작은 jog 동작으로 안전하게 움직일 수 있게 유지합니다.

기본 파라미터도 임의값보다 SDK example 경향을 따릅니다.
- `body minimum_time`: `17`, `22` 예제를 따라 5초 기준
- `head minimum_time`: `34` 예제를 따라 2초 기준
- `stiffness`, `damping_ratio`, `torque_limit`: `17`의 teleoperation impedance 설정 기준
- `control_hold_time`: stream 제어에서는 `17`을 따라 `1e6`

component behavior도 `17` 예제 쪽을 우선 따릅니다.
- torso는 항상 joint position command
- arm은 position / impedance 전환 가능
- stream 제어 arm command에는 velocity / acceleration limit를 함께 사용

interactive panel은 완료 대기보다 반응성과 연속성을 우선합니다.
- 모든 interactive 전송은 공용 `command_stream`을 사용하고
- 별도 `Load current pose` 없이 시작 시 한 번만 현재 자세를 target으로 동기화합니다.

### Visualization

추가 예정:

- URDF mesh scene
- target / marker overlay
- recorded session replay
- calibration residual visualization

## Near-Term Rule

당분간 새 기능은 아래 순서를 우선합니다.

1. frame and transform correctness
2. state visibility and debugging
3. calibration data flow
4. control abstraction
