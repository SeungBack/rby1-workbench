# Design Notes

Last updated: 2026-04-24

## Intent

`rby1-workbench`는 단순 예제 모음이 아니라, 다음 작업을 공통 구조 위에서 수행하기 위한 작업 공간입니다.

- 좌표계 / frame 추적
- 로봇 시각화
- head mounted camera calibration
- control app 구성
- config 기반 재사용
- camera stream 기반 perception 실험

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
- `perception/`
  camera stream, segmentation wrapper, prompt visualizer
- `viz/`
  Rerun integration
- `config/`
  structured config

현재 `visualize_robot`와 `realtime_sam3_realsense`는 CLI 앱이지만, 실제 동작 로직은 라이브러리 함수로 분리해 import 기반 재사용이 가능하도록 유지합니다.

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

인터랙션 입력과 관찰용 시각화는 같은 계층에 억지로 섞지 않습니다.

- `Rerun`
  로봇 state 관찰, frame 확인, mesh, target overlay
- `Viser`
  로봇 제어 입력 UI
- `OpenCV`
  camera 기반 실시간 prompt 입력과 segmentation overlay
- `control/`
  실제 SDK command builder와 streaming 로직
- `perception/`
  camera stream + segmentation model abstraction

이렇게 나누면 각 app이 책임을 선명하게 유지하면서도, calibration과 perception을 이후 다른 UI로 옮기기 쉬워집니다.

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

이번 단계에서는 calibration 전체를 넣기보다, 그 이전 단계인 `camera stream -> prompt -> segmentation -> visualization` 경로를 먼저 perception wrapper로 고정합니다.

### Perception

현재 perception 레이어의 기본 분리는 아래와 같습니다.

- `RealSenseStream`
  live color/depth source abstraction
- `Sam3RealtimePredictor`
  text/geometry prompt segmentation abstraction
- `OpenCVPromptVisualizer`
  prompt 입력과 결과 overlay를 담당하는 UI abstraction

text prompt와 point/box prompt는 한 화면에서 다루지만, SAM3 내부 추론 경로는 다르므로 wrapper가 이를 숨깁니다.
현재 단계에서는 geometry prompt가 있으면 geometry 결과를 메인 overlay로 사용합니다.

### Control

추가 예정 모듈:

- joint command presets
- joint impedance wrappers
- cartesian impedance wrappers
- head command utilities
- teleop app migration

현재 첫 단계는 `viser joint panel -> control command builder` 경로를 먼저 안정화하는 것입니다.
그 다음 단계에서 `EE target gizmo -> cartesian command stream`으로 확장합니다.

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
3. camera stream and perception abstraction
4. calibration data flow
5. control abstraction
