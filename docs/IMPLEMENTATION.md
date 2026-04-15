# Implementation Status

Last updated: 2026-04-15

## Current Scope

현재 `rby1-workbench`는 다음 범위를 우선 구현 중입니다.

- 로컬 pip-installable library 형태 정리
- import 이름을 `rby1_workbench`로 명확화
- live robot state 수집
- RB-Y1 link chain 기반 FK 계산
- frame graph 구성
- Rerun 기반 robot frame / skeleton 시각화
- viser 기반 joint control panel MVP

## Implemented

### Project Setup

- `pyproject.toml` 기반 패키지 구성
- `src/rby1_workbench/` 패키지 구조 분리
- Hydra config 진입점 추가
- console script entry point 추가
- public import API 추가

### Config

- `rby1_workbench.config.schema`
  - `RobotConfig`
  - `VizConfig`
  - `VisualizeRobotConfig`
  - `ViserConfig`
  - `JointControlConfig`
  - `ViserJointControlAppConfig`

### Geometry

- `rby1_workbench.geometry.se3`
  - identity
  - compose
  - inverse
  - relative_transform

- `rby1_workbench.geometry.transform_graph`
  - `TransformGraph`
  - `TransformEdge`

### Robot

- `rby1_workbench.robot.client`
  - 안전한 기본값의 robot connect helper
  - thread-safe latest state buffer
  - initial synchronous `get_state()` seed
  - callback error capture for debugging

- `rby1_workbench.robot.joints`
  - torso / arm / head link chain 정의
  - component joint 이름 조회

- `rby1_workbench.robot.kinematics`
  - SDK dynamics 기반 FK
  - named link transform 계산
  - skeleton point chain 생성
  - joint name -> position map 생성

### Visualization

- `rby1_workbench.viz.rerun_session`
  - Rerun session init
  - frame axes logging
  - frame point labels using joint names
  - optional GLB mesh asset logging
  - line strip logging
  - scalar logging

- `rby1_workbench.apps.visualize_robot`
  - live state polling
  - transform graph logging
  - optional mesh logging from local SDK assets
  - torso / arm / head skeleton logging
  - head joint scalar plot logging

- `rby1_workbench.viz.live_robot_viewer`
  - 앱 로직을 재사용 가능한 library function으로 분리
  - no-state warning and stream diagnostics

### Control

- `rby1_workbench.control.joint_commands`
  - component별 joint 이름/limit 조회
  - position / joint impedance command builder
  - shared command stream send path
  - expired stream auto-recreate
  - torso / arm / head target apply helper
  - body/head minimum time 분리
  - torso position-only behavior aligned with SDK teleoperation example
  - arm velocity / acceleration limit injection for stream commands

- `rby1_workbench.control.presets`
  - `22_joint_impedance_control.py` 기준 ready pose preset
  - model별 torso / arm 기본 자세 정의

- `rby1_workbench.control.viser_joint_control`
  - viser 기반 tabbed jog panel
  - head 기본 활성화
  - torso / arm opt-in
  - component tab UI
  - target number input
  - per-component jog send
  - direct callback-to-stream send path
  - jog update 기반 command streaming
  - ready pose button
  - startup-only current pose sync
  - SDK example 기준 default control parameter

- `rby1_workbench.apps.viser_joint_control_panel`
  - Hydra app entry point

### Examples

- `examples/library_visualize_robot.py`
- `examples/kinematics_snapshot.py`
- `examples/library_viser_joint_control.py`

### Agent Handoff

- `docs/AGENT_PROMPT.md`
  - 다른 code agent가 참고할 수 있는 handoff prompt

## Not Implemented Yet

- tool / camera mount frame registration
- head-eye calibration dataset pipeline
- cartesian impedance / EE target gizmo
- teleop app migration
- config group 분리
- replay / recording pipeline

## Notes

- 목표는 현재 `online package publish`가 아니라 `local package install`입니다.
- Python import 이름은 `rby1_workbench`입니다.
- `visualize_robot`는 현재 기본적으로 읽기 전용입니다.
- power / servo / control manager enable은 명시적으로 config를 바꿨을 때만 수행됩니다.
- 현재 viewer는 frame-first 디버깅을 목표로 합니다.
- `viser_joint_control_panel`은 첫 control MVP이며, cartesian control은 아직 포함하지 않습니다.
