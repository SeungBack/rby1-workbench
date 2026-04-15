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

### Examples

- `examples/library_visualize_robot.py`
- `examples/kinematics_snapshot.py`

### Agent Handoff

- `docs/AGENT_PROMPT.md`
  - 다른 code agent가 참고할 수 있는 handoff prompt

## Not Implemented Yet

- URDF mesh visualization
- tool / camera mount frame registration
- head-eye calibration dataset pipeline
- cartesian impedance / teleop app migration
- config group 분리
- replay / recording pipeline

## Notes

- 목표는 현재 `online package publish`가 아니라 `local package install`입니다.
- Python import 이름은 `rby1_workbench`입니다.
- `visualize_robot`는 현재 기본적으로 읽기 전용입니다.
- power / servo / control manager enable은 명시적으로 config를 바꿨을 때만 수행됩니다.
- 현재 viewer는 frame-first 디버깅을 목표로 합니다.
