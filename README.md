# rby1-workbench

RB-Y1용 시각화, 좌표계 추적, calibration, control 앱을 한 곳에 모으기 위한 새 작업 공간입니다.

현재 단계:
- 프로젝트 골격
- 패키지 구조
- live robot state + FK + rerun frame viewer

예정 단계:
- head camera calibration 파이프라인
- config 기반 control app 정리

## Documentation

- [docs/IMPLEMENTATION.md](/home/kimm/Workspaces/rby1-workbench/docs/IMPLEMENTATION.md)
- [docs/DESIGN.md](/home/kimm/Workspaces/rby1-workbench/docs/DESIGN.md)
- [docs/PROGRESS.md](/home/kimm/Workspaces/rby1-workbench/docs/PROGRESS.md)
- [docs/AGENT_PROMPT.md](/home/kimm/Workspaces/rby1-workbench/docs/AGENT_PROMPT.md)

앞으로 구현을 진행할 때 README와 위 문서들을 함께 갱신합니다.

## Naming

- repository name: `rby1-workbench`
- package metadata name: `rby1-workbench`
- Python import name: `rby1_workbench`
- console script: `rby1-visualize-robot`

## Install

이 프로젝트는 현재 `PyPI 배포용`보다는 `로컬에서 pip install 해서 쓰는 패키지`를 목표로 합니다.

```bash
pip install -e .
```

일반 설치:

```bash
pip install .
```

## Run

console script로 기본 viewer 실행:

```bash
rby1-visualize-robot
```

모듈 실행:

```bash
python -m rby1_workbench.apps.visualize_robot
```

주소나 모델 오버라이드:

```bash
python -m rby1_workbench.apps.visualize_robot robot.address=192.168.12.1:50051 robot.model=m
```

mesh까지 함께 시각화:

```bash
python -m rby1_workbench.apps.visualize_robot robot.address=192.168.12.1:50051 robot.model=m viz.log_meshes=true
```

## Library Usage

Python 라이브러리처럼 직접 import 해서 사용할 수 있습니다.

```python
from rby1_workbench import RobotConfig, VisualizeRobotConfig, VizConfig, run_visualize_robot

cfg = VisualizeRobotConfig(
    robot=RobotConfig(address="192.168.12.1:50051", model="m"),
    viz=VizConfig(application_id="my_rby1_viewer"),
)

run_visualize_robot(cfg)
```

추가 예시는 아래 파일들을 참고하면 됩니다.

- [examples/library_visualize_robot.py](/home/kimm/Workspaces/rby1-workbench/examples/library_visualize_robot.py)
- [examples/kinematics_snapshot.py](/home/kimm/Workspaces/rby1-workbench/examples/kinematics_snapshot.py)

이 예제들은 저장소 안의 reference example이며, 온라인 패키지 배포를 전제로 한 구성은 아닙니다.

현재 `visualize_robot` 앱은 기본적으로 읽기 전용입니다.
- `auto_power_on=false`
- `auto_servo_on=false`
- `auto_enable_control_manager=false`
- `viz.log_meshes=false`

즉, viewer 실행만으로 로봇 power/servo 상태를 바꾸지 않도록 해두었습니다.

## Package Layout

```text
src/rby1_workbench/
  apps/       # 실행 앱
  conf/       # Hydra config
  config/     # structured config
  geometry/   # SE3 / transform graph
  robot/      # robot state / joints / FK
  viz/        # rerun logging / scene
```
