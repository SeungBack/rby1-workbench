# rby1-workbench

RB-Y1용 시각화, 좌표계 추적, calibration, control 앱을 한 곳에 모으기 위한 새 작업 공간입니다.

현재 단계:
- 프로젝트 골격
- 패키지 구조
- live robot state + FK + rerun frame viewer
- viser 기반 joint control panel MVP

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
- console script: `rby1-viser-joint-control`

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

viser 기반 joint control panel 실행:

```bash
python -m rby1_workbench.apps.viser_joint_control_panel \
  robot.address=192.168.12.1:50051 \
  robot.model=m \
  command.enable_torso=true \
  command.enable_right_arm=true \
  command.enable_left_arm=true \
  command.enable_head=true
```

Rerun 3D viewer 함께 켜기 (`viz.enable=true`):

```bash
python -m rby1_workbench.apps.viser_joint_control_panel \
  robot.address=192.168.12.1:50051 \
  robot.model=m \
  command.enable_right_arm=true \
  command.enable_left_arm=true \
  viz.enable=true
```

Rerun viewer에서 확인할 수 있는 것:
- 로봇 skeleton + 전체 frame graph (live, 10 Hz)
- 현재 EE frame (live FK, 오른팔 빨강 / 왼팔 파랑)
- Cartesian jog target frame (jog 버튼 누를 때마다 즉시 갱신, 오른팔 오렌지 / 왼팔 시안)

패널에는 아래 동작이 포함됩니다.
- `Move to ready pose`: `22_joint_impedance_control.py`의 ready pose를 사용
- `Send target pose`: 현재 target state를 stream으로 전송
- `Head to zero`: head joint만 0도로 복귀
- component별 탭으로 UI를 나눠서 세로 길이를 줄였습니다.
- 각 joint는 숫자 입력과 `-5/-1/+1/+5 deg` jog 버튼으로 조작합니다.
- 기본값으로 `Send on jog=true`라서 jog 버튼을 누르면 바로 로봇에 반영됩니다.
- jog/숫자 입력은 변경된 component만 전송하고, `Send target pose`는 활성화된 모든 component target을 전송합니다.

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
- [examples/library_viser_joint_control.py](/home/kimm/Workspaces/rby1-workbench/examples/library_viser_joint_control.py)

이 예제들은 저장소 안의 reference example이며, 온라인 패키지 배포를 전제로 한 구성은 아닙니다.

현재 `visualize_robot` 앱은 기본적으로 읽기 전용입니다.
- `auto_power_on=false`
- `auto_servo_on=false`
- `auto_enable_control_manager=false`
- `viz.log_meshes=false`

즉, viewer 실행만으로 로봇 power/servo 상태를 바꾸지 않도록 해두었습니다.

현재 `viser_joint_control_panel`은 입력 UI만 담당합니다.
- `Rerun`은 관찰/시각화
- `Viser`는 component tab + number input + jog button 기반 입력
- torso/arm은 position 또는 joint impedance
- head는 joint position command
- ready pose는 `rby1-sdk/examples/python/22_joint_impedance_control.py`의 preset 하나를 그대로 사용합니다.
- 기본 제어값은 SDK 예제 쪽 패턴에 맞췄습니다.
- body는 `minimum_time=1.0`을 사용합니다.
  jog 반응성 기준으로 조정했습니다. (preset move는 별도 `move_j`로 처리 예정)
- head는 `minimum_time=2.0`을 사용합니다.
  `34_head_joint_control.py`의 기본값 기준입니다.
- `stiffness=50.0`, `damping_ratio=1.0`, `torque_limit=30.0`은
  `17_teleoperation_with_joint_mapping.py`의 impedance 설정을 따릅니다.
- `control_hold_time=1e6`은 `17_teleoperation_with_joint_mapping.py`의 stream command 기준입니다.
- torso는 `17_teleoperation_with_joint_mapping.py` 흐름에 맞춰 항상 `JointPositionCommandBuilder`를 사용합니다.
- arm streaming command에는 `17` 예제처럼 velocity / acceleration limit도 같이 넣습니다.
- panel 전송은 `send_command`가 아니라 공용 `command_stream` 기반입니다.
- `command_stream`이 만료되면 자동으로 다시 생성해서 재전송합니다.
- jog/number 입력은 별도 worker thread를 거치지 않고 callback에서 바로 stream으로 전송합니다.
- 시작 시 한 번 현재 자세를 target으로 읽어오고, 별도의 `Load current pose` 버튼은 두지 않습니다.

`viser`는 이 개발 환경에서 아직 설치 검증을 하지는 못했습니다. 패키지 의존성에는 추가해두었고, 실행 시 env에 `viser`가 있어야 합니다.

## Package Layout

```text
src/rby1_workbench/
  apps/       # 실행 앱
  conf/       # Hydra config
  config/     # structured config
  control/    # joint command / control panel helpers
  geometry/   # SE3 / transform graph
  robot/      # robot state / joints / FK
  viz/        # rerun logging / scene
```
