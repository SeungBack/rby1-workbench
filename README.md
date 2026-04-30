# rby1-workbench

RB-Y1용 시각화, 좌표계 추적, calibration, control 앱을 한 곳에 모으기 위한 새 작업 공간입니다.

현재 단계:
- 프로젝트 골격
- 패키지 구조
- live robot state + FK + rerun frame viewer
- viser 기반 joint control panel MVP
- RealSense + SAM3 realtime segmentation wrapper/app

예정 단계:
- viser head/torso를 움직이면 right arm이 움직이는 문제
- head camera calibration 파이프라인
- 기본 grasping 기능 통합

## TODO

- [ ] outputs 등의 경로가 rby1-workbench 기준으로 설정되어 있음.

## Documentation

- [docs/IMPLEMENTATION.md](/home/kimm/Workspaces/rby1-workbench/docs/IMPLEMENTATION.md)
- [docs/DESIGN.md](/home/kimm/Workspaces/rby1-workbench/docs/DESIGN.md)
- [docs/PROGRESS.md](/home/kimm/Workspaces/rby1-workbench/docs/PROGRESS.md)
- [docs/CLAUDE.md](/home/kimm/Workspaces/rby1-workbench/docs/CLAUDE.md)

앞으로 구현을 진행할 때 README와 위 문서들을 함께 갱신합니다.

## Naming

- repository name: `rby1-workbench`
- package metadata name: `rby1-workbench`
- Python import name: `rby1_workbench`
- console script: `rby1-realtime-sam3-realsense`
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

RealSense 컬러/깊이 스트림 위에 SAM3 prompt segmentation 실행:

```bash
python -m rby1_workbench.apps.realtime_sam3_realsense
```

console script로 실행:

```bash
rby1-realtime-sam3-realsense
```

체크포인트나 카메라 설정 오버라이드:

```bash
python -m rby1_workbench.apps.realtime_sam3_realsense \
  sam3.checkpoint_path=/home/kimm/Workspaces/sam3/sam3.1_multiplex.pt \
  realsense.serial_number=123456789 \
  realsense.color_width=848 \
  realsense.color_height=480
```

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
- `Apply joint targets`: 현재 편집한 joint target을 한 번에 blocking 적용
- component별 탭으로 UI를 나눠서 세로 길이를 줄였습니다.
- 각 joint는 숫자 입력과 `-5/-1/+1/+5 deg` jog 버튼으로 조작합니다.
- joint 숫자 입력은 로컬 target 편집용이고, joint jog/zero 버튼은 클릭 즉시 해당 component에 blocking 적용됩니다.
- Cartesian은 `Send on jog=true`일 때 long-lived stream으로 바로 반영됩니다.

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
- [examples/realtime_sam3_realsense.py](/home/kimm/Workspaces/rby1-workbench/examples/realtime_sam3_realsense.py)

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
- joint target 적용은 session-level `Apply joint targets` 하나로 통합했습니다.
- head는 별도 zero 버튼 없이 joint target 편집 후 같은 apply 경로를 사용합니다.
- joint jog/zero 버튼은 `keyboard_control.py`처럼 바로 움직이고, 숫자 입력값은 필요할 때 `Apply joint targets`로 보냅니다.
- ready pose는 `rby1-sdk/examples/python/22_joint_impedance_control.py`의 preset 하나를 그대로 사용합니다.
- body는 `minimum_time=1.0`을 사용합니다.
  joint target apply의 기본값입니다.
- head는 `minimum_time=2.0`을 사용합니다.
  `34_head_joint_control.py`의 기본값 기준입니다.
- arm Cartesian stream은 packaged `rby1.yaml`의 Cartesian impedance 설정을 재사용합니다.
- arm streaming command에는 velocity / acceleration limit을 같이 넣습니다.
- panel은 `RBY1` wrapper + 공용 `RBY1Stream` 기반으로 동작합니다.
- `RBY1Stream`이 만료되면 자동으로 다시 생성해서 재개합니다.
- 시작 시 한 번 현재 자세를 target으로 읽어오고, 별도의 joint sync 버튼은 두지 않습니다.

`viser`는 이 개발 환경에서 아직 설치 검증을 하지는 못했습니다. 패키지 의존성에는 추가해두었고, 실행 시 env에 `viser`가 있어야 합니다.

`realtime_sam3_realsense`는 다음 환경을 가정합니다.
- `pyrealsense2`가 설치되어 있고 RealSense 카메라가 연결되어 있어야 합니다.
- 로컬 `sam3` repo가 같은 워크스페이스에 있고, 해당 env에 `pip install -e /home/kimm/Workspaces/sam3` 형태로 설치되어 있어야 합니다.
- 기본 체크포인트 경로는 `/home/kimm/Workspaces/sam3/sam3.1_multiplex.pt`입니다.
- text prompt는 SAM3 grounding 경로를 사용하고, point/box prompt는 SAM3 interactive predictor 경로를 사용합니다.
- 현재 geometry prompt가 하나라도 있으면 geometry 추론이 우선되고, text prompt는 화면 정보용으로 유지됩니다.

OpenCV 창에서 사용할 기본 조작:
- `p`: positive point 추가 모드
- `n`: negative point 추가 모드
- `b`: box drag 모드
- `t`: text prompt 편집
- `i`: 새 instance 추가
- `d`: active instance 삭제
- `[` / `]`: instance 전환
- `c`: active instance의 point/box prompt 초기화
- `x`: active instance의 모든 prompt 초기화
- `q`: 종료

현재 기본 시각화 동작:
- color 결과와 depth를 좌우 `hconcat`으로 함께 표시합니다.
- 첫 창 크기는 기본적으로 `1600 x 900`으로 엽니다.
- active instance 하나를 편집하면서, 추론은 등록된 모든 instance에 대해 수행합니다.

## Package Layout

```text
src/rby1_workbench/
  apps/       # 실행 앱
  conf/       # Hydra config
  config/     # structured config
  perception/ # RealSense / SAM3 / OpenCV prompt wrappers
  control/    # joint command / control panel helpers
  geometry/   # SE3 / transform graph
  robot/      # robot state / joints / FK
  viz/        # rerun logging / scene
```
