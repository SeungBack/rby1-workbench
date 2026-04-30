# Project Conventions

> 이 파일을 포함한 모든 `docs/*.md`는 concise, clear하게 작성한다. 장황한 설명, 중복 내용, 추측성 메모는 쓰지 않는다.

## apps/ vs examples/

- `apps/` — 운용 도구. `pyproject.toml`에 console script 등록 필수.
- `examples/` — 라이브러리 사용 레퍼런스. console script 없음.

새 실행 도구는 `apps/`에 추가하고 `pyproject.toml [project.scripts]`에 등록한다.
entry point 이름은 `cli`. argparse 앱은 파일 하단에 `cli = main` alias 추가.

## Console Scripts

| 명령 | 모듈 |
|---|---|
| `rby1-camera-server` | `apps/camera_server.py` |
| `rby1-head-camera-calib` | `apps/head_camera_calib.py` |
| `rby1-visualize-robot` | `apps/visualize_robot.py` |
| `rby1-viser-joint-control` | `apps/viser_joint_control_panel.py` |
| `rby1-sam3` | `apps/sam3.py` |
| `rby1-found-grasp` | `apps/found_grasp.py` |

## Camera 아키텍처

여러 앱이 동시에 RealSense를 사용할 때:

```
rby1-camera-server   # 카메라 단독 점유 → SHM 기록 + ZMQ 알림
각 앱                # SharedMemoryCameraStream으로 SHM에서 읽음
```

- 카메라 이름은 `conf/camera_server.yaml`의 `cameras[].name`에서만 정의.
- 앱에서 스트림 생성: `shm_stream.create_camera_stream(cfg)`.
- `camera_source.mode`: `"server"` (기본) / `"direct"` (단독 실행).

## FK 보정

`RobotKinematics(flip_joints=[...])` — URDF와 물리 축이 반대인 관절 보정.
RBY1-M: `head_1` (pitch) 반전. `rby1.yaml`, `visualize_robot.yaml`에 설정됨.

## Head Camera Calibration

- 기준 프레임: `conf/head_camera_calib.yaml`의 `robot.head_link`
- 5가지 방법 전부 계산 후 board consistency 최소 방법 자동 저장
- 결과 `outputs/head_camera_calib_*.json` → `rby1-visualize-robot` 실행 시 자동 로드

## Config

- `conf/*.yaml` — 패키지 기본값. `pip install` 후에도 로드 가능.
- 사용자 override: `--config my.yaml` 또는 `OmegaConf.merge`.
- `conf/camera_server.yaml` — 카메라 serial → name 매핑. 이 파일에서만 정의.
