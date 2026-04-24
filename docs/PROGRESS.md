# Progress Log

## 2026-04-24

### Completed

- `perception/` 패키지 추가
- `pyrealsense2` 기반 `RealSenseStream` wrapper 추가
- `sam3/test.py` 흐름을 참고한 `Sam3RealtimePredictor` wrapper 추가
- text prompt용 SAM3 grounding 경로 래핑
- point/box prompt용 SAM3 interactive predictor 경로 래핑
- OpenCV 기반 live prompt visualizer 추가
- mouse로 point/box 입력, keyboard로 text prompt 편집 가능하도록 구성
- RealSense + SAM3 + visualizer를 묶는 realtime runtime loop 추가
- Hydra app `realtime_sam3_realsense` 추가
- console script `rby1-realtime-sam3-realsense` 추가
- example script `examples/realtime_sam3_realsense.py` 추가
- README / IMPLEMENTATION / DESIGN 문서 갱신
- visualizer를 color/depth `hconcat` 레이아웃으로 변경
- visualizer 초기 창 크기 확대
- multi-instance 추가 / 삭제 / 전환 기능 추가
- custom config dataclass 제거 및 `DictConfig` 기반 설정 로딩으로 전면 전환
- `load_rby1_config()` / `load_sam3_config()` 추가 및 public export 정리
- `robot/`, `viz/`, `control/`, `perception/`, `apps/`, `examples/` 전반에서 dataclass config 의존성 제거
- `stream.py` 안정성 수정
- `cancel()` idempotent 처리로 double-cancel 시 gRPC 채널 corruption 방지
- cancelled stream send 방어 및 expired stream 예외 처리
- OmegaConf `ListConfig`를 SDK 호출 전 Python `list`로 정규화
- `docs/CLAUDE.md`, `docs/IMPLEMENTATION.md`, `docs/PROGRESS.md`를 DictConfig 리팩터링 기준으로 갱신

### Current State

- `rby1-workbench` 안에서 로봇 viewer/control과 별개로 perception 실험을 위한 독립 wrapper 계층이 생김
- live camera stream에서 text, point, box prompt를 받아 SAM3 결과를 바로 overlay로 볼 수 있음
- prompt 입력 UI는 OpenCV에 두고, stream/model 쪽은 재사용 가능한 라이브러리 형태로 분리됨
- 로봇/시각화/perception 설정 계층은 custom dataclass 없이 packaged YAML + `DictConfig` 패턴으로 통일됨

### Notes

- 현재 text prompt와 geometry prompt는 동일 UI에서 관리되지만, geometry prompt가 있으면 geometry 추론이 우선됨
- 기본 SAM3 checkpoint는 `/home/kimm/Workspaces/sam3/sam3.1_multiplex.pt`를 가정
- local env에 `sam3`, `pyrealsense2`, `opencv-python`이 준비되어 있어야 함
- `docs/TROUBLE_SHOOTINGS.md`의 Cartesian panel 이슈 메모는 이번 config 리팩터링과 직접 충돌하지 않아 유지
