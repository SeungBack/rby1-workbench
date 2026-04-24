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

### Current State

- `rby1-workbench` 안에서 로봇 viewer/control과 별개로 perception 실험을 위한 독립 wrapper 계층이 생김
- live camera stream에서 text, point, box prompt를 받아 SAM3 결과를 바로 overlay로 볼 수 있음
- prompt 입력 UI는 OpenCV에 두고, stream/model 쪽은 재사용 가능한 라이브러리 형태로 분리됨

### Notes

- 현재 text prompt와 geometry prompt는 동일 UI에서 관리되지만, geometry prompt가 있으면 geometry 추론이 우선됨
- 기본 SAM3 checkpoint는 `/home/kimm/Workspaces/sam3/sam3.1_multiplex.pt`를 가정
- local env에 `sam3`, `pyrealsense2`, `opencv-python`이 준비되어 있어야 함
