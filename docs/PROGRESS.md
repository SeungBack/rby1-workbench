# Progress Log

## 2026-04-15

### Completed

- `rby1-workbench` 프로젝트 골격 생성
- `pyproject.toml`, README, 기본 Hydra config 추가
- geometry / robot / viz 코어 모듈 추가
- `visualize_robot` 앱 추가
- live FK와 frame graph를 Rerun에 로그하는 MVP 구성
- public library API 정리
- console script 추가
- `examples/` 예제 스크립트 추가
- 로컬 pip-installable 패키지 방향으로 문서 정리
- agent handoff용 `docs/AGENT_PROMPT.md` 추가
- viewer no-frame 상황을 위한 initial-state seed와 callback diagnostics 추가
- Python import package를 `rby1`에서 `rby1_workbench`로 변경
- frame XYZ 텍스트를 제거하고 joint 이름 라벨을 표시하도록 변경
- 기존 Rerun viewer에 `viz.log_meshes=true` 옵션 기반 mesh 시각화 추가
- `viser` 기반 joint control panel MVP 추가
- joint command library와 Hydra app entry point 추가
- SDK example 기반 ready pose preset과 ready pose button 추가
- joint control 기본값을 SDK example 기준으로 재조정
- head minimum time을 body와 분리하고, panel send를 non-blocking으로 변경
- ready pose source를 `22_joint_impedance_control.py` 하나로 고정
- panel 입력을 slider에서 jog button 중심으로 재구성
- panel 전송을 공용 `command_stream` 기반으로 통일
- expired command stream 자동 재생성과 jog 즉시 전송 기본값 추가
- panel UI를 component tab + number input 기반으로 압축
- jog/number 입력이 바뀐 component만 보내도록 수정
- jog/number 입력을 worker queue 대신 callback에서 바로 stream 전송하도록 단순화
- joint stream command를 `17_teleoperation_with_joint_mapping.py` 흐름에 더 가깝게 재정렬

### Current State

- 첫 번째 viewer 중심 MVP가 준비된 상태
- frame-first 구조가 잡혀서 calibration과 control을 얹을 기반이 생김
- 문서 관리용 `docs/` 체계 추가
- `Rerun = visualization`, `Viser = input` 방향이 정해짐

### Next Candidate Steps

1. tool / camera mount frame 정의 추가
2. EE target gizmo + cartesian impedance control
3. head camera calibration 데이터 구조 설계
4. config group 분리
5. replay / logging 구조 도입

### Working Rule

앞으로 기능 추가 시 아래 문서를 함께 갱신합니다.

- `README.md`
- `docs/IMPLEMENTATION.md`
- `docs/DESIGN.md`
- `docs/PROGRESS.md`
