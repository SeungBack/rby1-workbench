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

### Current State

- 첫 번째 viewer 중심 MVP가 준비된 상태
- frame-first 구조가 잡혀서 calibration과 control을 얹을 기반이 생김
- 문서 관리용 `docs/` 체계 추가

### Next Candidate Steps

1. tool / camera mount frame 정의 추가
2. head camera calibration 데이터 구조 설계
3. config group 분리
4. replay / logging 구조 도입
5. control app 이전 계획 수립

### Working Rule

앞으로 기능 추가 시 아래 문서를 함께 갱신합니다.

- `README.md`
- `docs/IMPLEMENTATION.md`
- `docs/DESIGN.md`
- `docs/PROGRESS.md`
