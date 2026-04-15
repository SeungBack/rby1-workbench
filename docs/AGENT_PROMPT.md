# Agent Prompt

Last updated: 2026-04-15

이 문서는 다른 code agent가 `rby1-workbench` 작업을 이어받을 때 참고하는 프로젝트 프롬프트입니다.
필요하면 그대로 복사해서 시스템/개발자 프롬프트 아래에 붙여 넣어도 됩니다.

## Copy-Paste Prompt

```md
You are working on `rby1-workbench`, a local pip-installable RB-Y1 robotics library in `/home/kimm/Workspaces/rby1-workbench`.

Project intent:
- Provide a reusable internal library for RB-Y1 visualization, coordinate frames, calibration, and control apps.
- Prioritize frame tracking, robot visualization, and calibration infrastructure over flashy UI or premature app complexity.
- This is not currently intended for online package publishing. Treat it as a local library installed with `pip install -e .` or `pip install .`.

Current architecture:
- `src/rby1_workbench/apps`: thin CLI/app entrypoints
- `src/rby1_workbench/config`: structured config/dataclasses
- `src/rby1_workbench/conf`: Hydra YAML configs
- `src/rby1_workbench/geometry`: SE3 and transform graph utilities
- `src/rby1_workbench/robot`: robot connection, state buffering, naming, FK
- `src/rby1_workbench/viz`: Rerun session and reusable viewer logic
- `examples/`: reference examples for library usage
- `docs/`: README-adjacent project memory

Naming:
- repository name: `rby1-workbench`
- Python import package: `rby1_workbench`
- console script: `rby1-visualize-robot`

Current implemented MVP:
- Live robot state buffer
- SDK dynamics-based forward kinematics
- Named frame graph construction
- Rerun frame and skeleton visualization
- Reusable `run_visualize_robot(...)` library function
- CLI app `rby1-visualize-robot`

Key design rules:
- Frame-first: preserve a stable frame vocabulary such as `base`, `link_torso_*`, `link_right_arm_*`, `link_left_arm_*`, `link_head_*`, `ee_right`, `ee_left`.
- Thin apps, thick library: reusable logic belongs in library modules, not inside CLI scripts.
- Safe defaults: visualization/read-only apps must not power on, servo on, or enable control manager unless explicitly configured.
- Prefer named interfaces over raw index slicing whenever possible.
- Keep the package locally pip-installable.

Documentation rules:
- Whenever you make meaningful code changes, also update the relevant docs:
  - `README.md`
  - `docs/IMPLEMENTATION.md`
  - `docs/DESIGN.md`
  - `docs/PROGRESS.md`
- If the change is specifically useful for future agents, update `docs/AGENT_PROMPT.md` too.

Current priorities:
1. Frame and transform correctness
2. State visibility and debugging
3. Calibration data flow
4. Control abstraction

Likely next work:
- Add tool/camera mount frames
- Add head camera calibration data structures
- Add replay/logging structure
- Split Hydra configs into groups
- Migrate control examples into reusable library modules

Before making risky structural changes:
- Check existing docs first.
- Preserve the frame-first design.
- Avoid pushing logic back into giant app scripts.
```

## Human Summary

### What This File Is For

- 다른 agent가 프로젝트 방향을 빠르게 이해하게 하기 위한 문서
- 작업을 재개할 때 문맥 손실을 줄이기 위한 문서
- 문서와 코드의 현재 상태를 연결하는 작업 메모

### What Agents Should Read Together

- [README.md](/home/kimm/Workspaces/rby1-workbench/README.md)
- [IMPLEMENTATION.md](/home/kimm/Workspaces/rby1-workbench/docs/IMPLEMENTATION.md)
- [DESIGN.md](/home/kimm/Workspaces/rby1-workbench/docs/DESIGN.md)
- [PROGRESS.md](/home/kimm/Workspaces/rby1-workbench/docs/PROGRESS.md)

### Agent Workflow

1. Read the current docs first.
2. Inspect the relevant module under `src/rby1_workbench/`.
3. Keep app entrypoints thin.
4. Put reusable logic into library modules.
5. Update docs after meaningful changes.

### When To Update This Prompt

- 새로운 큰 설계 원칙이 생겼을 때
- 작업 우선순위가 바뀌었을 때
- 다른 agent가 헷갈릴 만한 구조 변경이 들어갔을 때
