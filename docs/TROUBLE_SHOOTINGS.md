

WIFI RBY1이 느릴때 

sudo vi /etc/NetworkManager/conf.d/default-wifi-powersave-on.conf

wifi.powersave = 3을 2로 변경하고 Network Manager 다시 시작

sudo systemctl restart NetworkManager

## Cartesian Control Notes

### Confirmed Facts

- `examples/minimal_right_cartesian_stream.py` 는 실제 로봇에서 작동함.
- 즉, `rby1_workbench` 안에서도 `cartesian_command_stream.py` 에 가까운 최소 single-arm Cartesian stream 경로는 유효함.
- 반대로 `viser_joint_control_panel` 에서의 Cartesian control 은 한동안 전혀 동작하지 않았고, 이후에는 `두 번 클릭해야 작동` 하는 상태까지 개선됨.
- 따라서 현재 문제는 `RB-Y1 Cartesian 자체`가 아니라 `panel integration / mode transition / UI event ordering` 쪽일 가능성이 큼.

### What Helped

- Cartesian 경로를 generic multi-arm builder보다 공식 예제에 가까운 `single-arm stream command` 로 단순화하는 것이 효과적이었음.
- `right_arm` / `left_arm` Cartesian은 `BodyComponentBasedCommandBuilder().set_<arm>_command(CartesianCommandBuilder()...)` 형태가 핵심임.
- official stream example과 비슷하게 elbow hint를 주는 전용 경로를 추가함.
  - right: `add_joint_position_target("right_arm_2", 0.5, 1, 100)`
  - left: `add_joint_position_target("left_arm_2", -0.5, 1, 100)`
- Cartesian 기본값을 예제 쪽으로 완화했을 때가 더 나았음.
  - `cartesian_angular_velocity_limit = 100.0`
  - `stop_position_tracking_error = 1e-3`
  - `stop_orientation_tracking_error = 1e-4`
- `minimal_right_cartesian_stream.py` 같은 UI 없는 최소 재현 예제를 두는 것이 매우 중요했음.

### What Did Not Help Or Was Risky

- Cartesian command에 현재 arm의 모든 joint를 `add_joint_position_target(...)`로 넣는 방식
  - 예제의 “힌트” 수준을 넘어 arm을 사실상 고정할 수 있음.
- Cartesian를 joint teleop처럼 periodic resend loop로 다루는 방식
  - 공식 Cartesian 예제는 one-shot send + feedback 관찰 쪽에 더 가까움.
- joint/controller와 Cartesian controller가 각자 별도 stream을 만들거나,
  반대로 동일 stream에서 lifecycle을 분리하지 않고 섞는 방식 모두 주의가 필요함.
- UI에서 mode 전환과 target sync를 같은 callback 안에서 섞으면
  첫 클릭이 `mode switch / sync`에 소모되고 두 번째 클릭에서야 실제 target send가 일어날 수 있음.

### Current Unresolved Symptom

- `viser_joint_control_panel` 에서 Cartesian jog/apply가 `두 번 클릭해야` 실제 로봇에 반영됨.
- 이 문제는 minimal Cartesian example에는 없음.
- 따라서 unresolved area는 아래 중 하나일 가능성이 큼.
  - programmatic dropdown 변경과 GUI callback의 재진입 / 이벤트 순서
  - 첫 클릭에서 `sync_from_robot()` 가 새 Cartesian target을 덮어쓰는 흐름
  - mode transition 시 stream reset 이후 첫 send가 UI state보다 먼저 발생하는 흐름
  - `send_on_update` 와 `body control mode` 전환이 같은 이벤트에서 중첩되는 문제

### Recommended Next Debugging Order

1. `viser` callback에서 첫 클릭 시 실제 호출 순서를 로그로 남긴다.
   - `_set_body_control_mode`
   - `_apply_stream_mode`
   - `sync_from_robot`
   - `jog_position` / `jog_orientation`
   - `send_once`
2. 첫 클릭 직전/직후 Cartesian target translation 값을 로그로 남긴다.
3. 첫 클릭에서 target이 바뀌었는데 send된 command가 old target인지 확인한다.
4. 필요하면 Cartesian callback에서
   - mode 전환
   - sync
   - jog
   - send
   를 하나의 direct path로 묶고, dropdown UI state update는 마지막에 반영한다.
5. UI layer가 계속 꼬이면, Cartesian panel path를 `minimal_right_cartesian_stream.py` 호출과 최대한 동일하게 맞춘다.
