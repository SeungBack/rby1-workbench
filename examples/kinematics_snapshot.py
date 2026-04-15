"""Read one robot state and print a few FK results using the library API."""

from __future__ import annotations

import numpy as np

from rby1_workbench import RobotConfig, RobotKinematics, connect_robot


def main() -> None:
    robot = connect_robot(
        RobotConfig(
            address="192.168.12.1:50051",
            model="m",
        )
    )
    state = robot.get_state()
    kinematics = RobotKinematics(robot)
    result = kinematics.compute(np.asarray(state.position, dtype=np.float64))

    print("Head joints:")
    for joint_name in kinematics.head_joint_names:
        print(f"  {joint_name}: {result.joint_positions_by_name[joint_name]:.4f} rad")

    print("\nKey frames in base:")
    for frame_name in ["link_torso_5", "ee_right", "ee_left", "link_head_2"]:
        position = result.base_transforms[frame_name][:3, 3]
        print(f"  {frame_name}: {position}")


if __name__ == "__main__":
    main()
