"""Programmatic example for launching the viser joint control panel."""

from rby1_workbench import (
    JointControlConfig,
    RobotConfig,
    ViserConfig,
    ViserJointControlAppConfig,
)
from rby1_workbench.control.viser_joint_control import run_viser_joint_control_panel


def main() -> None:
    cfg = ViserJointControlAppConfig(
        robot=RobotConfig(
            address="192.168.12.1:50051",
            model="m",
        ),
        viser=ViserConfig(
            port=8080,
            title="RB-Y1 Joint Control Example",
        ),
        command=JointControlConfig(
            enable_head=True,
            enable_torso=False,
            enable_right_arm=False,
            enable_left_arm=False,
            send_on_update=False,
        ),
    )
    run_viser_joint_control_panel(cfg)


if __name__ == "__main__":
    main()
