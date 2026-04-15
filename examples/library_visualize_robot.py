"""Programmatic example for launching the live robot viewer as a library."""

from rby1_workbench import (
    RobotConfig,
    VisualizeRobotConfig,
    VizConfig,
    run_visualize_robot,
)


def main() -> None:
    cfg = VisualizeRobotConfig(
        robot=RobotConfig(
            address="192.168.12.1:50051",
            model="m",
            state_update_hz=10.0,
        ),
        viz=VizConfig(
            application_id="rby1_library_example",
            spawn_viewer=True,
        ),
    )
    run_visualize_robot(cfg)


if __name__ == "__main__":
    main()
