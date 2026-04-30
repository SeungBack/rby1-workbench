"""로봇 상태 및 모델 정보 확인 예제.

실행:
    python examples/check_robot_status.py
    python examples/check_robot_status.py --config /path/to/custom_rby1.yaml
"""

from __future__ import annotations

import argparse
import re

from rby1_workbench import RBY1, load_rby1_config


def main() -> None:
    parser = argparse.ArgumentParser(description="로봇 상태 및 모델 정보 확인")
    parser.add_argument("--config", default=None, help="rby1 YAML 설정 파일 경로")
    args = parser.parse_args()

    cfg = load_rby1_config(args.config)
    robot = RBY1(cfg)
    robot.connect()

    # ── 로봇 기본 정보 ────────────────────────────────────────────────
    info = robot.sdk_robot.get_robot_info()
    print(f"model_name:    {info.robot_model_name}")
    print(f"model_version: {info.robot_model_version}")

    # ── URDF 내용에서 직접 확인 ────────────────────────────────────────
    urdf_xml = robot.sdk_robot.get_robot_model()

    match = re.search(r'<robot[^>]+name="([^"]+)"', urdf_xml)
    print(f"urdf name:     {match.group(1) if match else 'unknown'}")

    match = re.search(r'joint name="head_0".*?xyz="([^"]+)"', urdf_xml, re.DOTALL)
    if match:
        print(f"head_0 xyz:    {match.group(1)}  (z=0.02→구버전, z=0.08→v1.2+)")


if __name__ == "__main__":
    main()
