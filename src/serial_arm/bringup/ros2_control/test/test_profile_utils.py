from pathlib import Path
import sys

import pytest
import yaml

sys.path.append(str(Path(__file__).resolve().parents[1] / "scripts"))
import profile_utils


def test_profile_without_moveit_loads_base_fields(monkeypatch, tmp_path):
    share = tmp_path / "serial_arm_robot_profiles"
    config = share / "config"
    config.mkdir(parents=True)
    (config / "robot_profiles.yaml").write_text(
        yaml.safe_dump(
            {
                "profiles": {
                    "minimal_arm": {
                        "core": {"package": "robot_pkg", "config": "config/core.yaml"},
                        "hardware": {
                            "plugin": "serial_arm_hardware_fake",
                            "config_package": "robot_pkg",
                            "config": "config/hardware.yaml",
                        },
                        "description": {
                            "package": "robot_pkg",
                            "urdf": "model/robot.urdf",
                            "ros2_control_xacro": "model/robot.ros2_control.xacro",
                        },
                        "controllers": {
                            "package": "robot_pkg",
                            "config": "config/ros2_controllers.yaml",
                        },
                    }
                }
            }
        ),
        encoding="utf-8",
    )

    def fake_share(package):
        return str(tmp_path / package)

    monkeypatch.setattr(profile_utils, "get_package_share_directory", fake_share)

    profile = profile_utils.load_profile("minimal_arm")
    assert profile["core_config_path"].endswith("robot_pkg/config/core.yaml")
    assert profile["hardware_plugin"] == "serial_arm_hardware_fake"
    assert profile["hardware_config_path"].endswith("robot_pkg/config/hardware.yaml")
    assert profile["description_urdf_path"].endswith("robot_pkg/model/robot.urdf")
    assert profile["ros2_control_xacro_path"].endswith("robot_pkg/model/robot.ros2_control.xacro")
    assert profile["controllers_path"].endswith("robot_pkg/config/ros2_controllers.yaml")
    assert "moveit_package" not in profile

    with pytest.raises(RuntimeError, match="Robot profile 'minimal_arm' does not define MoveIt support"):
        profile_utils.require_moveit_package(profile, "minimal_arm")
