from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def load_profile(robot_profile):
    config_dir = Path(get_package_share_directory("serial_arm_robot_profiles")) / "config"
    profiles_file = config_dir / "robot_profiles.yaml"
    with profiles_file.open("r", encoding="utf-8") as stream:
        profiles = yaml.safe_load(stream)["profiles"]
    if robot_profile not in profiles:
        available = ", ".join(sorted(profiles.keys()))
        raise RuntimeError(f"robot_profile must be one of: {available}")

    profile = dict(profiles[robot_profile])
    profile["core_config_path"] = resolve_package_path(profile["core"]["package"], profile["core"]["config"])
    profile["hardware_plugin"] = profile["hardware"]["plugin"]
    profile["hardware_config_path"] = resolve_package_path(profile["hardware"]["config_package"], profile["hardware"]["config"])
    profile["description_xacro_path"] = resolve_package_path(profile["description"]["package"], profile["description"]["xacro"])
    profile["ros2_control_xacro_path"] = resolve_package_path(profile["description"]["package"], profile["description"]["ros2_control_xacro"])
    profile["controllers_path"] = resolve_package_path(profile["controllers"]["package"], profile["controllers"]["config"])
    profile["moveit_package"] = profile["moveit"]["package"]
    return profile


def resolve_package_path(package, relative_path):
    return str(Path(get_package_share_directory(package)) / relative_path)
