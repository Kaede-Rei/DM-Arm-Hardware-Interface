from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def load_profile(robot_profile):
    config_dir = Path(get_package_share_directory("dm_arm_bringup")) / "config"
    profiles_file = config_dir / "robot_profiles.yaml"
    with profiles_file.open("r", encoding="utf-8") as stream:
        profiles = yaml.safe_load(stream)["profiles"]
    if robot_profile not in profiles:
        available = ", ".join(sorted(profiles.keys()))
        raise RuntimeError(f"robot_profile must be one of: {available}")

    profile = dict(profiles[robot_profile])
    profile["core_config_path"] = str(config_dir / profile["core_config"])
    profile["controllers_path"] = str(config_dir / "ros2_controllers.yaml")
    return profile
