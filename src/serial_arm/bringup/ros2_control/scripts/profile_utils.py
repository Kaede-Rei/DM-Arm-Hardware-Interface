from pathlib import Path

import yaml


def get_package_share_directory(package):
    from ament_index_python.packages import get_package_share_directory as resolve_share

    return resolve_share(package)


def load_profile(robot_profile):
    config_dir = Path(get_package_share_directory("serial_arm_robot_profiles")) / "config"
    profiles_file = config_dir / "robot_profiles.yaml"
    with profiles_file.open("r", encoding="utf-8") as stream:
        profiles = yaml.safe_load(stream)["profiles"]
    if robot_profile not in profiles:
        available = ", ".join(sorted(profiles.keys()))
        raise RuntimeError(f"robot_profile must be one of: {available}")

    profile = dict(profiles[robot_profile])
    core_profile = load_core_profile(robot_profile, str(profiles_file))
    profile["core_config_path"] = core_profile.core_config_path
    profile["hardware_plugin"] = core_profile.hardware_plugin
    profile["hardware_config_path"] = core_profile.hardware_config_path
    profile["description_urdf_path"] = resolve_package_path(profile["description"]["package"], profile["description"]["urdf"])
    profile["ros2_control_xacro_path"] = resolve_package_path(profile["description"]["package"], profile["description"]["ros2_control_xacro"])
    profile["controllers_path"] = resolve_package_path(profile["controllers"]["package"], profile["controllers"]["config"])
    moveit = profile.get("moveit")
    if moveit and moveit.get("package"):
        profile["moveit_package"] = moveit["package"]
    return profile


def load_core_profile(robot_profile, profiles_file):
    try:
        from serial_arm import load_robot_profile_core
    except ImportError as error:
        raise RuntimeError("serial_arm Python binding is required to resolve Core Robot Profile fields") from error
    return load_robot_profile_core(robot_profile, profiles_file)


def require_moveit_package(profile, robot_profile):
    moveit = profile.get("moveit")
    if not moveit or not moveit.get("package"):
        raise RuntimeError(f"Robot profile '{robot_profile}' does not define MoveIt support")
    return moveit["package"]


def resolve_package_path(package, relative_path):
    return str(Path(get_package_share_directory(package)) / relative_path)
