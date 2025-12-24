# Copyright (c) 2025 C.K. Robotics

import os
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def find_exe_preferring_local(local_exe, installed_exe):
    return local_exe if os.path.isfile(local_exe) else installed_exe


def get_share_file(package_name, *args):
    return os.path.join(get_package_share_directory(package_name), *args)


def get_sim_time_launch_arg():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False", description="Use simulation clock if True"
    )

    return declare_use_sim_time_cmd, {"use_sim_time": use_sim_time}


def to_lower(launch_configuration):
    return PythonExpression(["'", launch_configuration, "'", ".lower()"])


def check_val_in_list(val, list_to_check):
    return PythonExpression(["'", LaunchConfiguration(val), "' in ", str(list_to_check)])


def get_param_file(package_name, param_base=None, launch_config_var="race_type", prefix=False):
    if param_base is None:
        param_base = package_name
    if prefix:
        config = (
            get_share_file(package_name),
            "/param/",
            to_lower(LaunchConfiguration(launch_config_var)),
            "/",
            f"{param_base}.param.yaml",
        )
    else:
        config = (
            get_share_file(package_name),
            f"/param/{param_base}_",
            to_lower(LaunchConfiguration(launch_config_var)),
            ".param.yaml",
        )
    return config
