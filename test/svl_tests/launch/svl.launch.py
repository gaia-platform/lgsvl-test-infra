# Copyright (c) 2021-2022 Gaia Platform LLC
# SPDX-License-Identifier: MIT

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from iac_common import get_share_file, get_sim_time_launch_arg


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()
    default_launch_arguments = use_sim_time.items()

    av21_urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file("av21_description", "launch", "av21_description.launch.py")
        ),
        launch_arguments=default_launch_arguments,
    )

    lgsvl_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file("svl_tests", "launch", "lgsvl_interface.launch.py")
        ),
        launch_arguments=default_launch_arguments,
    )

    lgsvl_mocks_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file("lgsvl_mocks", "launch", "lgsvl_mocks.launch.py")
        ),
        launch_arguments=default_launch_arguments,
    )

    # TODO(user): Add the rest of your vehicle launch code here.

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            av21_urdf_launch,
            lgsvl_interface_launch,
            lgsvl_mocks_launch,
            # TODO(user): Any other vehicle code -- sensor fusion, telemetry, controller, etc.
        ]
    )
