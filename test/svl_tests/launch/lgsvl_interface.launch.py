# Copyright (c) 2021-2022 Gaia Platform LLC
# SPDX-License-Identifier: MIT

from launch import LaunchDescription
from launch_ros.actions import Node
from iac_common import get_share_file, get_sim_time_launch_arg


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()

    lgsvl_interface = Node(
        package="lgsvl_interface",
        executable="lgsvl_interface_exe",
        output="screen",
        parameters=[
            get_share_file("svl_tests", "param", "lgsvl_interface.param.yaml"),
            use_sim_time,
        ],
        remappings=[
            # TODO(user): Change these to the topic names used by your controller
            # for communicating with the `ne_raptor_interface`.
            ("raw_command", "user/raw_command"),
            ("state_command", "user/vehicle_state_command"),
        ],
    )

    return LaunchDescription([declare_use_sim_time_cmd, lgsvl_interface])
