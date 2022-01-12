# Copyright (c) 2021-2022 Gaia Platform LLC
# SPDX-License-Identifier: MIT

from launch import LaunchDescription
from launch_ros.actions import Node
from iac_common import get_sim_time_launch_arg


def generate_launch_description():
    declare_use_sim_time_cmd, _ = get_sim_time_launch_arg()

    return LaunchDescription(
        [
            declare_use_sim_time_cmd,
            Node(
                package="lgsvl_mocks",
                executable="lgsvl_mocks_node_exe",
                parameters=[],
                remappings=[
                    # TODO(user): Change these topic names as needed -- they should match the topics
                    # on the real car. The sensor config in SVL should be updated to match as needed.
                    ("wheel_speeds", "/lgsvl/wheel_speeds"),
                    ("gnss_fix", "/lgsvl/gnss_fix"),
                    ("gnss_odom", "/lgsvl/gnss_odom"),
                    ("wheel_speed_report", "/raptor_dbw_ros2/wheel_speed_report"),
                    ("accelerator_pedal", "/raptor_dbw_ros2/accelerator_pedal_cmd"),
                    ("state_report", "/state_report"),
                    ("pt_report", "/raptor_dbw_ros2/pt_report"),
                    ("top_bestvel", "/novatel_top/bestgnssvel"),
                    ("top_bestpos", "/novatel_top/bestgnsspos"),
                    ("top_heading2", "/novatel_top/heading2"),
                    ("top_fix", "/novatel_top/fix"),
                    ("bottom_bestvel", "/novatel_bottom/bestgnssvel"),
                    ("bottom_bestpos", "/novatel_bottom/bestgnsspos"),
                    ("bottom_heading2", "/novatel_bottom/heading2"),
                    ("misc_report", "/raptor_dbw_ros2/misc_report_do"),
                    ("rc_to_ct", "/raptor_dbw_ros2/rc_to_ct"),
                    ("ct_status", "/raptor_dbw_ros2/ct_report"),
                    ("imu", "/lgsvl/imu"),
                    ("top_imu", "/novatel_top/rawimu"),
                    ("bottom_imu", "/novatel_bottom/rawimu"),
                ],
            ),
        ]
    )
