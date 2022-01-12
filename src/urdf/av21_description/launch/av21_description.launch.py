from launch import LaunchDescription
from launch_ros.actions import Node
from iac_common import get_share_file, get_sim_time_launch_arg


def generate_launch_description():
    declare_use_sim_time_cmd, use_sim_time = get_sim_time_launch_arg()

    urdf = get_share_file("av21_description", "urdf", "av21.urdf")

    robot_desc = None
    with open(urdf, "r") as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "publish_frequency": 1.0,
                "ignore_timestamp": False,
                "use_tf_static": True,
                "robot_description": robot_desc,
            },
            use_sim_time
        ],
    )

    return LaunchDescription([declare_use_sim_time_cmd, robot_state_publisher])
