import os
import launch
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robots_env = os.getenv("ROBOT_INFO", "")
    if not robots_env:
        print("ROBOT_INFO env is not set")
        print("example: ROBOT_INFO=lino2:13a5")
        print("  robot is defined by 2 arguments which is separated by :")
        print("    arg1: robot_type\n    arg2: robot_sn")
        return LaunchDescription([])

    robot_first = [e.split(":") for e in robots_env.split(";")][0]
    robot_type, robot_sn = robot_first[0], robot_first[1]
    namespace = f"/{robot_type}_{robot_sn}"

    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="true | false (default)",
    )
    check_robot_status_node = launch_ros.actions.Node(
        package="fitrobotcpp",
        executable="check_robot_status",
        name="check_robot_status_node",
        output="screen",
        namespace=namespace,
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )
    master_node = launch_ros.actions.Node(
        package="fitrobotcpp",
        executable="master_node",
        name="master_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("sim")},
        ],
    )

    return LaunchDescription(
        [
            sim_arg,
            check_robot_status_node,
            master_node,
        ]
    )
