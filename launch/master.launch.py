import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


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
    check_robot_status_node = Node(
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
    rosbridge_ws_node = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml",
            )
        )
    )
    rosboard_node = Node(
        package='rosboard',
        executable='rosboard_node',
    )
    master_node = Node(
        package="fitrobotcpp",
        executable="master_node",
        name="master_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("sim")},
        ],
    )

    # waypoint_follower_node = Node(
    #     package='fitrobot',
    #     executable='waypoint_follower',
    # )

    return LaunchDescription(
        [
            check_robot_status_node,
            rosbridge_ws_node,
            rosboard_node,
            sim_arg,
            master_node,
            # waypoint_follower_node,
        ]
    )
