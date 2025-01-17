#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    turtlebot3_launch_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )
    world_file = os.path.join(
        get_package_share_directory("maze_runner"),
        "worlds",
        "maze_5x5.world",
    )

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    config_dir = os.path.join(get_package_share_directory("maze_runner"), "config")
    rviz_config = os.path.join(config_dir, "tb3_nav.rviz")
    params_file = os.path.join(config_dir, "tb3_nav_params.yaml")
    map_file = os.path.join(
        get_package_share_directory("maze_runner"), "maps", "maze_5x5_map.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world_file}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    rviz = Node(
        package="rviz2",
        output="screen",
        executable="rviz2",
        name="rviz2_node",
        arguments=["-d", rviz_config],
    )

    maze_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("nav2_bringup"),
                "/launch",
                "/bringup_launch.py",
            ]
        ),
        launch_arguments={
            "map": map_file,
            "params_file": params_file,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    ld.add_action(rviz)
    ld.add_action(maze_nav)

    return ld
