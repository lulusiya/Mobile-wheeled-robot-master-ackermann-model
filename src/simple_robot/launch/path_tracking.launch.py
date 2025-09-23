#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare(package="simple_robot").find("simple_robot")

    # Default world file
    default_world = os.path.join(pkg_share, "worlds", "empty.world")

    # Read the first two points from path.txt to set robot start position and orientation
    # 获取工作空间根目录下的path.txt文件
    workspace_root = os.getcwd()
    path_file = os.path.join(workspace_root, "path.txt")
    start_x = "0"  # Default values
    start_y = "0"
    start_yaw = "0.0"  # Default yaw

    try:
        # Try to read the first two lines from path.txt
        print(f"Looking for path file at: {path_file}")
        if os.path.exists(path_file):
            with open(path_file, "r") as f:
                lines = f.readlines()
                if len(lines) >= 2:
                    # Parse first point
                    first_line = lines[0].strip()
                    second_line = lines[1].strip()

                    if first_line and second_line:
                        first_parts = first_line.split(",")
                        second_parts = second_line.split(",")

                        if len(first_parts) == 2 and len(second_parts) == 2:
                            try:
                                # Get start position from first point
                                start_x = first_parts[0].strip()
                                start_y = first_parts[1].strip()

                                # Calculate initial yaw from first two points
                                x1, y1 = float(first_parts[0]), float(first_parts[1])
                                x2, y2 = float(second_parts[0]), float(second_parts[1])

                                # Calculate angle from point 1 to point 2
                                import math

                                dx = x2 - x1
                                dy = y2 - y1
                                calculated_yaw = math.atan2(dy, dx)
                                start_yaw = str(calculated_yaw)

                                print(
                                    f"Setting robot start position from path.txt: ({start_x}, {start_y})"
                                )
                                print(
                                    f"Setting robot start yaw from path direction: {calculated_yaw:.3f} rad ({math.degrees(calculated_yaw):.1f}°)"
                                )
                            except ValueError as ve:
                                print(f"Error parsing coordinates: {ve}")
                                print("Using default position and orientation")
                        else:
                            print("Invalid format in path.txt lines")
                            print("Using default position and orientation")
                    else:
                        print("Not enough valid lines in path.txt")
                        print("Using default position and orientation")
                elif len(lines) == 1:
                    # Only one point, use it for position but keep default yaw
                    first_line = lines[0].strip()
                    if first_line:
                        parts = first_line.split(",")
                        if len(parts) == 2:
                            start_x = parts[0].strip()
                            start_y = parts[1].strip()
                            print(
                                f"Setting robot start position from path.txt: ({start_x}, {start_y})"
                            )
                            print("Only one point in path.txt, using default yaw: 0.0")
                else:
                    print("path.txt is empty, using default position and orientation")
        else:
            print(
                f"path.txt not found at {path_file}, using default position ({start_x}, {start_y}) and yaw ({start_yaw})"
            )
    except Exception as e:
        print(
            f"Warning: Could not read path.txt: {e}, using default position ({start_x}, {start_y}) and yaw ({start_yaw})"
        )

    # Declare the world argument
    world_arg = DeclareLaunchArgument(
        "world", default_value=default_world, description="Path to the world file"
    )

    # Declare start position arguments
    start_x_arg = DeclareLaunchArgument(
        "start_x", default_value=start_x, description="Robot start X position"
    )
    start_y_arg = DeclareLaunchArgument(
        "start_y", default_value=start_y, description="Robot start Y position"
    )
    start_z_arg = DeclareLaunchArgument(
        "start_z", default_value="0.1", description="Robot start Z position"
    )
    start_yaw_arg = DeclareLaunchArgument(
        "start_yaw", default_value=start_yaw, description="Robot start yaw orientation"
    )

    # Path to the original robot launch file
    robot_launch = os.path.join(pkg_share, "launch", "robot.launch.py")

    # Include the robot launch file
    robot_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "start_x": LaunchConfiguration("start_x"),
            "start_y": LaunchConfiguration("start_y"),
            "start_z": LaunchConfiguration("start_z"),
            "start_yaw": LaunchConfiguration("start_yaw"),
        }.items(),
    )

    # Add a delay before starting path follower to ensure robot is fully initialized
    path_follower_node = ExecuteProcess(
        cmd=["bash", "-c", "sleep 5 && ros2 run simple_robot path_follower.py"],
        output="screen",
        shell=False,
    )

    return LaunchDescription(
        [
            world_arg,
            start_x_arg,
            start_y_arg,
            start_z_arg,
            start_yaw_arg,
            robot_launch_include,
            path_follower_node,
        ]
    )
