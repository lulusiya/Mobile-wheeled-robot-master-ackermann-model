#!/usr/bin/env python3
"""
用于单次演示的ROS 2启动文件。

这个脚本会加载一个固定的世界（office.world）并将机器人放置在预设的起始位置，
然后执行 task_executor.py 中的预设任务（向前行驶5秒）。

如何运行:
1. colcon build --packages-select simple_robot
2. source install/setup.bash
3. ros2 launch simple_robot demo.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory("simple_robot")

    # ============================ 演示参数 (可修改) ============================
    # 1. 选择一个用于演示的地图文件
    world_file = os.path.join(pkg_path, "worlds", "office.world")

    # 2. 为机器人设置一个固定的起始位置和朝向
    start_x = "0.0"
    start_y = "-2.0"
    start_z = "0.1"
    start_yaw = "1.57"  # 指向正 Y 轴方向 (在办公室地图里是走廊方向)

    # 3. 为机器人设置一个固定的目标位置
    goal_x = "0.0"
    goal_y = "3.0"  # 向前5米
    goal_z = "0.1"
    # ========================================================================

    # 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
        # 将我们设定的世界文件传递给 Gazebo
        launch_arguments={"world": world_file}.items(),
    )

    # 解析 URDF
    xacro_file = os.path.join(pkg_path, "urdf", "robot.urdf.xacro")
    robot_description_config = Command(["xacro ", xacro_file])
    params = {
        "robot_description": robot_description_config,
        "use_sim_time": True,
    }

    # 机器人状态发布器
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "simple_bot",
            "-x",
            start_x,
            "-y",
            start_y,
            "-z",
            start_z,
            "-Y",
            start_yaw,
        ],
        output="screen",
    )

    # 关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Ackermann 驱动控制器
    ackermann_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 任务执行节点 (导航到目标位置)
    task_executor_node = Node(
        package="simple_robot",
        executable="task_executor.py",
        name="task_executor",
        output="screen",
        parameters=[
            {
                "goal_x": float(goal_x),
                "goal_y": float(goal_y),
                "goal_z": float(goal_z),
            }
        ],
    )

    return LaunchDescription(
        [
            gazebo,
            node_robot_state_publisher,
            spawn_entity,
            joint_state_broadcaster_spawner,
            ackermann_steering_controller_spawner,
            task_executor_node,
            # 当任务节点退出时，关闭整个仿真
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=task_executor_node,
                    on_exit=[EmitEvent(event=Shutdown(reason="Demo task completed"))],
                )
            ),
        ]
    )
