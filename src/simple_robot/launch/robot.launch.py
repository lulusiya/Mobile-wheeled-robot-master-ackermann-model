import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():

    # Get the package path
    pkg_path = get_package_share_directory("simple_robot")
    world_path = os.path.join(pkg_path, "worlds")

    # Launch configuration variables
    world = LaunchConfiguration("world")

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_path, "worlds", "empty.world"),
        description="Full path to the world file to load",
    )

    # 新增：声明 headless 参数，用于控制是否启动Gazebo GUI
    declare_headless_cmd = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Whether to run Gazebo in headless mode (no GUI)",
    )

    # 声明机器人起始位置参数，您可以在 run_experiments.py 中覆盖这些默认值
    declare_start_x_cmd = DeclareLaunchArgument(
        "start_x", default_value="0.0", description="Robot start X position"
    )
    declare_start_y_cmd = DeclareLaunchArgument(
        "start_y", default_value="0.0", description="Robot start Y position"
    )
    declare_start_z_cmd = DeclareLaunchArgument(
        "start_z", default_value="0.1", description="Robot start Z position"
    )
    declare_start_yaw_cmd = DeclareLaunchArgument(
        "start_yaw", default_value="0.0", description="Robot start Yaw orientation"
    )

    # 声明机器人目标位置参数
    declare_goal_x_cmd = DeclareLaunchArgument(
        "goal_x", default_value="5.0", description="Robot goal X position"
    )
    declare_goal_y_cmd = DeclareLaunchArgument(
        "goal_y", default_value="0.0", description="Robot goal Y position"
    )
    declare_goal_z_cmd = DeclareLaunchArgument(
        "goal_z", default_value="0.1", description="Robot goal Z position"
    )

    # Start Gazebo
    # 当 headless 为 'true' 时，仅启动 gzserver
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gzserver.launch.py",
            ]
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
        condition=IfCondition(LaunchConfiguration("headless")),
    )

    # 当 headless 为 'false' 时 (默认情况)，启动完整的 Gazebo (gzserver + gzclient)
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    # Parse URDF
    xacro_file = os.path.join(pkg_path, "urdf", "robot.urdf.xacro")
    robot_description_config = Command(["xacro ", xacro_file])
    params = {
        "robot_description": robot_description_config,
        "use_sim_time": True,
    }

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # Spawn Robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        # 将机器人描述、实体名称和上面声明的位置参数传递给 spawner
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "simple_bot",
            "-x",
            LaunchConfiguration("start_x"),
            "-y",
            LaunchConfiguration("start_y"),
            "-z",
            LaunchConfiguration("start_z"),
            "-Y",
            LaunchConfiguration("start_yaw"),
        ],
        output="screen",
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Ackermann Drive Controller
    ackermann_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Obstacle Controller for dynamic obstacles
    obstacle_controller = Node(
        package="simple_robot",
        executable="move_obstacles.py",
        name="obstacle_controller",
        output="screen",
    )

    return LaunchDescription(
        [
            declare_world_cmd,
            declare_headless_cmd,  # 添加 headless 参数到启动描述
            # 声明机器人起始位置参数，您可以在 run_experiments.py 中覆盖这些默认值
            declare_start_x_cmd,
            declare_start_y_cmd,
            declare_start_z_cmd,
            declare_start_yaw_cmd,
            # 声明机器人目标位置参数
            declare_goal_x_cmd,
            declare_goal_y_cmd,
            declare_goal_z_cmd,
            # 根据 headless 参数选择性启动 Gazebo
            gzserver_cmd,
            gazebo_cmd,
            node_robot_state_publisher,
            spawn_entity,
            joint_state_broadcaster_spawner,
            ackermann_steering_controller_spawner,
            obstacle_controller,
        ]
    )
