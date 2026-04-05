"""
single_robot_nav.launch.py
--------------------------
ROS2 launch file for single-robot local navigation in a Gazebo world.

Brings up:
  - Gazebo simulation (world file from sim/world_models/)
  - Robot state publisher (URDF/XACRO)
  - LiDAR preprocessing node
  - IMU fusion node
  - SLAM front-end node
  - Occupancy grid mapper
  - Costmap generator
  - Global + local planner
  - Motion controller
  - RViz2 visualizer (nav_debug.rviz)
"""

# pyright: reportMissingImports=false, reportMissingModuleSource=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false, reportUnknownParameterType=false, reportMissingParameterType=false, reportUnusedImport=false

from launch import LaunchDescription  # type: ignore
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    pkg_share = FindPackageShare("from_signal_to_system")

    # --------------- Launch Arguments ---------------
    declare_world = DeclareLaunchArgument(
        "world",
        default_value="office_world",
        description="Gazebo world name (without extension).",
    )
    declare_headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without GUI.",
    )
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        description="ROS2 log level.",
    )
    declare_robot_params = DeclareLaunchArgument(
        "robot_params",
        default_value=PathJoinSubstitution([pkg_share, "config", "robot_params.yaml"]),
        description="Path to robot_params.yaml.",
    )
    declare_sensor_config = DeclareLaunchArgument(
        "sensor_config",
        default_value=PathJoinSubstitution([pkg_share, "config", "sensors.yaml"]),
        description="Path to sensors.yaml.",
    )

    # --------------- Robot Description ---------------
    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution([pkg_share, "urdf", "mobile_base.urdf.xacro"]),
        ]
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # --------------- Gazebo ---------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch/gazebo.launch.py"]
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [pkg_share, "sim", "world_models",
                 [LaunchConfiguration("world"), ".sdf"]]
            ),
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    # --------------- Perception Nodes ---------------
    lidar_preprocess = Node(
        package="from_signal_to_system",
        executable="lidar_preprocess_node",
        name="lidar_preprocess",
        parameters=[LaunchConfiguration("sensor_config")],
        remappings=[
            ("scan_raw", "/robot/scan/raw"),
            ("scan_filtered", "/robot/scan/filtered"),
        ],
    )
    imu_fusion = Node(
        package="from_signal_to_system",
        executable="imu_fusion_node",
        name="imu_fusion",
        parameters=[LaunchConfiguration("sensor_config")],
    )
    slam_frontend = Node(
        package="from_signal_to_system",
        executable="slam_frontend_node",
        name="slam_frontend",
        parameters=[LaunchConfiguration("robot_params")],
    )

    # --------------- Mapping Nodes ---------------
    occ_mapper = Node(
        package="from_signal_to_system",
        executable="occupancy_grid_mapper",
        name="occupancy_grid_mapper",
        parameters=[LaunchConfiguration("robot_params")],
    )
    costmap_gen = Node(
        package="from_signal_to_system",
        executable="costmap_generator",
        name="costmap_generator",
        parameters=[LaunchConfiguration("robot_params")],
    )

    # --------------- Planning Nodes ---------------
    global_planner = Node(
        package="from_signal_to_system",
        executable="global_planner_node",
        name="global_planner",
        parameters=[LaunchConfiguration("robot_params")],
    )
    local_planner = Node(
        package="from_signal_to_system",
        executable="local_planner_node",
        name="local_planner",
        parameters=[LaunchConfiguration("robot_params")],
    )

    # --------------- Controller ---------------
    motion_controller = Node(
        package="from_signal_to_system",
        executable="motion_controller",
        name="motion_controller",
        parameters=[LaunchConfiguration("robot_params")],
    )

    # --------------- RViz2 ---------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_share, "rviz", "nav_debug.rviz"]),
        ],
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    return LaunchDescription(
        [
            declare_world,
            declare_headless,
            declare_log_level,
            declare_robot_params,
            declare_sensor_config,
            robot_state_publisher,
            gazebo,
            lidar_preprocess,
            imu_fusion,
            slam_frontend,
            occ_mapper,
            costmap_gen,
            global_planner,
            local_planner,
            motion_controller,
            rviz,
        ]
    )
