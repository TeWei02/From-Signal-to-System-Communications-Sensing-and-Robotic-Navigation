"""
multi_robot_explore.launch.py
------------------------------
ROS2 launch file for multi-robot exploration with simulated communication constraints.

Spawns N robots, each in its own namespace (/robot_0, /robot_1, …), all sharing a
single Gazebo world.  Each robot runs a full perception/SLAM/planning stack.  A shared
base-station node receives map diffs over the link simulator.

Key parameters:
  n_robots          : number of robots to spawn (default 2)
  world             : Gazebo world name
  bandwidth_bps     : uplink bandwidth budget per robot
  base_latency_ms   : one-way link latency
  packet_loss_rate  : per-packet drop probability
  scenario_file     : path to exploration_scenarios.yaml
"""

# pyright: reportMissingImports=false, reportMissingModuleSource=false, reportUnknownMemberType=false, reportUnknownVariableType=false, reportUnknownArgumentType=false, reportUnknownParameterType=false, reportMissingParameterType=false

from launch import LaunchDescription  # type: ignore
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def spawn_robot_group(context, *args, **kwargs):
    """OpaqueFunction that dynamically creates N robot groups."""
    n_robots = int(LaunchConfiguration("n_robots").perform(context))
    robot_params = LaunchConfiguration("robot_params").perform(context)
    sensor_config = LaunchConfiguration("sensor_config").perform(context)
    comms_qos = LaunchConfiguration("comms_qos_file").perform(context)
    bw = LaunchConfiguration("bandwidth_bps").perform(context)
    lat = LaunchConfiguration("base_latency_ms").perform(context)
    loss = LaunchConfiguration("packet_loss_rate").perform(context)

    groups = []
    for i in range(n_robots):
        ns = f"robot_{i}"
        robot_nodes = [
            Node(
                package="from_signal_to_system",
                executable="lidar_preprocess_node",
                name="lidar_preprocess",
                parameters=[sensor_config, {"robot_id": i}],
            ),
            Node(
                package="from_signal_to_system",
                executable="imu_fusion_node",
                name="imu_fusion",
                parameters=[sensor_config, {"robot_id": i}],
            ),
            Node(
                package="from_signal_to_system",
                executable="slam_frontend_node",
                name="slam_frontend",
                parameters=[robot_params, {"robot_id": i}],
            ),
            Node(
                package="from_signal_to_system",
                executable="occupancy_grid_mapper",
                name="occupancy_grid_mapper",
                parameters=[robot_params],
            ),
            Node(
                package="from_signal_to_system",
                executable="costmap_generator",
                name="costmap_generator",
                parameters=[robot_params],
            ),
            Node(
                package="from_signal_to_system",
                executable="global_planner_node",
                name="global_planner",
                parameters=[robot_params],
            ),
            Node(
                package="from_signal_to_system",
                executable="local_planner_node",
                name="local_planner",
                parameters=[robot_params],
            ),
            Node(
                package="from_signal_to_system",
                executable="motion_controller",
                name="motion_controller",
                parameters=[robot_params],
            ),
            Node(
                package="from_signal_to_system",
                executable="edge_client_node",
                name="edge_client",
                parameters=[
                    comms_qos,
                    {
                        "robot_id": i,
                        "bandwidth_bps": int(bw),
                        "base_latency_ms": float(lat),
                        "packet_loss_rate": float(loss),
                    },
                ],
            ),
        ]
        groups.append(
            GroupAction(actions=[PushRosNamespace(ns)] + robot_nodes)
        )
    return groups


def generate_launch_description():
    pkg_share = FindPackageShare("from_signal_to_system")

    declare_n_robots = DeclareLaunchArgument("n_robots", default_value="2")
    declare_world = DeclareLaunchArgument("world", default_value="warehouse_world")
    declare_headless = DeclareLaunchArgument("headless", default_value="false")
    declare_bw = DeclareLaunchArgument("bandwidth_bps", default_value="500000")
    declare_lat = DeclareLaunchArgument("base_latency_ms", default_value="30.0")
    declare_loss = DeclareLaunchArgument("packet_loss_rate", default_value="0.02")
    declare_scenario = DeclareLaunchArgument(
        "scenario_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "exploration_scenarios.yaml"]
        ),
    )
    declare_qos = DeclareLaunchArgument(
        "comms_qos_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "comms_qos_profiles.yaml"]
        ),
    )
    declare_robot_params = DeclareLaunchArgument(
        "robot_params",
        default_value=PathJoinSubstitution([pkg_share, "config", "robot_params.yaml"]),
    )
    declare_sensor_config = DeclareLaunchArgument(
        "sensor_config",
        default_value=PathJoinSubstitution([pkg_share, "config", "sensors.yaml"]),
    )

    base_station = Node(
        package="from_signal_to_system",
        executable="base_station_node",
        name="base_station",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "comms_qos_profiles.yaml"])
        ],
    )

    return LaunchDescription(
        [
            declare_n_robots,
            declare_world,
            declare_headless,
            declare_bw,
            declare_lat,
            declare_loss,
            declare_scenario,
            declare_qos,
            declare_robot_params,
            declare_sensor_config,
            base_station,
            OpaqueFunction(function=spawn_robot_group),
        ]
    )
