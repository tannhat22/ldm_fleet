import os

from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("ldm_server_ros2"), "config.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="ldm_server_ros2",
                namespace="",
                executable="lift_server",
                name="lift_service",
                output="screen",
                emulate_tty=True,
                respawn=True,
                arguments=["-c", config_file],
            ),
            Node(
                package="ldm_server_ros2",
                namespace="",
                executable="lift_state_update",
                name="lift_state_update",
                output="screen",
                emulate_tty=True,
                respawn=True,
                arguments=["-c", config_file],
            ),
            Node(
                package="ldm_fleet_client_ros2",
                namespace="",
                executable="ldm_fleet_client_ros2",
                name="ldm_fleet_client_node",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[
                    {
                        "lift_name": "LIFT-TP2",
                        "lift_state_topic": "/lift_state",
                        "lift_trigger_server_name": "",
                        "dds_domain": 52,
                        "dds_state_topic": "lift_state",
                        "dds_lift_request_topic": "lift_request",
                        "update_frequency": 5.0,
                        "publish_frequency": 1.0,
                    }
                ],
            ),
        ]
    )
