import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="ldm_fleet_server_ros2",
                namespace="",
                executable="ldm_fleet_server_ros2",
                name="ldm_fleet_server_node",
                output="screen",
                emulate_tty=True,
                respawn=True,
                parameters=[
                    {
                        "fleet_state_topic": "fleet_ldm_state",
                        "lift_request_topic": "lift_ldm_request",
                        "dds_domain": 52,
                        "dds_lift_state_topic": "lift_state",
                        "dds_lift_request_topic": "lift_request",
                        "update_state_frequency": 5.0,
                        "publish_state_frequency": 1.0,
                    }
                ],
            ),
        ]
    )
