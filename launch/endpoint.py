from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                emulate_tty=True,
                parameters=[
                    {"ROS_IP": "0.0.0.0"},
                    {"ROS_TCP_PORT": 10000},
                    # seconds of silence from Unity before its connection is dropped, 0 = never
                    {"TCP_CLIENT_TIMEOUT": 10.0},
                ],
            )
        ]
    )
