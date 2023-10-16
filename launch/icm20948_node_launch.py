from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros2_icm20948",
                executable="icm20948_node",
                name="icm20948_node",
                parameters=[
                    {"i2c_address": 0x69},
                    {"frame_id": "imu_icm20948"},
                    {"pub_rate": 50},
                ],
            )
        ]
    )
