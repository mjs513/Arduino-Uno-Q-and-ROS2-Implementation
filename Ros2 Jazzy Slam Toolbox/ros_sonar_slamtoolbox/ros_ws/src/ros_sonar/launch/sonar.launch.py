from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros_sonar",
            executable="sonar_node.py",
            name="sonar_node",
            output="screen",
            parameters=[{
                "min_angle": 0.0,
                "max_angle": 180.0,
                "step": 2.0,
                "settle_sec": 0.15,
                "period_sec": 6.0,
            }],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.10", "0.0", "0.05",
                "0", "0", "0",
                "base_link",
                "sonar_link",
            ],
        ),
    ])
