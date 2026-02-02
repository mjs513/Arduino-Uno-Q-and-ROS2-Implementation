from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path to SLAM Toolbox parameters
    config = os.path.join(
        get_package_share_directory("ros_sonar"),
        "config",
        "slam_toolbox_params.yaml"
    )

    # Path to SLAM Toolbox bringup launch file
    slam_launch_file = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "launch",
        "online_sync_launch.py"
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            "slam_params_file": config,
        }.items(),
    )

    return LaunchDescription([

        # ---------------------------------------------------------
        # Sonar Node
        # ---------------------------------------------------------
        Node(
            package="ros_sonar",
            executable="sonar_node",
            name="sonar_node",
            output="screen",
        ),

        # ---------------------------------------------------------
        # TF: base_link -> sonar_link
        # ---------------------------------------------------------
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "sonar_link"],
            name="sonar_tf"
        ),

        # ---------------------------------------------------------
        # TF: odom -> base_link (fake odometry)
        # ---------------------------------------------------------
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            name="fake_odom_tf"
        ),

        # ---------------------------------------------------------
        # SLAM Toolbox (correct bringup)
        # ---------------------------------------------------------
        slam_toolbox,
    ])

