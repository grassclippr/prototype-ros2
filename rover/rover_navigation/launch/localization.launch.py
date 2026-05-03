from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = "rover_navigation"


def generate_launch_description():
    use_imu = LaunchConfiguration("use_imu")
    use_gnss = LaunchConfiguration("use_gnss")
    publish_wheel_tf = LaunchConfiguration("publish_wheel_tf")
    publish_map_odom_identity = LaunchConfiguration("publish_map_odom_identity")

    wheel_ekf_config = PathJoinSubstitution([
        FindPackageShare(PACKAGE_NAME),
        "config",
        "wheel_ekf.yaml",
    ])
    wheel_imu_ekf_config = PathJoinSubstitution([
        FindPackageShare(PACKAGE_NAME),
        "config",
        "wheel_imu_ekf.yaml",
    ])
    navsat_config = PathJoinSubstitution([
        FindPackageShare(PACKAGE_NAME),
        "config",
        "navsat.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_imu",
            default_value="true",
            description="Fuse /imu/data in the odom EKF.",
        ),
        DeclareLaunchArgument(
            "use_gnss",
            default_value="false",
            description="Start nmea_navsat_driver and navsat_transform_node.",
        ),
        DeclareLaunchArgument(
            "publish_wheel_tf",
            default_value="false",
            description="Let the wheel bridge publish odom->base_link. Keep false when EKF publishes TF.",
        ),
        DeclareLaunchArgument(
            "publish_map_odom_identity",
            default_value="true",
            description="Publish identity map->odom for early local-goal Nav2 bring-up.",
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_identity",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "map", "--child-frame-id", "odom",
            ],
            condition=IfCondition(publish_map_odom_identity),
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu_static_tf",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "base_link", "--child-frame-id", "imu_link",
            ],
            condition=IfCondition(use_imu),
        ),

        Node(
            package=PACKAGE_NAME,
            executable="wheel_velocity_odom_bridge.py",
            name="wheel_velocity_odom_bridge",
            output="screen",
            parameters=[{
                "input_topic": "/wheel_velocities",
                "odom_topic": "/wheel/odom",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_tf": publish_wheel_tf,
            }],
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[wheel_imu_ekf_config],
            condition=IfCondition(use_imu),
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[wheel_ekf_config],
            condition=UnlessCondition(use_imu),
        ),

        Node(
            package="nmea_navsat_driver",
            executable="nmea_topic_driver",
            name="nmea_topic_driver",
            output="screen",
            condition=IfCondition(use_gnss),
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[navsat_config],
            remappings=[
                ("imu", "/imu/data"),
                ("gps/fix", "/fix"),
                ("odometry/filtered", "/odometry/filtered"),
                ("gps/filtered", "/gps/filtered"),
            ],
            condition=IfCondition(use_gnss),
        ),
    ])
