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
    imu_i2c_bus = LaunchConfiguration("imu_i2c_bus")
    imu_address = LaunchConfiguration("imu_address")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    imu_publish_rate = LaunchConfiguration("imu_publish_rate")
    imu_x = LaunchConfiguration("imu_x")
    imu_y = LaunchConfiguration("imu_y")
    imu_z = LaunchConfiguration("imu_z")
    imu_roll = LaunchConfiguration("imu_roll")
    imu_pitch = LaunchConfiguration("imu_pitch")
    imu_yaw = LaunchConfiguration("imu_yaw")

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
        DeclareLaunchArgument(
            "imu_i2c_bus",
            default_value="1",
            description="Linux I2C bus number for the ICM20948.",
        ),
        DeclareLaunchArgument(
            "imu_address",
            default_value="0x69",
            description="I2C address for the ICM20948.",
        ),
        DeclareLaunchArgument(
            "imu_frame_id",
            default_value="imu_link",
            description="Frame id for published IMU messages.",
        ),
        DeclareLaunchArgument(
            "imu_publish_rate",
            default_value="100.0",
            description="Publish rate in Hz for the ICM20948 driver.",
        ),
        DeclareLaunchArgument("imu_x", default_value="0.0"),
        DeclareLaunchArgument("imu_y", default_value="0.0"),
        DeclareLaunchArgument("imu_z", default_value="0.0"),
        DeclareLaunchArgument("imu_roll", default_value="1.5707963267948966"),
        DeclareLaunchArgument("imu_pitch", default_value="0.0"),
        DeclareLaunchArgument("imu_yaw", default_value="1.5707963267948966"),

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
                "--x", imu_x, "--y", imu_y, "--z", imu_z,
                "--roll", imu_roll, "--pitch", imu_pitch, "--yaw", imu_yaw,
                "--frame-id", "base_link", "--child-frame-id", imu_frame_id,
            ],
            condition=IfCondition(use_imu),
        ),

        Node(
            package=PACKAGE_NAME,
            executable="icm20948_node.py",
            name="icm20948",
            output="screen",
            parameters=[{
                "i2c_bus": imu_i2c_bus,
                "address": imu_address,
                "frame_id": imu_frame_id,
                "publish_rate_hz": imu_publish_rate,
            }],
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
