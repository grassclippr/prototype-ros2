from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_imu = LaunchConfiguration("use_imu")
    use_gnss = LaunchConfiguration("use_gnss")
    publish_map_odom_identity = LaunchConfiguration("publish_map_odom_identity")
    start_nav2 = LaunchConfiguration("start_nav2")

    localization_launch = PathJoinSubstitution([
        FindPackageShare("rover_navigation"),
        "launch",
        "localization.launch.py",
    ])
    nav2_launch = PathJoinSubstitution([
        FindPackageShare("rover_navigation"),
        "launch",
        "nav2.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("use_gnss", default_value="false"),
        DeclareLaunchArgument("publish_map_odom_identity", default_value="true"),
        DeclareLaunchArgument("start_nav2", default_value="false"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                "use_imu": use_imu,
                "use_gnss": use_gnss,
                "publish_map_odom_identity": publish_map_odom_identity,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={}.items(),
            condition=IfCondition(start_nav2),
        ),
    ])
