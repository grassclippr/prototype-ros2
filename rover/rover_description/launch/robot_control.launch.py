from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = "rover_description"

declare_log_level_arg = DeclareLaunchArgument(
    name="log_level",
    default_value="info",
    description="Logging level for all nodes"
)
log_level = LaunchConfiguration("log_level")

xacro_file = PathJoinSubstitution([
    FindPackageShare(PACKAGE_NAME),
    "urdf",
    "robomow_rl2000.xacro"
])

robot_description_content = ParameterValue(
    Command([
        FindExecutable(name="xacro"),
        " ",
        xacro_file
    ]),
    value_type=str
)

ros2_control_config = PathJoinSubstitution([
    FindPackageShare(PACKAGE_NAME),
    "config",
    "robomow_ros2_control.yaml"
])


def generate_launch_description():
    return LaunchDescription([
        declare_log_level_arg,

        # robot_state_publisher with xacro-based URDF
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description_content}],
            output="screen",
            arguments=["--ros-args", "--log-level", log_level]
   #         env={'RCUTILS_COLORIZED_OUTPUT': '1'}
        ),

        # ros2_control_node with URDF and controller config
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_content},
                ros2_control_config
            ],
            output="screen",
            arguments=["--ros-args", "--log-level", log_level]
        ),

        # joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # diff_drive_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),
    ])
