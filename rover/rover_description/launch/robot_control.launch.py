from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
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
declare_mode_arg = DeclareLaunchArgument(
    name="mode",
    default_value="sim",
    description="Control backend mode: sim or hw"
)
mode = LaunchConfiguration("mode")
declare_use_joystick_arg = DeclareLaunchArgument(
    name="use_joystick",
    default_value="false",
    description="Start joy_node and teleop_twist_joy"
)
use_joystick = LaunchConfiguration("use_joystick")
declare_joy_backend_arg = DeclareLaunchArgument(
    name="joy_backend",
    default_value="game_controller_node",
    description="Joystick backend: joy_node or game_controller_node"
)
joy_backend = LaunchConfiguration("joy_backend")
declare_joy_dev_arg = DeclareLaunchArgument(
    name="joy_dev",
    default_value="/dev/input/js0",
    description="Linux joystick device for joy_node"
)
joy_dev = LaunchConfiguration("joy_dev")
declare_joy_device_id_arg = DeclareLaunchArgument(
    name="joy_device_id",
    default_value="0",
    description="Joystick device index for joy/game_controller nodes"
)
joy_device_id = LaunchConfiguration("joy_device_id")
declare_joy_device_name_arg = DeclareLaunchArgument(
    name="joy_device_name",
    default_value="",
    description="Optional joystick device name to match exactly"
)
joy_device_name = LaunchConfiguration("joy_device_name")

xacro_file = PathJoinSubstitution([
    FindPackageShare(PACKAGE_NAME),
    "urdf",
    "robomow_rl2000.xacro"
])

hardware_plugin = PythonExpression([
    "'mock_components/GenericSystem' if '",
    mode,
    "' == 'sim' else 'rover_hardware/RoverBaseboardSystem'"
])

robot_description_content = ParameterValue(
    Command([
        FindExecutable(name="xacro"),
        " ",
        xacro_file,
        " ",
        "hardware_plugin:=",
        hardware_plugin
    ]),
    value_type=str
)

ros2_control_config = PathJoinSubstitution([
    FindPackageShare(PACKAGE_NAME),
    "config",
    "robomow_ros2_control.yaml"
])
joystick_config = PathJoinSubstitution([
    FindPackageShare(PACKAGE_NAME),
    "config",
    "ps_controller_teleop.yaml"
])


def generate_launch_description():
    return LaunchDescription([
        declare_log_level_arg,
        declare_mode_arg,
        declare_use_joystick_arg,
        declare_joy_backend_arg,
        declare_joy_dev_arg,
        declare_joy_device_id_arg,
        declare_joy_device_name_arg,

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

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                "dev": joy_dev,
                "device_id": joy_device_id,
                "deadzone": 0.2,
                "autorepeat_rate": 5.0,
            }],
            condition=IfCondition(PythonExpression(["'", use_joystick, "' == 'true' and '", joy_backend, "' == 'joy_node'"]))
        ),

        Node(
            package='joy',
            executable='game_controller_node',
            name='joy_node',
            output='screen',
            parameters=[{
                "device_id": joy_device_id,
                "device_name": joy_device_name,
                "deadzone": 0.2,
                "autorepeat_rate": 5.0,
            }],
            condition=IfCondition(PythonExpression(["'", use_joystick, "' == 'true' and '", joy_backend, "' != 'joy_node'"]))
        ),

        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joystick_config],
            condition=IfCondition(use_joystick)
        ),

        Node(
            package='nmea_navsat_driver',
            executable='nmea_topic_driver',
            condition=IfCondition(PythonExpression(["'", mode, "' == 'hw'"]))
        )
    ])
