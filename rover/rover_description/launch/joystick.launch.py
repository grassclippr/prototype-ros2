from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PACKAGE_NAME = "rover_description"

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

joystick_config = PathJoinSubstitution([
    FindPackageShare(PACKAGE_NAME),
    "config",
    "ps_controller_teleop.yaml"
])


def generate_launch_description():
    return LaunchDescription([
        declare_joy_backend_arg,
        declare_joy_dev_arg,
        declare_joy_device_id_arg,
        declare_joy_device_name_arg,
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
            condition=IfCondition(PythonExpression(["'", joy_backend, "' == 'joy_node'"]))
        ),
        Node(
            package='joy',
            executable='game_controller_node',
            name='joy_node',
            output='screen',
            parameters=[{
                "device_id": joy_device_id,
                "device_name": ParameterValue(joy_device_name, value_type=str),
                "deadzone": 0.2,
                "autorepeat_rate": 5.0,
            }],
            condition=UnlessCondition(PythonExpression(["'", joy_backend, "' == 'joy_node'"]))
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joystick_config],
        ),
    ])
