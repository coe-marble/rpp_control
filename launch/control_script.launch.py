"""Start the generic ROS adapter for an RPP control script."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'rpp_workspace',
            default_value=FindPackageShare('rpp_control'),
            description='Directory containing the RPP workspace.',
        ),
        DeclareLaunchArgument('script_library', default_value='rpp_control'),
        DeclareLaunchArgument('script_name', default_value='control_script'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('control_frequency_hz', default_value='50.0'),
        DeclareLaunchArgument('state_timeout_sec', default_value='1.0'),
        DeclareLaunchArgument('expose_navigation_topics', default_value='false'),
        DeclareLaunchArgument('is_dev', default_value='false'),
        DeclareLaunchArgument('enable_nav2_cmd_vel', default_value='true'),
        DeclareLaunchArgument('navigation_reference_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('navigation_resource', default_value=''),
        DeclareLaunchArgument('navigation_generator', default_value=''),
        DeclareLaunchArgument('navigation_goal_monitor', default_value=''),
        DeclareLaunchArgument('navigation_frame_prefix', default_value=''),
        Node(
            package='rpp_control',
            executable='control_script_node',
            name='control_script',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'rpp_workspace': ParameterValue(
                    LaunchConfiguration('rpp_workspace'), value_type=str
                ),
                'script_library': ParameterValue(
                    LaunchConfiguration('script_library'), value_type=str
                ),
                'script_name': ParameterValue(
                    LaunchConfiguration('script_name'), value_type=str
                ),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                ),
                'control_frequency_hz': ParameterValue(
                    LaunchConfiguration('control_frequency_hz'), value_type=float
                ),
                'state_timeout_sec': ParameterValue(
                    LaunchConfiguration('state_timeout_sec'), value_type=float
                ),
                'expose_navigation_topics': ParameterValue(
                    LaunchConfiguration('expose_navigation_topics'), value_type=bool
                ),
                'is_dev': ParameterValue(
                    LaunchConfiguration('is_dev'), value_type=bool
                ),
                'enable_nav2_cmd_vel': ParameterValue(
                    LaunchConfiguration('enable_nav2_cmd_vel'), value_type=bool
                ),
                'navigation_reference_timeout_sec': ParameterValue(
                    LaunchConfiguration('navigation_reference_timeout_sec'), value_type=float
                ),
                'navigation_resource': ParameterValue(
                    LaunchConfiguration('navigation_resource'), value_type=str
                ),
                'navigation_generator': ParameterValue(
                    LaunchConfiguration('navigation_generator'), value_type=str
                ),
                'navigation_goal_monitor': ParameterValue(
                    LaunchConfiguration('navigation_goal_monitor'), value_type=str
                ),
                'navigation_frame_prefix': ParameterValue(
                    LaunchConfiguration('navigation_frame_prefix'), value_type=str
                ),
            }],
        ),
    ])
