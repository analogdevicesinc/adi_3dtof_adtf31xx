import os
import launch
import json
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    var_ns_prefix = "adi_3dtof_adtf31xx_read_rosbag"

    # If the bagfile contains topics from diffeent cameras(with different prefix)
    # pass it as a list of strings example "[cam1 cam2]"
    arg_camera_prefixes_desc = DeclareLaunchArgument(
        'arg_camera_prefixes', default_value="[cam1]")

    # Name of the input bag file
    arg_in_file_name_desc = DeclareLaunchArgument(
        'arg_in_file_name', default_value="no name")

    # Node description
    adi_3dtof_adtf31xx_read_rosbag_node_desc = Node(
        package='adi_3dtof_adtf31xx',
        namespace=var_ns_prefix,
        executable='adi_3dtof_adtf31xx_read_rosbag_node',
        name='adi_3dtof_adtf31xx_read_rosbag_node',
        output="screen",
        parameters=[{
            'param_input_file_name': LaunchConfiguration('arg_in_file_name'),
            'param_camera_prefixes': LaunchConfiguration('arg_camera_prefixes'),
        }],
        on_exit=launch.actions.Shutdown()
    )

    # Launch description
    return LaunchDescription([
        arg_in_file_name_desc,
        arg_camera_prefixes_desc,
        adi_3dtof_adtf31xx_read_rosbag_node_desc
    ])
