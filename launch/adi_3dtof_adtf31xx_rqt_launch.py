import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    # Arguments
    var_ns_prefix_cam1 = "cam1"

    # Node description
    rqt_gui_launch_desc = Node(
        package='rqt_gui',
        namespace=var_ns_prefix_cam1,
        executable='rqt_gui',
        name='rqt_gui_launch',
        output="screen",
        arguments=['--perspective-file', [PathJoinSubstitution(([
            FindPackageShare('adi_3dtof_adtf31xx'),
            'rqt_config',
            'adi_3dtof_adtf31xx_rqt.perspective'
        ]))]]
    )

    # Launch description
    return LaunchDescription([
        rqt_gui_launch_desc,
    ])
