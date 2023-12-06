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
    adi_3dtof_adtf31xx_compressed_image_subscriber_node_desc = Node(
        package='adi_3dtof_adtf31xx',
        namespace=var_ns_prefix_cam1,
        executable='adi_3dtof_adtf31xx_compressed_image_subscriber_node',
        name='adi_3dtof_adtf31xx_compressed_image_subscriber_node',
        output="screen",
        parameters=[{
            'camera_prefix': var_ns_prefix_cam1
        }],
        on_exit=launch.actions.Shutdown()
    )

    # Rviz description
    rviz_desc = Node(
        package='rviz2',
        namespace='var_ns_prefix_cam1',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [PathJoinSubstitution(([
            FindPackageShare('adi_3dtof_adtf31xx'),
            'rviz',
            'adi_3dtof_adtf31xx_raw.rviz'
        ]))]]
    )

    # Launch
    return LaunchDescription([
        adi_3dtof_adtf31xx_compressed_image_subscriber_node_desc,
        rviz_desc
    ])
