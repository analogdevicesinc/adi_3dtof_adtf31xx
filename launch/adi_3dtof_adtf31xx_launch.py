import os
import launch
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.substitutions import LaunchConfiguration, PythonExpression

# configuration file names
config_json_file_names = ['config_adsd3500_adsd3100.json',
                          'config_adsd3500_adsd3030.json']

package_dir = get_package_share_directory('adi_3dtof_adtf31xx') + "/../../../../src/adi_3dtof_adtf31xx/"

def generate_launch_description():

    # Height of the sensor from camera in meters
    arg_camera_height_from_ground_in_mtr_desc = DeclareLaunchArgument(
        'arg_camera_height_from_ground_in_mtr', default_value="0.15")

    # Input mode => 0:Real Time Sensor, 2:Rosbag bin, 3:Network
    arg_input_sensor_mode_desc = DeclareLaunchArgument(
        'arg_input_sensor_mode', default_value="3")

    # Input filename : Applicable only if the input mode is 2
    # Relative path to the launch file which gets executed. Here, launch file from install folder is considered
    arg_in_file_name_desc = DeclareLaunchArgument('arg_in_file_name', default_value= package_dir + "../adi_3dtof_input_video_files/adi_3dtof_height_170mm_yaw_135degrees_cam1.bin")

    # Input Sensor IP Address : Applicable only if inpiut sensor mode is 3    
    arg_input_sensor_ip_desc = DeclareLaunchArgument(
        'arg_input_sensor_ip', default_value="10.43.0.1")
    
    # Enable RVL compression for depth and ab images
    arg_enable_depth_ab_compression_desc = DeclareLaunchArgument(
        'arg_enable_depth_ab_compression', default_value="False")
    
    # Enable Depth image publishing 
    arg_enable_depth_publish_desc = DeclareLaunchArgument(
        'arg_enable_depth_publish', default_value="True")
    
    # Enable AB image publishing 
    arg_enable_ab_publish_desc = DeclareLaunchArgument(
        'arg_enable_ab_publish', default_value="True")
    
    # Enable Confidence image publishing 
    arg_enable_conf_publish_desc = DeclareLaunchArgument(
        'arg_enable_conf_publish', default_value="True")
    
    # Enable Point Cloud image publishing 
    arg_enable_point_cloud_publish_desc = DeclareLaunchArgument(
        'arg_enable_point_cloud_publish', default_value="False")

    # abThreshold
    arg_ab_threshold_desc = DeclareLaunchArgument(
        'arg_ab_threshold', default_value="10")

    # confidenceThreshold
    arg_confidence_threshold_desc = DeclareLaunchArgument(
        'arg_confidence_threshold', default_value="10")

    # Configuration file name of ToF SDK 
    # config_adsd3500_adsd3100.json - For ADSd3100(MP sensor)
    # config_adsd3500_adsd3030.json - For ADSd3030(VGA sensor)
    config_json_file_name = "config_adsd3500_adsd3100.json"
    arg_config_file_name_of_tof_sdk_desc = DeclareLaunchArgument(
        'arg_config_file_name_of_tof_sdk', default_value= package_dir + "config/" + config_json_file_name)

    # Frame Type
    #MP(1024x01024) sensor:
    #    "sr-native" - 0
    #    "lr-native" - 1
    #    "sr-qnative" - 2
    #    "lr-qnative" - 3
    #    "sr-mixed" - 5
    #    "lr-mixed" - 6

    #VGA(640x512) sensor:
    #    "sr-native" - 0
    #    "lr-native" - 1
    #    "lr-qnative" - 3
    #    "sr-mixed" - 5
    #    "lr-mixed" - 6    
    arg_camera_mode_desc = DeclareLaunchArgument(
        'arg_camera_mode', default_value="3")

    # Encoding String
    #   "mono16" - 16 bit sincle color channel
    #   "16UC1" - 16 bit unsigned integer
    arg_encoding_type_desc = DeclareLaunchArgument(
        'arg_encoding_type', default_value="mono16")
        
    # Parameters for TF
    var_ns_prefix_cam1 = "cam1"
    var_cam1_base_frame_optical = f"{var_ns_prefix_cam1}_adtf31xx_optical"
    var_cam1_base_frame = f"{var_ns_prefix_cam1}_adtf31xx"
    """
         map
          ^
          |
        camera_device
          ^
          |
        camera_optical
    """
    var_cam1_parent_frame = "map"
    var_cam1_child_frame = var_cam1_base_frame_optical

    # Optical to Device rotation, these are standard values, would never change
    var_cam_optical_to_base_roll = "-1.57"
    var_cam_optical_to_base_pitch = "0"
    var_cam_optical_to_base_yaw = "-1.57"

    # Camera position wrt map
    var_cam1_pos_x = "0.0"
    var_cam1_pos_y = "0.0"
    var_cam1_roll = "0.0"
    var_cam1_pitch = "0.0"
    var_cam1_yaw = "0.0"

    # adi_3dtof_adtf31xx_node Node description
    adi_3dtof_adtf31xx_node_desc = Node(
        package='adi_3dtof_adtf31xx',
        namespace=var_ns_prefix_cam1,
        executable='adi_3dtof_adtf31xx_node',
        name='adi_3dtof_adtf31xx_node',
        output="screen",
        parameters=[{
            'param_camera_link': var_cam1_base_frame_optical,
            'param_input_sensor_mode': LaunchConfiguration('arg_input_sensor_mode'),
            'param_input_file_name': LaunchConfiguration('arg_in_file_name'),
            'param_config_file_name_of_tof_sdk': LaunchConfiguration('arg_config_file_name_of_tof_sdk'),
            'param_camera_mode': LaunchConfiguration('arg_camera_mode'),
            'param_enable_depth_ab_compression': LaunchConfiguration('arg_enable_depth_ab_compression'),
            'param_enable_depth_publish': LaunchConfiguration('arg_enable_depth_publish'),            
            'param_enable_ab_publish': LaunchConfiguration('arg_enable_ab_publish'),
            'param_enable_conf_publish': LaunchConfiguration('arg_enable_conf_publish'),
            'param_enable_point_cloud_publish': LaunchConfiguration('arg_enable_point_cloud_publish'),
            'param_ab_threshold': LaunchConfiguration('arg_ab_threshold'),
            'param_confidence_threshold': LaunchConfiguration('arg_confidence_threshold'),
            'param_encoding_type' : LaunchConfiguration('arg_encoding_type'),
            'param_input_sensor_ip' : LaunchConfiguration('arg_input_sensor_ip')
        }],
        on_exit=launch.actions.Shutdown()
    )

    # cam1_base_to_optical TF description
    cam1_base_to_optical_tf_desc = Node(
        package='tf2_ros',
        namespace=var_ns_prefix_cam1,
        executable='static_transform_publisher',
        name=f'{var_cam1_base_frame_optical}_tf',
        output="screen",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                                                  "--roll", f"{var_cam_optical_to_base_roll}",
                                                  "--pitch", f"{var_cam_optical_to_base_pitch}",
                                                  "--yaw", f"{var_cam_optical_to_base_yaw}",
                                                  "--frame-id", f"{var_cam1_base_frame}",
                                                  "--child-frame-id", f"{var_cam1_child_frame}"]
    )

    # map_to_cam1_base TF description
    map_to_cam1_base_tf_desc = Node(
        package='tf2_ros',
        namespace=var_ns_prefix_cam1,
        executable='static_transform_publisher',
        name=f'{var_cam1_base_frame}_tf',
        output="screen",
        arguments=["--x", f"{var_cam1_pos_x}",
                   "--y", f"{var_cam1_pos_y}",
                   "--z", PythonExpression(LaunchConfiguration(
                       'arg_camera_height_from_ground_in_mtr')),
                   "--roll", f"{var_cam1_roll}",
                   "--pitch", f"{var_cam1_pitch}",
                   "--yaw", f"{var_cam1_yaw}",
                   "--frame-id", f"{var_cam1_parent_frame}",
                   "--child-frame-id", f"{var_cam1_base_frame}"]
    )

    # RVIZ description
    rviz_desc = Node(
        package='rviz2',
        namespace=var_ns_prefix_cam1,
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [PathJoinSubstitution(([
            FindPackageShare('adi_3dtof_adtf31xx'),
            'rviz',
            'adi_3dtof_adtf31xx.rviz'
        ]))]]
    )

    # Launch
    return LaunchDescription([
        arg_camera_height_from_ground_in_mtr_desc,
        arg_input_sensor_mode_desc,
        arg_in_file_name_desc,
        arg_input_sensor_ip_desc,
        arg_enable_depth_ab_compression_desc,
        arg_enable_depth_publish_desc,
        arg_enable_ab_publish_desc,
        arg_enable_conf_publish_desc,
        arg_enable_point_cloud_publish_desc,
        arg_ab_threshold_desc,
        arg_confidence_threshold_desc,
        arg_config_file_name_of_tof_sdk_desc,
        arg_camera_mode_desc,
        arg_encoding_type_desc,
        adi_3dtof_adtf31xx_node_desc,
        cam1_base_to_optical_tf_desc,
        map_to_cam1_base_tf_desc,
        # rviz_desc
    ])
