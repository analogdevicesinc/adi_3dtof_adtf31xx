cmake_minimum_required(VERSION 3.8)
project(adi_3dtof_adtf31xx)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(compressed_depth_image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

#target_compile_definitions(${PROJECT_NAME} PRIVATE "ENABLE_FUNCTION_PROFILING")

include_directories(
  include
)

if(NOT HOST_BUILD)
  message(WARNING "If the package is being built on the TOF module, ignore this warning; otherwise, use the command below to build the package on the host.
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DHOST_BUILD=TRUE")
endif()
#List of common source files.
SET( COMMON_SRC_FILES src/adi_3dtof_adtf31xx.cpp src/input_sensor_file.cpp src/input_sensor_file_rosbagbin.cpp src/adi_3dtof_adtf31xx_input_thread.cpp src/adi_3dtof_adtf31xx_output_thread.cpp src/ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.cpp)

if(NOT HOST_BUILD)
  # Compile for platforms with 3D ToF Sensor
  add_compile_definitions(ENABLE_ADI_3DTOF_ADTF31XX_SENSOR)
  add_executable(${PROJECT_NAME}_node ${COMMON_SRC_FILES} src/input_sensor_adtf31xx.cpp)
else()
  # Compilation for non 3D ToF sensor platform(ex:Host machines)
  add_executable(${PROJECT_NAME}_node ${COMMON_SRC_FILES})
endif()

ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs pcl_ros cv_bridge image_geometry image_transport compressed_depth_image_transport OpenCV)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if(NOT HOST_BUILD)
  # We need this library only for Sensor platforms.
	set(ADITOF_LIB1 ${PROJECT_SOURCE_DIR}/libs/libaditof.so)
endif()

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${Boost_SYSTEM_LIBRARY}
  ${ADITOF_LIB1}
 )

## Mark executables for installation
 install(TARGETS ${PROJECT_NAME}_node
   DESTINATION lib/${PROJECT_NAME}
 )


#This is the executable to be run on the Host to uncompress the depth/ir images.
if(HOST_BUILD)
  add_executable(adi_3dtof_adtf31xx_compressed_image_subscriber_node src/adi_3dtof_adtf31xx_compressed_image_subscriber.cpp src/ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.cpp)

  ament_target_dependencies(adi_3dtof_adtf31xx_compressed_image_subscriber_node rclcpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs pcl_ros cv_bridge image_geometry image_transport OpenCV)

  target_link_libraries(adi_3dtof_adtf31xx_compressed_image_subscriber_node
     ${OpenCV_LIBRARIES}
     )

  install(TARGETS adi_3dtof_adtf31xx_compressed_image_subscriber_node
     DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(HOST_BUILD)
  add_executable(adi_3dtof_adtf31xx_read_rosbag_node src/read_rosbags.cpp src/ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.cpp)

  ament_target_dependencies(adi_3dtof_adtf31xx_read_rosbag_node rclcpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs pcl_ros cv_bridge image_geometry image_transport OpenCV compressed_depth_image_transport rosbag2_cpp)

  target_link_libraries(adi_3dtof_adtf31xx_read_rosbag_node
    ${OpenCV_LIBRARIES}
  )
  install(TARGETS adi_3dtof_adtf31xx_read_rosbag_node
     DESTINATION lib/${PROJECT_NAME})
endif()

 install(DIRECTORY rviz 
 DESTINATION share/${PROJECT_NAME})

 install(DIRECTORY launch
 DESTINATION share/${PROJECT_NAME})

 install(DIRECTORY config 
 DESTINATION share/${PROJECT_NAME})

 install(DIRECTORY rqt_config
 DESTINATION share/${PROJECT_NAME})

ament_package()
