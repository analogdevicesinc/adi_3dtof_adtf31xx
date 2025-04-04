cmake_minimum_required(VERSION 3.0.2)
project(adi_3dtof_adtf31xx)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS cv_bridge
             image_view
             image_geometry
             image_transport
             roscpp
             sensor_msgs
             std_msgs
             eigen_conversions
             tf2
             tf2_ros
             pcl_ros
             dynamic_reconfigure)

find_package(Eigen3 QUIET)
find_package(OpenCV REQUIRED)

# Variable to indicate that the sensor is connected to the platform
set(SENSOR_CONNECTED TRUE CACHE BOOL "Indicates if the sensor is connected to the platform")

# Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(cfg/ADTF31xxSensorParams.cfg)

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################
catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  adi_3dtof_adtf31xx
  CATKIN_DEPENDS
  roscpp
  std_msgs
  image_geometry
  sensor_msgs)

# Specify additional locations of header files Your package locations should be
# listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Declare a C++ executable With catkin_make all packages are built within a
# single CMake context The recommended prefix ensures that target names across
# packages don't collide The macro should be passed using cmake command ex:

if(SENSOR_CONNECTED)
  find_package(aditof REQUIRED)
  # Compile for platforms with 3D ToF Sensor
	add_compile_definitions(ENABLE_ADTF31XX_SENSOR)
  add_executable(
    ${PROJECT_NAME}_node
    src/adi_3dtof_adtf31xx.cpp
    src/ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.cpp
    src/input_sensor_file_rosbagbin.cpp
    src/input_sensor_adtf31xx.cpp
    src/adi_3dtof_adtf31xx_input_thread.cpp
    src/adi_3dtof_adtf31xx_output_thread.cpp)
else()
  # Compilation for non 3D ToF sensor platform(ex:Host machines)
  add_executable(
    ${PROJECT_NAME}_node
    src/adi_3dtof_adtf31xx.cpp
    src/ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.cpp
    src/input_sensor_file_rosbagbin.cpp
    src/adi_3dtof_adtf31xx_input_thread.cpp
    src/adi_3dtof_adtf31xx_output_thread.cpp)
endif()

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES} ${aditof_LIBRARIES})

# ##############################################################################
# Install ##
# ##############################################################################
# Copy the RVIZ config files
set(RQT_RVIZ_ORIGINAL_FILE
    ${PROJECT_SOURCE_DIR}/rqt_config/adi_3dtof_adtf31xx_rqt_rviz.rviz)
set(RQT_RVIZ_DEST_FILE ~/.ros/adi_3dtof_adtf31xx_rqt_rviz.rviz)
add_custom_command(
  TARGET ${PROJECT_NAME}_node
  POST_BUILD
  COMMAND cp ${RQT_RVIZ_ORIGINAL_FILE} ${RQT_RVIZ_DEST_FILE})

# Mark executables for installation
install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Copy the ToF sensor config folder
add_custom_command(
  TARGET ${PROJECT_NAME}_node
  POST_BUILD
  COMMAND
    ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/config
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/${PROJECT_NAME}/config
)

if(BUILD_EXAMPLE_SUBSCRIBER_NODE)
  # This is the executable to be run on the Host to uncompress the depth/ab
  # images.
  add_executable(adi_3dtof_adtf31xx_compressed_image_subscriber_node
                 src/adi_3dtof_adtf31xx_compressed_image_subscriber.cpp)

  target_link_libraries(adi_3dtof_adtf31xx_compressed_image_subscriber_node
                        ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

  install(TARGETS adi_3dtof_adtf31xx_compressed_image_subscriber_node
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
# This is the executable to be run to convert ros bag format to binary file
# format.
if(BUILD_READ_ROSBAG_NODE)
  add_executable(
    read_rosbags_node
    src/read_rosbags.cpp
    src/ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.cpp
  )

  target_link_libraries(read_rosbags_node ${catkin_LIBRARIES}
                        ${OpenCV_LIBRARIES})

  install(TARGETS read_rosbags_node
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
