/******************************************************************************
  Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
  This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
  and its licensors.
 ******************************************************************************/
#include "adi_3dtof_adtf31xx.h"

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/msg/image.h>

#include <boost/thread/thread.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "adi_3dtof_adtf31xx_frame_info.h"
#include "input_sensor.h"
#include "input_sensor_factory.h"
#include "module_profile.h"
#include "ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.h"

namespace enc = sensor_msgs::image_encodings;

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 */

/**
 *
 * @brief This function is the entry point to the sensor node
 *
 *
 */
bool ADI3DToFADTF31xx::readNextFrame()
{
  //Update the tunable parameters
  updateTunableParameters();

  ADI3DToFADTF31xxFrameInfo * inframe = nullptr;
  try {
    inframe = adtf31xxSensorGetNextFrame();
  } catch (const std::exception & e) {
    std::cerr << "Error getting next frame" << e.what() << '\n';
    return false;
  }

  if (inframe == nullptr) {
    return false;
  }

  ADI3DToFADTF31xxOutputInfo * new_output_frame =
    new ADI3DToFADTF31xxOutputInfo(image_width_, image_height_);
  if (new_output_frame == nullptr) {
    delete inframe;
    return false;
  }

  curr_frame_timestamp_ = inframe->getFrameTimestamp();
  depth_frame_ = inframe->getDepthFrame();
  ir_frame_ = inframe->getIRFrame();
  xyz_frame_ = inframe->getXYZFrame();

  if ((depth_frame_ == nullptr) || (ir_frame_ == nullptr) || (xyz_frame_ == nullptr)) {
    delete new_output_frame;
    return false;
  }

  PROFILE_FUNCTION_START(adtf31xx_readNextFrame)
  PROFILE_FUNCTION_START(adtf31xx_depthFrameCompression)
  compressed_depth_image_transport::RvlCodec rvl;
  unsigned short * raw_depth_frame = depth_frame_;
  unsigned char * compressed_depth_frame = new unsigned char[image_width_ * image_height_ * 2];
  int compressed_size_depth_frame = 0;
  if (enable_depth_ir_compression_) {
    compressed_size_depth_frame = rvl.CompressRVL(
      &raw_depth_frame[0], &compressed_depth_frame[0], image_width_ * image_height_);
  }
  PROFILE_FUNCTION_END(adtf31xx_depthFrameCompression)

  if (new_output_frame != nullptr) {
    new_output_frame->frame_number_ = frame_number_;
    new_output_frame->compressed_depth_frame_size_ = compressed_size_depth_frame;
    memcpy(
      new_output_frame->depth_frame_, depth_frame_,
      image_width_ * image_height_ * sizeof(depth_frame_[0]));
    memcpy(
      new_output_frame->ir_frame_, ir_frame_, image_width_ * image_height_ * sizeof(ir_frame_[0]));
    memcpy(
      new_output_frame->xyz_frame_, xyz_frame_,
      3 * image_width_ * image_height_ * sizeof(xyz_frame_[0]));
    memcpy(
      new_output_frame->compressed_depth_frame_, compressed_depth_frame,
      compressed_size_depth_frame * sizeof(compressed_depth_frame[0]));

    // Push
    adtf31xxSensorPushOutputNode(new_output_frame);
  }

  delete[] compressed_depth_frame;

  delete inframe;

  frame_number_++;
  PROFILE_FUNCTION_END(adtf31xx_readNextFrame)

  return true;
}

/**
 * @brief This is main function
 *
 * @param argc number of input arguments to the function
 * @param argv array of pointer to the input arguments
 * @return int
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  INIT_FUNCTION_PROFILE();

  // Create an instance of ADI3DToFADTF31xx
  auto adi_3dtof_adtf31xx = std::make_shared<ADI3DToFADTF31xx>();

  // Spawn the read input thread..
  std::thread read_input_thread;
  bool thread_spawn_status = true;
  try {
    read_input_thread = std::thread(&ADI3DToFADTF31xx::readInput, adi_3dtof_adtf31xx);
  } catch (const std::exception & e) {
    thread_spawn_status = false;
    std::cerr << "Exception when trying to spawn read_input_thread : " << e.what() << '\n';
  }

  // Spawn the process output thread..
  std::thread process_output_thread;
  try {
    process_output_thread = std::thread(&ADI3DToFADTF31xx::processOutput, adi_3dtof_adtf31xx);
  } catch (const std::exception & e) {
    thread_spawn_status = false;
    std::cerr << "Exception when trying to spawn process_output_thread : " << e.what() << '\n';
  }

  if (thread_spawn_status) {
    rclcpp::Rate loop_rate(100);

    try {
      rclcpp::spin(adi_3dtof_adtf31xx);
    } catch (const std::exception & e) {
      std::cerr << " Exception, shutting down the node  : " << e.what() << '\n';
    }

    // Signal thread abort
    adi_3dtof_adtf31xx->readInputAbort();
    try {
      // Wait for the thread to complete
      read_input_thread.join();
    } catch (const std::exception & e) {
      std::cerr << " Exception in read_input_thread.join() : " << e.what() << '\n';
    }
    // Signal thread abort
    adi_3dtof_adtf31xx->processOutputAbort();
    try {
      // Wait for the thread to complete
      process_output_thread.join();
    } catch (const std::exception & e) {
      std::cerr << " Exception in process_output_thread.join() : " << e.what() << '\n';
    }
  }

  CLOSE_FUNCTION_PROFILE();

  rclcpp::shutdown();

  return 0;
}
