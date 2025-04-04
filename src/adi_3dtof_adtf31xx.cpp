/******************************************************************************
  Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
  This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
  and its licensors.
 ******************************************************************************/
#include "input_sensor.h"
#include "module_profile.h"
#include "input_sensor_factory.h"
#include "adi_3dtof_adtf31xx_frame_info.h"
#include "adi_3dtof_adtf31xx.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <compressed_depth_image_transport/rvl_codec.h>
#include <boost/thread/thread.hpp>

namespace enc = sensor_msgs::image_encodings;

/**
 * @mainpage
 *
 * @htmlinclude manifest.html
 */

/**
 * @brief updates the parameter of input image based on dynamic reconfigure.
 *
 */

void ADI3DToFADTF31xx::updateDynamicReconfigureVariablesInputThread()
{
  // setting AB threshold and confidence threshold values if they are changed.
  if (ab_threshold_ != dynamic_reconfigure_config_.ab_threshold)
  {
    ab_threshold_ = dynamic_reconfigure_config_.ab_threshold;
    ROS_INFO("Changed AB threshold value is %d", ab_threshold_);
    input_sensor_->setABinvalidationThreshold(ab_threshold_);
  }

  if (confidence_threshold_ != dynamic_reconfigure_config_.confidence_threshold)
  {
    confidence_threshold_ = dynamic_reconfigure_config_.confidence_threshold;
    ROS_INFO("Changed Confidence threshold value is %d", confidence_threshold_);
    input_sensor_->setConfidenceThreshold(confidence_threshold_);
  }

  if (jblf_filter_state_ != dynamic_reconfigure_config_.jblf_filter_state)
  {
    jblf_filter_state_ = dynamic_reconfigure_config_.jblf_filter_state;
    ROS_INFO("Changed JBLF filter state is %d", jblf_filter_state_);
    input_sensor_->setJBLFFilterState(jblf_filter_state_);
  }

  if (jblf_filter_size_ != dynamic_reconfigure_config_.jblf_filter_size)
  {
    jblf_filter_size_ = dynamic_reconfigure_config_.jblf_filter_size;
    ROS_INFO("Changed JBLF filter size is %d", jblf_filter_size_);
    input_sensor_->setJBLFFilterSize(jblf_filter_size_);
  }

  if (radial_filter_min_threshold_ != dynamic_reconfigure_config_.radial_threshold_min)
  {
    radial_filter_min_threshold_ = dynamic_reconfigure_config_.radial_threshold_min;
    ROS_INFO("Changed Radial filter min threshold is %d", radial_filter_min_threshold_);
    input_sensor_->setRadialFilterMinThreshold(radial_filter_min_threshold_);
  }

  if (radial_filter_max_threshold_ != dynamic_reconfigure_config_.radial_threshold_max)
  {
    radial_filter_max_threshold_ = dynamic_reconfigure_config_.radial_threshold_max;
    ROS_INFO("Changed Radial filter max threshold is %d", radial_filter_max_threshold_);
    input_sensor_->setRadialFilterMaxThreshold(radial_filter_max_threshold_);
  }
}

/**
 *
 * @brief This function is the entry point to the sensor node
 *
 *
 */
bool ADI3DToFADTF31xx::readNextFrame()
{
  updateDynamicReconfigureVariablesInputThread();

  ADI3DToFADTF31xxFrameInfo* inframe;
  try
  {
    inframe = adtf31xxSensorGetNextFrame();
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error getting next frame" << e.what() << '\n';
    return false;
  }

  if (inframe == nullptr)
  {
    return false;
  }

  ADI3DToFADTF31xxOutputInfo* new_output_frame = new ADI3DToFADTF31xxOutputInfo(image_width_, image_height_);
  if (new_output_frame == nullptr)
  {
    delete inframe;
    return false;
  }

  curr_frame_timestamp_ = inframe->getFrameTimestamp();
  depth_frame_ = inframe->getDepthFrame();
  ab_frame_ = inframe->getABFrame();
  xyz_frame_ = inframe->getXYZFrame();
  conf_frame_ = inframe->getConfFrame();

  if ((depth_frame_ == nullptr) || (ab_frame_ == nullptr) || (xyz_frame_ == nullptr) || (conf_frame_ == nullptr))
  {
    delete new_output_frame;
    return false;
  }
  if (new_output_frame != nullptr)
  {
    PROFILE_FUNCTION_START(adtf31xx_readNextFrame)
    if (enable_depth_ab_compression_)
    {
      // Compress depth and confidence frame. ab image compression is done in the output thread
      PROFILE_FUNCTION_START(adtf31xx_depthFrameCompression)
      compressed_depth_image_transport::RvlCodec rvl;
      new_output_frame->compressed_depth_frame_size_ = rvl.CompressRVL(
          &depth_frame_[0], &new_output_frame->compressed_depth_frame_[0], image_width_ * image_height_);
      new_output_frame->compressed_conf_frame_size_ =
          rvl.CompressRVL(&conf_frame_[0], &new_output_frame->compressed_conf_frame_[0], image_width_ * image_height_);
      PROFILE_FUNCTION_END(adtf31xx_depthFrameCompression)
    }

    new_output_frame->frame_number_ = frame_number_;
    memcpy(new_output_frame->depth_frame_, depth_frame_, image_width_ * image_height_ * sizeof(depth_frame_[0]));
    memcpy(new_output_frame->ab_frame_, ab_frame_, image_width_ * image_height_ * sizeof(ab_frame_[0]));
    memcpy(new_output_frame->conf_frame_, conf_frame_, image_width_ * image_height_ * sizeof(conf_frame_[0]));
    memcpy(new_output_frame->xyz_frame_, xyz_frame_, 3 * image_width_ * image_height_ * sizeof(xyz_frame_[0]));

    // Push
    adtf31xxSensorPushOutputNode(new_output_frame);
  }

  delete inframe;

  frame_number_++;
  PROFILE_FUNCTION_END(adtf31xx_readNextFrame)

  return true;
}

/**
 * @brief new values from dynamic reconfigure are copied to a structure variable here, actual update to individual
 * parameters happens in updateDynamicReconfigureVariablesInputThread function.
 *
 * @param config Config parameters present in GUI
 * @param level
 */
void ADI3DToFADTF31xx::dynamicallyReconfigureVariables(adi_3dtof_adtf31xx::ADTF31xxSensorParamsConfig& config,
                                                       uint32_t /*level*/)
{
  dynamic_reconfigure_config_ = config;

  ROS_INFO(
      "Configuration variables: ab_threshold: %d, confidence_threshold: %d, jblf_filter_state: %d, jblf_filter_size: "
      "%d, radial_filter_min_threshold: %d, radial_filter_max_threshold: %d",
      ab_threshold_, confidence_threshold_, jblf_filter_state_, jblf_filter_size_, radial_filter_min_threshold_,
      radial_filter_max_threshold_);
}

/**
 * @brief Dynamic reconfigure initialization
 *
 */
void ADI3DToFADTF31xx::initSettingsForDynamicReconfigure()
{
  dynamic_reconfigure_config_.ab_threshold = ab_threshold_;
  dynamic_reconfigure_config_.confidence_threshold = confidence_threshold_;
  dynamic_reconfigure_config_.jblf_filter_state = jblf_filter_state_;
  dynamic_reconfigure_config_.jblf_filter_size = jblf_filter_size_;
  dynamic_reconfigure_config_.radial_threshold_min = radial_filter_min_threshold_;
  dynamic_reconfigure_config_.radial_threshold_max = radial_filter_max_threshold_;
}

/**
 * @brief This is main function
 *
 * @param argc number of input arguments to the function
 * @param argv array of pointer to the input arguments
 * @return int
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "adi_3dtof_adtf31xx");

  INIT_FUNCTION_PROFILE();

  // Create an instance of ADI3DToFADTF31xx
  ADI3DToFADTF31xx adi_3dtof_adtf31xx;

  // Spawn the read input thread..
  boost::thread read_input_thread;
  bool thread_spawn_status = true;
  try
  {
    read_input_thread = boost::thread(&ADI3DToFADTF31xx::readInput, &adi_3dtof_adtf31xx);
  }
  catch (const std::exception& e)
  {
    thread_spawn_status = false;
    std::cerr << "Exception when trying to spawn read_input_thread : " << e.what() << '\n';
  }

  // Spawn the process output thread..
  boost::thread process_output_thread;
  try
  {
    process_output_thread = boost::thread(&ADI3DToFADTF31xx::processOutput, &adi_3dtof_adtf31xx);
  }
  catch (const std::exception& e)
  {
    thread_spawn_status = false;
    std::cerr << "Exception when trying to spawn process_output_thread : " << e.what() << '\n';
  }

  ros::Rate loop_rate(100);
  int frame_num = 0;

  while ((ros::ok()) && (thread_spawn_status))
  {
    ROS_INFO_STREAM("Running loop : " << frame_num++);

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    if (!adi_3dtof_adtf31xx.readNextFrame())
    {
      break;
    }

    FLUSH_FUNCTION_PROFILE();

    loop_rate.sleep();

    ros::spinOnce();
  }

  if (thread_spawn_status)
  {
    // Signal thread abort
    adi_3dtof_adtf31xx.readInputAbort();
    try
    {
      // Wait for the thread to complete
      read_input_thread.join();
    }
    catch (const std::exception& e)
    {
      std::cerr << " Exception in read_input_thread.join() : " << e.what() << '\n';
    }
    // Signal thread abort
    adi_3dtof_adtf31xx.processOutputAbort();
    try
    {
      // Wait for the thread to complete
      process_output_thread.join();
    }
    catch (const std::exception& e)
    {
      std::cerr << " Exception in process_output_thread.join() : " << e.what() << '\n';
    }
  }

  CLOSE_FUNCTION_PROFILE();

  adi_3dtof_adtf31xx.shutDownAllNodes();

  return 0;
}
