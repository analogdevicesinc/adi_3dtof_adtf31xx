/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include "module_profile.h"
#include "adi_3dtof_adtf31xx_frame_info.h"
#include "adi_3dtof_adtf31xx_output_info.h"
#include "adi_3dtof_adtf31xx.h"
#include <compressed_depth_image_transport/rvl_codec.h>
#include <sensor_msgs/Image.h>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

namespace enc = sensor_msgs::image_encodings;

/**
 * @brief This function sets the abort flag for the output thread,
 * the function is normally called by the main function to abort the output thread.
 *
 */
void ADI3DToFADTF31xx::processOutputAbort()
{
  process_output_thread_abort_ = true;
}

/**
 * @brief This function pushes the debug node to the output queue.
 * If the queue is full, then the last item in the queue gets replaced
 * with the latest node.
 *
 * @param new_output_node : Pointer to the debug node o be published
 */
void ADI3DToFADTF31xx::adtf31xxSensorPushOutputNode(ADI3DToFADTF31xxOutputInfo* new_output_node)
{
  output_thread_mtx_.lock();
  int queue_size = output_node_queue_.size();
  output_thread_mtx_.unlock();

  if (queue_size <= (max_debug_queue_length_ - 1))
  {
    // Push this one
    output_thread_mtx_.lock();
    output_node_queue_.push(new_output_node);
    output_thread_mtx_.unlock();
  }
  else
  {
    [[maybe_unused]] ADI3DToFADTF31xxOutputInfo* last_node = nullptr;
    // Replace the last item with the current one.
    output_thread_mtx_.lock();
    last_node = (ADI3DToFADTF31xxOutputInfo*)output_node_queue_.back();
    output_thread_mtx_.unlock();

    // Copy the contents of new node into the old one and then delete the new node.
    last_node = new_output_node;

    // Delete this one
    delete new_output_node;
  }
}

/**
 * @brief The output process function, this is running a loop
 * which reads the frame from the putput queue, generates the visualization output
 * and publishes the output as ROS messages.
 *
 */
void ADI3DToFADTF31xx::processOutput()
{
  int debug_queue_size = output_node_queue_.size();

  while ((!process_output_thread_abort_) || (debug_queue_size > 0))
  {
    // std::cout << "Inside processOutput" << std::endl;
    output_thread_mtx_.lock();
    debug_queue_size = output_node_queue_.size();
    output_thread_mtx_.unlock();
    if (debug_queue_size > 0)
    {
      PROFILE_FUNCTION_START(processOutput_Thread)
      output_thread_mtx_.lock();
      ADI3DToFADTF31xxOutputInfo* new_frame = (ADI3DToFADTF31xxOutputInfo*)output_node_queue_.front();
      output_node_queue_.pop();
      output_thread_mtx_.unlock();

      if (enable_depth_ab_compression_)
      {
        // Compress ab frame
        PROFILE_FUNCTION_START(adtf31xx_abFrameCompression)
        compressed_depth_image_transport::RvlCodec rvl;
        new_frame->compressed_ab_frame_size_ = rvl.CompressRVL(
            &new_frame->ab_frame_[0], &new_frame->compressed_ab_frame_[0], image_width_ * image_height_);
        PROFILE_FUNCTION_END(adtf31xx_abFrameCompression)
      }
      // Publish the output
      publishImageAndCameraInfo(new_frame);
      delete new_frame;
      PROFILE_FUNCTION_END(processOutput_Thread)
    }

    // Sleep
    boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
  }
}
