/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include "module_profile.h"
#include "adi_3dtof_adtf31xx_frame_info.h"
#include "adi_3dtof_adtf31xx.h"
#include <compressed_depth_image_transport/rvl_codec.h>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

/**
 * @brief Function to read Abort flag, this function will be called by the main function to exit the thread.
 *
 */
void ADI3DToFADTF31xx::readInputAbort()
{
  read_input_thread_abort_ = true;
}

/**
 * @brief Function to read the next frame from the input Queue
 *
 * @return ADI3DToFADTF31xxFrameInfo* - Pointer to the next frame
 */
ADI3DToFADTF31xxFrameInfo* ADI3DToFADTF31xx::adtf31xxSensorGetNextFrame()
{
  ADI3DToFADTF31xxFrameInfo* inframe = nullptr;
  input_thread_mtx_.lock();
  int queue_size = input_frames_queue_.size();
  input_thread_mtx_.unlock();

  // We will try till we fill read the buffer, this is just to allow the read thread to fill the queue
  while ((inframe == nullptr) && ((!error_in_frame_read_) || queue_size > 0))
  {
    input_thread_mtx_.lock();
    queue_size = input_frames_queue_.size();
    input_thread_mtx_.unlock();
    if (queue_size > 0)
    {
      input_thread_mtx_.lock();
      inframe = (ADI3DToFADTF31xxFrameInfo*)input_frames_queue_.front();
      input_frames_queue_.pop();
      input_thread_mtx_.unlock();
    }
    if (inframe == nullptr)
    {
      // Wait for the buffers to be filled
      boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
    }
  }
  return inframe;
}

/**
 * @brief This function reads the frmae from the sensor. This runs in a loop,
 * reading the frames and adding them to the input Queue,
 * if the queue is fulll, then the recent buffer is overwritten with the
 * newly read frame.
 *
 */
void ADI3DToFADTF31xx::readInput()
{
  int frame_count = 0;

  while (!read_input_thread_abort_)
  {
    // ROS_INFO("Read loop");

    // Create a new node
    ADI3DToFADTF31xxFrameInfo* new_frame = new ADI3DToFADTF31xxFrameInfo(image_width_, image_height_);
    if (new_frame != nullptr)
    {
      PROFILE_FUNCTION_START(readInput_Thread)
      bool result = input_sensor_->readNextFrame(new_frame->getDepthFrame(), new_frame->getIRFrame());
      input_sensor_->getFrameTimestamp(new_frame->getFrameTimestampPtr());
      PROFILE_FUNCTION_END(readInput_Thread)
      if (!result)
      {
        // free memory
        delete new_frame;
        error_in_frame_read_ = true;
        continue;
      }
    }
    else
    {
      error_in_frame_read_ = true;
      read_input_thread_abort_ = true;
      break;
    }

    // Add it to the queue
    input_thread_mtx_.lock();
    int queue_size = input_frames_queue_.size();
    input_thread_mtx_.unlock();
    if (queue_size <= (max_input_queue_length_ - 1))
    {
      input_thread_mtx_.lock();
      input_frames_queue_.push(new_frame);
      input_thread_mtx_.unlock();
    }
    else
    {
      std::cout << "Overwrite buffer" << std::endl;
      // If the Queue is full, then overwrite the last buffer with the latest frame
      input_thread_mtx_.lock();
      ADI3DToFADTF31xxFrameInfo* last_node = (ADI3DToFADTF31xxFrameInfo*)input_frames_queue_.back();
      input_thread_mtx_.unlock();
      last_node = new_frame;
      delete new_frame;
    }

    // sleep
    boost::this_thread::sleep_for(boost::chrono::milliseconds(2));
  }

  // Destroy the queue
  input_thread_mtx_.lock();
  int queue_size = input_frames_queue_.size();
  while (queue_size > 0)
  {
    // pop and delete
    ADI3DToFADTF31xxFrameInfo* new_frame = (ADI3DToFADTF31xxFrameInfo*)input_frames_queue_.front();
    input_frames_queue_.pop();
    delete new_frame;
    queue_size = input_frames_queue_.size();
  }
  input_thread_mtx_.unlock();
}
