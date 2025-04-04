/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef INPUT_SENSOR_FILE_ROSBAGBIN_H
#define INPUT_SENSOR_FILE_ROSBAGBIN_H

#include <stdint.h>

#include <fstream>

#include "input_sensor.h"

/**
 * @brief This is input class for sensor as camera
 *
 */
class InputSensorFileRosbagBin : public IInputSensor
{
public:
  InputSensorFileRosbagBin()
  {
    input_frame_width_ = 1024;
    input_frame_height_ = 1024;
    processing_scale_ = 2;

    camera_intrinsics_.camera_matrix[0] = 781.291565f;
    camera_intrinsics_.camera_matrix[1] = 0.0f;
    camera_intrinsics_.camera_matrix[2] = 520.714905f;
    camera_intrinsics_.camera_matrix[3] = 0.0f;
    camera_intrinsics_.camera_matrix[4] = 781.87738f;
    camera_intrinsics_.camera_matrix[5] = 513.001709f;
    camera_intrinsics_.camera_matrix[6] = 0.0f;
    camera_intrinsics_.camera_matrix[7] = 0.0f;
    camera_intrinsics_.camera_matrix[8] = 1.0f;

    camera_intrinsics_.distortion_coeffs[0] = -0.0693829656f;
    camera_intrinsics_.distortion_coeffs[1] = 0.115561306f;
    camera_intrinsics_.distortion_coeffs[2] = 0.000196631983f;
    camera_intrinsics_.distortion_coeffs[3] = -0.00011414945f;
    camera_intrinsics_.distortion_coeffs[4] = 0.0944529548f;
    camera_intrinsics_.distortion_coeffs[5] = 0.269195855f;
    camera_intrinsics_.distortion_coeffs[6] = -0.00811018609f;
    camera_intrinsics_.distortion_coeffs[7] = 0.190516844f;

    // Initializing extrinsics
    camera_extrinsics_.rotation_matrix[0] = 1.0f;
    camera_extrinsics_.rotation_matrix[1] = 0.0f;
    camera_extrinsics_.rotation_matrix[2] = 0.0f;
    camera_extrinsics_.rotation_matrix[3] = 0.0f;
    camera_extrinsics_.rotation_matrix[4] = 1.0f;
    camera_extrinsics_.rotation_matrix[5] = 0.0f;
    camera_extrinsics_.rotation_matrix[6] = 0.0f;
    camera_extrinsics_.rotation_matrix[7] = 0.0f;
    camera_extrinsics_.rotation_matrix[8] = 1.0f;

    camera_extrinsics_.translation_matrix[0] = 0.0f;
    camera_extrinsics_.translation_matrix[1] = 0.0f;
    camera_extrinsics_.translation_matrix[2] = 0.0f;
  }
  void openSensor(
    std::string sensor_name, int input_image_width, int input_image_height,
    std::string config_file_name, std::string input_sensor_ip) override;
  void configureSensor(int camera_mode) override;
  void getIntrinsics(CameraIntrinsics * camera_intrinsics_data) override;
  void getExtrinsics(CameraExtrinsics * camera_extrinsics_data) override;
  bool readNextFrame(
    unsigned short * out_depth_frame, unsigned short * out_ab_frame,
    unsigned short * out_conf_frame, short * out_xyz_frame) override;
  bool getFrameTimestamp(rclcpp::Time * timestamp) override;
  void closeSensor() override;

  /**
   * @brief set ABinvalidation threshold
   *
   * @param threshold ABinvalidation threshold
   */
  void setABinvalidationThreshold(int /*threshold*/) override
  {
    // Does nothing here.
    return;
  }

  /**
   * @brief Set Confidence Threshold
   *
   * @param threshold Confidence threshold
   */
  void setConfidenceThreshold(int /*threshold*/) override
  {
    // Does nothing here, should be overridden in derived class.
    return;
  }

  /**
   * @brief sets the state of JBLF filter
   *
   * @param enable_jblf_filter parameter to enable or disable JBLF filter
   */
  void setJBLFFilterState(bool /*enable_jblf_filter*/) override
  {
    // Does nothing here, should be overridden in derived class.
    return;
  }

  /**
   * @brief sets the size of JBLF filter
   *
   * @param jbfl_filter_size parameter to set JBLF filter size
   */
  void setJBLFFilterSize(int /*jbfl_filter_size*/) override
  {
    // Does nothing here, should be overridden in derived class.
    return;
  }

  /**
   * @brief sets the minimum threshold for radial filter
   *
   * @param radial_threshold_min parameter to set minimum threshold for radial filter
   */
  void setRadialFilterMinThreshold(int /*radial_min_threshold*/) override
  {
    // Does nothing here, should be overridden in derived class.
    return;
  }

  /**
   * @brief sets the maximum threshold for radial filter
   *
   * @param radial_threshold_max parameter to set maximum threshold for radial filter
   */
  void setRadialFilterMaxThreshold(int /*radial_max_threshold*/) override
  {
    // Does nothing here, should be overridden in derived class.
    return;
  }

private:
  std::string in_file_name_;
  std::ifstream in_file_;
  uint32_t input_frame_width_;
  uint32_t input_frame_height_;
  int processing_scale_;
  uint32_t header_version_ = 0;
  uint32_t frame_pitch_ = 0;
  uint64_t device_timestamp_ = 0;
  uint64_t frame_timestamp_ = 0;
  uint32_t bytes_per_pixel_ = 2;
  uint32_t total_frames_ = 0;
  uint8_t header_buffer_[36];
  uint8_t cam_info_buffer_[400];
  CameraIntrinsics camera_intrinsics_;
  CameraExtrinsics camera_extrinsics_;
};

#endif
