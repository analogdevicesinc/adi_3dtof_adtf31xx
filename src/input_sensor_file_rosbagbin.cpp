/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#include "input_sensor_file_rosbagbin.h"

#include <assert.h>

#include <fstream>
#include <thread>

/**
 * @brief opens the bag file.
 *
 * @param sensor_name the input bag file name
 * @param input_image_width image width
 * @param input_image_height image height
 * @param processing_scale scales the image dimensions and camera intrinsics.
 * @param config_file_name config file name of ToF SDK is not used as this is file io mode.
 */
void InputSensorFileRosbagBin::openSensor(
  std::string sensor_name, int input_image_width, int input_image_height,
  std::string /*config_file_name*/, std::string /*input_sensor_ip*/)
{
  in_file_name_ = sensor_name;
  frame_counter_ = 0;
  sensor_open_flag_ = false;
  frame_width_ = input_image_width;
  frame_height_ = input_image_height;

  // Open file for streaming.
  in_file_.open(sensor_name, std::ifstream::binary);
  if (in_file_.is_open()) {
    // Update flag.
    sensor_open_flag_ = true;
  }
}

/**
 * @brief Configures the sensor
 *
 * @param camera_mode camera mode, not used in file-io mode
 */
void InputSensorFileRosbagBin::configureSensor(int /*camera_mode*/)
{
  total_frames_ = 0;
  if (in_file_.is_open()) {
    in_file_.seekg(0, std::ios::end);
    std::fstream::pos_type file_length = in_file_.tellg();
    in_file_.seekg(0, std::ios::beg);
    in_file_.read((char *)header_buffer_, 36);  // 36 bytes header
    // Populate header data
    uint8_t * header_ptr = header_buffer_;
    uint32_t first_frame_pos;
    memcpy(&total_frames_, header_ptr, sizeof(uint32_t));
    header_ptr += 4;  // skipping head byte
    memcpy(&input_frame_width_, header_ptr, sizeof(uint32_t));
    header_ptr += 4;
    printf("input_frame_width= %d \n", input_frame_width_);
    memcpy(&input_frame_height_, header_ptr, sizeof(uint32_t));
    header_ptr += 4;
    printf("input_frame_height= %d \n", input_frame_height_);
    memcpy(&bytes_per_pixel_, header_ptr, sizeof(uint32_t));
    header_ptr += 4;
    memcpy(&header_version_, header_ptr, sizeof(uint32_t));
    header_ptr += 4;
    memcpy(&first_frame_pos, header_ptr, sizeof(uint32_t));
    header_ptr += 4;
    memcpy(&frame_pitch_, header_ptr, sizeof(uint32_t));
    header_ptr += 4;
    memcpy(&device_timestamp_, header_ptr, sizeof(uint64_t));
    header_ptr += 8;

    in_file_.read((char *)cam_info_buffer_, first_frame_pos - 36);  // 36 bytes header excluded
    uint8_t * cam_info_ptr = cam_info_buffer_;
    if (
      (input_frame_width_ == 1024 && input_frame_height_ == 1024) ||
      (input_frame_width_ == 512 && input_frame_height_ == 640)) {
      processing_scale_ = 1;
      setProcessingScale(processing_scale_);
    } else {
      processing_scale_ = 2;
      setProcessingScale(processing_scale_);
    }

    // Scale Intrinsics
    camera_intrinsics_.camera_matrix[0] /= input_scale_factor_;
    camera_intrinsics_.camera_matrix[2] /= input_scale_factor_;
    camera_intrinsics_.camera_matrix[4] /= input_scale_factor_;
    camera_intrinsics_.camera_matrix[5] /= input_scale_factor_;

    setFrameWidth(input_frame_width_);
    setFrameHeight(input_frame_height_);

    // Reading camera info

    uint32_t size_of_d;
    double k_cam[9] = {0};
    double r_cam[9] = {0};
    double p_cam[12] = {0};

    memcpy(&k_cam, cam_info_ptr, sizeof(k_cam));
    cam_info_ptr += sizeof(k_cam);

    memcpy(&size_of_d, cam_info_ptr, sizeof(size_of_d));
    cam_info_ptr += sizeof(size_of_d);

    double * d_cam = (double *)malloc(sizeof(double) * size_of_d);

    memcpy(d_cam, cam_info_ptr, sizeof(double) * size_of_d);
    cam_info_ptr += (size_of_d * sizeof(double));

    memcpy(&r_cam, cam_info_ptr, sizeof(r_cam));
    cam_info_ptr += sizeof(r_cam);

    memcpy(&p_cam, cam_info_ptr, sizeof(p_cam));

    camera_intrinsics_.camera_matrix[0] = (float)k_cam[0];
    camera_intrinsics_.camera_matrix[1] = 0.0f;
    camera_intrinsics_.camera_matrix[2] = (float)k_cam[2];
    camera_intrinsics_.camera_matrix[3] = 0.0f;
    camera_intrinsics_.camera_matrix[4] = (float)k_cam[4];
    camera_intrinsics_.camera_matrix[5] = (float)k_cam[5];
    camera_intrinsics_.camera_matrix[6] = 0.0f;
    camera_intrinsics_.camera_matrix[7] = 0.0f;
    camera_intrinsics_.camera_matrix[8] = 1.0f;

    /*when the header version is 1, the size of distortion coefficients is 16.
    they are represented as [0 0 0 0 0 0 0 0 k1 k2 p1 p2 k3 k4 k5 k6] hence offset of 8 is needed to read distortion
    coefficients When the header version is 2, the size of distortion coefficients is 8.
    they are represented as [k1 k2 p1 p2 k3 k4 k5 k6] hence no offset is needed to read the distortion coefficients*/

    if (header_version_ == 1) {
      for (int i = 0; i < 8; i++) {
        camera_intrinsics_.distortion_coeffs[i] = (float)d_cam[i + 8];
      }
    } else if (header_version_ >= 2) {
      for (int i = 0; i < 8; i++) {
        camera_intrinsics_.distortion_coeffs[i] = (float)d_cam[i];
      }
    }

    free(d_cam);
  }
  return;
}

/**
 * @brief Gets the camera intrinsics
 *
 * @param camera_intrinsics_data camera intrinsics
 */
void InputSensorFileRosbagBin::getIntrinsics(CameraIntrinsics * camera_intrinsics_data)
{
  memcpy(camera_intrinsics_data, &camera_intrinsics_, sizeof(camera_intrinsics_));
  return;
}

/**
 * @brief gets camera extrinsics
 *
 * @param camera_extrinsics_data camera extrinsics
 */
void InputSensorFileRosbagBin::getExtrinsics(CameraExtrinsics * camera_extrinsics_data)
{
  memcpy(camera_extrinsics_data, &camera_extrinsics_, sizeof(camera_extrinsics_));
  return;
}

/**
 * @brief reads next frame
 *
 * @param out_depth_frame pointer to read depth frame
 * @param out_ab_frame pointer to read ab frame

 * @return true if reading next frame is successful.
 * @return false if reading next frame is failure.
 */
bool InputSensorFileRosbagBin::readNextFrame(
  unsigned short * out_depth_frame, unsigned short * out_ab_frame, __attribute__((unused)) unsigned short * out_conf_frame,
  __attribute__((unused)) short * out_xyz_frame)
{
  // Read next frame

  // Sleep for 100ms - To simulate 10FPS
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  int num_samples_in_frame = input_frame_width_ * input_frame_height_;
  int num_bytes_in_combo_frame =
    ((num_samples_in_frame * bytes_per_pixel_) + 8) * 2;  // 8 header bytes per frame
  char * frame_buffer = new char[num_bytes_in_combo_frame];
  bool error_reading_frame = false;

  // Allocate memory
  unsigned short * depth_frame = new unsigned short[num_samples_in_frame];
  unsigned short * ab_frame = new unsigned short[num_samples_in_frame];

  assert(out_depth_frame != nullptr);
  assert(out_ab_frame != nullptr);

  if (header_version_ >= 3) {
    // Version 3 and above would have confidence image as well
    num_bytes_in_combo_frame += ((num_samples_in_frame * bytes_per_pixel_) + 8);
    assert(out_conf_frame != nullptr);
  }

  if (header_version_ == 4) {
    // Version 4 will have xyz image as well
    num_bytes_in_combo_frame += (num_samples_in_frame * bytes_per_pixel_ * 3);
    assert(out_xyz_frame != nullptr);
  }

  if (in_file_.is_open() && (frame_counter_ < total_frames_)) {
    in_file_.read(frame_buffer, num_bytes_in_combo_frame);
    if (in_file_.gcount() != num_bytes_in_combo_frame) {
      // Error in reading frame.
      error_reading_frame = true;
    }

    ++frame_counter_;
  } else {
    // EOF is reached or file is not openned
    error_reading_frame = true;
  }

  // unsigned short* temp_frame_buffer = reinterpret_cast<unsigned short*>(frame_buffer);
  uint8_t * pframe_buffer_ptr = reinterpret_cast<uint8_t *>(frame_buffer);
  uint8_t * ptemp_frame_buffer_ptr;
  bool read_pass = false;
  uint64_t timestamp_ab, timestamp_depth;
  if (!error_reading_frame) {
    while (!read_pass) {
      ptemp_frame_buffer_ptr = pframe_buffer_ptr;
      memcpy(&timestamp_depth, ptemp_frame_buffer_ptr, sizeof(uint64_t));
      ptemp_frame_buffer_ptr += 8;  // skipping frame header
      // Copy depth data and ab data
      // First comes depth frame,
      memcpy(depth_frame, ptemp_frame_buffer_ptr, num_samples_in_frame * bytes_per_pixel_);
      pframe_buffer_ptr += 8 + (num_samples_in_frame * bytes_per_pixel_);  // proceeding to ab data
      ptemp_frame_buffer_ptr = pframe_buffer_ptr;
      memcpy(&timestamp_ab, ptemp_frame_buffer_ptr, sizeof(uint64_t));
      if (timestamp_ab == timestamp_depth) {
        ptemp_frame_buffer_ptr += 8;  // skipping frame header
        // followed by ab frame.
        memcpy(ab_frame, ptemp_frame_buffer_ptr, num_samples_in_frame * bytes_per_pixel_);
        pframe_buffer_ptr += 8 + (num_samples_in_frame * bytes_per_pixel_);
        read_pass = true;
        frame_timestamp_ = timestamp_ab;
      }
    }
  }

  if (error_reading_frame) {
    // close file
    closeSensor();

    // delete memory
    delete[] frame_buffer;
    delete[] depth_frame;
    delete[] ab_frame;
    return false;
  }

  unsigned short * temp_depth_frame = depth_frame;
  unsigned short * temp_ab_frame = ab_frame;

  for (unsigned int i = 0; i < input_frame_height_; i++) {
    for (unsigned int j = 0; j < input_frame_width_; j++) {
      *out_depth_frame++ = temp_depth_frame[j];
      *out_ab_frame++ = temp_ab_frame[j];
    }

    temp_depth_frame += (input_frame_width_);
    temp_ab_frame += (input_frame_width_);
  }

  //Compute XYZ

  delete[] frame_buffer;
  delete[] depth_frame;
  delete[] ab_frame;

  return true;
}

/**
 * @brief returns frame timestamp
 *
 */
bool InputSensorFileRosbagBin::getFrameTimestamp(rclcpp::Time * timestamp)
{
  if (frame_timestamp_ > 0) {
    rclcpp::Time temptimestamp(
      (uint32_t)(frame_timestamp_ / 1000000000), (uint32_t)frame_timestamp_);
    *timestamp = temptimestamp;
  } else {
    *timestamp = rclcpp::Clock{}.now();
  }
  return true;
}

/**
 * @brief closes the input file
 *
 */
void InputSensorFileRosbagBin::closeSensor()
{
  if (in_file_.is_open()) {
    in_file_.close();
  }
}
