/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_ADTF31XX_FRAME_INFO_H
#define ADI_3DTOF_ADTF31XX_FRAME_INFO_H

#include <cstring>
#include <rclcpp/rclcpp.hpp>
/**
 * @brief This is the class for adtf31xx sensor frame
 *
 */
class ADI3DToFADTF31xxFrameInfo
{
public:
  /**
   * @brief Constructor
   *
   * @param image_width - Image Width
   * @param image_height - Image Height
   */
  ADI3DToFADTF31xxFrameInfo(int image_width, int image_height)
  {
    // Create the node.
    depth_frame_ = nullptr;
    ab_frame_ = nullptr;
    xyz_frame_ = nullptr;
    conf_frame_ = nullptr;
    compressed_depth_frame_ = nullptr;
    compressed_ab_frame_ = nullptr;
    compressed_conf_frame_ = nullptr;

    image_width_ = image_width;
    image_height_ = image_height;

    depth_frame_ = new unsigned short[image_width * image_height];
    ab_frame_ = new unsigned short[image_width * image_height];
    xyz_frame_ = new short[image_width * image_height * 3];
    conf_frame_ = new unsigned short[image_width * image_height];

    // Worst case, RVL compression can take ~1.5x the input data.
    compressed_depth_frame_ = new unsigned char[3 * image_width * image_height];
    compressed_ab_frame_ = new unsigned char[3 * image_width * image_height];
    compressed_conf_frame_ = new unsigned char[3 * image_width * image_height];
    compressed_depth_frame_size_ = 0;
    compressed_ab_frame_size_ = 0;
    compressed_conf_frame_size_ = 0;
    frame_timestamp_ = rclcpp::Clock{}.now();
  }

  /**
   * @brief Destructor
   */
  ~ADI3DToFADTF31xxFrameInfo()
  {
    if (depth_frame_ != nullptr) {
      delete[] depth_frame_;
    }
    if (ab_frame_ != nullptr) {
      delete[] ab_frame_;
    }
    if (xyz_frame_ != nullptr) {
      delete[] xyz_frame_;
    }
    if (conf_frame_ != nullptr) {
      delete[] conf_frame_;
    }
    if (compressed_depth_frame_ != nullptr) {
      delete[] compressed_depth_frame_;
    }
    if (compressed_ab_frame_ != nullptr) {
      delete[] compressed_ab_frame_;
    }
    if (compressed_conf_frame_ != nullptr) {
      delete[] compressed_conf_frame_;
    }
  }

  /**
   * @brief Get the depth image frame
   *
   * @return unsigned short* depth image pointer
   */
  unsigned short * getDepthFrame() const { return depth_frame_; }

  /**
   * @brief Get the AB image frame
   *
   * @return unsigned short* AB image pointer
   */
  unsigned short * getABFrame() const { return ab_frame_; }

  /**
   * @brief Get point cloud frame
   *
   * @return short* point cloud pointer
   */
  short * getXYZFrame() const { return xyz_frame_; }

  /**
   * @brief Get Confidence frame
   *
   * @return unsigned short* Confidence pointer
   */
  unsigned short * getConfFrame() const { return conf_frame_; }

  /**
   * @brief Get Compressed depth image frame
   *
   * @return unsigned char* compressed depth image pointer
   */
  unsigned char * getCompressedDepthFrame() const { return compressed_depth_frame_; }

  /**
   * @brief Get Compressed AB image frame
   *
   * @return unsigned char* compressed AB image pointer
   */
  unsigned char * getCompressedABFrame() const { return compressed_ab_frame_; }

  /**
   * @brief Get Compressed Confidence image frame
   *
   * @return unsigned char* compressed AB image pointer
   */
  unsigned char * getCompressedConfFrame() const { return compressed_conf_frame_; }

  /**
   * @brief Get Frame Timestamp Pointer
   *
   * @return rclcpp::Time* Frame Timnestamp pointer
   */
  rclcpp::Time * getFrameTimestampPtr() { return &frame_timestamp_; }

  /**
   * @brief Get Frame Timestamp
   *
   * @return rclcpp::Time Frame Timestamp
   */
  rclcpp::Time getFrameTimestamp() const { return frame_timestamp_; }

  /**
   * @brief Gives compressed depth image size
   *
   * @return int size of compressed depth image
   */
  int getCompressedDepthFrameSize() const { return compressed_depth_frame_size_; }

  /**
   * @brief Gives compressed AB image size
   *
   * @return int size of compressed AB image
   */
  int getCompressedABFrameSize() const { return compressed_ab_frame_size_; }

  /**
   * @brief Gives compressed confidence image size
   *
   * @return int size of compressed confidence image
   */
  int getCompressedConfFrameSize() const { return compressed_conf_frame_size_; }

  /**
   * @brief Set the Compressed depth image size
   *
   * @param compressed_depth_frame_size size of compressed depth image
   */
  void setCompressedDepthFrameSize(int compressed_depth_frame_size)
  {
    compressed_depth_frame_size_ = compressed_depth_frame_size;
  }

  /**
   * @brief Set the Compressed AB Image Size
   *
   * @param compressed_ab_frame_size size of compressed AB image
   */
  void setCompressedABFrameSize(int compressed_ab_frame_size)
  {
    compressed_ab_frame_size_ = compressed_ab_frame_size;
  }

  /**
   * @brief Set the Compressed confidence Image Size
   *
   * @param compressed_conf_frame_size size of compressed confidence image
   */
  void setCompressedConfFrameSize(int compressed_conf_frame_size)
  {
    compressed_conf_frame_size_ = compressed_conf_frame_size;
  }

  // Assignment operator
  ADI3DToFADTF31xxFrameInfo & operator=(const ADI3DToFADTF31xxFrameInfo & rhs)
  {
    image_width_ = rhs.image_width_;
    image_height_ = rhs.image_height_;
    memcpy(depth_frame_, rhs.depth_frame_, sizeof(depth_frame_[0]) * image_width_ * image_height_);
    memcpy(ab_frame_, rhs.ab_frame_, sizeof(ab_frame_[0]) * image_width_ * image_height_);
    memcpy(xyz_frame_, rhs.xyz_frame_, sizeof(xyz_frame_[0]) * image_width_ * image_height_ * 3);
    memcpy(conf_frame_, rhs.conf_frame_, sizeof(conf_frame_[0]) * image_width_ * image_height_);
    compressed_depth_frame_size_ = rhs.compressed_depth_frame_size_;
    compressed_ab_frame_size_ = rhs.compressed_ab_frame_size_;
    memcpy(
      compressed_depth_frame_, rhs.compressed_depth_frame_,
      sizeof(compressed_depth_frame_[0]) * compressed_depth_frame_size_);
    memcpy(
      compressed_ab_frame_, rhs.compressed_ab_frame_,
      sizeof(compressed_ab_frame_[0]) * compressed_ab_frame_size_);
    memcpy(
      compressed_conf_frame_, rhs.compressed_conf_frame_,
      sizeof(compressed_conf_frame_[0]) * compressed_conf_frame_size_);
    frame_timestamp_ = rhs.frame_timestamp_;
    return *this;
  }

private:
  /**
   * @brief Depth image
   */
  unsigned short * depth_frame_;

  /**
   * @brief AB image
   */
  unsigned short * ab_frame_;
  /**
   * @brief xyz frame
   *
   */
  short * xyz_frame_;
  /**
   * @brief conf frame
   *
   */
  unsigned short * conf_frame_;
  /**
   * @brief compressed depth frame
   *
   */
  unsigned char * compressed_depth_frame_;
  /**
   * @brief compressed ab frame
   *
   */
  unsigned char * compressed_ab_frame_;
  /**
   * @brief compressed conf frame
   *
   */
  unsigned char * compressed_conf_frame_;
  /**
   * @brief compressed depth frame size
   *
   */
  int compressed_depth_frame_size_;
  /**
   * @brief compressed ab frame size
   *
   */
  int compressed_ab_frame_size_;
  /**
   * @brief compressed conf frame size
   *
   */
  int compressed_conf_frame_size_;
  /**
   * @brief Image width
   */
  int image_width_;
  /**
   * @brief Image height
   */
  int image_height_;
  /**
   * @brief Frame Timestamp
   */
  rclcpp::Time frame_timestamp_;
};

#endif
