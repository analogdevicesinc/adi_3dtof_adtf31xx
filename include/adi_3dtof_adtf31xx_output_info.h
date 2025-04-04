/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_ADTF31XX_OUTPUT_INFO_H
#define ADI_3DTOF_ADTF31XX_OUTPUT_INFO_H

#include <cstring>
/**
 * @brief This is the class for adtf31xx sensor node output information.
 *
 */
class ADI3DToFADTF31xxOutputInfo
{
public:
  /**
   * @brief Constructor
   *
   * @param image_width - Image Width
   * @param image_height - Image Height
   */
  ADI3DToFADTF31xxOutputInfo(int image_width, int image_height)
  {
    // Create the node.
    frame_number_ = -1;
    depth_frame_ = nullptr;
    ab_frame_ = nullptr;
    xyz_frame_ = nullptr;
    conf_frame_ = nullptr;
    compressed_depth_frame_ = nullptr;
    compressed_ab_frame_ = nullptr;
    compressed_conf_frame_ = nullptr;
    image_width_ = image_width;
    image_height_ = image_height;
    compressed_depth_frame_size_ = 0;
    compressed_ab_frame_size_ = 0;
    compressed_conf_frame_size_ = 0;

    depth_frame_ = new unsigned short[image_width * image_height];
    ab_frame_ = new unsigned short[image_width * image_height];
    conf_frame_ = new unsigned short[image_width * image_height];
    // Worst case, RVL compression can take ~1.5x the input data.
    compressed_depth_frame_ = new unsigned char[3 * image_width * image_height];
    compressed_ab_frame_ = new unsigned char[3 * image_width * image_height];
    compressed_conf_frame_ = new unsigned char[3 * image_width * image_height];
    xyz_frame_ = new short[3 * image_width * image_height];
  }

  /**
   * @brief Destructor
   */
  ~ADI3DToFADTF31xxOutputInfo()
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

  // Assignment operator
  ADI3DToFADTF31xxOutputInfo & operator=(const ADI3DToFADTF31xxOutputInfo & rhs)
  {
    frame_number_ = rhs.frame_number_;
    image_width_ = rhs.image_width_;
    image_height_ = rhs.image_height_;
    compressed_depth_frame_size_ = rhs.compressed_depth_frame_size_;
    compressed_ab_frame_size_ = rhs.compressed_ab_frame_size_;
    compressed_conf_frame_size_ = rhs.compressed_conf_frame_size_;

    memcpy(depth_frame_, rhs.depth_frame_, sizeof(depth_frame_[0]) * image_width_ * image_height_);
    memcpy(ab_frame_, rhs.ab_frame_, sizeof(ab_frame_[0]) * image_width_ * image_height_);
    memcpy(xyz_frame_, rhs.xyz_frame_, sizeof(xyz_frame_[0]) * image_width_ * image_height_ * 3);
    memcpy(conf_frame_, rhs.conf_frame_, sizeof(conf_frame_[0]) * image_width_ * image_height_);
    memcpy(
      compressed_depth_frame_, rhs.compressed_depth_frame_,
      sizeof(compressed_depth_frame_[0]) * compressed_depth_frame_size_);
    memcpy(
      compressed_ab_frame_, rhs.compressed_ab_frame_,
      sizeof(compressed_ab_frame_[0]) * compressed_ab_frame_size_);
    memcpy(
      compressed_conf_frame_, rhs.compressed_conf_frame_,
      sizeof(compressed_conf_frame_[0]) * compressed_conf_frame_size_);

    return *this;
  }

  int frame_number_;
  unsigned short * depth_frame_;
  unsigned short * ab_frame_;
  unsigned short * conf_frame_;
  unsigned char * compressed_depth_frame_;
  unsigned char * compressed_ab_frame_;
  unsigned char * compressed_conf_frame_;
  int compressed_depth_frame_size_;
  int compressed_ab_frame_size_;
  int compressed_conf_frame_size_;
  short * xyz_frame_;
  int image_width_;
  int image_height_;
};

#endif
