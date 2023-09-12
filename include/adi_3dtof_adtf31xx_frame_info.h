/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_ADTF31XX_FRAME_INFO_H
#define ADI_3DTOF_ADTF31XX_FRAME_INFO_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cstring>
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
    ir_frame_ = nullptr;
    xyz_frame_ = nullptr;
    image_width_ = image_width;
    image_height_ = image_height;

    depth_frame_ = new unsigned short[image_width * image_height];
    ir_frame_ = new unsigned short[image_width * image_height];
    xyz_frame_ = new short[image_width * image_height * 3];

    // Worst case, RVL compression can take ~1.5x the input data.
    compressed_depth_frame_ = new unsigned char[2 * image_width * image_height];
    compressed_ir_frame_ = new unsigned char[2 * image_width * image_height];
    compressed_depth_frame_size_ = 0;
    compressed_ir_frame_size_ = 0;
    frame_timestamp_ = ros::Time::now();
  }

  /**
   * @brief Destructor
   */
  ~ADI3DToFADTF31xxFrameInfo()
  {
    if (depth_frame_ != nullptr)
    {
      delete[] depth_frame_;
    }
    if (ir_frame_ != nullptr)
    {
      delete[] ir_frame_;
    }
    if (xyz_frame_ != nullptr)
    {
      delete[] xyz_frame_;
    }
    if (compressed_depth_frame_ != nullptr)
    {
      delete[] compressed_depth_frame_;
    }
    if (compressed_ir_frame_ != nullptr)
    {
      delete[] compressed_ir_frame_;
    }
  }

  /**
   * @brief Get the depth image frame
   *
   * @return unsigned short* depth image pointer
   */
  unsigned short* getDepthFrame() const
  {
    return depth_frame_;
  }

  /**
   * @brief Get the IR image frame
   *
   * @return unsigned short* IR image pointer
   */
  unsigned short* getIRFrame() const
  {
    return ir_frame_;
  }

  /**
   * @brief Get point cloud frame
   *
   * @return short* point cloud pointer
   */
  short* getXYZFrame() const
  {
    return xyz_frame_;
  }

  /**
   * @brief Get Compressed depth image frame
   *
   * @return unsigned char* compressed depth image pointer
   */
  unsigned char* getCompressedDepthFrame() const
  {
    return compressed_depth_frame_;
  }

  /**
   * @brief Get Compressed IR image frame
   *
   * @return unsigned char* compressed IR image pointer
   */
  unsigned char* getCompressedIRFrame() const
  {
    return compressed_ir_frame_;
  }

  /**
   * @brief Get Frame Timestamp Pointer
   *
   * @return ros::Time* Frame Timnestamp pointer
   */
  ros::Time* getFrameTimestampPtr()
  {
    return &frame_timestamp_;
  }

  /**
   * @brief Get Frame Timestamp
   *
   * @return ros::Time Frame Timestamp
   */
  ros::Time getFrameTimestamp() const
  {
    return frame_timestamp_;
  }

  /**
   * @brief Gives compressed depth image size
   *
   * @return int size of compressed depth image
   */
  int getCompressedDepthFrameSize() const
  {
    return compressed_depth_frame_size_;
  }

  /**
   * @brief Gives compressed IR image size
   *
   * @return int size of compressed IR image
   */
  int getCompressedIRFrameSize() const
  {
    return compressed_ir_frame_size_;
  }

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
   * @brief Set the Compressed IR Image Size
   *
   * @param compressed_ir_frame_size size of compressed IR image
   */
  void setCompressedIRFrameSize(int compressed_ir_frame_size)
  {
    compressed_ir_frame_size_ = compressed_ir_frame_size;
  }

  // Assignment operator
  ADI3DToFADTF31xxFrameInfo& operator=(const ADI3DToFADTF31xxFrameInfo& rhs)
  {
    image_width_ = rhs.image_width_;
    image_height_ = rhs.image_height_;
    memcpy(depth_frame_, rhs.depth_frame_, sizeof(depth_frame_[0]) * image_width_ * image_height_);
    memcpy(ir_frame_, rhs.ir_frame_, sizeof(ir_frame_[0]) * image_width_ * image_height_);
    memcpy(xyz_frame_, rhs.xyz_frame_, sizeof(xyz_frame_[0]) * image_width_ * image_height_ * 3);
    compressed_depth_frame_size_ = rhs.compressed_depth_frame_size_;
    compressed_ir_frame_size_ = rhs.compressed_ir_frame_size_;
    memcpy(compressed_depth_frame_, rhs.compressed_depth_frame_,
           sizeof(compressed_depth_frame_[0]) * compressed_depth_frame_size_);
    memcpy(compressed_ir_frame_, rhs.compressed_ir_frame_, sizeof(compressed_ir_frame_[0]) * compressed_ir_frame_size_);
    frame_timestamp_ = rhs.frame_timestamp_;
    return *this;
  }

private:
  /**
   * @brief Depth image
   */
  unsigned short* depth_frame_;

  /**
   * @brief IR image
   */
  unsigned short* ir_frame_;
  /**
   * @brief xyz frame
   *
   */
  short* xyz_frame_;
  /**
   * @brief compressed depth frame
   *
   */
  unsigned char* compressed_depth_frame_;
  /**
   * @brief compressed ir frame
   *
   */
  unsigned char* compressed_ir_frame_;
  /**
   * @brief compressed depth frame size
   *
   */
  int compressed_depth_frame_size_;
  /**
   * @brief compressed ir frame size
   *
   */
  int compressed_ir_frame_size_;
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
  ros::Time frame_timestamp_;
};

#endif
