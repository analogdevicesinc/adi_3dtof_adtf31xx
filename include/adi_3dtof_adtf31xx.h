/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_ADTF31XX__H
#define ADI_3DTOF_ADTF31XX__H

#include "input_sensor.h"
#include "input_sensor_factory.h"
#include "image_proc_utils.h"
#include "adi_3dtof_adtf31xx_output_info.h"
#include "adi_3dtof_adtf31xx_frame_info.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <compressed_depth_image_transport/rvl_codec.h>
#include <queue>
#include <dynamic_reconfigure/server.h>
#include <compressed_depth_image_transport/compression_common.h>
#include <adi_3dtof_adtf31xx/ADTF31xxSensorParamsConfig.h>

namespace enc = sensor_msgs::image_encodings;

/**
 * @brief This is main class for this package
 *
 */
class ADI3DToFADTF31xx : public ros::NodeHandle
{
public:
  /**
   * @brief Construct a new ADI3DToFADTF31xx object
   *
   * @param camera_link
   * @param virtual_camera_link
   * @param safety_zone_radius_mtr
   * @param input_sensor_mode
   */
  ADI3DToFADTF31xx()
  {
    dynamic_reconfigure_callbacktype_ = boost::bind(&ADI3DToFADTF31xx::dynamicallyReconfigureVariables, this, _1, _2);
    server_.setCallback(dynamic_reconfigure_callbacktype_);

    ros::NodeHandle nh("~");

    // Get Parameters
    // ToF camera link
    std::string camera_link;
    nh.param<std::string>("param_camera_link", camera_link, "adi_camera_link");

    // Input sensor Mode:Camera/File/ROSBag
    int input_sensor_mode;
    nh.param<int>("param_input_sensor_mode", input_sensor_mode, 0);

    std::string input_file_name;
    nh.param<std::string>("param_input_file_name", input_file_name, "no name");

    int enable_depth_ab_compression;
    nh.param<int>("param_enable_depth_ab_compression", enable_depth_ab_compression, 0);

    int ab_threshold;
    nh.param<int>("param_ab_threshold", ab_threshold, 10);

    int confidence_threshold;
    nh.param<int>("param_confidence_threshold", confidence_threshold, 10);

    // JBLF Filter State
    int jblf_filter_state;
    nh.param<int>("param_jblf_filter_state", jblf_filter_state, 1);

    // JBLF Filter Size
    int jblf_filter_size;
    nh.param<int>("param_jblf_filter_size", jblf_filter_size, 7);

    // Radial Filter Min Threshold
    int radial_filter_min_threshold;
    nh.param<int>("param_radial_filter_min_threshold", radial_filter_min_threshold, 100);

    // Radial Filter Max Threshold
    int radial_filter_max_threshold;
    nh.param<int>("param_radial_filter_max_threshold", radial_filter_max_threshold, 10000);

    std::string config_file_name_of_tof_sdk;
    nh.param<std::string>("param_config_file_name_of_tof_sdk", config_file_name_of_tof_sdk, "no name");

    int camera_mode;
    nh.param<int>("param_camera_mode", camera_mode, 3);

    std::string input_sensor_ip;
    nh.param<std::string>("param_input_sensor_ip", input_sensor_ip, "no name");

    int enable_point_cloud_publish;
    nh.param<int>("param_enable_point_cloud_publish", enable_point_cloud_publish, 0);

    std::string encoding_type;
    nh.param<std::string>("param_encoding_type", encoding_type, "mono16");

    enable_point_cloud_publish_ = (enable_point_cloud_publish == 1) ? true : false;

    camera_link_ = std::move(camera_link);
    input_sensor_mode_ = input_sensor_mode;
    input_file_name_ = std::move(input_file_name);
    input_sensor_ip_ = std::move(input_sensor_ip);
    encoding_type_ = std::move(encoding_type);
    frame_number_ = 0;
    enable_depth_ab_compression_ = (enable_depth_ab_compression == 1) ? true : false;

    ab_threshold_ = ab_threshold;
    confidence_threshold_ = confidence_threshold;
    jblf_filter_state_ = (jblf_filter_state == 1) ? true : false;
    jblf_filter_size_ = jblf_filter_size;
    radial_filter_min_threshold_ = radial_filter_min_threshold;
    radial_filter_max_threshold_ = radial_filter_max_threshold;

    // Get input sensor module
    input_sensor_ = InputSensorFactory::getInputSensor(input_sensor_mode_);

    // Open the sensor
    if (input_sensor_mode_ != 3)
    {
      // If the mode is not ADTF31xx sensor over Network, then the ip should be set to ""
      input_sensor_ip_.clear();
    }
    input_sensor_->openSensor(input_file_name_, image_width_, image_height_, config_file_name_of_tof_sdk,
                              input_sensor_ip_);
    if (!input_sensor_->isOpened())
    {
      ROS_ERROR("Could not open the sensor %s", input_file_name_.c_str());
      shutDownAllNodes();
    }

    // Configure the sensor
    input_sensor_->configureSensor(camera_mode);

    input_sensor_->setRadialFilterMinThreshold(radial_filter_min_threshold_);

    input_sensor_->setRadialFilterMaxThreshold(radial_filter_max_threshold_);

    input_sensor_->setJBLFFilterSize(jblf_filter_size_);

    // Buffer allocations.
    image_width_ = input_sensor_->getFrameWidth();
    image_height_ = input_sensor_->getFrameHeight();
    xyz_frame_ = nullptr;

    // Get intrinsics and extrinsics
    input_sensor_->getIntrinsics(&depth_intrinsics_);
    input_sensor_->getExtrinsics(&depth_extrinsics_);

    // Create publishers.
    // Input and Intermediate Debug Images
    if (enable_depth_ab_compression_ == 1)
    {
      depth_image_publisher_ = this->advertise<sensor_msgs::CompressedImage>("depth_image/compressedDepth", 10);
      ab_image_publisher_ = this->advertise<sensor_msgs::CompressedImage>("ab_image/compressedDepth", 10);
      conf_image_publisher_ = this->advertise<sensor_msgs::CompressedImage>("conf_image/compressedDepth", 10);
    }
    else
    {
      depth_image_publisher_ = this->advertise<sensor_msgs::Image>("depth_image", 10);
      ab_image_publisher_ = this->advertise<sensor_msgs::Image>("ab_image", 10);
      conf_image_publisher_ = this->advertise<sensor_msgs::Image>("conf_image", 10);
    }

    if (enable_point_cloud_publish_ == 1)
    {
      xyz_image_publisher_ = this->advertise<sensor_msgs::PointCloud2>("point_cloud", 10);
    }

    // Camera Infos
    depth_info_publisher_ = this->advertise<sensor_msgs::CameraInfo>("camera_info", 10);

    image_proc_utils_ = new ImageProcUtils(&depth_intrinsics_, image_width_, image_height_);

    input_sensor_->setABinvalidationThreshold(ab_threshold_);

    input_sensor_->setConfidenceThreshold(confidence_threshold_);

    input_sensor_->setJBLFFilterState(jblf_filter_state_);

    // Printing the output in console as parameters are overwritten by launch file.
    ROS_INFO(
        "Configuration variables: ab_threshold: %d, confidence_threshold: %d, jblf_filter_state: %d, jblf_filter_size: "
        "%d, radial_filter_min_threshold: %d, radial_filter_max_threshold: %d",
        ab_threshold_, confidence_threshold_, jblf_filter_state_, jblf_filter_size_, radial_filter_min_threshold_,
        radial_filter_max_threshold_);

    // Initially setting dynamic reconfigure values to same as launch file
    initSettingsForDynamicReconfigure();
  }

  /**
   * @brief This function shuts down all the nodes running in a roscore
   *
   */
  void shutDownAllNodes()
  {
    int status = system("rosnode kill -a");
    if (status < 0)
    {
      ROS_INFO_STREAM("Error in \"rosnode kill -a\": " << status);
    }
    ros::shutdown();
  }

  /**
   * @brief Destroy the ADI3DToFADTF31xx object
   *
   */
  ~ADI3DToFADTF31xx()
  {
    if (image_proc_utils_ != nullptr)
    {
      delete image_proc_utils_;
      image_proc_utils_ = nullptr;
    }
  }

  bool readNextFrame();
  void processOutputAbort();
  void adtf31xxSensorPushOutputNode(ADI3DToFADTF31xxOutputInfo* new_output_node);
  void processOutput();
  void readInputAbort();
  ADI3DToFADTF31xxFrameInfo* adtf31xxSensorGetNextFrame();
  void readInput();
  void dynamicallyReconfigureVariables(adi_3dtof_adtf31xx::ADTF31xxSensorParamsConfig& config, uint32_t level);
  void updateDynamicReconfigureVariablesInputThread();

private:
  IInputSensor* input_sensor_;
  std::string camera_link_;
  int input_sensor_mode_;
  int image_width_ = 1024;
  int image_height_ = 1024;
  int frame_number_;
  bool enable_depth_ab_compression_;
  int ab_threshold_ = 10;
  int confidence_threshold_ = 10;
  bool jblf_filter_state_ = true;
  int jblf_filter_size_ = 7;
  int radial_filter_min_threshold_ = 100;
  int radial_filter_max_threshold_ = 10000;
  std::string input_file_name_;
  std::string input_sensor_ip_;
  std::string encoding_type_;
  bool enable_point_cloud_publish_ = false;
  sensor_msgs::CameraInfo cam_info_msg_;
  unsigned short* depth_frame_;
  unsigned short* ab_frame_;
  short* xyz_frame_;
  unsigned short* conf_frame_;
  ros::Publisher depth_image_publisher_;
  ros::Publisher ab_image_publisher_;
  ros::Publisher conf_image_publisher_;
  ros::Publisher xyz_image_publisher_;
  ros::Publisher depth_info_publisher_;
  CameraIntrinsics depth_intrinsics_;
  CameraExtrinsics depth_extrinsics_;
  CameraExtrinsics depth_extrinsics_external_;
  ImageProcUtils* image_proc_utils_;

  bool process_output_thread_abort_ = false;
  bool read_input_thread_abort_ = false;

  boost::mutex output_thread_mtx_;
  boost::mutex input_thread_mtx_;

  bool error_in_frame_read_ = false;

  int max_input_queue_length_ = 10;
  std::queue<ADI3DToFADTF31xxFrameInfo*> input_frames_queue_;

  int max_debug_queue_length_ = 10;
  std::queue<ADI3DToFADTF31xxOutputInfo*> output_node_queue_;

  // dynamic reconfigure parameters
  dynamic_reconfigure::Server<adi_3dtof_adtf31xx::ADTF31xxSensorParamsConfig> server_;
  dynamic_reconfigure::Server<adi_3dtof_adtf31xx::ADTF31xxSensorParamsConfig>::CallbackType
      dynamic_reconfigure_callbacktype_;
  adi_3dtof_adtf31xx::ADTF31xxSensorParamsConfig dynamic_reconfigure_config_;

  /**
   * @brief    This is the scale factor to scale the input image.
               The processing will happen on the scaled down image
               The topics and the output coordinates will correspond
               to the scaled image size.
   *
   */
  int processing_scale_ = 2;
  ros::Time curr_frame_timestamp_ = ros::Time::now();

  /**
   * @brief This image fills and publishes the camera information
   *
   * @param frame_id  frame_id of camera_info
   * @param publisher This is Ros publisher
   */
  void fillAndPublishCameraInfo(std::string frame_id, const ros::Publisher& publisher)
  {
    cam_info_msg_.header.seq = input_sensor_->getFrameCounter();
    cam_info_msg_.header.stamp = curr_frame_timestamp_;
    cam_info_msg_.header.frame_id = std::move(frame_id);

    cam_info_msg_.width = image_width_;
    cam_info_msg_.height = image_height_;

    cam_info_msg_.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

    cam_info_msg_.K.fill(0.0f);
    cam_info_msg_.K[0] = depth_intrinsics_.camera_matrix[0];
    cam_info_msg_.K[2] = depth_intrinsics_.camera_matrix[2];
    cam_info_msg_.K[4] = depth_intrinsics_.camera_matrix[4];
    cam_info_msg_.K[5] = depth_intrinsics_.camera_matrix[5];
    cam_info_msg_.K[8] = 1.0f;

    cam_info_msg_.P.fill(0.0);
    cam_info_msg_.P[0] = depth_intrinsics_.camera_matrix[0];
    cam_info_msg_.P[2] = depth_intrinsics_.camera_matrix[2];
    cam_info_msg_.P[3] = depth_extrinsics_.translation_matrix[0];
    cam_info_msg_.P[5] = depth_intrinsics_.camera_matrix[4];
    cam_info_msg_.P[6] = depth_intrinsics_.camera_matrix[5];
    cam_info_msg_.P[7] = depth_extrinsics_.translation_matrix[1];
    cam_info_msg_.P[10] = 1.0f;
    cam_info_msg_.P[11] = depth_extrinsics_.translation_matrix[2];

    cam_info_msg_.D.resize(0);
    for (float distortion_coeff : depth_intrinsics_.distortion_coeffs)
    {
      cam_info_msg_.D.push_back(distortion_coeff);
    }

    cam_info_msg_.R.fill(0.0f);
    for (int i = 0; i < 9; i++)
    {
      cam_info_msg_.R[i] = depth_extrinsics_.rotation_matrix[i];
    }

    cam_info_msg_.binning_x = 0;
    cam_info_msg_.binning_y = 0;
    cam_info_msg_.roi.do_rectify = false;
    cam_info_msg_.roi.height = 0;
    cam_info_msg_.roi.width = 0;
    cam_info_msg_.roi.x_offset = 0;
    cam_info_msg_.roi.y_offset = 0;

    publisher.publish(cam_info_msg_);
  }

  /**
   * @brief This function publishes images as Ros messages.
   *
   * @param img This is input image
   * @param encoding_type number of bits used to represent one pixel of image.
   * @param frame_id frame id of image
   * @param publisher This is ros publisher
   */
  void publishImageAsRosMsg(cv::Mat img, const std::string& encoding_type, std::string frame_id,
                            const ros::Publisher& publisher)
  {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    cv_ptr->encoding = encoding_type;
    cv_ptr->header.seq = input_sensor_->getFrameCounter();
    cv_ptr->header.stamp = curr_frame_timestamp_;
    cv_ptr->header.frame_id = std::move(frame_id);
    cv_ptr->image = std::move(img);

    publisher.publish(cv_ptr->toImageMsg());
  }

  /**
   * @brief This function publishes the point cloud
   *
   * @param xyz_frame - Pointer to xyz frame
   *
   * Note: Assumes that cam_info_msg_ is already populated
   */
  void publishPointCloud(short* xyz_frame)
  {
    sensor_msgs::PointCloud2::Ptr pointcloud_msg(new sensor_msgs::PointCloud2);

    pointcloud_msg->header.seq = input_sensor_->getFrameCounter();
    pointcloud_msg->header.stamp = curr_frame_timestamp_;
    pointcloud_msg->header.frame_id = camera_link_;
    pointcloud_msg->width = image_width_;
    pointcloud_msg->height = image_height_;
    pointcloud_msg->is_dense = false;
    pointcloud_msg->is_bigendian = false;

    // XYZ data from sensor.
    // This data is in 16 bpp format.
    short* xyz_sensor_buf = xyz_frame;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*pointcloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud_msg, "z");
    for (int i = 0; i < image_height_; i++)
    {
      for (int j = 0; j < image_width_; j++)
      {
        *iter_x = (float)(*xyz_sensor_buf++) / 1000.0f;
        *iter_y = (float)(*xyz_sensor_buf++) / 1000.0f;
        *iter_z = (float)(*xyz_sensor_buf++) / 1000.0f;
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
    }

    // Publisher
    xyz_image_publisher_.publish(pointcloud_msg);
  }

  /**
   * @brief Publishes the image and camera information.
   * This function publishes the camera information and
   * depth, AB, and confidence images.
   *
   * @param out_frame Pointer to the output frame containing the compressed depth frame and its size.
   */
  void publishImageAndCameraInfo(ADI3DToFADTF31xxOutputInfo* out_frame)
  {
    // Publish Camera info
    fillAndPublishCameraInfo(camera_link_, depth_info_publisher_);

    if (enable_depth_ab_compression_ == true)
    {
      // Publish compressed depth,ab and confidence image
      PROFILE_FUNCTION_START(Publish_CompressImg)
      publishRVLCompressedImageAsRosMsg(out_frame->compressed_depth_frame_, out_frame->compressed_depth_frame_size_,
                                        encoding_type_, camera_link_, depth_image_publisher_);

      publishRVLCompressedImageAsRosMsg(out_frame->compressed_ab_frame_, out_frame->compressed_ab_frame_size_,
                                        encoding_type_, camera_link_, ab_image_publisher_);

      publishRVLCompressedImageAsRosMsg(out_frame->compressed_conf_frame_, out_frame->compressed_conf_frame_size_,
                                        encoding_type_, camera_link_, conf_image_publisher_);
      PROFILE_FUNCTION_END(Publish_CompressImg)
    }
    else
    {
      // Publish uncompressed depth, ab and confidence image
      // convert to CV format.
      cv::Mat m_disp_image_depth, m_disp_image_ab, m_disp_image_conf;
      m_disp_image_depth = cv::Mat(image_height_, image_width_, CV_16UC1, out_frame->depth_frame_);
      m_disp_image_ab = cv::Mat(image_height_, image_width_, CV_16UC1, out_frame->ab_frame_);
      m_disp_image_conf = cv::Mat(image_height_, image_width_, CV_16UC1, out_frame->conf_frame_);

      // Publish image as Ros message
      publishImageAsRosMsg(m_disp_image_depth, encoding_type_, camera_link_, depth_image_publisher_);
      publishImageAsRosMsg(m_disp_image_ab, encoding_type_, camera_link_, ab_image_publisher_);
      publishImageAsRosMsg(m_disp_image_conf, encoding_type_, camera_link_, conf_image_publisher_);
    }

    PROFILE_FUNCTION_START(publish_PointCloud)
    if (enable_point_cloud_publish_ == true)
    {
      // Publish point cloud
      publishPointCloud(out_frame->xyz_frame_);
    }
    PROFILE_FUNCTION_END(publish_PointCloud)
  }

  /**
   * @brief This function publishes a compressed image as Ros message.
   *
   * @param compressed_img Compressed image buffer
   * @param compressed_img_size Compressed image size in bytes
   * @param encoding_type number of bits used to represent one pixel of image.
   * @param frame_id frame id of image
   * @param publisher ROS publisher handle
   */
  void publishRVLCompressedImageAsRosMsg(unsigned char* compressed_img, int compressed_img_size,
                                         const std::string& encoding_type, std::string frame_id,
                                         const ros::Publisher& publisher)
  {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    sensor_msgs::CompressedImage::Ptr compressed_payload_ptr(new sensor_msgs::CompressedImage());

    compressed_payload_ptr->format = encoding_type + ";compressedDepth rvl";
    compressed_payload_ptr->header.seq = input_sensor_->getFrameCounter();
    compressed_payload_ptr->header.stamp = curr_frame_timestamp_;
    compressed_payload_ptr->header.frame_id = std::move(frame_id);
    compressed_payload_ptr->data.resize(compressed_img_size + 8 +
                                        sizeof(compressed_depth_image_transport::ConfigHeader));

    // Adding header to compressed depth data as image transport subscribers can decompress.
    compressed_depth_image_transport::ConfigHeader compressionConfiguration{};
    compressionConfiguration.format = compressed_depth_image_transport::INV_DEPTH;

    float depthQuantization = 0;
    float maximumDepth = 1;

    // Inverse depth quantization parameters
    float depthQuantizationA = depthQuantization * (depthQuantization + 1.0f);
    float depthQuantizationB = 1.0f - depthQuantizationA / maximumDepth;
    compressionConfiguration.depthParam[0] = depthQuantizationA;
    compressionConfiguration.depthParam[1] = depthQuantizationB;

    memcpy(&compressed_payload_ptr->data[0], &compressionConfiguration,
           sizeof(compressed_depth_image_transport::ConfigHeader));
    memcpy(&compressed_payload_ptr->data[0] + sizeof(compressed_depth_image_transport::ConfigHeader), &image_width_,
           sizeof(int));
    memcpy(&compressed_payload_ptr->data[4] + sizeof(compressed_depth_image_transport::ConfigHeader), &image_height_,
           sizeof(int));

    memcpy(&compressed_payload_ptr->data[8] + sizeof(compressed_depth_image_transport::ConfigHeader), compressed_img,
           compressed_img_size);

    publisher.publish(compressed_payload_ptr);
  }

  void initSettingsForDynamicReconfigure();
};

#endif
