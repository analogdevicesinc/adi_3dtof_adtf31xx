/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_ADTF31XX__H
#define ADI_3DTOF_ADTF31XX__H

#include <compressed_depth_image_transport/compression_common.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/point_cloud2.h>

#include <chrono>
#include <functional>
#include <memory>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <string>

#include "adi_3dtof_adtf31xx_frame_info.h"
#include "adi_3dtof_adtf31xx_output_info.h"
#include "image_proc_utils.h"
#include "input_sensor.h"
#include "input_sensor_factory.h"
#include "module_profile.h"
#include "ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.h"

using namespace std::chrono_literals;
namespace enc = sensor_msgs::image_encodings;

/**
 * @brief This is main class for this package
 *
 */
class ADI3DToFADTF31xx : public rclcpp::Node
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
  ADI3DToFADTF31xx() : Node("adi_3dtof_adtf31xx_node")
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

    // The values represent the configuration values used in "rmw_qos_profile_default" profile
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_profile.depth = 10;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    qos_profile.deadline = RMW_QOS_DEADLINE_DEFAULT;
    qos_profile.lifespan = RMW_QOS_LIFESPAN_DEFAULT;
    qos_profile.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    qos_profile.liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    qos_profile.avoid_ros_namespace_conventions = false;

    // ToF camera link
    rcl_interfaces::msg::ParameterDescriptor base_camera_link_descriptor{};
    base_camera_link_descriptor.read_only = true;
    base_camera_link_descriptor.description = "Base camera frame name";
    this->declare_parameter<std::string>(
      "param_camera_link", "adi_camera_link_def", base_camera_link_descriptor);

    // Input sensor Mode:Camera/File/ROSBag
    rcl_interfaces::msg::ParameterDescriptor input_sensor_mode_descriptor{};
    input_sensor_mode_descriptor.read_only = true;
    input_sensor_mode_descriptor.description = "Input mode: 0:Camera, 2:FileIO";
    this->declare_parameter<int>("param_input_sensor_mode", 0, input_sensor_mode_descriptor);

    // Input file name in FileIO mode
    rcl_interfaces::msg::ParameterDescriptor input_file_name_descriptor{};
    input_file_name_descriptor.read_only = true;
    input_file_name_descriptor.description = "Input file name in FileIO mode";
    this->declare_parameter<std::string>(
      "param_input_file_name", "no name", input_file_name_descriptor);

    // Enable option to publish depth and ir compressed image
    rcl_interfaces::msg::ParameterDescriptor enable_depth_ir_compressed_image_descriptor{};
    enable_depth_ir_compressed_image_descriptor.read_only = true;
    enable_depth_ir_compressed_image_descriptor.description =
      "Unchecked: Publishes uncompressed images, Checked: Publishes compressed images";
    this->declare_parameter<bool>(
      "param_enable_depth_ir_compression", false, enable_depth_ir_compressed_image_descriptor);

    // Path of the config file to read from tof sdk
    rcl_interfaces::msg::ParameterDescriptor path_of_the_config_file_descriptor{};
    path_of_the_config_file_descriptor.read_only = true;
    path_of_the_config_file_descriptor.description = "Path of the configuaration files";
    this->declare_parameter<std::string>(
      "param_config_file_name_of_tof_sdk", "no name", path_of_the_config_file_descriptor);

    // Frame type supported in TOF SDK
    rcl_interfaces::msg::ParameterDescriptor frame_type_descriptor{};
    frame_type_descriptor.read_only = true;
    frame_type_descriptor.description = "Frame type";
    this->declare_parameter<std::string>("param_frame_type", "no name", frame_type_descriptor);

    // ab threshold to be set to device
    rcl_interfaces::msg::ParameterDescriptor ab_threshold_descriptor{};
    rcl_interfaces::msg::IntegerRange ab_threshold_range;
    ab_threshold_range.set__from_value(1).set__to_value(255).set__step(1);
    ab_threshold_descriptor.integer_range = {ab_threshold_range};
    ab_threshold_descriptor.description = "Set ab threshold value to device";
    this->declare_parameter<int>("param_ab_threshold", 10, ab_threshold_descriptor);

    // confidence threshold to be set to device
    rcl_interfaces::msg::ParameterDescriptor confidence_threshold_descriptor{};
    rcl_interfaces::msg::IntegerRange confidence_threshold_range;
    confidence_threshold_range.set__from_value(1).set__to_value(255).set__step(1);
    confidence_threshold_descriptor.integer_range = {confidence_threshold_range};
    confidence_threshold_descriptor.description = "Set confidence threshold value to device";
    this->declare_parameter<int>("param_confidence_threshold", 10, confidence_threshold_descriptor);

    std::string config_file_name_of_tof_sdk;
    std::string frame_type;
    camera_link_ =
      this->get_parameter("param_camera_link").get_parameter_value().get<std::string>();
    input_sensor_mode_ =
      this->get_parameter("param_input_sensor_mode").get_parameter_value().get<int>();

    input_file_name_ =
      this->get_parameter("param_input_file_name").get_parameter_value().get<std::string>();
    enable_depth_ir_compression_ =
      this->get_parameter("param_enable_depth_ir_compression").get_parameter_value().get<bool>();
    ab_threshold_ = this->get_parameter("param_ab_threshold").get_parameter_value().get<int>();
    confidence_threshold_ =
      this->get_parameter("param_confidence_threshold").get_parameter_value().get<int>();
    config_file_name_of_tof_sdk = this->get_parameter("param_config_file_name_of_tof_sdk")
                                    .get_parameter_value()
                                    .get<std::string>();
    frame_type = this->get_parameter("param_frame_type").get_parameter_value().get<std::string>();

    tunable_params_.ab_threshold = ab_threshold_;
    tunable_params_.confidence_threshold = confidence_threshold_;
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ADI3DToFADTF31xx::parametersCallback, this, std::placeholders::_1));

    frame_number_ = 0;

    timer_ = this->create_wall_timer(25ms, std::bind(&ADI3DToFADTF31xx::timer_callback, this));

    // Get input sensor module
    input_sensor_ = InputSensorFactory::getInputSensor(input_sensor_mode_);

    // Open the sensor
    input_sensor_->openSensor(
      input_file_name_, image_width_, image_height_, processing_scale_,
      config_file_name_of_tof_sdk);
    if (!input_sensor_->isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open the sensor %s", input_file_name_.c_str());
      rclcpp::shutdown();
    }

    input_sensor_->setProcessingScale(processing_scale_);

    // Configure the sensor
    input_sensor_->configureSensor(frame_type);

    // Buffer allocations.
    image_width_ = input_sensor_->getFrameWidth();
    image_height_ = input_sensor_->getFrameHeight();
    xyz_frame_ = nullptr;

    // Get intrinsics and extrinsics
    input_sensor_->getIntrinsics(&depth_intrinsics_);
    input_sensor_->getExtrinsics(&depth_extrinsics_);

    // Create publishers.
    // Input and Intermediate Debug Images
    if (enable_depth_ir_compression_ == 1) {
      compressed_depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "depth_image/compressedDepth",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));
      compressed_ir_image_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "ir_image/compressedDepth",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));
    } else {
      depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "depth_image", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));
      ir_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "ir_image", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));
    }

    //xyz_image_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud",
    //                  rclcpp::QoS( rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile ) );

    // Camera Infos
    depth_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));

    image_proc_utils_ = new ImageProcUtils(&depth_intrinsics_, image_width_, image_height_);

    input_sensor_->setABinvalidationThreshold(ab_threshold_);

    input_sensor_->setConfidenceThreshold(confidence_threshold_);
  }

  /**
   * @brief Destroy the ADI3DToFADTF31xx object
   *
   */
  ~ADI3DToFADTF31xx()
  {
    if (image_proc_utils_ != nullptr) {
      delete image_proc_utils_;
      image_proc_utils_ = nullptr;
    }
  }

  void timer_callback()
  {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "adi_3dtof_adtf31xx_node : Running loop : " << frame_number_);
    if (!readNextFrame()) {
      //Raise an exception to shutdown the Node.
      throw std::runtime_error("Error Reading next Frame..");
    }
  }

  bool readNextFrame();
  void processOutputAbort();
  void adtf31xxSensorPushOutputNode(ADI3DToFADTF31xxOutputInfo * new_output_node);
  void processOutput();
  void readInputAbort();
  ADI3DToFADTF31xxFrameInfo * adtf31xxSensorGetNextFrame();
  void readInput();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  IInputSensor * input_sensor_;
  std::string camera_link_;
  int input_sensor_mode_;
  int image_width_ = 1024;
  int image_height_ = 1024;
  int frame_number_;
  bool enable_depth_ir_compression_;
  int ab_threshold_ = 10;
  int confidence_threshold_ = 10;
  std::string input_file_name_;
  sensor_msgs::msg::CameraInfo cam_info_msg_;
  unsigned short * depth_frame_;
  unsigned short * ir_frame_;
  short * xyz_frame_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_ir_image_publisher_;
  //rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr xyz_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_publisher_;
  CameraIntrinsics depth_intrinsics_;
  CameraExtrinsics depth_extrinsics_;
  CameraExtrinsics depth_extrinsics_external_;
  ImageProcUtils * image_proc_utils_;

  bool process_output_thread_abort_ = false;
  bool read_input_thread_abort_ = false;

  std::mutex output_thread_mtx_;
  std::mutex input_thread_mtx_;

  bool error_in_frame_read_ = false;

  int max_input_queue_length_ = 10;
  std::queue<ADI3DToFADTF31xxFrameInfo *> input_frames_queue_;

  int max_debug_queue_length_ = 10;
  std::queue<ADI3DToFADTF31xxOutputInfo *> output_node_queue_;

  /**
   * @brief    This is the scale factor to scale the input image.
               The processing will happen on the scaled down image
               The topics and the output coordinates will correspond
               to the scaled image size.
   *
   */
  int processing_scale_ = 2;
  rclcpp::Time curr_frame_timestamp_ = rclcpp::Clock{}.now();

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  struct TunableParameters
  {
    int ab_threshold;
    int confidence_threshold;
  };

  TunableParameters tunable_params_;

  /**
   * @brief Callback for set parameter event, here the parameter struct is copied, the actual values get updated in the process loop.
   *
   * @param config Config parameters present in GUI
   * @param level
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    // Copy the parameters vector to a local variable.
    for (const auto & param : parameters) {
      if (param.get_name() == "param_ab_threshold") {
        if (tunable_params_.ab_threshold != param.as_int()) {
          tunable_params_.ab_threshold = param.as_int();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_ab_threshold is changed to %s",
            param.value_to_string().c_str());
        }
      }

      if (param.get_name() == "param_confidence_threshold") {
        if (tunable_params_.confidence_threshold != param.as_int()) {
          tunable_params_.confidence_threshold = param.as_int();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_confidence_threshold is changed to %s",
            param.value_to_string().c_str());
        }
      }
    }
    return result;
  }

  /**
   * @brief updates the parameter of input image based on dynamic reconfigure.
   *
   */
  void updateTunableParameters()
  {
    // setting AB threshold and confidence threshold values if they are changed.
    if (ab_threshold_ != tunable_params_.ab_threshold) {
      ab_threshold_ = tunable_params_.ab_threshold;
      RCLCPP_INFO(this->get_logger(), "Changed AB threshold value is %d", ab_threshold_);
      input_sensor_->setABinvalidationThreshold(ab_threshold_);
    }

    if (confidence_threshold_ != tunable_params_.confidence_threshold) {
      confidence_threshold_ = tunable_params_.confidence_threshold;
      RCLCPP_INFO(
        this->get_logger(), "Changed Confidence threshold value is %d", confidence_threshold_);
      input_sensor_->setConfidenceThreshold(confidence_threshold_);
    }
  }

  /**
   * @brief This image fills and publishes the camera information
   *
   * @param frame_id  frame_id of camera_info
   * @param publisher This is Ros publisher
   */
  void fillAndPublishCameraInfo(
    std::string frame_id,
    const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher)
  {
    cam_info_msg_.header.stamp = curr_frame_timestamp_;
    cam_info_msg_.header.frame_id = std::move(frame_id);

    cam_info_msg_.width = image_width_;
    cam_info_msg_.height = image_height_;

    cam_info_msg_.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;

    cam_info_msg_.k.fill(0.0f);
    cam_info_msg_.k[0] = depth_intrinsics_.camera_matrix[0];
    cam_info_msg_.k[2] = depth_intrinsics_.camera_matrix[2];
    cam_info_msg_.k[4] = depth_intrinsics_.camera_matrix[4];
    cam_info_msg_.k[5] = depth_intrinsics_.camera_matrix[5];
    cam_info_msg_.k[8] = 1.0f;

    cam_info_msg_.p.fill(0.0);
    cam_info_msg_.p[0] = depth_intrinsics_.camera_matrix[0];
    cam_info_msg_.p[2] = depth_intrinsics_.camera_matrix[2];
    cam_info_msg_.p[3] = depth_extrinsics_.translation_matrix[0];
    cam_info_msg_.p[5] = depth_intrinsics_.camera_matrix[4];
    cam_info_msg_.p[6] = depth_intrinsics_.camera_matrix[5];
    cam_info_msg_.p[7] = depth_extrinsics_.translation_matrix[1];
    cam_info_msg_.p[10] = 1.0f;
    cam_info_msg_.p[11] = depth_extrinsics_.translation_matrix[2];

    cam_info_msg_.d.resize(0);
    for (float distortion_coeff : depth_intrinsics_.distortion_coeffs) {
      cam_info_msg_.d.push_back(distortion_coeff);
    }

    cam_info_msg_.r.fill(0.0f);
    for (int i = 0; i < 9; i++) {
      cam_info_msg_.r[i] = depth_extrinsics_.rotation_matrix[i];
    }

    cam_info_msg_.binning_x = 0;
    cam_info_msg_.binning_y = 0;
    cam_info_msg_.roi.do_rectify = false;
    cam_info_msg_.roi.height = 0;
    cam_info_msg_.roi.width = 0;
    cam_info_msg_.roi.x_offset = 0;
    cam_info_msg_.roi.y_offset = 0;

    publisher->publish(cam_info_msg_);
  }

  /**
   * @brief This function publishes images as Ros messages.
   *
   * @param img This is input image
   * @param encoding_type number of bits used to represent one pixel of image.
   * @param frame_id frame id of image
   * @param publisher This is ros publisher
   */
  void publishImageAsRosMsg(
    cv::Mat img, const std::string & encoding_type, std::string frame_id,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher)
  {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    cv_ptr->encoding = encoding_type;
    cv_ptr->header.stamp = curr_frame_timestamp_;
    cv_ptr->header.frame_id = std::move(frame_id);
    cv_ptr->image = std::move(img);

    publisher->publish(*cv_ptr->toImageMsg());
  }

  /**
   * @brief This function publishes the point cloud
   *
   * Note: Assumes that cam_info_msg_ is already populated
   */
  void publishPointCloud(short * xyz_frame)
  {
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg(new sensor_msgs::msg::PointCloud2);

    pointcloud_msg->header.stamp = curr_frame_timestamp_;
    pointcloud_msg->header.frame_id = camera_link_;
    pointcloud_msg->width = image_width_;
    pointcloud_msg->height = image_height_;
    pointcloud_msg->is_dense = false;
    pointcloud_msg->is_bigendian = false;

    // XYZ data from sensor.
    // This data is in 16 bpp format.
    short * xyz_sensor_buf = xyz_frame;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*pointcloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud_msg, "z");
    for (int i = 0; i < image_height_; i++) {
      for (int j = 0; j < image_width_; j++) {
        *iter_x = (float)(*xyz_sensor_buf++) / 1000.0f;
        *iter_y = (float)(*xyz_sensor_buf++) / 1000.0f;
        *iter_z = (float)(*xyz_sensor_buf++) / 1000.0f;
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
    }

    // Publisher
    //xyz_image_publisher_->publish(*pointcloud_msg);
  }

  /**
   * @brief This function publihes the camera info, compressed depth and ir images.
   *
   * @param compressed_depth_frame - Pointer to compressed depth image
   * @param compressed_depth_frame_size - Buffer size(compressed depth image)
   * @param compressed_ir_frame - Pointer to compressed ir image
   * @param compressed_ir_frame_size - Buffer size(compressed ir image)
   */
  void publishImageAndCameraInfo(
    unsigned char * compressed_depth_frame, int compressed_depth_frame_size,
    unsigned char * compressed_ir_frame, int compressed_ir_frame_size, short * /*xyz_frame*/)
  {
    // Publish image as Ros message
    cv::Mat m_disp_image_depth, m_disp_image_ir, m_disp_virtual_depth;

    // convert to 16 bit depth and IR image of CV format.
    m_disp_image_depth = cv::Mat(image_height_, image_width_, CV_16UC1, depth_frame_);
    m_disp_image_ir = cv::Mat(image_height_, image_width_, CV_16UC1, ir_frame_);

    fillAndPublishCameraInfo(camera_link_, depth_info_publisher_);

    if (enable_depth_ir_compression_ == true) {
      PROFILE_FUNCTION_START(Publish_CompressImg)
      publishRVLCompressedImageAsRosMsg(
        compressed_depth_frame, compressed_depth_frame_size, "mono16", camera_link_,
        compressed_depth_image_publisher_);

      publishRVLCompressedImageAsRosMsg(
        compressed_ir_frame, compressed_ir_frame_size, "mono16", camera_link_,
        compressed_ir_image_publisher_);
      PROFILE_FUNCTION_END(Publish_CompressImg)
    } else {
      publishImageAsRosMsg(m_disp_image_depth, "mono16", camera_link_, depth_image_publisher_);
      publishImageAsRosMsg(m_disp_image_ir, "mono16", camera_link_, ir_image_publisher_);
    }

    PROFILE_FUNCTION_START(publish_PointCloud)
    // PublishPointCloud
    //publishPointCloud(xyz_frame);
    PROFILE_FUNCTION_END(publish_PointCloud)
  }

  /**
   * @brief This function publishes depth image , ir image, point-cloud and camera info.
   *
   * @param depth_frame - Pointer to the depth frame buffer
   * @param ir_frame - Pointer to the ir frame buffer
   */
  void publishImageAndCameraInfo(
    unsigned short * depth_frame, unsigned short * ir_frame, short * /*xyz_frame*/)
  {
    // Publish image as Ros message
    cv::Mat m_disp_image_depth, temp_depth, m_disp_image_ir, m_disp_virtual_depth;

    // convert to 16 bit depth and IR image of CV format.
    m_disp_image_depth = cv::Mat(image_height_, image_width_, CV_16UC1, depth_frame);
    m_disp_image_ir = cv::Mat(image_height_, image_width_, CV_16UC1, ir_frame);

    fillAndPublishCameraInfo(camera_link_, depth_info_publisher_);
    publishImageAsRosMsg(m_disp_image_depth, "mono16", camera_link_, depth_image_publisher_);
    publishImageAsRosMsg(m_disp_image_ir, "mono16", camera_link_, ir_image_publisher_);

    //publishPointCloud(xyz_frame);
  }

  /**
   * @brief This function publishes a compressed image as Ros message.
   *
   * @param compressed_img Compressed image buffer
   * @param compressed_img_size Compressed image size in bytes
   * @param frame_id frame id of image
   * @param publisher ROS publisher handle
   */
  void publishRVLCompressedImageAsRosMsg(
    unsigned char * compressed_img, int compressed_img_size, const std::string & encoding_type,
    std::string frame_id,
    const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher)
  {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    sensor_msgs::msg::CompressedImage::SharedPtr compressed_payload_ptr(
      new sensor_msgs::msg::CompressedImage());

    compressed_payload_ptr->format = encoding_type + ";compressedDepth rvl";
    compressed_payload_ptr->header.stamp = curr_frame_timestamp_;
    compressed_payload_ptr->header.frame_id = std::move(frame_id);
    compressed_payload_ptr->data.resize(
      compressed_img_size + 8 + sizeof(compressed_depth_image_transport::ConfigHeader));

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

    memcpy(
      &compressed_payload_ptr->data[0], &compressionConfiguration,
      sizeof(compressed_depth_image_transport::ConfigHeader));
    memcpy(
      &compressed_payload_ptr->data[0] + sizeof(compressed_depth_image_transport::ConfigHeader),
      &image_width_, sizeof(int));
    memcpy(
      &compressed_payload_ptr->data[4] + sizeof(compressed_depth_image_transport::ConfigHeader),
      &image_height_, sizeof(int));
    memcpy(
      &compressed_payload_ptr->data[8] + sizeof(compressed_depth_image_transport::ConfigHeader),
      compressed_img, compressed_img_size);

    publisher->publish(*compressed_payload_ptr);
  }
};

#endif
