/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#ifndef ADI_3DTOF_ADTF31XX__H
#define ADI_3DTOF_ADTF31XX__H

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#endif
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
#include "ros-perception/image_transport_plugins/compressed_depth_image_transport/compression_common.h"

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

    // Enable option to publish depth and ab compressed image
    rcl_interfaces::msg::ParameterDescriptor enable_depth_ab_compressed_image_descriptor{};
    enable_depth_ab_compressed_image_descriptor.read_only = true;
    enable_depth_ab_compressed_image_descriptor.description =
      "Unchecked: Publishes uncompressed images, Checked: Publishes compressed images";
    this->declare_parameter<bool>(
      "param_enable_depth_ab_compression", false, enable_depth_ab_compressed_image_descriptor);

    // Enable option to publish depth image
    rcl_interfaces::msg::ParameterDescriptor enable_depth_image_publish_descriptor{};
    enable_depth_image_publish_descriptor.read_only = true;
    enable_depth_image_publish_descriptor.description =
      "Unchecked: Do not publish depth image, Checked: Publish depth image";
    this->declare_parameter<bool>(
      "param_enable_depth_publish", true, enable_depth_image_publish_descriptor);

    // Enable option to publish ab image
    rcl_interfaces::msg::ParameterDescriptor enable_ab_image_publish_descriptor{};
    enable_ab_image_publish_descriptor.description =
      "Unchecked: Do not publish ab image, Checked: Publish ab image";
    this->declare_parameter<bool>(
      "param_enable_ab_publish", true, enable_ab_image_publish_descriptor);

    // Enable option to publish confidence image
    rcl_interfaces::msg::ParameterDescriptor enable_conf_image_publish_descriptor{};
    enable_conf_image_publish_descriptor.description =
      "Unchecked: Do not publish confidence image, Checked: Publish confidence image";
    this->declare_parameter<bool>(
      "param_enable_conf_publish", true, enable_conf_image_publish_descriptor);

    // Enable option to publish xyz image
    rcl_interfaces::msg::ParameterDescriptor enable_point_cloud_image_publish_descriptor{};
    enable_point_cloud_image_publish_descriptor.description =
      "Unchecked: Do not publish Point cloud image, Checked: Publish Point cloud image";
    this->declare_parameter<bool>(
      "param_enable_point_cloud_publish", false, enable_point_cloud_image_publish_descriptor);

    // Path of the config file to read from tof sdk
    rcl_interfaces::msg::ParameterDescriptor path_of_the_config_file_descriptor{};
    path_of_the_config_file_descriptor.read_only = true;
    path_of_the_config_file_descriptor.description = "Path of the configuaration files";
    this->declare_parameter<std::string>(
      "param_config_file_name_of_tof_sdk", "no name", path_of_the_config_file_descriptor);

    // Frame type supported in TOF SDK
    rcl_interfaces::msg::ParameterDescriptor camera_mode_descriptor{};
    camera_mode_descriptor.read_only = true;
    camera_mode_descriptor.description = "Camera Mode";
    this->declare_parameter<int>("param_camera_mode", 3, camera_mode_descriptor);

    // ip address of the sensor
    rcl_interfaces::msg::ParameterDescriptor ip_address_of_sensor_descriptor{};
    ip_address_of_sensor_descriptor.read_only = true;
    ip_address_of_sensor_descriptor.description = "IP address of the sensor";
    this->declare_parameter<std::string>(
      "param_input_sensor_ip", "no name", ip_address_of_sensor_descriptor);

    // Encoding type
    rcl_interfaces::msg::ParameterDescriptor param_encoding_type_descriptor{};    
    param_encoding_type_descriptor.description = "Frame encoding type, allowed values: mono16, 16UC1";
    this->declare_parameter<std::string>(
      "param_encoding_type", "mono16", param_encoding_type_descriptor);

    // ab threshold to be set to device
    rcl_interfaces::msg::ParameterDescriptor ab_threshold_descriptor{};
    rcl_interfaces::msg::IntegerRange ab_threshold_range;
    ab_threshold_range.set__from_value(1).set__to_value(255);
    ab_threshold_descriptor.integer_range = {ab_threshold_range};
    ab_threshold_descriptor.description = "Set ab threshold value to device";
    this->declare_parameter<int>("param_ab_threshold", 10, ab_threshold_descriptor);

    // confidence threshold to be set to device
    rcl_interfaces::msg::ParameterDescriptor confidence_threshold_descriptor{};
    rcl_interfaces::msg::IntegerRange confidence_threshold_range;
    confidence_threshold_range.set__from_value(1).set__to_value(255);
    confidence_threshold_descriptor.integer_range = {confidence_threshold_range};
    confidence_threshold_descriptor.description = "Set confidence threshold value to device";
    this->declare_parameter<int>("param_confidence_threshold", 10, confidence_threshold_descriptor);

    rcl_interfaces::msg::ParameterDescriptor enable_jblf_filter_descriptor{};
    enable_jblf_filter_descriptor.description =
      "Unchecked: Disable JBLF Filter, Checked: Enable JBLF Filter";
    this->declare_parameter<bool>("param_enable_jblf_filter", true, enable_jblf_filter_descriptor);

    rcl_interfaces::msg::ParameterDescriptor jblf_filter_size_descriptor{};
    rcl_interfaces::msg::IntegerRange jblf_filter_size;
    jblf_filter_size_descriptor.description =
      "Set JBLF Filter size,Allowed values: 3, 5, 7, 9, 11, 13, 15";
    jblf_filter_size_descriptor.additional_constraints = "Allowed values: 3, 5, 7, 9, 11, 13, 15";
    this->declare_parameter<int>("param_jblf_filter_size", 7, jblf_filter_size_descriptor);

    rcl_interfaces::msg::ParameterDescriptor radial_threshold_min_descriptor{};
    rcl_interfaces::msg::IntegerRange radial_threshold_min_range;
    radial_threshold_min_range.set__from_value(1).set__to_value(300);
    radial_threshold_min_descriptor.integer_range = {radial_threshold_min_range};
    radial_threshold_min_descriptor.description = "Set minimum value for Radial threshold";
    this->declare_parameter<int>(
      "param_radial_threshold_min", 100, radial_threshold_min_descriptor);

    rcl_interfaces::msg::ParameterDescriptor radial_threshold_max_descriptor{};
    rcl_interfaces::msg::IntegerRange radial_threshold_max_range;
    radial_threshold_max_range.set__from_value(6000).set__to_value(12000);
    radial_threshold_max_descriptor.integer_range = {radial_threshold_max_range};
    radial_threshold_max_descriptor.description = "Set maximum value for Radial threshold";
    this->declare_parameter<int>(
      "param_radial_threshold_max", 10000, radial_threshold_max_descriptor);

    enable_jblf_filter_ =
      this->get_parameter("param_enable_jblf_filter").get_parameter_value().get<bool>();

    jblf_filter_size_ =
      this->get_parameter("param_jblf_filter_size").get_parameter_value().get<int>();

    radial_threshold_min_ =
      this->get_parameter("param_radial_threshold_min").get_parameter_value().get<int>();

    radial_threshold_max_ =
      this->get_parameter("param_radial_threshold_max").get_parameter_value().get<int>();

    std::string config_file_name_of_tof_sdk;
    int camera_mode;
    camera_link_ =
      this->get_parameter("param_camera_link").get_parameter_value().get<std::string>();
    input_sensor_mode_ =
      this->get_parameter("param_input_sensor_mode").get_parameter_value().get<int>();

    input_file_name_ =
      this->get_parameter("param_input_file_name").get_parameter_value().get<std::string>();
    enable_depth_ab_compression_ =
      this->get_parameter("param_enable_depth_ab_compression").get_parameter_value().get<bool>();
    enable_depth_publish_ =
      this->get_parameter("param_enable_depth_publish").get_parameter_value().get<bool>();
    enable_ab_publish_ =
      this->get_parameter("param_enable_ab_publish").get_parameter_value().get<bool>();
    enable_conf_publish_ =
      this->get_parameter("param_enable_conf_publish").get_parameter_value().get<bool>();
    enable_xyz_publish_ =
      this->get_parameter("param_enable_point_cloud_publish").get_parameter_value().get<bool>();
    ab_threshold_ = this->get_parameter("param_ab_threshold").get_parameter_value().get<int>();
    confidence_threshold_ =
      this->get_parameter("param_confidence_threshold").get_parameter_value().get<int>();
    config_file_name_of_tof_sdk = this->get_parameter("param_config_file_name_of_tof_sdk")
                                    .get_parameter_value()
                                    .get<std::string>();
    camera_mode = this->get_parameter("param_camera_mode").get_parameter_value().get<int>();
    input_sensor_ip_ =
      this->get_parameter("param_input_sensor_ip").get_parameter_value().get<std::string>();
    encoding_type_ =
      this->get_parameter("param_encoding_type").get_parameter_value().get<std::string>();

    tunable_params_.ab_threshold = ab_threshold_;
    tunable_params_.confidence_threshold = confidence_threshold_;
    tunable_params_.enable_depth_publish = enable_depth_publish_;
    tunable_params_.enable_ab_publish = enable_ab_publish_;
    tunable_params_.enable_conf_publish = enable_conf_publish_;
    tunable_params_.enable_xyz_publish = enable_xyz_publish_;
    tunable_params_.enable_jblf_filter = enable_jblf_filter_;
    tunable_params_.jblf_filter_size = jblf_filter_size_;
    tunable_params_.radial_threshold_min = radial_threshold_min_;
    tunable_params_.radial_threshold_max = radial_threshold_max_;
    tunable_params_.encoding_type = encoding_type_;

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&ADI3DToFADTF31xx::parametersCallback, this, std::placeholders::_1));

    frame_number_ = 0;

    timer_ = this->create_wall_timer(25ms, std::bind(&ADI3DToFADTF31xx::timerCallback, this));

    // Get input sensor module
    input_sensor_ = InputSensorFactory::getInputSensor(input_sensor_mode_);

    // Open the sensor
    if (input_sensor_mode_ != 3) {
      // If the mode is not ADTF31xx sensor over Network, then the ip should be set to ""
      input_sensor_ip_.clear();
    }
    input_sensor_->openSensor(
      input_file_name_, image_width_, image_height_, config_file_name_of_tof_sdk, input_sensor_ip_);
    if (!input_sensor_->isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open the sensor %s", input_file_name_.c_str());
      rclcpp::shutdown();
    }

    // Configure the sensor
    input_sensor_->configureSensor(camera_mode);

    input_sensor_->setJBLFFilterState(enable_jblf_filter_);

    input_sensor_->setJBLFFilterSize(jblf_filter_size_);

    // Buffer allocations.
    image_width_ = input_sensor_->getFrameWidth();
    image_height_ = input_sensor_->getFrameHeight();

    // Get intrinsics and extrinsics
    input_sensor_->getIntrinsics(&depth_intrinsics_);
    input_sensor_->getExtrinsics(&depth_extrinsics_);

    // Create publishers.
    // Input and Intermediate Debug Images
    if (enable_depth_ab_compression_ == 1) {
      compressed_depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "depth_image/compressedDepth",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));
      compressed_ab_image_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "ab_image/compressedDepth",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));
      compressed_conf_image_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "conf_image/compressedDepth",
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));
    } else {
      depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "depth_image", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));
      ab_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "ab_image", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));
      conf_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "conf_image", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));
    }

    xyz_image_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "point_cloud", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));

    // Camera Infos
    depth_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));

    image_proc_utils_ = new ImageProcUtils(&depth_intrinsics_, image_width_, image_height_);

    input_sensor_->setABinvalidationThreshold(ab_threshold_);

    input_sensor_->setConfidenceThreshold(confidence_threshold_);
       
    input_sensor_->setRadialFilterMinThreshold(radial_threshold_min_);

    input_sensor_->setRadialFilterMaxThreshold(radial_threshold_max_);
  }

  /**
   * @brief Destroy the ADI3DToFADTF31xx object
   *
   */
  ~ADI3DToFADTF31xx() override
  {
    if (image_proc_utils_ != nullptr) {
      delete image_proc_utils_;
      image_proc_utils_ = nullptr;
    }
  }

  void timerCallback()
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
  bool enable_depth_ab_compression_;
  bool enable_depth_publish_;
  bool enable_ab_publish_;
  bool enable_conf_publish_;
  bool enable_xyz_publish_;
  int ab_threshold_ = 10;
  int confidence_threshold_ = 10;
  std::string input_file_name_;
  std::string input_sensor_ip_;
  std::string encoding_type_;
  sensor_msgs::msg::CameraInfo cam_info_msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ab_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr conf_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_ab_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_conf_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr xyz_image_publisher_;
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

  bool enable_jblf_filter_ = false;
  int jblf_filter_size_ = 7;
  int radial_threshold_min_ = 100;
  int radial_threshold_max_ = 10000;

  /**
   * @brief    This is the scale factor to scale the input image.
               The processing will happen on the scaled down image
               The topics and the output coordinates will correspond
               to the scaled image size.
   *
   */  
  rclcpp::Time curr_frame_timestamp_ = rclcpp::Clock{}.now();

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  struct TunableParameters
  {
    int ab_threshold;
    int confidence_threshold;
    bool enable_depth_publish;
    bool enable_ab_publish;
    bool enable_conf_publish;
    bool enable_xyz_publish;
    bool enable_jblf_filter;
    int jblf_filter_size;
    int radial_threshold_min;
    int radial_threshold_max;
    std::string encoding_type;
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

      if (param.get_name() == "param_enable_depth_publish") {
        if (tunable_params_.enable_depth_publish != param.as_bool()) {
          tunable_params_.enable_depth_publish = param.as_bool();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_enable_depth_publish is changed to %s",
            param.value_to_string().c_str());
        }
      }

      if (param.get_name() == "param_enable_ab_publish") {
        if (tunable_params_.enable_ab_publish != param.as_bool()) {
          tunable_params_.enable_ab_publish = param.as_bool();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_enable_ab_publish is changed to %s",
            param.value_to_string().c_str());
        }
      }

      if (param.get_name() == "param_enable_conf_publish") {
        if (tunable_params_.enable_conf_publish != param.as_bool()) {
          tunable_params_.enable_conf_publish = param.as_bool();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_enable_conf_publish is changed to %s",
            param.value_to_string().c_str());
        }
      }

      if (param.get_name() == "param_enable_point_cloud_publish") {
        if (tunable_params_.enable_xyz_publish != param.as_bool()) {
          tunable_params_.enable_xyz_publish = param.as_bool();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_enable_point_cloud_publish is changed to %s",
            param.value_to_string().c_str());
        }
      }

      if (param.get_name() == "param_enable_jblf_filter") {
        if (tunable_params_.enable_jblf_filter != param.as_bool()) {
          tunable_params_.enable_jblf_filter = param.as_bool();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_enable_jblf_filter is changed to %s",
            param.value_to_string().c_str());
        }
      }

      // JBLF Filter Size(only use the allowed values, stored in jblf_filter_size_allowed_values array, others will be ignored)
      std::vector<int> jblf_filter_size_allowed_values = {3, 5, 7, 9, 11, 13, 15};
      if (param.get_name() == "param_jblf_filter_size") {
        if (
          std::find(
            jblf_filter_size_allowed_values.begin(), jblf_filter_size_allowed_values.end(),
            param.as_int()) != jblf_filter_size_allowed_values.end()) {
          if (tunable_params_.jblf_filter_size != param.as_int()) {
            tunable_params_.jblf_filter_size = param.as_int();
            RCLCPP_INFO(
              this->get_logger(), "The value of param_jblf_filter_size is changed to %s",
              param.value_to_string().c_str());
          }
        }
      }

      //Radial Threshold Min
      if (param.get_name() == "param_radial_threshold_min") {
        if (tunable_params_.radial_threshold_min != param.as_int()) {
          tunable_params_.radial_threshold_min = param.as_int();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_radial_threshold_min is changed to %s",
            param.value_to_string().c_str());
        }
      }

      //Radius Threshold Max
      if (param.get_name() == "param_radial_threshold_max") {
        if (tunable_params_.radial_threshold_max != param.as_int()) {
          tunable_params_.radial_threshold_max = param.as_int();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_radial_threshold_max is changed to %s",
            param.value_to_string().c_str());
        }
      }

      if (param.get_name() == "param_encoding_type") {
        if (tunable_params_.encoding_type != param.as_string()) {
          tunable_params_.encoding_type = param.as_string();
          RCLCPP_INFO(
            this->get_logger(), "The value of param_encoding_type is changed to %s",
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

    if (enable_depth_publish_ != tunable_params_.enable_depth_publish) {
      enable_depth_publish_ = tunable_params_.enable_depth_publish;
      RCLCPP_INFO(
        this->get_logger(), "Changed Enable/Disable Depth image publish value is %d",
        enable_depth_publish_);
    }

    if (enable_ab_publish_ != tunable_params_.enable_ab_publish) {
      enable_ab_publish_ = tunable_params_.enable_ab_publish;
      RCLCPP_INFO(
        this->get_logger(), "Changed Enable/Disable AB image publish value is %d",
        enable_ab_publish_);
    }

    if (enable_conf_publish_ != tunable_params_.enable_conf_publish) {
      enable_conf_publish_ = tunable_params_.enable_conf_publish;
      RCLCPP_INFO(
        this->get_logger(), "Changed Enable/Disable Confidence image publish  value is %d",
        enable_conf_publish_);
    }

    if (enable_xyz_publish_ != tunable_params_.enable_xyz_publish) {
      enable_xyz_publish_ = tunable_params_.enable_xyz_publish;
      RCLCPP_INFO(
        this->get_logger(), "Changed Enable/Disable XYZ image publish value is %d",
        enable_xyz_publish_);
    }

    if (enable_jblf_filter_ != tunable_params_.enable_jblf_filter) {
      enable_jblf_filter_ = tunable_params_.enable_jblf_filter;
      RCLCPP_INFO(
        this->get_logger(), "Changed Enable/Disable JBLF filter value is %d", enable_jblf_filter_);
      input_sensor_->setJBLFFilterState(enable_jblf_filter_);
    }

    if (jblf_filter_size_ != tunable_params_.jblf_filter_size) {
      jblf_filter_size_ = tunable_params_.jblf_filter_size;
      RCLCPP_INFO(this->get_logger(), "Changed JBLF filter size value is %d", jblf_filter_size_);
      input_sensor_->setJBLFFilterSize(jblf_filter_size_);
    }

    if (radial_threshold_min_ != tunable_params_.radial_threshold_min) {
      radial_threshold_min_ = tunable_params_.radial_threshold_min;
      RCLCPP_INFO(
        this->get_logger(), "Changed Radial Threshold Min value is %d", radial_threshold_min_);
      input_sensor_->setRadialFilterMinThreshold(radial_threshold_min_);
    }

    if (radial_threshold_max_ != tunable_params_.radial_threshold_max) {
      radial_threshold_max_ = tunable_params_.radial_threshold_max;
      RCLCPP_INFO(
        this->get_logger(), "Changed Radial Threshold Max value is %d", radial_threshold_max_);
      input_sensor_->setRadialFilterMaxThreshold(radial_threshold_max_);
    }

    if (encoding_type_ != tunable_params_.encoding_type) {
      encoding_type_ = tunable_params_.encoding_type;
      RCLCPP_INFO(this->get_logger(), "Changed Encoding type is %s", encoding_type_.c_str());
    }
  }

  /**
   * @brief This image fills and publishes the camera information
   *
   * @param frame_id  frame_id of camera_info
   * @param publisher This is Ros publisher
   */
  void fillAndPublishCameraInfo(
    const std::string & frame_id,
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
    const cv::Mat & img, const std::string & encoding_type, const std::string & frame_id,
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
    xyz_image_publisher_->publish(*pointcloud_msg);
  }

  /**
   * @brief This function publishes the images as Ros message.
   * 
   * @param out_frame Pointer to the output frame containing the compressed depth frame and its size.
   */
  void publishImageAndCameraInfo(ADI3DToFADTF31xxOutputInfo * out_frame)
  {
    // Publish image as Ros message
    cv::Mat m_disp_image_depth, m_disp_image_ab, m_disp_virtual_depth, m_disp_image_conf;

    // convert to 16 bit depth and AB image of CV format.
    m_disp_image_depth = cv::Mat(image_height_, image_width_, CV_16UC1, out_frame->depth_frame_);
    m_disp_image_ab = cv::Mat(image_height_, image_width_, CV_16UC1, out_frame->ab_frame_);
    m_disp_image_conf = cv::Mat(image_height_, image_width_, CV_16UC1, out_frame->conf_frame_);

    fillAndPublishCameraInfo(camera_link_, depth_info_publisher_);

    if (enable_depth_ab_compression_) {
      PROFILE_FUNCTION_START(Publish_CompressImg)
      if (enable_depth_publish_) {
        publishRVLCompressedImageAsRosMsg(
          out_frame->compressed_depth_frame_, out_frame->compressed_depth_frame_size_,
          encoding_type_, camera_link_, compressed_depth_image_publisher_);
      }

      if (enable_ab_publish_) {
        publishRVLCompressedImageAsRosMsg(
          out_frame->compressed_ab_frame_, out_frame->compressed_ab_frame_size_, encoding_type_,
          camera_link_, compressed_ab_image_publisher_);
      }
      if (enable_conf_publish_) {
        publishRVLCompressedImageAsRosMsg(
          out_frame->compressed_conf_frame_, out_frame->compressed_conf_frame_size_, encoding_type_,
          camera_link_, compressed_conf_image_publisher_);
      }
      PROFILE_FUNCTION_END(Publish_CompressImg)
    } else {
      if (enable_depth_publish_) {
        publishImageAsRosMsg(
          m_disp_image_depth, encoding_type_, camera_link_, depth_image_publisher_);
      }
      if (enable_ab_publish_) {
        publishImageAsRosMsg(m_disp_image_ab, encoding_type_, camera_link_, ab_image_publisher_);
      }
      if (enable_conf_publish_) {
        publishImageAsRosMsg(
          m_disp_image_conf, encoding_type_, camera_link_, conf_image_publisher_);
      }
    }

    if (enable_xyz_publish_) {
      PROFILE_FUNCTION_START(publish_PointCloud)
      // PublishPointCloud
      publishPointCloud(out_frame->xyz_frame_);
      PROFILE_FUNCTION_END(publish_PointCloud)
    }
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
    const std::string & frame_id,
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
    compressed_depth_image_transport::ConfigHeader compression_configuration{};
    compression_configuration.format = compressed_depth_image_transport::INV_DEPTH;

    float depth_quantization = 0;
    float maximum_depth = 1;

    // Inverse depth quantization parameters
    float depth_quantization_a = depth_quantization * (depth_quantization + 1.0f);
    float depth_quantization_b = 1.0f - depth_quantization_a / maximum_depth;
    compression_configuration.depthParam[0] = depth_quantization_a;
    compression_configuration.depthParam[1] = depth_quantization_b;

    memcpy(
      &compressed_payload_ptr->data[0], &compression_configuration,
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
