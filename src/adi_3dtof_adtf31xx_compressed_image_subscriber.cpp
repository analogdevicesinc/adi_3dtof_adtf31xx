/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#include <image_geometry/pinhole_camera_model.hpp>
#endif
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/image.h>
#include <std_msgs/msg/bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <functional>
#include <memory>
#include <pcl_ros/transforms.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

#include "image_proc_utils.h"
#include "ros-perception/image_transport_plugins/compressed_depth_image_transport/rvl_codec.h"
#include "ros-perception/image_transport_plugins/compressed_depth_image_transport/compression_common.h"

using namespace std::chrono_literals;
namespace enc = sensor_msgs::image_encodings;
#define MAX_QUEUE_SIZE_FOR_TIME_SYNC 10

/**
 * @brief This is main class for the HOST node
 *
 *
 */
class ADI3DToFADTF31xxCompressedImageSubscriber : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new ADI3DToFADTF31xxCompressedImageSubscriber object
   *
   */
  ADI3DToFADTF31xxCompressedImageSubscriber()
  : Node("adi_3dtof_adtf31xx_compressed_image_subscriber"),
    depth_ab_image_sync_(
      sync_depth_ab_image(MAX_QUEUE_SIZE_FOR_TIME_SYNC), depth_image_subscriber_,
      ab_image_subscriber_)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "adi_3dtof_adtf31xx_compressed_image_subscriber::Inside "
      "ADI3DToFADTF31xxCompressedImageSubscriber()");

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

    // Get Parameters
    this->declare_parameter<std::string>("camera_prefix", "cam1");
    std::string cam_prefix =
      this->get_parameter("camera_prefix").get_parameter_value().get<std::string>();

    depth_image_subscriber_.subscribe(
      this, "/" + cam_prefix + "/depth_image/compressedDepth", qos_profile);
    ab_image_subscriber_.subscribe(
      this, "/" + cam_prefix + "/ab_image/compressedDepth", qos_profile);
    depth_ab_image_sync_.registerCallback(
      &ADI3DToFADTF31xxCompressedImageSubscriber::syncDepthABImageCallback, this);

    camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/" + cam_prefix + "/camera_info",
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile),
      std::bind(
        &ADI3DToFADTF31xxCompressedImageSubscriber::camInfoCallback, this, std::placeholders::_1));

    depth_image_recvd_ = false;
    ab_image_recvd_ = false;
    depth_image_ = nullptr;
    ab_image_ = nullptr;
    camera_parameters_updated_ = false;
    xyz_image_ = nullptr;

    // Create TF listerner instance
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create publishers.
    depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "raw_depth_image",
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));
    ab_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "raw_ab_image", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "pointcloud", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile), qos_profile));

    timer_ = this->create_wall_timer(
      10ms, std::bind(&ADI3DToFADTF31xxCompressedImageSubscriber::timerCallback, this));

    // init
    frame_counter_ = 0;
  }

  void timerCallback()
  {
    // ADI_LOG_INFO("adi_3dtof_adtf31xx_compressed_image_subscriber::Running loop");

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    publishFrames();
  }

  /**
   * @brief Destroy the ADI3DToFADTF31xxCompressedImageSubscriber object
   *
   */
  ~ADI3DToFADTF31xxCompressedImageSubscriber() override
  {
    if (ab_image_ != nullptr) {
      delete[] ab_image_;
      ab_image_ = nullptr;
    }

    if (depth_image_ != nullptr) {
      delete[] depth_image_;
      depth_image_ = nullptr;
    }

    if (xyz_image_ != nullptr) {
      delete[] xyz_image_;
      xyz_image_ = nullptr;
    }

    delete image_proc_utils_;
  }

  /**
   * @brief Call back for synchronised topics(depth and AB image pair)
   *
   * @param depth_image_cam1 - Cam1 depth image pointer
   * @param ab_image_cam1 - Cam1 AB image pointer
   */
  void syncDepthABImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & depth_image_cam1,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & ab_image_cam1)
  {
    // Call respective callbacks with the id.
    depthABImageCallback(depth_image_cam1, true);
    depthABImageCallback(ab_image_cam1, false);
    depth_image_recvd_ = true;
    ab_image_recvd_ = true;
  }

  /**
   * @brief Low-level callback for Com-Info the ID would indicate the source camera
   *
   * @param cam_info - Cam Info Pointer
   */
  void camInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
  {
    // Save camera info only once
    if (!camera_parameters_updated_) {
      // Save
      image_width_ = cam_info->width;
      image_height_ = cam_info->height;
      
      // Check whether original or modified camera intrinsic are sent
      camera_intrinsics_.camera_matrix[0] = cam_info->k[0];
      camera_intrinsics_.camera_matrix[1] = 0.0f;
      camera_intrinsics_.camera_matrix[2] = cam_info->k[2];
      camera_intrinsics_.camera_matrix[3] = 0.0f;
      camera_intrinsics_.camera_matrix[4] = cam_info->k[4];
      camera_intrinsics_.camera_matrix[5] = cam_info->k[5];
      camera_intrinsics_.camera_matrix[6] = 0.0f;
      camera_intrinsics_.camera_matrix[7] = 0.0f;
      camera_intrinsics_.camera_matrix[8] = 1.0f;

      for (int i = 0; i < 8; i++) {
        camera_intrinsics_.distortion_coeffs[i] = cam_info->d[i];
      }

      image_proc_utils_ = new ImageProcUtils(&camera_intrinsics_, image_width_, image_height_);
      camera_parameters_updated_ = true;
    }
  }

  /**
   * @brief Low-level callback for depth/ab image
   *
   * @param depth_ab_image_message - Pointer to depth/ab compressed image message
   * @param is_depth_image - true:depth image,false:ab image
   */
  void depthABImageCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & depth_ab_image_message,
    bool is_depth_image)
  {
    compressed_depth_image_transport::RvlCodec rvl;
    if ((depth_ab_image_message == nullptr)) {
      return;
    }

    /* The Format of the message is
     * 0th Position : ConfigHeader (sizeof(compressed_depth_image_transport::ConfigHeader bytes)
     * sizeof(compressed_depth_image_transport::ConfigHeader) : image_width(4 bytes)
     * sizeof(compressed_depth_image_transport::ConfigHeader) + 4: image_height(4 bytes)
     * sizeof(compressed_depth_image_transport::ConfigHeader) + 8: compressed image
     * */
    unsigned char * compressed_image_buf =
      (unsigned char *)&depth_ab_image_message
        ->data[sizeof(compressed_depth_image_transport::ConfigHeader) + 8];
    unsigned short * raw_image_buf = nullptr;
    bool * message_recvd_flag;

    int * image_width = (int *)&depth_ab_image_message
                          ->data[sizeof(compressed_depth_image_transport::ConfigHeader) + 0];
    int * image_height = (int *)&depth_ab_image_message
                           ->data[sizeof(compressed_depth_image_transport::ConfigHeader) + 4];
    
    image_width_ = *image_width;
    image_height_ = *image_height;

    if (is_depth_image) {
      if (depth_image_ == nullptr) {
        depth_image_ = new unsigned short[image_width_ * image_height_ * 2];
      }
      // depth image
      raw_image_buf = depth_image_;
      message_recvd_flag = &depth_image_recvd_;
    } else {
      // ab image
      if (ab_image_ == nullptr) {
        ab_image_ = new unsigned short[image_width_ * image_height_ * 2];
      }
      raw_image_buf = ab_image_;
      message_recvd_flag = &ab_image_recvd_;
    }

    // decompress
    rvl.DecompressRVL(compressed_image_buf, raw_image_buf, image_width_ * image_height_);

    if (is_depth_image) {
      generatePointCloud(depth_ab_image_message);
    }
    *message_recvd_flag = true;
  }

  sensor_msgs::msg::PointCloud2::SharedPtr convert2ROSPointCloudMsg(
    short * xyz_frame, rclcpp::Time stamp, std::string frame_id)
  {
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg(new sensor_msgs::msg::PointCloud2);

    pointcloud_msg->header.stamp = stamp;
    pointcloud_msg->header.frame_id = std::move(frame_id);
    pointcloud_msg->width = image_width_;
    pointcloud_msg->height = image_height_;
    pointcloud_msg->is_dense = false;
    pointcloud_msg->is_bigendian = false;

    // XYZ data from sensor.
    // This data is in 16 bpp format.
    short * xyz_sensor_buf;
    xyz_sensor_buf = xyz_frame;
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
    return pointcloud_msg;
  }

  void generatePointCloud(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr & depth_image_message)
  {
    if (!camera_parameters_updated_) {
      return;
    }

    // Compute point cloud
    if (xyz_image_ == nullptr) {
      xyz_image_ = new short[image_width_ * image_height_ * 3];
    }
    image_proc_utils_->computePointCloud(depth_image_, xyz_image_);

    rclcpp::Time stamp;
    std::string frame_id;
    if (depth_image_message != nullptr) {
      stamp = depth_image_message->header.stamp;
      frame_id = depth_image_message->header.frame_id;
    }

    // Transform to map
    // Get the transform wrt to "map"
    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud =
      convert2ROSPointCloudMsg(xyz_image_, stamp, frame_id);
    tf_buffer_->canTransform(
      "map", point_cloud->header.frame_id, point_cloud->header.stamp, rclcpp::Duration(5, 0));
    pcl_ros::transformPointCloud("map", *point_cloud, transformed_pc_, *tf_buffer_);
  }

  /**
   * @brief Publish function
   *
   * @return true
   * @return false
   */
  bool publishFrames()
  {
    // Make sure we have received frames from all the sensors
    bool all_callbacks_recvd = false;
    if ((depth_image_recvd_) && (ab_image_recvd_)) {
      all_callbacks_recvd = true;
    }

    if (all_callbacks_recvd) {
      RCLCPP_INFO(
        this->get_logger(), "adi_3dtof_adtf31xx_compressed_image_subscriber::Running loop");

      cv::Mat m_depth_image, m_ab_image;
      // convert to 16 bit depth and AB image of CV format.
      m_depth_image = cv::Mat(image_height_, image_width_, CV_16UC1, depth_image_);
      m_ab_image = cv::Mat(image_height_, image_width_, CV_16UC1, ab_image_);

      publishImageAsRosMsg(m_depth_image, "mono16", "raw_depth_image", depth_image_publisher_);
      publishImageAsRosMsg(m_ab_image, "mono16", "raw_ab_image", ab_image_publisher_);

      pointcloud_publisher_->publish(transformed_pc_);

      depth_image_recvd_ = false;
      ab_image_recvd_ = false;

      // Update frame count
      frame_counter_++;
    }

    return true;
  }

private:
  int image_width_;
  int image_height_;
  int frame_counter_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> depth_image_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> ab_image_subscriber_;
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage>
    sync_depth_ab_image;
  message_filters::Synchronizer<sync_depth_ab_image> depth_ab_image_sync_;

  bool depth_image_recvd_;
  bool ab_image_recvd_;
  unsigned short * depth_image_;
  unsigned short * ab_image_;
  short * xyz_image_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ab_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

  rclcpp::Time curr_frame_timestamp_ = rclcpp::Clock{}.now();

  ImageProcUtils * image_proc_utils_ = nullptr;
  bool camera_parameters_updated_;
  CameraIntrinsics camera_intrinsics_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  sensor_msgs::msg::PointCloud2 transformed_pc_;

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
};

/**
 * @brief This is main function
 *
 * @param argc number of input arguments to the function
 * @param argv array of pointer to the input arguments
 * @return int
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto adi_3dtof_adtf31xx_compressed_image_subscriber =
    std::make_shared<ADI3DToFADTF31xxCompressedImageSubscriber>();

  rclcpp::spin(adi_3dtof_adtf31xx_compressed_image_subscriber);
  rclcpp::shutdown();

  return 0;
}
