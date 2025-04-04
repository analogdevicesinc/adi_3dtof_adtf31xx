/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/
#include "image_proc_utils.h"
#include <ros/ros.h>
#include <utility>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/distortion_models.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <compressed_depth_image_transport/compression_common.h>

namespace enc = sensor_msgs::image_encodings;
#define MAX_QUEUE_SIZE_FOR_TIME_SYNC 10

/**
 * @brief This is main class for the HOST node
 *
 *
 */
class ADI3DToFADTF31xxCompressedImageSubscriber : public ros::NodeHandle
{
public:
  /**
   * @brief Construct a new ADI3DToFADTF31xxCompressedImageSubscriber object
   *
   * @param nh - Node handle
   * @param cam_prefix - vector of camera prefix for all the cameras
   * @param depth_ab_image_sync - Synchronizer handle depth and ab image
   */
  ADI3DToFADTF31xxCompressedImageSubscriber(ros::NodeHandle& nh, const std::string& cam_prefix)
    : it_(nh)
    , depth_ab_image_sync_(sync_depth_ab_image_(MAX_QUEUE_SIZE_FOR_TIME_SYNC), depth_image_subscriber_,
                           ab_image_subscriber_)
  {
    ROS_INFO("adi_3dtof_adtf31xx_compressed_image_subscriber::Inside ADI3DToFADTF31xxCompressedImageSubscriber()");

    camera_info_subscriber_ = this->subscribe<sensor_msgs::CameraInfo>(
        "/" + cam_prefix + "/camera_info", 1,
        boost::bind(&ADI3DToFADTF31xxCompressedImageSubscriber::camInfoCallback, this, _1));

    depth_image_subscriber_.subscribe(it_, "/" + cam_prefix + "/depth_image", 5);
    ab_image_subscriber_.subscribe(it_, "/" + cam_prefix + "/ab_image", 5);

    depth_ab_image_sync_.registerCallback(
        boost::bind(&ADI3DToFADTF31xxCompressedImageSubscriber::syncDepthABImageCallback, this, _1, _2));

    depth_image_recvd_ = false;
    ab_image_recvd_ = false;
    depth_image_ = nullptr;
    ab_image_ = nullptr;
    camera_parameters_updated_ = false;
    xyz_image_ = nullptr;
    depth_image_message_ = nullptr;

    // Create TF listerner instance
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

    // Create publishers.
    depth_image_publisher_ = this->advertise<sensor_msgs::Image>("raw_depth_image", 10);
    ab_image_publisher_ = this->advertise<sensor_msgs::Image>("raw_ab_image", 10);
    pointcloud_publisher_ = this->advertise<sensor_msgs::PointCloud2>("pointcloud", 10);

    // init
    frame_counter_ = 0;
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
   * @brief Destroy the ADI3DToFADTF31xxCompressedImageSubscriber object
   *
   */
  ~ADI3DToFADTF31xxCompressedImageSubscriber()
  {
    if (ab_image_ != nullptr)
    {
      delete[] ab_image_;
      ab_image_ = nullptr;
    }

    if (depth_image_ != nullptr)
    {
      delete[] depth_image_;
      depth_image_ = nullptr;
    }

    if (xyz_image_ != nullptr)
    {
      delete[] xyz_image_;
      xyz_image_ = nullptr;
    }

    delete tf_listener_;
    delete image_proc_utils_;
  }

  /**
   * @brief Call back for synchronised topics(depth and AB image pair)
   *
   * @param depth_image_cam1 - Cam1 depth image pointer
   * @param ab_image_cam1 - Cam1 AB image pointer
   */
  void syncDepthABImageCallback(const sensor_msgs::ImageConstPtr& depth_image_cam1,
                                const sensor_msgs::ImageConstPtr& ab_image_cam1)
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
  void camInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    // Save camera info only once
    if (!camera_parameters_updated_)
    {
      // Save
      image_width_ = cam_info->width;
      image_height_ = cam_info->height;
      if ((image_width_ != 512) && (image_height_ != 512))
      {
        ROS_INFO_STREAM("Image width and Height are not set to 512x512");
        return;
      }

      // Check whether original or modified camera intrinsic are sent
      camera_intrinsics_.camera_matrix[0] = cam_info->K[0];
      camera_intrinsics_.camera_matrix[1] = 0.0f;
      camera_intrinsics_.camera_matrix[2] = cam_info->K[2];
      camera_intrinsics_.camera_matrix[3] = 0.0f;
      camera_intrinsics_.camera_matrix[4] = cam_info->K[4];
      camera_intrinsics_.camera_matrix[5] = cam_info->K[5];
      camera_intrinsics_.camera_matrix[6] = 0.0f;
      camera_intrinsics_.camera_matrix[7] = 0.0f;
      camera_intrinsics_.camera_matrix[8] = 1.0f;

      for (int i = 0; i < 8; i++)
      {
        camera_intrinsics_.distortion_coeffs[i] = cam_info->D[i];
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
  void depthABImageCallback(const sensor_msgs::ImageConstPtr& depth_ab_image_message, bool is_depth_image)
  {
    if ((depth_ab_image_message == nullptr))
    {
      return;
    }

    unsigned char* image_buf = (unsigned char*)&depth_ab_image_message->data[0];
    unsigned short* raw_image_buf = nullptr;
    bool* message_recvd_flag;

    int* image_width = (int*)&depth_ab_image_message->width;
    int* image_height = (int*)&depth_ab_image_message->height;
    if ((*image_width != 512) && (*image_height != 512))
    {
      return;
    }
    image_width_ = *image_width;
    image_height_ = *image_height;

    if (is_depth_image)
    {
      if (depth_image_ == nullptr)
      {
        depth_image_ = new unsigned short[image_width_ * image_height_];
      }
      // depth image
      raw_image_buf = depth_image_;
      message_recvd_flag = &depth_image_recvd_;
      depth_image_message_ = depth_ab_image_message;
    }
    else
    {
      // ab image
      if (ab_image_ == nullptr)
      {
        ab_image_ = new unsigned short[image_width_ * image_height_];
      }
      raw_image_buf = ab_image_;
      message_recvd_flag = &ab_image_recvd_;
    }

    memcpy(raw_image_buf, image_buf, image_width_ * image_height_ * 2);
    *message_recvd_flag = true;
  }

  sensor_msgs::PointCloud2::Ptr convert2ROSPointCloudMsg(short* xyz_frame, ros::Time stamp, std::string frame_id)
  {
    sensor_msgs::PointCloud2::Ptr pointcloud_msg(new sensor_msgs::PointCloud2);

    pointcloud_msg->header.seq = frame_counter_;
    pointcloud_msg->header.stamp = stamp;
    pointcloud_msg->header.frame_id = std::move(frame_id);
    pointcloud_msg->width = image_width_;
    pointcloud_msg->height = image_height_;
    pointcloud_msg->is_dense = false;
    pointcloud_msg->is_bigendian = false;

    // XYZ data from sensor.
    // This data is in 16 bpp format.
    short* xyz_sensor_buf;
    xyz_sensor_buf = xyz_frame;
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
    return pointcloud_msg;
  }
  void generatePointCloud()
  {
    if (!camera_parameters_updated_)
    {
      return;
    }

    // Compute point cloud
    if (xyz_image_ == nullptr)
    {
      xyz_image_ = new short[image_width_ * image_height_ * 3];
    }
    image_proc_utils_->computePointCloud(depth_image_, xyz_image_);

    ros::Time stamp;
    std::string frame_id;
    if (depth_image_message_ != nullptr)
    {
      stamp = depth_image_message_->header.stamp;
      frame_id = depth_image_message_->header.frame_id;
    }

    // Transform to map
    // Get the transform wrt to "map"
    sensor_msgs::PointCloud2::Ptr point_cloud = convert2ROSPointCloudMsg(xyz_image_, stamp, frame_id);
    tf_buffer_.canTransform("map", point_cloud->header.frame_id, point_cloud->header.stamp, ros::Duration(5.0));
    pcl_ros::transformPointCloud("map", *point_cloud, transformed_pc_, tf_buffer_);
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
    if ((depth_image_recvd_) && (ab_image_recvd_))
    {
      all_callbacks_recvd = true;
    }

    if (all_callbacks_recvd)
    {
      cv::Mat m_depth_image, m_ab_image;

      ROS_INFO("adi_3dtof_adtf31xx_compressed_image_subscriber::Running loop");

      // convert to 16 bit depth and AB image of CV format.
      m_depth_image = cv::Mat(image_height_, image_width_, CV_16UC1, depth_image_);
      m_ab_image = cv::Mat(image_height_, image_width_, CV_16UC1, ab_image_);

      publishImageAsRosMsg(m_depth_image, "mono16", "raw_depth_image", depth_image_publisher_);
      publishImageAsRosMsg(m_ab_image, "mono16", "raw_ab_image", ab_image_publisher_);

      generatePointCloud();
      pointcloud_publisher_.publish(transformed_pc_);

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

  ros::Subscriber camera_info_subscriber_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter depth_image_subscriber_;
  image_transport::SubscriberFilter ab_image_subscriber_;
  bool depth_image_recvd_;
  bool ab_image_recvd_;
  unsigned short* depth_image_;
  unsigned short* ab_image_;
  short* xyz_image_;

  ros::Publisher depth_image_publisher_;
  ros::Publisher ab_image_publisher_;
  ros::Publisher pointcloud_publisher_;
  sensor_msgs::ImageConstPtr depth_image_message_;

  ros::Time curr_frame_timestamp_ = ros::Time::now();

  ImageProcUtils* image_proc_utils_ = nullptr;
  bool camera_parameters_updated_;
  CameraIntrinsics camera_intrinsics_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener* tf_listener_;
  sensor_msgs::PointCloud2 transformed_pc_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_depth_ab_image_;

  message_filters::Synchronizer<sync_depth_ab_image_> depth_ab_image_sync_;

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
    cv_ptr->header.seq = frame_counter_;
    cv_ptr->header.stamp = curr_frame_timestamp_;
    cv_ptr->header.frame_id = std::move(frame_id);
    cv_ptr->image = std::move(img);

    publisher.publish(cv_ptr->toImageMsg());
  }
};

/**
 * @brief This is main function
 *
 * @param argc number of input arguments to the function
 * @param argv array of pointer to the input arguments
 * @return int
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "adi_3dtof_adtf31xx_compressed_image_subscriber");
  ros::NodeHandle nh("~");

  // Get Parameters
  std::string cam_prefix;
  nh.param<std::string>("param_camera_prefix", cam_prefix, "no name");

  // Create an instance of ADI3DToFADTF31xxCompressedImageSubscriber
  ADI3DToFADTF31xxCompressedImageSubscriber adi_3dtof_adtf31xx_compressed_image_subscriber(nh, cam_prefix);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    if (!adi_3dtof_adtf31xx_compressed_image_subscriber.publishFrames())
    {
      break;
    }

    loop_rate.sleep();

    ros::spinOnce();
  }

  adi_3dtof_adtf31xx_compressed_image_subscriber.shutDownAllNodes();

  return 0;
}
