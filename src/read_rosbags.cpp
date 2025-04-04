#include <bits/stdc++.h>
#include <boost/thread/thread.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <utility>

#include <vector>
#include <string>
#include <stdio.h>
#include <sensor_msgs/CompressedImage.h>
#include <compressed_depth_image_transport/rvl_codec.h>
#include <compressed_depth_image_transport/compression_common.h>

namespace enc = sensor_msgs::image_encodings;

// version of bin files
int CAM_VERSION = 2;

/**
 * @brief This structure indicates header of bin files.
 *
 */
#pragma pack(1)
struct HeaderOfBinFile
{
  int cam_total_frames = 0;            // Total number of frames in cameras
  int cam_image_width = 0;             // width of the image
  int cam_image_height = 0;            // Height of the image.
  int cam_bytes_per_pixel = 2;         // Number of bytes per pixel of image
  int cam_version = 2;                 // version of bin file
  int cam_first_frame_position = 408;  // position of first frame in the file.
  int cam_frame_pitch = 1048592;  // length of a single frame. each frame contains "depth time stamp, depth image, ab
                                  // time stamp, ab image"
  long cam_device_timestamp = 0;  // device timestamp
};

/**
 * @brief comparison function
 *
 * @param first_element first element
 * @param second_element second element
 * @return true if first pair element's first value is less than second pair element's first value
 * @return false if first pair element's first value is greater than second pair element's first value
 */
bool comparator(std::pair<long, int>& first_element, std::pair<long, int>& second_element)
{
  return first_element.first < second_element.first;
}

/**
 * @brief sorts the map
 *
 * @param mapped_array input mapped array to sort function
 */
void sort_map(std::map<long, int>& mapped_array)
{
  // Declare vector of pairs
  std::vector<std::pair<long, int>> vector_of_pairs;

  // Copy key-value pair from Map
  // to vector of pairs
  vector_of_pairs.reserve(mapped_array.size());

for (auto& it : mapped_array)
  {
    vector_of_pairs.push_back(it);
  }

  // Sort using comparator function
  sort(vector_of_pairs.begin(), vector_of_pairs.end(), comparator);
}

/**
 * @brief deleting the intermediate files.
 * remove() function does not work with a null terminated string, so here null character at the end of the string is
 * removed then passed remove function
 *
 * @param file_name file name
 */
void deleteFile(std::string file_name)
{
  const std::string& str = std::move(file_name);
  size_t null_char_pos = str.find_last_not_of('\0');
  char* char_array = new char[null_char_pos + 1];
  std::strcpy(char_array, str.substr(0, null_char_pos + 1).c_str());

  const char* filename = char_array;

  // Delete the file
  if (std::remove(filename) != 0)
  {
    // perror("Error deleting the file");
  }
  else
  {
    // printf("File deleted successfully\n");
  }
  delete[] char_array;
}

/**
 * @brief This function finds whether any depth frame do not have the AB frame corresponding to it.
 *
 * @param file_name Input file name
 * @param num_depth_frames Total number of depth frames
 * @param num_ab_frames Total number of AB frames
 */
void findMissingABOrDepthFrames(const std::string& file_name, int num_depth_frames, int num_ab_frames)
{
  std::ifstream fin;
  fin.open(file_name, std::ios::binary | std::ios::in);

  uint8_t image_indicator;
  long time_stamp_depth;
  long time_stamp_ab;
  std::vector<long> list_depth_timestamps;
  std::vector<long> list_ab_timestamps;

  int total_frames;
  int image_width;
  int image_height;
  fin.read((char*)&total_frames, sizeof(uint32_t));
  fin.read((char*)&image_width, sizeof(uint32_t));
  fin.read((char*)&image_height, sizeof(uint32_t));

  uint32_t header_size;
  fin.seekg(20);
  fin.read((char*)&header_size, sizeof(uint32_t));
  char* header = new char[header_size];
  fin.seekg(0);
  fin.read(header, header_size);

  uint64_t startframe = header_size;  // first frame position
  long lowest_timestamp_depth = 0;
  int j = 0;
  // loop through entire file and append depth image timestamps
  while (j < num_depth_frames)
  {
    fin.seekg(startframe);
    fin.read((char*)&image_indicator, sizeof(uint8_t));
    if (image_indicator == 0)
    {
      fin.read((char*)&lowest_timestamp_depth, sizeof(long));
      list_depth_timestamps.push_back(lowest_timestamp_depth);
      j++;
    }
    startframe = startframe + (image_width * image_height * 2) + 9;  // use frame_pitch here
  }
  // sort depth image timestamps
  std::sort(list_depth_timestamps.begin(), list_depth_timestamps.end());

  startframe = header_size;
  long lowest_timestamp_ab = 0;
  int k = 0;
  // loop through entire file and append AB image timestamps
  while (k < num_ab_frames)
  {
    fin.seekg(startframe);
    fin.read((char*)&image_indicator, sizeof(uint8_t));
    if (image_indicator == 1)
    {
      fin.read((char*)&lowest_timestamp_ab, sizeof(long));
      list_ab_timestamps.push_back(lowest_timestamp_ab);
      k++;
    }
    startframe = startframe + (image_width * image_height * 2) + 9;
  }
  // sort AB image timestamps
  std::sort(list_ab_timestamps.begin(), list_ab_timestamps.end());

  std::cout << "\n\n" << file_name << std::endl;
  std::cout << "Checking whether any Depth frame missing its AB pair....\n\n";

  bool found_ab_match;
  long depth_time_stamp;
  // If depth timestamp does not have matching AB timestamp then print appropriate message.
  for (int i = 0; i < num_depth_frames; i++)
  {
    depth_time_stamp = list_depth_timestamps[i];
    found_ab_match =
        (std::find(list_ab_timestamps.begin(), list_ab_timestamps.end(), depth_time_stamp) != list_ab_timestamps.end());
    if (found_ab_match)
    {
      // std::cout << ".." ;
    }
    else
    {
      std::cout << "depth frame with timestamp " << depth_time_stamp << " do not have AB frame" << std::endl;
    }
  }

  std::cout << "\n\nChecking whether any AB frame missing its Depth pair....\n\n";
  bool found_depth_match;
  long ab_time_stamp;
  // If AB timestamp does not have matching depth timestamp then print appropriate message.
  for (int i = 0; i < num_ab_frames; i++)
  {
    ab_time_stamp = list_ab_timestamps[i];
    found_depth_match = (std::find(list_depth_timestamps.begin(), list_depth_timestamps.end(), ab_time_stamp) !=
                         list_depth_timestamps.end());
    if (found_depth_match)
    {
      // std::cout << "..";
    }
    else
    {
      std::cout << "ab frame with timestamp " << ab_time_stamp << " do not have depth frame" << std::endl;
    }
  }

  fin.close();
}

/**
 * @brief Notifies if capture is less than 10FPS
 *
 * @param file_name file name
 */
void notifyFrameLoss(const std::string& file_name)
{
  std::fstream fin;
  fin.open(file_name, std::ios::binary | std::ios::in);

  int total_frames;
  int image_width;
  int image_height;
  // reads total number of frames in file
  fin.read((char*)&total_frames, sizeof(uint32_t));
  // reads image width
  fin.read((char*)&image_width, sizeof(uint32_t));
  // reads image height
  fin.read((char*)&image_height, sizeof(uint32_t));

  uint32_t header_size;
  fin.seekg(20);
  // reads total header size
  fin.read((char*)&header_size, sizeof(uint32_t));
  char* header = new char[header_size];
  fin.seekg(0);
  // reads entire header
  fin.read(header, header_size);

  long time_stamp;
  long prev_time_stamp;
  fin.read((char*)&time_stamp, sizeof(long));
  prev_time_stamp = time_stamp;
  int frame_position;
  frame_position = header_size;
  int k = 0;
  // loop through entire file and compare old time stamp with current timestamp if the difference is more than 110ms
  // then say that capture exceeds 10FPS.
  while (k < total_frames)
  {
    fin.seekg(frame_position);
    fin.read((char*)&time_stamp, sizeof(long));
    if ((time_stamp - prev_time_stamp) > 110000000)
    {
      std::cout << time_stamp << " - " << prev_time_stamp << " = " << time_stamp - prev_time_stamp << " ns"
                << std::endl;
      std::cout << "Capture is less than 10FPS" << std::endl;
    }
    prev_time_stamp = time_stamp;
    frame_position = frame_position + ((image_width * image_height * 2) + 8) * 2;
    k++;
  }
  fin.close();
}

/**
 * @brief AB and depth images with the same timestamp are treated as frames and written to the final binary file, which
 * is arranged in ascending order of timestamps.
 *
 * @param file_name file name
 * @param num_depth_frames number of depth frames
 * @param num_ab_frames number of AB frames
 */
void arrangeFramesWithTimeStamps(const std::string& file_name, int num_depth_frames, int num_ab_frames)
{
  std::ifstream fin;
  std::ofstream fout;
  fin.open(file_name, std::ios::binary | std::ios::in);

  std::string output_file;
  // output file name
  output_file = file_name.substr(0, file_name.find_last_of('.')) + "_" + "out" + ".bin";
  // delete if any file with the same output file name exists.
  deleteFile(output_file);
  // open output file
  fout.open(output_file, std::ios::binary | std::ios::out);

  // image indicator to indicate whether it is depth frame or AB frame.
  uint8_t image_indicator;
  long time_stamp_depth;
  long time_stamp_ab;
  long lowest_timestamp_depth;

  int total_frames;
  int image_width;
  int image_height;
  // reading total number of frames, image width and image height.
  fin.read((char*)&total_frames, sizeof(uint32_t));
  fin.read((char*)&image_width, sizeof(uint32_t));
  fin.read((char*)&image_height, sizeof(uint32_t));

  // reading size of header of file.
  uint32_t header_size;
  fin.seekg(20);
  fin.read((char*)&header_size, sizeof(uint32_t));
  char* header = new char[header_size];
  fin.seekg(0);
  // reading header from input file
  fin.read(header, header_size);
  // Writing header to output file
  fout.write(header, header_size);

  char* frame_buffer_depth = new char[image_width * image_height * 2];
  char* frame_buffer_ab = new char[image_width * image_height * 2];

  // it is map of timestamp and byte number in file related to depth data
  std::map<long, int> depth_data;

  // first frame address starts after the header.
  uint64_t startframe = header_size;
  int j = 0;
  while (j < num_depth_frames)
  {
    fin.seekg(startframe);
    fin.read((char*)&image_indicator, sizeof(uint8_t));
    if (image_indicator == 0)
    {
      fin.read((char*)&lowest_timestamp_depth, sizeof(long));
      // if the next image is depth image then add the timestamp and byte number of depth image to depth data.
      depth_data.insert({ lowest_timestamp_depth, startframe });
      j++;
    }
    startframe = startframe + (image_width * image_height * 2) + 9;
  }
  // sort map in ascending order of depth timestamp
  sort_map(depth_data);

  // it is map of timestamp and byte number in file related to AB data
  std::map<long, int> ab_data;

  startframe = header_size;
  int k = 0;
  while (k < num_ab_frames)
  {
    fin.seekg(startframe);
    fin.read((char*)&image_indicator, sizeof(uint8_t));
    if (image_indicator == 1)
    {
      fin.read((char*)&lowest_timestamp_depth, sizeof(long));
      // if the next image is AB image then add the timestamp and byte number of AB image to ab data.
      ab_data.insert({ lowest_timestamp_depth, startframe });
      k++;
    }
    startframe = startframe + (image_width * image_height * 2) + 9;
  }
  // sort map in ascending order of ab timestamp
  sort_map(ab_data);

  fin.close();
  fin.open(file_name, std::ios::binary | std::ios::in);

  long frame_time_stamp;
  uint32_t total_frames_count = 0;
  // if depth timestamp matches with AB timestamp then both AB and depth images are written as a single frame.
  for (auto & it : depth_data)
  {
    frame_time_stamp = it.first;
    if (ab_data.find(frame_time_stamp) != ab_data.end())
    {
      int frame_ptr_ab = ab_data.find(frame_time_stamp)->second;
      int frame_ptr_depth = it.second;
      frame_ptr_depth = frame_ptr_depth + 9;  // to go to image data
      frame_ptr_ab = frame_ptr_ab + 9;        // to go to image data
      fin.seekg(frame_ptr_depth);
      fin.read(frame_buffer_depth, image_width * image_height * 2);
      fin.seekg(frame_ptr_ab);
      fin.read(frame_buffer_ab, image_width * image_height * 2);
      fout.write((char*)&frame_time_stamp, sizeof(long));
      fout.write(frame_buffer_depth, image_width * image_height * 2);
      fout.write((char*)&frame_time_stamp, sizeof(long));
      fout.write(frame_buffer_ab, image_width * image_height * 2);
      total_frames_count++;
    }
  }

  fout.seekp(0);
  fout.write((char*)&total_frames_count, sizeof(uint32_t));

  fin.close();
  fout.close();

  // Notifies if the recording takes less than 10 FPS.
  notifyFrameLoss(output_file);
}

/**
 * @brief Stores camera topics in binary file
 *
 * @param bag_file_name input bag file name
 * @param cam_name camera prefix names
 */
void storeCameraTopicsInbinFile(std::string bag_file_name, std::vector<std::string> cam_name)
{
  std::cout << "Reading image topics from one or more cameras and writing to bin files.." << std::endl;

  const std::string& input_bag_file = std::move(bag_file_name);

  // output file names
  std::vector<std::string> cam_bin_file_name;
  // ab image topic names
  std::vector<std::string> cam_ab_image_topic_names;
  // compressed ab image topic names
  std::vector<std::string> cam_compressed_ab_image_topic_names;
  // depth image topic names
  std::vector<std::string> cam_depth_image_topic_names;
  // compressed depth image topic names
  std::vector<std::string> cam_compressed_depth_image_topic_names;
  // camera info topics
  std::vector<std::string> cam_info_topic_names;

  cv::Mat image;
  rosbag::Bag bag;

  HeaderOfBinFile header_of_bin_file;

  // Header parameters of bin file
  std::vector<HeaderOfBinFile> headers_of_bin_files;

  // intermediate variables
  std::vector<int> camera_total_ab_frames;
  std::vector<int> camera_total_depth_frames;
  std::vector<bool> camera_device_time_stamp_written;
  std::vector<bool> camera_info_written;

  // file handlers
  std::vector<std::ofstream*> files;
  files.resize(cam_name.size());

  for (int j = 0; j < cam_name.size(); j++)
  {
    // creating output file names
    std::string cam_bin_file;
    cam_bin_file = input_bag_file.substr(0, input_bag_file.find_last_of('.')) + "_" + cam_name[j] + ".bin";
    cam_bin_file_name.push_back(cam_bin_file);

    // creating AB image topics
    std::string cam_ab_image = "/" + cam_name[j] + "/ab_image";
    cam_ab_image_topic_names.push_back(cam_ab_image);

    std::string compressed_cam_ab_image = "/" + cam_name[j] + "/ab_image/compressedDepth";
    cam_compressed_ab_image_topic_names.push_back(compressed_cam_ab_image);

    // creating depth image topics
    std::string cam_depth_image = "/" + cam_name[j] + "/depth_image";
    cam_depth_image_topic_names.push_back(cam_depth_image);

    // creating depth image topics
    std::string compressed_cam_depth_image = "/" + cam_name[j] + "/depth_image/compressedDepth";
    cam_compressed_depth_image_topic_names.push_back(compressed_cam_depth_image);

    // creating camera info topics
    std::string cam_info = "/" + cam_name[j] + "/camera_info";
    cam_info_topic_names.push_back(cam_info);

    headers_of_bin_files.push_back(header_of_bin_file);

    camera_total_ab_frames.push_back(0);
    camera_total_depth_frames.push_back(0);
    camera_device_time_stamp_written.push_back(false);
    camera_info_written.push_back(false);

    files[j] = new std::ofstream();
    files[j]->open(cam_bin_file_name[j], std::ios::out);
    files[j]->write((char*)&headers_of_bin_files[j], sizeof(HeaderOfBinFile));
  }

  uint8_t depth = 0;
  uint8_t ab = 1;
  bool all_camera_info_written = false;

  // Reading starts..........
  bag.open(input_bag_file);
  compressed_depth_image_transport::RvlCodec rvl;
  for (rosbag::MessageInstance const message : rosbag::View(bag))
  {
    const std::string& img_topic = message.getTopic();

    for (int i = 0; i < cam_name.size(); i++)
    {
      // Writing camera info first.
      if ((img_topic == cam_info_topic_names[i]) && (!camera_info_written[i]))
      {
        sensor_msgs::CameraInfoPtr camera_info_ptr = message.instantiate<sensor_msgs::CameraInfo>();
        headers_of_bin_files[i].cam_image_width = camera_info_ptr->width;
        headers_of_bin_files[i].cam_image_height = camera_info_ptr->height;
        files[i]->write((char*)&camera_info_ptr->K, sizeof(double) * 9);
        uint32_t size_of_d;
        size_of_d = camera_info_ptr->D.size();
        files[i]->write((char*)&size_of_d, sizeof(uint32_t));
        files[i]->write((char*)camera_info_ptr->D.data(), sizeof(double) * size_of_d);
        headers_of_bin_files[i].cam_first_frame_position =
            (headers_of_bin_files[i].cam_first_frame_position - 16 * sizeof(double)) + (size_of_d * sizeof(double));
        files[i]->write((char*)&camera_info_ptr->R, sizeof(double) * 9);
        files[i]->write((char*)&camera_info_ptr->P, sizeof(double) * 12);
        camera_info_written[i] = true;
      }

      // Writing camera image topics..
      if (((img_topic == cam_ab_image_topic_names[i]) || (img_topic == cam_depth_image_topic_names[i])) &&
          (camera_info_written[i]))
      {
        try
        {
          cv_bridge::CvImagePtr cv_ptr;
          sensor_msgs::ImageConstPtr img_msg_ptr = message.instantiate<sensor_msgs::Image>();
          long timestamp;
          timestamp = (long)img_msg_ptr->header.stamp.toNSec();
          // std::cout << timestamp;
          image = cv_bridge::toCvCopy(img_msg_ptr, "mono16")->image;

          if (img_topic == cam_ab_image_topic_names[i])
          {
            files[i]->write((char*)&ab, sizeof(bool));
            camera_total_ab_frames[i]++;
          }
          else if (img_topic == cam_depth_image_topic_names[i])
          {
            files[i]->write((char*)&depth, sizeof(bool));
            camera_total_depth_frames[i]++;
          }
          if (!camera_device_time_stamp_written[i])
          {
            headers_of_bin_files[i].cam_device_timestamp = timestamp;
            camera_device_time_stamp_written[i] = true;
          }
          files[i]->write((char*)&timestamp, sizeof(long));
          files[i]->write((char*)image.data, sizeof(unsigned short) * (headers_of_bin_files[i].cam_image_width *
                                                                       headers_of_bin_files[i].cam_image_height));
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Failed while reading raw images from bag file");
        }
      }

      /* Writing camera image comprssed topics..
         This decompression is similar to image transport decompression. as adi_3dtof_adtf31xx package publishes in
         similar format of image transport. */
      if (((img_topic == cam_compressed_ab_image_topic_names[i]) ||
           (img_topic == cam_compressed_depth_image_topic_names[i])) &&
          (camera_info_written[i]))
      {
        try
        {
          cv_bridge::CvImagePtr cv_ptr;
          sensor_msgs::CompressedImageConstPtr compressed_message = message.instantiate<sensor_msgs::CompressedImage>();

          // data is present after config header, image width (4 bytes) and image height(4 bytes)
          unsigned char* compressed_image_buf =
              (unsigned char*)&compressed_message->data[sizeof(compressed_depth_image_transport::ConfigHeader) + 8];
          int compressed_image_buf_size = compressed_message->data.size();

          // reading image width
          int* image_width = (int*)&compressed_message->data[sizeof(compressed_depth_image_transport::ConfigHeader)];
          // reading image height
          int* image_height =
              (int*)&compressed_message->data[sizeof(compressed_depth_image_transport::ConfigHeader) + 4];

          headers_of_bin_files[i].cam_image_width = *image_width;
          headers_of_bin_files[i].cam_image_height = *image_height;

          unsigned short* raw_image = new unsigned short[headers_of_bin_files[i].cam_image_width *
                                                         headers_of_bin_files[i].cam_image_height * 2];

          rvl.DecompressRVL(compressed_image_buf, raw_image,
                            headers_of_bin_files[i].cam_image_width * headers_of_bin_files[i].cam_image_height);

          long timestamp;
          timestamp = (long)compressed_message->header.stamp.toNSec();

          if (img_topic == cam_compressed_ab_image_topic_names[i])
          {
            files[i]->write((char*)&ab, sizeof(bool));
            camera_total_ab_frames[i]++;
          }
          else if (img_topic == cam_compressed_depth_image_topic_names[i])
          {
            files[i]->write((char*)&depth, sizeof(bool));
            camera_total_depth_frames[i]++;
          }
          if (!camera_device_time_stamp_written[i])
          {
            headers_of_bin_files[i].cam_device_timestamp = timestamp;
            camera_device_time_stamp_written[i] = true;
          }
          files[i]->write((char*)&timestamp, sizeof(long));
          files[i]->write((char*)raw_image, sizeof(unsigned short) * (headers_of_bin_files[i].cam_image_width *
                                                                      headers_of_bin_files[i].cam_image_height));
          free(raw_image);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("Failed while reading compressed images from bag file");
        }
      }
    }
  }

  // closing files and rearranging topics according to timestamps
  bag.close();
  for (int k = 0; k < cam_name.size(); k++)
  {
    // writing header
    files[k]->seekp(0);
    headers_of_bin_files[k].cam_total_frames = std::min(camera_total_ab_frames[k], camera_total_depth_frames[k]);
    files[k]->write((char*)&headers_of_bin_files[k].cam_total_frames, sizeof(uint32_t));
    files[k]->write((char*)&headers_of_bin_files[k].cam_image_width, sizeof(uint32_t));
    files[k]->write((char*)&headers_of_bin_files[k].cam_image_height, sizeof(uint32_t));
    files[k]->seekp(20);
    files[k]->write((char*)&headers_of_bin_files[k].cam_first_frame_position, sizeof(uint32_t));
    files[k]->seekp(28);
    files[k]->write((char*)&headers_of_bin_files[k].cam_device_timestamp, sizeof(long));
    files[k]->close();

    findMissingABOrDepthFrames(cam_bin_file_name[k], camera_total_depth_frames[k], camera_total_ab_frames[k]);

    arrangeFramesWithTimeStamps(cam_bin_file_name[k], camera_total_depth_frames[k], camera_total_ab_frames[k]);

    // delete intermediate files
    deleteFile(cam_bin_file_name[k]);
  }

  // clearing all vectors.
  cam_bin_file_name.clear();
  cam_bin_file_name.shrink_to_fit();
  cam_ab_image_topic_names.clear();
  cam_ab_image_topic_names.shrink_to_fit();
  cam_compressed_ab_image_topic_names.clear();
  cam_compressed_ab_image_topic_names.shrink_to_fit();
  cam_depth_image_topic_names.clear();
  cam_depth_image_topic_names.shrink_to_fit();
  cam_compressed_depth_image_topic_names.clear();
  cam_compressed_depth_image_topic_names.shrink_to_fit();
  cam_info_topic_names.clear();
  cam_info_topic_names.shrink_to_fit();
  headers_of_bin_files.clear();
  headers_of_bin_files.shrink_to_fit();
  camera_total_ab_frames.clear();
  camera_total_ab_frames.shrink_to_fit();
  camera_total_depth_frames.clear();
  camera_total_depth_frames.shrink_to_fit();
  camera_device_time_stamp_written.clear();
  camera_device_time_stamp_written.shrink_to_fit();
  camera_info_written.clear();
  camera_device_time_stamp_written.shrink_to_fit();
  files.clear();
  files.shrink_to_fit();
}

/**
 * @brief This is main function
 *
 * @param argc number of input arguments to the function
 * @param argv array of pointer to the input arguments
 * @return int
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "adi_read_rosbags");
  ros::NodeHandle nh("~");

  std::string input_file_name;
  nh.param<std::string>("param_input_file_name", input_file_name, "no name");

  // Get Parameters
  std::vector<std::string> cam_prefix;
  XmlRpc::XmlRpcValue cam_prefix_arr;
  nh.param("param_camera_prefixes", cam_prefix_arr, cam_prefix_arr);
  for (int i = 0; i < cam_prefix_arr.size(); i++)
  {
    cam_prefix.push_back(cam_prefix_arr[i]);
    std::cerr << "camera_prefixes: " << cam_prefix[i] << std::endl;
  }

  storeCameraTopicsInbinFile(input_file_name, cam_prefix);

  std::cout << "Completed." << std::endl;

  ros::shutdown();

  return 0;
}
