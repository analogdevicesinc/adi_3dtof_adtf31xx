/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef INPUT_SENSOR_FACTORY_H
#define INPUT_SENSOR_FACTORY_H

#include "input_sensor.h"
#ifdef ENABLE_ADTF31XX_SENSOR
#include "input_sensor_adtf31xx.h"
#endif // ENABLE_ADTF31XX_SENSOR
#include "input_sensor_file_rosbagbin.h"

/**
 * @brief This class is input sensor factory
 *
 */
class InputSensorFactory
{
public:
  /**
   * @brief Get the Input Sensor object
   *
   * @param input_sensor_type
   * @return IInputSensor*
   */
  static IInputSensor * getInputSensor(int input_sensor_type)
  {
    IInputSensor * input_sensor = nullptr;
    switch (input_sensor_type) {
      case 0:
// Camera
#ifdef ENABLE_ADTF31XX_SENSOR
        input_sensor = new InputSensorADTF31XX;
#else

        RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"),
          "Since the ROS node is now executing on the host(not connected to sensor), the value of "
          "arg_input_sensor_mode = 0 "
          "is not supported."
          "Please check for argument arg_input_sensor_mode in related launch files.");
        input_sensor = nullptr;
#endif
        break;
      case 1:
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Not a valid sensor type.");
        input_sensor = nullptr;
        break;
      case 2:
        // ROS Bag Bin
        input_sensor = new InputSensorFileRosbagBin;
        break;
      case 3:
        //Sensor via Network
#ifdef ENABLE_ADTF31XX_SENSOR
        return new InputSensorADTF31XX;
#else
        RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"),
          "Since the ROS node is now executing on the host(not connected to sensor), the value of "
          "arg_input_sensor_mode = 3 is not supported."
          "Please check for argument arg_input_sensor_mode in related launch files.");
        return nullptr;
#endif
      default:
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Not a valid sensor type.");
        input_sensor = nullptr;
        break;
    }
    return input_sensor;
  }
};
#endif
