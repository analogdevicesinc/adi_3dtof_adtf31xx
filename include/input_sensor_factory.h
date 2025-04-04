/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef INPUT_SENSOR_FACTORY_H
#define INPUT_SENSOR_FACTORY_H
#ifdef ENABLE_ADTF31XX_SENSOR
#include "input_sensor_adtf31xx.h"
#endif
#include "input_sensor_file_rosbagbin.h"
#include "input_sensor.h"

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
  static IInputSensor* getInputSensor(int input_sensor_type)
  {
    switch (input_sensor_type)
    {
      case 0:
// Camera
#ifdef ENABLE_ADTF31XX_SENSOR
        return new InputSensorADTF31XX;
#else
        ROS_ERROR(
            "Since the ROS node is now executing on the host, the value of arg_input_sensor_mode = 0 is not supported."
            "Please check for argument arg_input_sensor_mode in related launch files.");
        return nullptr;
#endif
        break;
      case 1:
        // File mode is depricated
        ROS_ERROR("File mode is deprecated, use ROS Bag Bin mode instead.");
        return nullptr;
      case 2:
        // ROS Bag Bin
        return new InputSensorFileRosbagBin;
        break;
      case 3:
        // Sensor via Network
#ifdef ENABLE_ADTF31XX_SENSOR
        return new InputSensorADTF31XX;
#else
        ROS_ERROR(
            "Since the ROS node is now executing on the host, the value of arg_input_sensor_mode = 0/3 is not "
            "supported."
            "Please check for argument arg_input_sensor_mode in related launch files.");
        return nullptr;
#endif

      default:
        ROS_INFO_STREAM("Not a valid senor type.");
        return nullptr;
    }
  }
};
#endif