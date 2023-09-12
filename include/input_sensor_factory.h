/******************************************************************************
Copyright (c), 2023 - Analog Devices Inc. All Rights Reserved.
This software is PROPRIETARY & CONFIDENTIAL to Analog Devices, Inc.
and its licensors.
******************************************************************************/

#ifndef INPUT_SENSOR_FACTORY_H
#define INPUT_SENSOR_FACTORY_H

#include "input_sensor_adtf31xx.h"
#include "input_sensor_file.h"
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
        // File
        return new InputSensorFile;
      case 2:
        // ROS Bag Bin
        return new InputSensorFileRosbagBin;
        break;
      default:
        ROS_INFO_STREAM("Not a valid senor type.");
        return nullptr;
    }
  }
};
#endif