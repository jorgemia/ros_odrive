#ifndef ODRIVE_HPP_
#define ODRIVE_HPP_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <libusb-1.0/libusb.h>
//#include "ros_odrive/odrive_msg.h" can get rid of this
//#include "ros_odrive/odrive_ctrl.h" can get rid of this
#include <string>
#include <fstream>

#include "ros/ros.h"
#include "ros_odrive/odrive_endpoint.hpp"
#include "ros_odrive/odrive_utils.hpp"
#include "ros_odrive/odrive_enums.hpp"
#include <jsoncpp/json/json.h>

#define ODRIVE_OK    0
#define ODRIVE_ERROR 1

// Listener commands - Dont think I need this?
//enum commands {
//    CMD_AXIS_RESET,
//    CMD_AXIS_IDLE,
//    CMD_AXIS_CLOSED_LOOP,
//    CMD_AXIS_SET_VELOCITY,
//    CMD_REBOOT
//};

namespace odrive
{

    class ODriveDriver {

    public:
        ODriveDriver(
                const std::string* odrive_serial_numbers,
                const uint8_t num_odrives,
                const std::string* motor_to_odrive_serial_number_map,
                const uint8_t* motor_index_map,
                const uint8_t num_motors); 
        ~ODriveDriver();

        int init(); // start communication

        int getPosCPR(int motor_index, float &pos);

        int setMotorSpeed(int motor_index, float motor_speed);
        int setMotorSpeeds(float* motor_speeds); // assumed to match num_motors

        int getMotorSpeed(int motor_index, float &motor_speed);
        int getBusVoltage(int motor_index, float &voltage);
        
        int checkErrors(uint8_t* error_codes_array); // assumed to match num_motors

    private:
        // read settings
        uint8_t num_odrives_;
        uint8_t num_motors_;

        // saved for use between creation and init
        std::string* odrive_serial_numbers_;
        std::string* motor_to_odrive_serial_number_map_;
        uint8_t* motor_index_map_;

    };
}

#endif
