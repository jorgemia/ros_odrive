include "ros_odrive/odrive.hpp"

using namespace std;

Json::Value odrive_json; //define odrive_json type to json
bool targetJsonValid = false;
//endpoint is a pointer to an object of class odrive_endpoint 
//odrive_endpoint *endpoint[2] = NULL; //Here or below?


ODriveDriver::ODriveDriver(const std::string* odrive_serial_numbers,
               const uint8_t num_odrives,
               const std::string* motor_to_odrive_serial_number_map, //array of 4 containing motor serial numbers
               const uint8_t* motor_index_map, //array of 4 containing if motor is axis 0 or 1
               const uint8_t num_motors) {
   
    // read settings
    num_odrives_ = num_odrives;
    num_motors_ = num_motors;

    // saved for use between creation and init
    // save serial numbers to array of size 2
    odrive_serial_numbers_ = new std::string[num_odrives]();
    for (uint8_t i = 0; i < num_odrives; ++i) {
        odrive_serial_numbers_[i].assign(odrive_serial_numbers[i]);
    }

    motor_to_odrive_serial_number_map_ = new std::string[num_motors]();
    motor_index_map_ = new uint8_t[num_motors]();
    for (uint8_t i = 0; i < num_motors; ++i) {
        //save array containing serial numbers of each motor //////USE THIS OR MOTOR TO ODRIVE MAP??
        motor_to_odrive_serial_number_map_[i].assign(motor_to_odrive_serial_number_map[i]);
        //save array saying motor axis
        motor_index_map_[i] = motor_index_map[i];
    }

    //OR CAN I JUST USE THIS???
    uint8_t motor_to_odrive_map[4] = {0, 0, 1, 1};

}

//This is a destructor - used to shut stuff down
ODriveDriver::~ODriveDriver() {
    
    for (uint8_t i = 0; i < 2; i++){
        
        endpoint[i]->remove();

        delete endpoint[i];

    }

    if (motor_index_map_) delete [] motor_index_map_;
    if (odrive_serial_numbers_) delete [] odrive_serial_numbers_;
    if (motor_to_odrive_serial_number_map_) delete [] motor_to_odrive_serial_number_map_;

    return 0;

}

/**
 *
 * Initialise odrives
 *
 */
int ODriveDriver::init() {

    //The part below of endpoints and json is what I need to modify for 2 boards
    // Get odrive endpoint instance
    //endpoint needs to be an array of 2 containing the endpoints for each serial number
    endpoint0 = new odrive_endpoint();
    endpoint1 = new odrive_endpoint();
    odrive_endpoint *endpoint[2] = {endpoint0, endpoint1};

    for (i = 0; i < 2; ++i) {
    // Enumerate Odrive target
        if (endpoint[i]->init(stoull(odrive_serial_numbers_[i], 0, 16)))
        {
            ROS_ERROR("Device not found!");
            return 1;
        }

        // Read JSON from target
        if (getJson(endpoint[i], &odrive_json)) {
            return 1;
        }
        targetJsonValid = true;
    }

    //Got rid of watchdog as diff drive supposedly has a timeout thing

}

int ODriveDriver::setMotorSpeed(int motor_index, float motor_speed) {
    std::string cmd;

    //Get which axis depending on index
    cmd = "axis"
    cmd.append(std::to_string(motor_index_map_[motor_index]))
    
    //odrive json will depend on odrive being targeted
    writeOdriveData(endpoint[motor_to_odrive_map[motor_index]], odrive_json, cmd.append(".controller.vel_setpoint"), motor_speed);

}

int ODriveDriver::setMotorSpeeds(float* motor_speeds) {
    for (uint8_t i = 0; i < num_motors_; ++i) {
        setMotorSpeed(i, motor_speeds[i]);
        
    }
    
}

//Get encoder position reading for motor
int ODriveDriver::getPosCPR(int motor_index, float &pos) {
    std::string cmdpos;
    
    cmdpos = "axis"
    cmdpos.append(std::to_string(motor_index_map_[motor_index]))

    //Encoder Position
    readOdriveData(endpoint[motor_to_odrive_map[motor_index]], odrive_json, cmdpos.append(".encoder.pos_estimate"), pos);
    
}

//Get encoder speed reading for motor
int ODriveDriver::getMotorSpeed(int motor_index, float &motor_speed) {
    std::string cmdvel;
    
    cmdvel = "axis"
    cmdvel.append(std::to_string(motor_index_map_[motor_index]))

    //Encoder Velocity
    readOdriveData(endpoint[motor_to_odrive_map[motor_index]], odrive_json, cmdvel.append("encoder.vel_estimate"), motor_speed);

}


/***Not used for now
int ODriveDriver::getBusVoltage(int motor_index, float &voltage) {
    getFloat(motor_index, voltage, VBUS_VOLTAGE);
    std::cout << "Voltage: " << motor_index << " : " << voltage << std::endl;
    //To this
    readOdriveData(endpoint, odrive_json, string("vbus_voltage"), fval);
    msg.vbus = fval;
}

int ODriveDriver::checkErrors(uint8_t* error_codes_array) {

    int cmd;
    int axis_offset;
    for (uint8_t i = 0; i < num_motors_; ++i) {
        axis_offset = (motor_index_map_[i] == 1) ? per_axis_offset : 0;
        cmd = AXIS__ERROR + axis_offset;
        
        uint8_t handle_index = motor_to_odrive_handle_index_[i];
        
        uint8_t motor_error_output;


        readOdriveData(endpoint, odrive_json, string("axis0.error"), u16val);
        msg.error0 = u16val;
        readOdriveData(endpoint, odrive_json, string("axis1.error"), u16val);
        msg.error1 = u16val;


        int result = odriveEndpointGetUInt8(odrive_handles_[handle_index], cmd, motor_error_output);
        if (result != LIBUSB_SUCCESS) {
            std::cerr << "CppSdk::checkErrors couldn't send `" << std::to_string(cmd) << "` to '" << odrive_serial_numbers_[handle_index] << "': `" << result << "` (see prior error message)" << std::endl;
            return ODRIVE_SDK_UNEXPECTED_RESPONSE;
        }
        error_codes_array[i] = motor_error_output;
    }
    return ODRIVE_SDK_COMM_SUCCESS;
}***/

