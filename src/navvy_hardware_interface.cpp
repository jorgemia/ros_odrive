//Load hardware interface stuff
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

//Load controller manager
#include "controller_manager/controller_manager.h"

//Load odrive stuff
#include "odrive_cpp_ros/odrive_cpp_ros.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"

#include "std_msgs/String.h"

#define RIGHT_ODRIVE_SERIAL "35503147790414"//"35730780926030" //Will need to check mine 547849592832
#define LEFT_ODRIVE_SERIAL  "35722209348941"//"35503147790414"
#define ENCODER_CPR 450 //Count per revolution
#define PI 3.14159265358979
#define TWO_PI (2 * PI)
#define RAD_PER_CPR (TWO_PI / ENCODER_CPR)



class Navvy : public hardware_interface::RobotHW {
    public:
    Navvy() {
        hardware_interface::JointStateHandle state_handle_front_left("front_left_wheel", &pos[0], &vel[0], &eff[0]);
        hardware_interface::JointStateHandle state_handle_rear_left("rear_left_wheel", &pos[1], &vel[1], &eff[1]);
        hardware_interface::JointStateHandle state_handle_front_right("front_right_wheel", &pos[2], &vel[2], &eff[2]);
        hardware_interface::JointStateHandle state_handle_rear_right("rear_right_wheel", &pos[3], &vel[3], &eff[3]);
        
        jnt_state_interface.registerHandle(state_handle_front_left);
        jnt_state_interface.registerHandle(state_handle_rear_left);
        jnt_state_interface.registerHandle(state_handle_front_right);
        jnt_state_interface.registerHandle(state_handle_rear_right);

        registerInterface(&jnt_state_interface);

        hardware_interface::JointHandle vel_handle_front_left(jnt_state_interface.getHandle("front_left_wheel"), &cmd[0]);
        hardware_interface::JointHandle vel_handle_rear_left(jnt_state_interface.getHandle("rear_left_wheel"), &cmd[1]);
        hardware_interface::JointHandle vel_handle_front_right(jnt_state_interface.getHandle("front_right_wheel"), &cmd[2]);
        hardware_interface::JointHandle vel_handle_rear_right(jnt_state_interface.getHandle("rear_right_wheel"), &cmd[3]);
        
        for (int i = 0; i < 4; ++i) {
            cmd[i] = 0.0;
        }

        jnt_vel_interface.registerHandle(vel_handle_front_left);
        jnt_vel_interface.registerHandle(vel_handle_rear_left);
        jnt_vel_interface.registerHandle(vel_handle_front_right);
        jnt_vel_interface.registerHandle(vel_handle_rear_right);

        registerInterface(&jnt_vel_interface); //up to here it follows the standard stuff from ros tutorial

        //Create an array with 2 strings, which are the odrive serial numbers
        std::string ser_nums[2] = {LEFT_ODRIVE_SERIAL, RIGHT_ODRIVE_SERIAL};
        //Create and array of strings, assign odrive to each motor FL, BL, FR, BR
        std::string set_motor_map[4] = {LEFT_ODRIVE_SERIAL, LEFT_ODRIVE_SERIAL, RIGHT_ODRIVE_SERIAL, RIGHT_ODRIVE_SERIAL};
        //Create an array defining the motor index to be used by odrive - either 0 or 1 - Remember when wiring up
        uint8_t motor_indexes[4] = {0, 1, 0, 1};
        //Create new motor driver for 2 odrives and initialise it. If it starts up, set motors enabled to true.
        motor_driver = new odrive::ODriveDriver(ser_nums, 2, set_motor_map, motor_indexes, 4);
        int result = motor_driver->init();
        if (result != 0) {
            std::cout << "Could not connect to odrives!"<< std::endl;
            motors_enabled = false;
        } else {
            motor_driver->sendWatchdog();
            motor_driver->setErrors();
            motor_driver->setStates(); 
            motors_enabled = true; //If connected to odrive, set motors_enable to true
            
        }
    }

    void publishMessage(ros::Publisher voltage_pub) {
        float voltage;
        for (int i = 0; i < 4; ++i) {
            motor_driver->getBusVoltage(i, voltage);

            batvol[i] = voltage;
        }

        sensor_msgs::BatteryState msg;
        msg.voltage = batvol[0];
        msg.power_supply_status = 0;
        msg.power_supply_health = 0;
        msg.power_supply_technology = 0;
        msg.present = true;

        voltage_pub.publish(msg);

    }

    //Get encoder readings
    void updateJointsFromHardware() {
        //std::cout << "Updating joints" << std::endl;
        float speed;
	    float motor_pos;
        float pos_cpr;
        float pos_delta;
        double angle_delta;
        //For each joint
        for (int i = 0; i < 4; ++i) {
            //get speed of motor
            motor_driver->getMotorSpeed(i, speed);
            //Get speed reading for each wheel - multiply speed in count/s to get it in rad/s
            vel[i] = ((speed * direction_multipliers[i]) / ENCODER_CPR) * 2 * 3.141592;
            //std::cout << "speed: " << vel[i] << std::endl; so this only goes positive and negative (no switching)
            
            motor_driver->getMotorPosition(i, motor_pos); //in counts
            
            if (first_pos_yet){
            	first_motor_pos[i] = motor_pos;
            	std::cout << "first motor pos : " << first_motor_pos[i] << std::endl;
            }
            
            pos[i] = (((motor_pos-first_motor_pos[i]) * direction_multipliers[i])/ENCODER_CPR) * 2 * 3.141592;
            
            //std::cout << "position: " << pos[i] << std::endl; // so this only goes positive and negative (no switching)
            
            //Get position from encoders
            //motor_driver->getPosCPR(i, pos_cpr); //In counts per revolution
            	//If previously had encoder readings
            //if (last_cpr_populated) {
                //change in pos = current reading vs last reading
                //pos_delta = pos_cpr - last_pos_cpr[i];
                //std::cout << "pos delta : " << pos_delta << std::endl;
                
                //if (pos_delta < - ENCODER_CPR / 2.0) { //If position delta smaller than -45 cpr (ie. -180) transform it to positive
                    //overflow will go from eg 80 to 10. Delta = -70
                    // want to add on ENCODER_CPR to change to delta = 20
                    //pos_delta += ENCODER_CPR;
                //} else if (pos_delta > ENCODER_CPR / 2.0) { //If position delta bigger than 45 cpr (ie. 180) transform it to negative
                    //underflow will go from eg 10 to 80, Delta = 70
                 //   pos_delta -= ENCODER_CPR;
                //}
                //pos_delta *= direction_multipliers[i]; //Multiply to get direction of wheels correct 
                //angle_delta = pos_delta * RAD_PER_CPR; // Transform to angle in radians

                //Position in radians
                //pos[i] = pos[i] + angle_delta; //Initially pos will be 0 (Setting initial offset). Then pos added incrementally.

                //std::cout << "Motor " << i << " pos " << pos[i] << " delta: " << angle_delta << std::endl;

            //}         
            //last_pos_cpr[i] = pos_cpr; //exit loop and save reading as latest

            // int motor_pos;
            // motor_driver->readCurrentMotorPosition(i, motor_pos);
            // std::cout << "sad: " << i << " : " << motor_pos << std::endl;
            // motor_driver->getMotorPosition(i, motor_pos);
            
        }
        //last_cpr_populated = true; //After first reading set variable to true
        first_pos_yet = false; 
    }   

    //Write commands to motors
    void writeCommandsToHardware() {
 
        float target_speeds[4];//Create target speeds array of floats
        //For each wheel
        for (int i = 0; i < 4; ++i) {
            //If command too big or too small - ignore. Why 200?
            if (cmd[i] < -1000 || cmd[i] > 1000) {
                std::cout << "Motor speed request: " << cmd[i] << " ignored." << std::endl;
                target_speeds[i] = 0.0;
            } else {
                target_speeds[i] = cmd[i] / TWO_PI * ENCODER_CPR * direction_multipliers[i]; //Convert from rad/s to CPR (Divide by 2pi to get rev then multiply to get count), correct direction, save to target speeds
            }
            //std::cout << target_speeds[i] << " ";
        }
        // std::cout << std::endl;
        
        //if motors connected
        if (motors_enabled) {
            motor_driver->sendWatchdog();
            motor_driver->setMotorSpeeds(target_speeds);
        }
    }

	void vgain_callback(const std_msgs::String::ConstPtr& msg) {

		int space = msg->data.find(' ');

		if (space != -1) {

			int motor = std::stoi(msg->data.substr(0, space));
			float gain = std::stof(msg->data.substr(space + 1));
			
			if (motor > -1 && motor < 4) motor_driver->setVGain(motor, gain);
		}
	}

	void vigain_callback(const std_msgs::String::ConstPtr& msg) {

		int space = msg->data.find(' ');

		if (space != -1) {

			int motor = std::stoi(msg->data.substr(0, space));
			float gain = std::stof(msg->data.substr(space + 1));
			
			if (motor > -1 && motor < 4) motor_driver->setVIGain(motor, gain);
		}
	}

	void ccbandwidth_callback(const std_msgs::String::ConstPtr& msg) {

		int space = msg->data.find(' ');

		if (space != -1) {

			int motor = std::stoi(msg->data.substr(0, space));
			float bandwidth = std::stof(msg->data.substr(space + 1));
			
			if (motor > -1 && motor < 4) motor_driver->setCCBandwidth(motor, bandwidth);
		}
	}

		

    //This seems to follow the ROS tutorial but check the lower parts
    private:
        odrive::ODriveDriver *motor_driver;
        hardware_interface::JointStateInterface jnt_state_interface; //Define here and used at the top
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        //Define arrays and set to 0
        double cmd[4] = {0, 0, 0, 0}; //Initially set to 0. Controller Manager gets this from diff drive controller
        double pos[4] = {0, 0, 0, 0}; //Controller manager will read these and feed to controller
        double vel[4] = {0, 0, 0, 0};
        double eff[4] = {0, 0, 0, 0};

        float last_pos_cpr[4]; //Create an array of 4 floats for last position CPR of each wheel use above
        float first_motor_pos[4];
        bool last_cpr_populated = false; //Create last_cpr_populated variable and set to false initally
        float batvol[4];
	    bool first_pos_yet = true;
	
        int direction_multipliers[4] = {1, 1, -1, -1}; //Define direction to turn - used at top
        bool motors_enabled; //Create motors enabled variable which will be true when connected

};





//This file combines navvy_hardware.h, navvy_hardware.cpp and navvy_base.cpp into a single file
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "navvy_odrive_interface");
    Navvy robot;
    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::NodeHandle n;
    
    ros::Publisher voltage_pub = n.advertise<sensor_msgs::BatteryState>("/battery", 10);

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0); //Is this rate enough?

	// PID tuning
	ros::Subscriber pid_sub1 = n.subscribe("v_gain", 1, &Navvy::vgain_callback, &robot);
	ros::Subscriber pid_sub2 = n.subscribe("vi_gain", 1, &Navvy::vigain_callback, &robot);
	ros::Subscriber pid_sub3 = n.subscribe("cc_bandwidth", 1, &Navvy::ccbandwidth_callback, &robot);

    while(ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;
        
        robot.publishMessage(voltage_pub);
        robot.updateJointsFromHardware();
        cm.update(time, period); //Not 100% sure how the timing stuff works and what the best way to do it is.
        robot.writeCommandsToHardware();
        rate.sleep();
    }
    return 0;
}
