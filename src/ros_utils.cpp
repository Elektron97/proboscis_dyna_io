/****************************************
 * Source code for ROS Node Library     *
 ****************************************/
#include "proboscis_dyna_io/ros_utils.h"

// --- ROS DYNAMIXEL NODE CLASS --- //
Ros_Dynamixel_Node::Ros_Dynamixel_Node()
{
    // Init current vector
    motor_currents.data = std::vector<float>(N_MOTORS);
}

Ros_Dynamixel_Node::~Ros_Dynamixel_Node()
{
    // Deconstructor
    ROS_INFO("Turning OFF all the motors."); 
}

// To rewrite
void Ros_Dynamixel_Node::torque_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    ROS_INFO("Turns command received.");

    // Verify n of motors
    if(msg->data.size() == N_MOTORS)
    {
        //Extract array of torques (for now: currents) | save for publishing the sensor data
        if(dyna_obj.set_currents(msg->data, ros::Time::now().toSec(), motor_currents.data))
            ROS_INFO("Torques command written correctly on Dynamixels.");
        else
        {
            ROS_ERROR("Torques command written uncorrectly on Dynamixels.");
            return;
        }
    }
    else
    {
        ROS_ERROR("/cmd_torques msg is uncorrect. Wrong number of motors.");
        return;
    }
}

void Ros_Dynamixel_Node::main_loop(const ros::TimerEvent& event)
{
    // Publish /read_currents
    current_pub.publish(motor_currents);
}