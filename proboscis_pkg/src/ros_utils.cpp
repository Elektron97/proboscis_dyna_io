/****************************************
 * Source code for ROS Node Library     *
 ****************************************/
#include "proboscis_pkg/ros_utils.h"

// --- ROS DYNAMIXEL NODE CLASS --- //
Ros_Dynamixel_Node::Ros_Dynamixel_Node()
{
    // Constructor
}

Ros_Dynamixel_Node::~Ros_Dynamixel_Node()
{
    // Deconstructor
    ROS_INFO("Turning OFF all the motors."); 
}

void Ros_Dynamixel_Node::turns_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    ROS_INFO("Turns command received.");

    // Verify n of motors
    if(msg->data.size() == N_MOTORS)
    {
        //Extract array of torques
        if(dyna_obj.set_turns(msg->data))
            ROS_INFO("Turns command written correctly on Dynamixels.");
        else
        {
            ROS_ERROR("Turns command written uncorrectly on Dynamixels.");
            return;
        }
    }
    else
    {
        ROS_ERROR("/cmd_turns msg is uncorrect. Wrong number of motors.");
        return;
    }
}