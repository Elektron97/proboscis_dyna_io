/****************************
 * Motor Input/Output Node  *
 ****************************/
// --- Includes --- //
#include "proboscis_pkg/dynamixel_utils.hpp"

// --- Define --- //
#define QUEUE_SIZE 10

// --- Namespace --- //
using namespace std;        // std io

// --- Global Variables --- //
string topic_tag = "/proboscis";
string torque_topic_name = "/cmd_torque";

// Functions
void torque_callBack(const std_msgs::Float64MultiArray::ConstPtr& msg);

// --- Main --- //
int main(int argc, char** argv)
{
    // --- Init ROS Node --- //
    ros::init(argc, argv, "motor_io");
    ros::NodeHandle node_obj;

    // --- Define Subscribers and Publishers --- //
    // Subscribing: Writing into the motors
    ros::Subscriber torque_sub = node_obj.subscribe(topic_tag.append(torque_topic_name), QUEUE_SIZE, torque_callBack);

    // Dynamixel Object
    Dynamixel_Motors dyna_obj = Dynamixel_Motors();

    // --- Main Loop --- //
    ros::spin(); // only subscribing ros node

    // --- End of Program --- //
    ROS_INFO("Turning OFF all the motors.");    
    return 0;
}

void torque_callBack(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    ROS_INFO("Dynamixel Node here.");
}