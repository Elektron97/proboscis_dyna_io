/****************************
 * Motor Input/Output Node  *
 ****************************/

// --- Includes --- //
// ROS for C++
#include "ros.h"
// msgs
#include "std_msgs/Float64MultiArray.h"

// Dynamixels
#include "dynamixel_sdk/dynamixel_sdk.h"

// --- Define --- //
#define QUEUE_SIZE 10

// --- Namespace --- //
using namespace std;        // std io
using namespace dynamixel;  // dynamixel

// --- Global Variables --- //
const string topic_tag = "/proboscis";
const string torque_topic_name = "/cmd_torque";

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

    // Publishing: Effective Torque and Velocity -> Power (?)

    // --- Main Loop --- //
    ros::spin(); // only subscribing ros node

    // --- End of Program --- //
    ROS_INFO("Turning OFF all the motors.");    
    return 0;
}