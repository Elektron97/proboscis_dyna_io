/****************************************
 * Control Node for Proboscis Prototype *
 ****************************************/

#ifndef CONTROL_UTILS_H_
#define CONTROL_UTILS_H_

// --- Includes --- //
// ROS for C++
#include "ros/ros.h"
// ROS msgs
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32MultiArray.h"

// --- Define --- //
#define NODE_FREQUENCY  50.0    // [Hz] Max publish rate is ~31 Hz
#define QUEUE_SIZE      1

// --- Namespace --- //
using namespace std;        // std io

// --- Global Variables --- //
const string topic_tag = "/proboscis";
const string turns_topic_name = "/cmd_turns";

// ---  Function Signatures --- //

// ---  Classes --- //
class Control_Node
{
    // - Ros objects - //
    ros::NodeHandle node_handle;
    // Sub & Pub objects

};



#endif /* CONTROL_UTILS_H_ */