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
// PID Library
#include "pid/pid.h"

// --- Define --- //
#define NODE_FREQUENCY  30.0
#define QUEUE_SIZE      1

// n° motors
#define N_MOTORS        7

// n° of buttons
#define N_BUTTONS       11

// PID Parameters
#define KP 5.0
#define KI 0.1
#define KD 0.001
// Saturation
#define MAX_OUTPUT  5.0
#define MIN_OUTPUT  0.0

// --- Namespace --- //
using namespace std;        // std io

// --- Enum --- //
//enum Joystick_Buttons {A, B, X, Y, LB, RB, BACK, START, POWER, L3, R3};

// --- Global Variables --- //
// Topic Names
const string topic_tag = "/proboscis";
const string turns_topic_name = "/cmd_turns";
const string current_topic_name = "/read_currents";
const string joy_topic_name = "/joy";

// ---  Function Signatures --- //
void joy2Motors(sensor_msgs::Joy joystick_input, std_msgs::Float32MultiArray& motor_cmd, float max_value);

// ---  Classes --- //
class Control_Node
{
    // - Ros objects - //
    ros::NodeHandle node_handle;
    // Sub & Pub objects
    ros::Subscriber sub_joy     = node_handle.subscribe(joy_topic_name, QUEUE_SIZE, &Control_Node::joy_callBack, this);
    ros::Publisher  pub_turns   = node_handle.advertise<std_msgs::Float32MultiArray>(topic_tag + turns_topic_name, QUEUE_SIZE);
    // Timer for main loop
    ros::Timer timer_obj        = node_handle.createTimer(ros::Duration(1/NODE_FREQUENCY), &Control_Node::main_loop, this);

    // Turn commands
    std_msgs::Float32MultiArray turn_commands;

    // Callbacks (private method)
    void joy_callBack(const sensor_msgs::Joy::ConstPtr& msg);
    void current_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg);

    public:
        // Constructor
        Control_Node();

        // Deconstructor
        ~Control_Node();

        // Publisher
        void publish_turns();

        // Main Loop
        void main_loop(const ros::TimerEvent& event);
};

#endif /* CONTROL_UTILS_H_ */