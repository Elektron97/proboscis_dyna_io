/****************************
 * Motor Input/Output Node  *
 ****************************/
#ifndef ROS_UTILS_H_
#define ROS_UTILS_H_

/****************************
 * Motor Input/Output Node  *
 ****************************/
// --- Includes --- //
// #include "ros_dynamixel_pkg/dynamixel_utils.h"
#include "ros_dynamixel_pkg/custom_current_control.h"
// msgs
#include "std_msgs/Float32MultiArray.h"

// --- Define --- //
#define NODE_FREQUENCY  50.0    // [Hz] Max publish rate is ~31 Hz
#define QUEUE_SIZE      10
#define N_MOTORS        7

// --- Namespace --- //
using namespace std;        // std io

// --- Global Variables --- //
const string topic_tag = "/proboscis";
const string torque_topic_name = "/cmd_torque";
const string current_topic_name = "/read_currents";

// ---  Function Signatures --- //

class Ros_Dynamixel_Node
{
    //--- Private Attributes ---//
    // - Ros objects - //
    ros::NodeHandle node_handle;
    // Sub & Pub objects
    ros::Subscriber torque_sub   = node_handle.subscribe(topic_tag + torque_topic_name, QUEUE_SIZE, &Ros_Dynamixel_Node::torque_callBack, this);
    // ros::Publisher current_pub  = node_handle.advertise<std_msgs::Float32MultiArray>(topic_tag + current_topic_name, QUEUE_SIZE);

    // Timer
    // ros::Timer timer_obj        = node_handle.createTimer(ros::Duration(1/NODE_FREQUENCY), &Ros_Dynamixel_Node::main_loop, this);

    // - Dynamixel Object - //
    Current_PID dyna_obj   = Current_PID(N_MOTORS);

    // Useful Variables
    // std_msgs::Float32MultiArray motor_currents;

    // Callbacks
    void torque_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg);
    
    public:
        // Constructor
        Ros_Dynamixel_Node();

        // Deconstructor
        ~Ros_Dynamixel_Node();

        // Publisher
        // void publish_currents();

        // Main Loop
        // void main_loop(const ros::TimerEvent& event);
};


#endif /* ROS_UTILS_H_ */