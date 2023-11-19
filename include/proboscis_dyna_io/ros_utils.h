/****************************
 * Motor Input/Output Node  *
 ****************************/
#ifndef ROS_UTILS_H_
#define ROS_UTILS_H_

/****************************
 * Motor Input/Output Node  *
 ****************************/
// --- Includes --- //
#include "ros_dynamixel_pkg/dynamixel_utils.h"
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
const string turns_topic_name = "/cmd_turns";
const string current_topic_name = "/read_currents";

// ---  Function Signatures --- //

class Ros_Dynamixel_Node
{
    //--- Private Attributes ---//
    // - Ros objects - //
    ros::NodeHandle node_handle;
    // Sub & Pub objects
    ros::Subscriber turns_sub   = node_handle.subscribe(topic_tag + turns_topic_name, QUEUE_SIZE, &Ros_Dynamixel_Node::turns_callBack, this);
    ros::Publisher current_pub  = node_handle.advertise<std_msgs::Float32MultiArray>(topic_tag + current_topic_name, QUEUE_SIZE);

    // Timer
    ros::Timer timer_obj        = node_handle.createTimer(ros::Duration(1/NODE_FREQUENCY), &Ros_Dynamixel_Node::main_loop, this);

    // - Dynamixel Object - //
    ExtPos_Dynamixel dyna_obj   = ExtPos_Dynamixel(N_MOTORS);

    // Useful Variables
    std_msgs::Float32MultiArray motor_currents;

    // Callbacks
    void turns_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg);
    
    public:
        // Constructor
        Ros_Dynamixel_Node();

        // Deconstructor
        ~Ros_Dynamixel_Node();

        // Publisher
        void publish_currents();

        // Main Loop
        void main_loop(const ros::TimerEvent& event);
};


#endif /* ROS_UTILS_H_ */