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
const string path_name = "Motor_CurrentPID";

// ---  Function Signatures --- //
template <typename T>
T my_getParam(ros::NodeHandle node_obj, string param_path)
{
    T param_value;
    node_obj.getParam(param_path, param_value);
    return param_value;
}

class Ros_Dynamixel_Node
{
    //--- Private Attributes ---//
    // - Ros objects - //
    ros::NodeHandle node_handle;
    // Sub & Pub objects
    ros::Subscriber torque_sub   = node_handle.subscribe(topic_tag + torque_topic_name, QUEUE_SIZE, &Ros_Dynamixel_Node::torque_callBack, this);
    ros::Publisher current_pub  = node_handle.advertise<std_msgs::Float32MultiArray>(topic_tag + current_topic_name, QUEUE_SIZE);

    // Timer
    ros::Timer timer_obj        = node_handle.createTimer(ros::Duration(1/NODE_FREQUENCY), &Ros_Dynamixel_Node::main_loop, this);

    // - Dynamixel Object - //
    // Internal PID gains
    float Kp = my_getParam<float>(node_handle, path_name + "/P");
    float Ki = my_getParam<float>(node_handle, path_name + "/I");
    float Kd = my_getParam<float>(node_handle, path_name + "/D");

    // Init Dynamixel Object
    //Current_PID dyna_obj   = Current_PID(N_MOTORS);   // Using after tuning process
    Current_PID dyna_obj   = Current_PID(N_MOTORS, Kp, Ki, Kd);

    // Useful Variables
    std_msgs::Float32MultiArray motor_currents;

    // Callbacks
    void torque_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg);
    
    public:
        // Constructor
        Ros_Dynamixel_Node();

        // Deconstructor
        ~Ros_Dynamixel_Node();

        // Main Loop
        void main_loop(const ros::TimerEvent& event);
};


#endif /* ROS_UTILS_H_ */