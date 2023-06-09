/****************************
 * Motor Input/Output Node  *
 ****************************/
#ifndef ROS_UTILS_H_
#define ROS_UTILS_H_

/****************************
 * Motor Input/Output Node  *
 ****************************/
// --- Includes --- //
#include "proboscis_pkg/dynamixel_utils.h"
// msgs
#include "std_msgs/Float32MultiArray.h"

// --- Define --- //
#define QUEUE_SIZE  10
#define N_MOTORS    7

// --- Namespace --- //
using namespace std;        // std io

// --- Global Variables --- //
string topic_tag = "/proboscis";
string torque_topic_name = "/cmd_torque";
string turns_topic_name = "/cmd_turns";

class Ros_Dynamixel_Node
{
    //--- Private Attributes ---//
    // Ros objects
    ros::NodeHandle node_handle;
    ros::Subscriber turns_sub = node_handle.subscribe(topic_tag.append(turns_topic_name), QUEUE_SIZE, &Ros_Dynamixel_Node::turns_callBack, this);

    // Callbacks
    void turns_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg);
    
    public:
        // Dynamixel Object
        ExtPos_Dynamixel dyna_obj = ExtPos_Dynamixel(N_MOTORS);

        // Constructor
        Ros_Dynamixel_Node();

        // Deconstructor
        ~Ros_Dynamixel_Node();
};


#endif /* ROS_UTILS_H_ */