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

// Current Dynamixel Object
Current_Dynamixel dyna_obj(N_MOTORS);

// --- CallBacks --- //
void torque_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg);

// --- Main --- //
int main(int argc, char** argv)
{
    // --- Init ROS Node --- //
    ros::init(argc, argv, "motor_io");
    ros::NodeHandle node_obj;

    // --- Define Subscribers and Publishers --- //
    // Subscribing: Writing into the motors
    ros::Subscriber torque_sub = node_obj.subscribe(topic_tag.append(torque_topic_name), QUEUE_SIZE, torque_callBack);

    // --- Main Loop --- //
    ros::spin(); // only subscribing ros node

    // --- End of Program --- //
    ROS_INFO("Turning OFF all the motors.");    
    return 0;
}

void torque_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg)
{    
    ROS_INFO("Torque command received.");

    // Verify n of motors
    if(msg->layout.dim[0].size == N_MOTORS)
    {
        //Extract array of torques
        if(dyna_obj.set_torques(msg->data))
            ROS_INFO("Torque command written correctly on Dynamixels.");
        else
        {
            ROS_ERROR("Torque command written uncorrectly on Dynamixels.");
            return;
        }
    }
    else
    {
        ROS_ERROR("/cmd_torque msg is uncorrect. Wrong number of motors.");
        return;
    }
}