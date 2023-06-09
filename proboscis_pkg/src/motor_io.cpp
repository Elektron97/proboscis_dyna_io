/****************************
 * Motor Input/Output Node  *
 ****************************/
// --- Includes --- //
#include "proboscis_pkg/dynamixel_utils.h"
#include "proboscis_pkg/ros_utils.h"

// --- Main --- //
int main(int argc, char** argv)
{
    // --- Init ROS Node --- //
    ros::init(argc, argv, "motor_io");

    Ros_Dynamixel_Node node_obj;

    // --- Main Loop --- //
    ros::spin(); // only subscribing ros node

    // --- End of Program --- //;    
    return 0;
}