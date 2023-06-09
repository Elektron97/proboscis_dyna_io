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

    // Init Motors
    Ros_Dynamixel_Node node_obj;

    // --- Main Loop --- //
    // only subscribing ros node
    //ros::spin();

    // publishing node
    while(ros::ok())
    {
        // Read Commands
        ros::spinOnce();

        // Publish
        node_obj.publish_currents();
        node_obj.sleep();
    }
    // --- End of Program --- //    
    return 0;
}