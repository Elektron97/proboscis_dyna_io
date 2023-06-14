/*****************
 * Control Node  *
 *****************/
// --- Includes --- //
#include "proboscis_pkg/control_utils.h"

// --- Main --- //
int main(int argc, char** argv)
{
    // --- Init ROS Node --- //
    ros::init(argc, argv, "controller");

    // Init Motors
    Control_Node node_obj;

    // --- Main Loop --- //
    ros::spin();

    // --- End of Program --- // 
    ROS_INFO("End of Main");   
    return 0;
}