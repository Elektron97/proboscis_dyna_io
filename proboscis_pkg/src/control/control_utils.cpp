/********************************************
 * Source code for ROS Control Node Library *
 ********************************************/
#include "proboscis_pkg/control/control_utils.h"

// --- ROS CONTROL NODE CLASS --- //
// Constructor
Control_Node::Control_Node()
{
    // Init vector objs
    turn_commands.data = vector<float>(N_MOTORS);   // store command

    // Init iterator
    //i = 0;
}

Control_Node::~Control_Node()
{
    // Deconstructor
    ROS_INFO("Control Node turning off."); 
}

void Control_Node::joy_callBack(const sensor_msgs::Joy::ConstPtr& msg)
{
    // --- Read Joystick and map into turn commands --- //
    joy2Motors(*msg, turn_commands, MAX_OUTPUT);  // just set position
}

void Control_Node::publish_turns()
{
    pub_turns.publish(turn_commands);
}

void Control_Node::main_loop(const ros::TimerEvent& event)
{
    // Publish /cmd_turns
    publish_turns();
}

// --- FUNCTIONS --- //
void joy2Motors(sensor_msgs::Joy joystick_input, std_msgs::Float32MultiArray& motor_cmd, float max_value)
{
    // Simple boolean mapping (only for testing)
    for(int i = 0; i < N_MOTORS; i++)
    {   
        if(joystick_input.buttons[i] == 1)
            motor_cmd.data[i] = max_value;
        else
            motor_cmd.data[i] = 0.0;
    }

    // --- Extract Information from LEFT stick --- //
    //float rho, theta;
    //cartesian2Polar(-joystick_input.axes[0], joystick_input.axes[1], rho, theta);
    
    // Mapping Bending
    // Including Optimization Tool? -> RobOptim or fmincon() in C++
}