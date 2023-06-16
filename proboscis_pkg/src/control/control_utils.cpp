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
    ref_currents.data = vector<float>(N_MOTORS);    // store ref

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
    //joy2Motors(*msg, turn_commands, MAX_OUTPUT);  // [Old] -> just set position
    joy2Motors(*msg, ref_currents, 0.2);    // for now: setting to 0.2 A all active motors
}

void Control_Node::current_callBack(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // --- Read Current and Implement PID Controller --- //
    for(int i = 0; i < N_MOTORS; i++)
    {
        if(i + 1 != 5)
            turn_commands.data[i] = pid_obj.calculate(ref_currents.data[i], msg->data[i]);      // Fixing dt and gains for every motors
        else
            continue;
        //ROS_INFO("PID Control Law for Motor %d | turn = %f", i+1, turn_commands.data[i]);
    }

    // Publish only when a new PID command is computed 
    publish_turns();
}

void Control_Node::publish_turns()
{
    pub_turns.publish(turn_commands);
}

void Control_Node::main_loop(const ros::TimerEvent& event)
{
    // Publish /cmd_turns
    //publish_turns();
    // Maybe it's better to publish only when a new PID command is computed
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
}