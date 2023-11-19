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
//[OLD Safe Copy]
/*void joy2Motors(sensor_msgs::Joy joystick_input, std_msgs::Float32MultiArray& motor_cmd, float max_value)
{
    // Simple boolean mapping (only for testing)
    for(int i = 0; i < N_MOTORS; i++)
    {   
        if(joystick_input.buttons[i] == 1)
            motor_cmd.data[i] = max_value;
        else
            motor_cmd.data[i] = 0.0;
    }

    // Clear array
    int i = 0;
    for(i; i < N_MOTORS; i++)
    {
        motor_cmd.data[i] = 0.0;
    }

    // --- Extract Information from LEFT stick --- //
    float rho, theta;
    cartesian2Polar(-joystick_input.axes[0], joystick_input.axes[1], rho, theta);
    
    // Including Optimization Tool? Not Working Yet!:(((
    // Mapping Bending
    // Longitudinal 90° and Longitudinal 30°
    if((theta >= PI/6.0) && (theta <= PI/2.0))
    {
        // 30°
        motor_cmd.data[2] = (1.1547*rho*cos(theta))*(MAX_OUTPUT/SQRT_2);
        // 90°
        motor_cmd.data[0] = (rho*sin(theta) - 0.5774*rho*cos(theta))*(MAX_OUTPUT/SQRT_2);
    }
    else if((theta > PI/2.0) && (theta <= PI/2.0 + PI/3.0))
    {
        // 90°
        motor_cmd.data[0] = (rho*sin(theta) + 0.5774*rho*cos(theta))*(MAX_OUTPUT/SQRT_2);
        // 150°
        motor_cmd.data[1] = (- 1.1547*rho*cos(theta))*(MAX_OUTPUT/SQRT_2);
    }
    else
    {
        ROS_ERROR("Currently not implemented");
    }

    return;
}*/

Eigen::VectorXd build_desXi(float rho, float theta, float rt, float lt)
{
    Eigen::VectorXd des_xi(6);
    des_xi << rho*cos(theta), rho*sin(theta), rt, 0.0, 0.0, 1.0 - lt;
    return des_xi;
}

Eigen::VectorXd solve(Eigen::VectorXd _xi_des)
{
    // 0. Update Desired Strain Modes
    xi_des = _xi_des;

    // 1. define the problem
    Problem nlp;
    nlp.AddVariableSet  (std::make_shared<ExVariables>());
    nlp.AddConstraintSet(std::make_shared<ExConstraint>());
    nlp.AddCostSet      (std::make_shared<ExCost>());

    // 2. choose solver and options
    IpoptSolver ipopt;
    ipopt.SetOption("linear_solver", "mumps");
    ipopt.SetOption("jacobian_approximation", "finite-difference-values"); // approximate jacobian numerically (not exact)
    // 3 . solve
    ipopt.Solve(nlp);
    return nlp.GetOptVariables()->GetValues();
}

void joy2Motors(sensor_msgs::Joy joystick_input, std_msgs::Float32MultiArray& motor_cmd, float max_value)
{
    // Clear array
    int i = 0;
    for(i; i < N_MOTORS; i++)
    {
        motor_cmd.data[i] = 0.0;
    }

    // --- Extract Information from LEFT stick --- //
    float rho, theta;
    cartesian2Polar(-joystick_input.axes[0], joystick_input.axes[1], rho, theta);

    // --- Compose Desired Strain & Solve Optimal Problem --- //
    Eigen::VectorXd solution = solve(build_desXi(rho, theta, joystick_input.axes[4], joystick_input.axes[5]));
    Eigen::VectorXd tau = solution.tail(N_MOTORS); // Extract

    // --- Update motor_cmd: Mapping from dynamixel indeces and notation --- //
    // Longitudinal
    motor_cmd.data[0] = tau[0];
    motor_cmd.data[1] = tau[2];
    motor_cmd.data[2] = tau[1];
    // Oblique
    motor_cmd.data[3] = tau[5];
    motor_cmd.data[4] = tau[3];
    motor_cmd.data[5] = tau[4];
    motor_cmd.data[6] = tau[6];
    return;
}