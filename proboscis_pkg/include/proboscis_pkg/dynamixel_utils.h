/**************************************************
 * Useful Functions to use Dynamixel in your Robot*
 **************************************************/

// Dynamixel XM430-W210-R.
// E-Manual: https://emanual.robotis.com/docs/en/dxl/x/xm430-w210

#ifndef DYNAMIXEL_UTILS_H_
#define DYNAMIXEL_UTILS_H_

// Dynamixel SDK
#include "dynamixel_sdk/dynamixel_sdk.h"
// ROS for C++
#include "ros/ros.h"

// Dynamixel Namespace
using namespace dynamixel;

/* DEFINE*/
// Address
#define ADDR_TORQUE_ENABLE      64
#define ADDR_GOAL_CURRENT       102
#define ADDR_LED                65
#define ADDR_OP_MODE            11    

// Value
#define LED_ON                  1
#define LED_OFF                 0
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define CURRENT_MODE            0
#define CURRENT_POSITION_MODE   5

// Data Length
#define CURRENT_BYTE            2

#define PROTOCOL_VERSION        2.0             // Default Protocol version of DYNAMIXEL X series.

// Hardware Parameters
#define BAUDRATE                115200
#define DEVICE_NAME             "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

// Limit
#define MAX_CURRENT             3.209           // [A] | Max current permitted in the motors.
#define MAX_VELOCITY            24.5324         // [rad/s]

#define MAX_CURRENT_REGISTER    1193            // uint16_t | Max value in the current Register. Corresponds to MAX_CURRENT.

// Mapping Torque - Current
// current(torque) = coeff_2*torque^2 + coeff_1*torque + coeff_0
#define COEFF_0 0.1327
#define COEFF_1 0.5753
#define COEFF_2 0.2030

// Functions
int16_t current2Register(float current_value);
float register2Current(int16_t register_value);
bool register_saturation(int16_t &register_value);
float torque2Current(float current);
int16_t torque2Register(float torque);
float sign(float x);

// Dynamixel Class
class Dynamixel_Motors
{
    // --- Private Attributes --- //
    // Communication Utils
    PortHandler *portHandler        = PortHandler::getPortHandler(DEVICE_NAME);
    PacketHandler *packetHandler    = PacketHandler::getPacketHandler(BAUDRATE);
    GroupSyncWrite motors_syncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, CURRENT_BYTE);
    // N° of motors
    int n_motors;

    // Methods
    public:
        // --- Constructor --- //
        Dynamixel_Motors(int n_dyna);
        
        // --- Deconstructor --- //
        ~Dynamixel_Motors();

        // --- Methods --- //
        // Low Level Set: Register
        bool set_currentRegisters(int16_t registers[]);
        bool set_currentRegisters(std::vector<int16_t> registers);  // Overwrite
        
        // Mid Level Set: Current
        bool set_currents(float currents[]);
        bool set_currents(std::vector<float> currents);             // Overwrite

        // High Level Set: Torque   
        bool set_torques(float torques[]);
        bool set_torques(std::vector<float> torques);               // Overwrite

        // Power Off Function
        void powerOFF();
};

#endif /* DYNAMIXEL_UTILS_H_ */