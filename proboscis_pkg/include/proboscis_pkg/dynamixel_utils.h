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
// Addresses
#define ADDR_TORQUE_ENABLE      64
#define ADDR_LED                65
#define ADDR_OP_MODE            11
#define ADDR_GOAL_POSITION      116
#define ADDR_GOAL_VELOCITY      104
#define ADDR_GOAL_CURRENT       102   

// Value
#define LED_ON                  1
#define LED_OFF                 0
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define CURRENT_MODE            0
#define VELOCITY_MODE           1
#define EXTENDED_POSITION_MODE  5

// Data Length
#define POSITION_BYTE           4
#define VELOCITY_BYTE           4
#define CURRENT_BYTE            2

#define PROTOCOL_VERSION        2.0             // Default Protocol version of DYNAMIXEL X series.

// Hardware Parameters
#define BAUDRATE                115200
#define DEVICE_NAME             "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

// Limit
#define MAX_CURRENT             3.209           // [A] | Max current permitted in the motors.
#define MAX_VELOCITY            24.5324         // [rad/s]

#define MAX_CURRENT_REGISTER    1193            // uint16_t | Max value in the current Register. Corresponds to MAX_CURRENT.

// (Abstract) Dynamixel Class
template <typename T>
class Dynamixel_Motors
{
    // --- Private Attributes --- //
    // Communication Utils    
    PortHandler *portHandler        = PortHandler::getPortHandler(DEVICE_NAME);
    PacketHandler *packetHandler    = PacketHandler::getPacketHandler(BAUDRATE);
    // N° of motors
    int n_motors;

    // Methods
    public:
        // --- Constructor --- //
        Dynamixel_Motors(int n_dyna);
        
        // --- Deconstructor --- //
        ~Dynamixel_Motors();

        // --- Methods --- //
        virtual bool set2registers(T registers[]);
        virtual bool set2registers(std::vector<T> registers);

        // Power Off Functions
        virtual bool set2Zeros();
        void powerOFF();                // Turn Off every Dynamixels
};

#endif /* DYNAMIXEL_UTILS_H_ */