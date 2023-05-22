/**************************************************
 * Useful Functions to use Dynamixel in your Robot*
 **************************************************/
// Dynamixel SDK
#include "dynamixel_sdk/dynamixel_sdk.h"
// ROS for C++
#include "ros.h"
// msgs
#include "std_msgs/Float64MultiArray.h"

// Dynamixel Namespace
using namespace dynamixel;

/* DEFINE*/
// Address
#define ADDR_TORQUE_ENABLE      64
#define ADDR_GOAL_CURRENT       102
#define ADDR_LED                65

// Value
#define LED_ON                  1
#define LED_OFF                 0
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0

// Data Length
#define CURRENT_BYTE            2

#define PROTOCOL_VERSION        2.0             // Default Protocol version of DYNAMIXEL X series.

#define BAUDRATE                57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME             "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

#define MAX_CURRENT             3.209           //[A] | Max current permitted in the motors.
#define MAX_VELOCITY            24.5324         //[rad/s]

#define MAX_CURRENT_REGISTER    1193            //uint16_t | Max value in the current Register. Corrispond to MAX_CURRENT.

// Dynamixel Class
class Dynamixel_Motors
{
    // Private Attributes
    PortHandler *portHandler;
    PacketHandler *packetHandler;
    GroupSyncWrite motors_syncWrite;
    int n_motors;

    // Methods
    public:
        Dynamixel_Motors()
        {
            // Init Port&Packet Handler
            portHandler = PortHandler::getPortHandler(DEVICE_NAME);
            packetHandler = PacketHandler::getPacketHandler(BAUDRATE);

            // Init GroupSync Object
            motors_syncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, CURRENT_BYTE);

            // Open Communication
            uint8_t dxl_error = 0;
            int dxl_comm_result = COMM_TX_FAIL;

            if (!portHandler->openPort()) 
                ROS_ERROR("Failed to open the port!");

            if (!portHandler->setBaudRate(BAUDRATE))
                ROS_ERROR("Failed to set the baudrate!");
            
            // Turn On LED and Enable Torque
            int i = 0;
            for(i; i < n_motors; i++) // Supposing that Motors idx are from 1 to n_motors
            {
                // LED
                dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_LED, LED_ON, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS) 
                {
                    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i+1);
                    break;
                }

                // Enable Torque
                dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS) 
                {
                    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i+1);
                    break;
                }
            }
        }

}