/**************************************************
 * Useful Functions to use Dynamixel in your Robot*
 **************************************************/
// Dynamixel SDK
#include "dynamixel_sdk/dynamixel_sdk.h"
// ROS for C++
#include "ros/ros.h"
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
    // --- Private Attributes --- //
    // Communication Utils
    PortHandler *portHandler        = PortHandler::getPortHandler(DEVICE_NAME);
    PacketHandler *packetHandler    = PacketHandler::getPacketHandler(BAUDRATE);
    GroupSyncWrite motors_syncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, CURRENT_BYTE);
    // N° of motors
    int n_motors;

    // Methods
    public:
        // Constructor
        Dynamixel_Motors()
        {
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

            // Set Current to Zero in every motors
            int16_t rest_current[n_motors];
            if(!set_currents(rest_current))
                ROS_ERROR("Failed to set all the torques to zero.");
        }

        // --- Methods --- //
        bool set_currents(int16_t currents[])
        {
            // Assert: check that dim(currents) = n_motors
            //assert(sizeof(currents)/sizeof(int16_t) == n_motors);

            // Error Handling
            uint8_t dxl_error = 0;
            int dxl_comm_result = COMM_TX_FAIL;
            int dxl_addparam_result = false;

            // Double Array for cycle every motors
            uint8_t param_goal_currents[n_motors][CURRENT_BYTE];

            // Add parameters to sync_write obj
            int i, j = 0;
            for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
            {
                param_goal_currents[i][0] = DXL_LOBYTE(currents[i]);
                param_goal_currents[i][1] = DXL_HIBYTE(currents[i]);

                dxl_addparam_result = motors_syncWrite.addParam((uint8_t) i + 1, param_goal_currents[i]); // da sistemare
                if (dxl_addparam_result != true)
                {
                    ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
                    break;
                }
            }

            // Send all data
            dxl_comm_result = motors_syncWrite.txPacket();
            if (dxl_comm_result == COMM_SUCCESS) 
            {
                for(i; i < n_motors; i++)
                {
                   ROS_INFO("setCurrent : [ID:%d] [CURRENT (register):%d]", i+1, currents[i]); 
                }
                
                // Clear Parameters
                motors_syncWrite.clearParam();
                return true;
            } 
            else 
            {
                ROS_ERROR("Failed to set current! Result: %d", dxl_comm_result);
                
                // Clear Parameters
                motors_syncWrite.clearParam();
                return false;
            }
        }
};