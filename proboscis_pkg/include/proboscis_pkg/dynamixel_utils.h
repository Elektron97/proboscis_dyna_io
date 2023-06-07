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

// Mapping Torque - Current
// current(torque) = coeff_2*torque^2 + coeff_1*torque + coeff_0
#define COEFF_0 0.1327
#define COEFF_1 0.5753
#define COEFF_2 0.2030

// Functions
int16_t current2Register(float current_value);
float   register2Current(int16_t register_value);
bool    register_saturation(int16_t &register_value);
float   torque2Current(float current);
int16_t torque2Register(float torque);
float   sign(float x);

// (Abstract) Dynamixel Class
template <typename T>
class Dynamixel_Motors
{
    // --- Protected Attributes --- //
    protected:
        // Communication Utils    
        PortHandler *portHandler        = PortHandler::getPortHandler(DEVICE_NAME);
        PacketHandler *packetHandler    = PacketHandler::getPacketHandler(BAUDRATE);
        
        // N° of motors
        int n_motors = 0;

        // Error Handling
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

    // Methods
    public:
        // --- Constructor --- //
        Dynamixel_Motors();
        
        // --- Deconstructor --- //
        ~Dynamixel_Motors();

        // --- Methods --- //
        virtual bool set2registers(T registers[]);
        virtual bool set2registers(std::vector<T> registers);

        // Power Off Functions
        virtual bool set2Zeros();
        void powerOFF();                // Turn Off every Dynamixels
};

// Current Dynamixel
class Current_Dynamixel: public Dynamixel_Motors<int16_t>
{
    // Init with CURRENT address
    GroupSyncWrite motors_syncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, CURRENT_BYTE);

    public:
        // --- Constructor --- //
        Current_Dynamixel(int n_dyna);

        // --- Methods --- //
        // Low Level Set: Register
        bool set2registers(int16_t registers[]);
        bool set2registers(std::vector<int16_t> registers);         // Overwrite (vector<T>)
        
        // Mid Level Set: Current
        bool set_currents(float currents[]);
        bool set_currents(std::vector<float> currents);             // Overwrite (vector<T>)

        // High Level Set: Torque   
        bool set_torques(float torques[]);
        bool set_torques(std::vector<float> torques);               // Overwrite (vector<T>)

        // Power Off Functions
        bool set2Zeros();
};

#endif /* DYNAMIXEL_UTILS_H_ */