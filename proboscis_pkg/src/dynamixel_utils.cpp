/****************************************
 * Source code for Dynamixel Library    *
 ****************************************/
// Include
#include "proboscis_pkg/dynamixel_utils.h"

// --- Dynamixel Class --- //
// Constructor 
template <typename T> Dynamixel_Motors<T>::Dynamixel_Motors(int n_dyna)
{
    // Init n_motors
    n_motors = n_dyna;

    // Open Communication
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    if(!portHandler->openPort()) 
        ROS_ERROR("Failed to open the port!");

    if(!portHandler->setBaudRate(BAUDRATE))
        ROS_ERROR("Failed to set the baudrate!");
}

// Deconstructor
template <typename T> Dynamixel_Motors<T>::~Dynamixel_Motors()
{
    ROS_WARN("Terminating Dynamixel object...");
    
    // Set Current to Zero in every motors
    if(!set2Zeros())
        ROS_ERROR("Failed to set all the torques to zero.");
    
    // Turn Off every motors
    powerOFF();
    
    ROS_WARN("Dynamixel Object terminated.");
}

template <typename T> void Dynamixel_Motors<T>::powerOFF()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // Turn Off LED and Disable Torque
    int i = 0;
    for(i; i < n_motors; i++) // Supposing that Motors idx are from 1 to n_motors
    {
        // LED
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_LED, LED_OFF, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to turn off LED for Dynamixel ID %d", i+1);
            //break;
        }

        // Disable Torque
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to disable torque for Dynamixel ID %d", i+1);
            //break;
        }
    }
}