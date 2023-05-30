/****************************************
 * Source code for Dynamixel Library    *
 ****************************************/
// Include
#include "proboscis_pkg/dynamixel_utils.h"

// --- Dynamixel Class --- //
// Constructor 
Dynamixel_Motors::Dynamixel_Motors(int n_dyna)
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
    
    // Turn On LED, Current Mode and Enable Torque
    int i = 0;
    for(i; i < n_motors; i++) // Supposing that Motors idx are from 1 to n_motors
    {
        // LED
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_LED, LED_ON, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to turn on LED for Dynamixel ID %d", i+1);
            break;
        }

        // Current Drive Mode
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, i + 1, ADDR_OP_MODE, CURRENT_MODE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) 
        {
            ROS_ERROR("Failed to set Current Mode for Dynamixel ID %d", i+1);
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
    if(!set2Zeros())
        ROS_ERROR("Failed to set all the torques to zero.");
}

// Deconstructor
Dynamixel_Motors::~Dynamixel_Motors()
{
    ROS_WARN("Terminating Dynamixel object...");
    
    // Set Current to Zero in every motors
    if(!set2Zeros())
        ROS_ERROR("Failed to set all the torques to zero.");
    
    // Turn Off every motors
    powerOFF();
    
    ROS_WARN("Dynamixel Object terminated.");
}

bool Dynamixel_Motors::set_currentRegisters(int16_t registers[])
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
    int i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        // Security Saturation on register values
        if(!register_saturation(registers[i]))
            ROS_WARN("Commanded Current are out of limits. Saturating...");


        param_goal_currents[i][0] = DXL_LOBYTE(registers[i]);
        param_goal_currents[i][1] = DXL_HIBYTE(registers[i]);

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
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setCurrent : [ID:%d] [CURRENT (register):%d]", i+1, registers[i]); 
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

bool Dynamixel_Motors::set_currentRegisters(std::vector<int16_t> registers)
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
    int i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        param_goal_currents[i][0] = DXL_LOBYTE(registers[i]);
        param_goal_currents[i][1] = DXL_HIBYTE(registers[i]);

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
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setCurrent : [ID:%d] [CURRENT (register):%d]", i+1, registers[i]); 
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

bool Dynamixel_Motors::set_currents(float currents[])
{
    int16_t registers[n_motors];

    // Convert in Register Values
    int i = 0;
    for(i; i < n_motors; i++)
    {
        registers[i] = current2Register(currents[i]);
    }

    return set_currentRegisters(registers);
}

// Overloading
bool Dynamixel_Motors::set_currents(std::vector<float> currents)
{
    int16_t registers[n_motors];

    // Convert in Register Values
    int i = 0;
    for(i; i < n_motors; i++)
    {
        registers[i] = current2Register(currents[i]);
    }

    return set_currentRegisters(registers);
}

bool Dynamixel_Motors::set_torques(float torques[])
{
    int16_t registers[n_motors];

    // Convert in Register Values
    int i = 0;
    for(i; i < n_motors; i++)
    {
        registers[i] = torque2Register(torques[i]);
    }

    return set_currentRegisters(registers);
}

// Overloading
bool Dynamixel_Motors::set_torques(std::vector<float> torques)
{
    int16_t registers[n_motors];

    // Convert in Register Values
    int i = 0;
    for(i; i < n_motors; i++)
    {
        registers[i] = torque2Register(torques[i]);
    }

    return set_currentRegisters(registers);
}

bool Dynamixel_Motors::set2Zeros()
{
    // Error Handling
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_addparam_result = false;

    // Double Array for cycle every motors
    uint8_t param_goal_currents[n_motors][CURRENT_BYTE];

    // Add parameters to sync_write obj
    int i = 0;
    for(i; i < n_motors; i++) // supposing motors idx are from 1 to n_motors
    {
        param_goal_currents[i][0] = DXL_LOBYTE(0);
        param_goal_currents[i][1] = DXL_HIBYTE(0);

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
        for(i = 0; i < n_motors; i++)
        {
            ROS_INFO("setCurrent : [ID:%d] [CURRENT (register):%d]", i+1, 0); 
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

void Dynamixel_Motors::powerOFF()
{
    /************SECURITY POWER OFF**********
     * This function tries DISABLE torque   *
     * and turn off LEDs for every motor.   *
     ****************************************/

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

/**************************************************************************/
// --- Functions --- //
int16_t current2Register(float current_value)
{
    return MAX_CURRENT_REGISTER*((int16_t) (current_value/MAX_CURRENT));
}

float register2Current(int16_t register_value)
{
    return MAX_CURRENT*((float) (register_value/MAX_CURRENT_REGISTER));
}

bool register_saturation(int16_t &register_value)
{
    // No need of sign function, because is only positive values
    if(abs(register_value) > MAX_CURRENT_REGISTER)
    {
        register_value = ((int16_t) sign(register_value))*MAX_CURRENT_REGISTER;
        return false;
    }
    else
    {
        return true;
    }
}

float sign(float x)
{
    /*SIGN FUNCTION:*/
    if(x > 0)
        return 1.0;
    if(x < 0)
        return -1.0;
    else
        return 0.0;
}

float torque2Current(float torque)
{
    return COEFF_2*torque*torque + COEFF_1*torque + COEFF_0;
}

int16_t torque2Register(float torque)
{
    return current2Register(torque2Current(torque));
}