/**
 * @file   EposMotor
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:15:22
 * 
 * @author modified by Farshad Nozad Heravi (f.n.heravi@gmail.com)
 * @date    June 2024
 */

#include <iostream>
#include "Definitions.h"
#include "EposMotor.hpp"
#include "Macros.hpp"
#include "EposException.hpp"
#include "EposProfilePositionMode.hpp"
#include "EposProfileVelocityMode.hpp"
#include "EposCurrentMode.hpp"

// #include "maxon_epos_msgs/MotorState.h"

/**
 * @brief Constructor
 */
EposMotor::EposMotor(std::string motor_name, std::string EposModel, std::string protocol_stack, std::string interface, std::string port,
                int baudrate, int timeout,
                int encoder_type, int encoder_resolution, int gear_ratio, int encoder_inverted_polarity, std::string control_mode) : _motor_name(motor_name),
        m_position(0), m_velocity(0), m_effort(0), m_current(0),
        _EposModel(EposModel), _protocol_stack(protocol_stack), _interface(interface), _port(port),
        _encoder_type(encoder_type), _encoder_resolution(encoder_resolution), _gear_ratio(gear_ratio), _encoder_inverted_polarity(encoder_inverted_polarity),
        _baudrate(baudrate), _timeout (timeout),
        _control_mode (control_mode)
{
    _position_mode_velocity = 10; // ,,,
    _position_mode_acceleration = 100; // ,,,
    _position_mode_deceleration = 100; // ,,,
}

/**
 * @brief Destructor
 *        change the device state to "disable"
 */
EposMotor::~EposMotor()
{
    try {
        VCS_NODE_COMMAND_NO_ARGS(SetDisableState, m_epos_handle);
    } catch (const EposException &e) {
        std::cout << e.what() << std::endl;
    }
}

void EposMotor::init()
{
    initEposDeviceHandle();

    VCS_NODE_COMMAND_NO_ARGS(SetDisableState, m_epos_handle);

    initDeviceError();
    initProtocolStackChanges();
    initControlMode(_control_mode);
    initEncoderParams();
    initProfilePosition();
    enableMotor();
}

std::vector<double> EposMotor::read()
{
    try {
        if (m_control_mode) {
            m_control_mode->read();
        }
        m_position = ReadPosition();
        m_velocity = ReadVelocity();
        m_current = ReadCurrent();
    } catch (const EposException &e) {
        std::cout << "ERROR: " << e.what() << std::endl;
    }
    
    std::vector<double> output = {m_position, m_velocity, m_current};
    return output;
}

void EposMotor::write(const double position, const double velocity, const double current)
{
    try {
        if (m_control_mode) {
            m_control_mode->write(position, velocity, current);
        }
    } catch (const EposException &e) {
        std::cout << e.what() << std::endl;
    }
}


/**
 * @brief Initialize Epos Node Handle
 */
void EposMotor::initEposDeviceHandle()
{
    const DeviceInfo device_info(_EposModel, _protocol_stack, _interface, _port);
    const unsigned short node_id(0);

    // create epos handle
    m_epos_handle = HandleManager::CreateEposHandle(device_info, node_id);
}

void EposMotor::enableMotor()
{
    VCS_NODE_COMMAND_NO_ARGS(SetEnableState, m_epos_handle);
}

/**
 * @brief Clear initial errors
 */
void EposMotor::initDeviceError()
{
    unsigned char num_of_device_errors;
    // Get Current Error nums
    VCS_NODE_COMMAND(GetNbOfDeviceError, m_epos_handle, &num_of_device_errors);
    for (int i = 1; i <= num_of_device_errors; i++) {
        unsigned int device_error_code;
        VCS_NODE_COMMAND(GetDeviceErrorCode, m_epos_handle, i, &device_error_code);
        std::cout << "EPOS Device Error: 0x" << std::hex << device_error_code << std::endl;
    }

    // Clear Errors
    VCS_NODE_COMMAND_NO_ARGS(ClearFault, m_epos_handle);
    // Get Current Error nums again
    VCS_NODE_COMMAND(GetNbOfDeviceError, m_epos_handle, &num_of_device_errors);
    if (num_of_device_errors > 0) {
        throw EposException(_motor_name + ": " + std::to_string(num_of_device_errors) + " faults uncleared");
    }
}

/**
 * @brief Set Protocol Stack for Epos Node Handle
 *
 * @param motor_nh ros NodeHandle of motor
 */
void EposMotor::initProtocolStackChanges()
{
    // load values from ros parameter server
    const unsigned int baudrate( _baudrate );
    const unsigned int timeout( _timeout );
    if (_baudrate == 0 && timeout == 0) 
        return;

    if (baudrate > 0 && timeout > 0) 
        VCS_COMMAND(SetProtocolStackSettings, m_epos_handle.ptr.get(), baudrate, timeout);
    else {
        unsigned int current_baudrate, current_timeout;
        VCS_COMMAND(GetProtocolStackSettings, m_epos_handle.ptr.get(), &current_baudrate, &current_timeout);
        VCS_COMMAND(SetProtocolStackSettings, m_epos_handle.ptr.get(),
                baudrate > 0 ? baudrate : current_baudrate,
                timeout > 0 ? timeout : current_timeout);
    }
}

/**
 * @brief Initialize Control Mode
 *
 * @param root_nh NodeHandle of TopLevel
 * @param motor_nh NodeHandle of motor
 */
void EposMotor::initControlMode(std::string control_mode)
{
    if (control_mode == "profile_position")
        m_control_mode.reset(new EposProfilePositionMode());
    else if (control_mode == "profile_velocity")
        m_control_mode.reset(new EposProfileVelocityMode());
    else if (control_mode == "current") 
        m_control_mode.reset(new EposCurrentMode());
    else
        throw EposException("Unsupported control mode (" + control_mode + ")");
    m_control_mode->init();
}

/**
 * @brief Initialize EPOS sensor(Encoder) parameters
 *
 * @param motor_nh NodeHandle of motor
 */
void EposMotor::initEncoderParams()
{   
    const int type( _encoder_type );
    VCS_NODE_COMMAND(SetSensorType, m_epos_handle, type);

    if (type == 1 || type == 2) {
        // Incremental Encoder
        const int resolution(_encoder_resolution);
        const int gear_ratio(_gear_ratio);
        if (resolution == 0 || gear_ratio == 0)
            throw EposException("Please set parameter 'resolution' and 'gear_ratio'");
        const bool inverted_polarity(_encoder_inverted_polarity);
        std::cout << "Epos motor ( " << _motor_name << ") has set." << std::endl;
        if (inverted_polarity) {
            std::cout << "Epos motor ( " << _motor_name << ") : Inverted polarity is True" << std::endl;
        }
        
        VCS_NODE_COMMAND(SetIncEncoderParameter, m_epos_handle, resolution, inverted_polarity);

        m_max_qc = 4 * resolution * gear_ratio;

    // } else if (type == 4 || type == 5) {
    //     // SSI Abs Encoder
    //     bool inverted_polarity;
    //     int data_rate, number_of_multiturn_bits, number_of_singleturn_bits;
    //     if (encoder_nh.hasParam("data_rate") && encoder_nh.hasParam("number_of_singleturn_bits") && encoder_nh.hasParam("number_of_multiturn_bits")) {
    //         encoder_nh.getParam("data_rate", data_rate);
    //         encoder_nh.getParam("number_of_multiturn_bits", number_of_multiturn_bits);
    //         encoder_nh.getParam("number_of_singleturn_bits", number_of_singleturn_bits);
    //     } else {
    //         std::cout << "Please set 'data_rate', 'number_of_singleturn_bits', and 'number_of_multiturn_bits'" << std::endl;
    //     }
    //     VCS_NODE_COMMAND(SetSsiAbsEncoderParameter, m_epos_handle, data_rate, number_of_multiturn_bits, number_of_singleturn_bits, inverted_polarity);
    //     m_max_qc = inverted_polarity ? -(1 << number_of_singleturn_bits) : (1 << number_of_singleturn_bits);
    } else {
        // Invalid Encoder
        throw EposException("Invalid Encoder Type: " + std::to_string(type));
    }
}

void EposMotor::initProfilePosition()
{
    VCS_NODE_COMMAND(SetPositionProfile, m_epos_handle, _position_mode_velocity, _position_mode_acceleration, _position_mode_deceleration);
}

/**
 * @brief Read Motor Position Function
 *
 * @return motor position in ticks
 */
double EposMotor::ReadPosition()
{
    int position;
    VCS_NODE_COMMAND(GetPositionIs, m_epos_handle, &position);
    return position;
}

/**
 * @brief Read Motor Velocity Function
 *
 * @return motor velocity in tick/s
 */
double EposMotor::ReadVelocity()
{
    int velocity;
    VCS_NODE_COMMAND(GetVelocityIs, m_epos_handle, &velocity);
    return velocity;
}

/**
 * @brief Read Motor Current Funciton
 *
 * @return motor current
 */
double EposMotor::ReadCurrent()
{
    short current;
    VCS_NODE_COMMAND(GetCurrentIs, m_epos_handle, &current);
    return current;
}


bool EposMotor::set_velocity(int velocity)
{
    _position_mode_velocity = velocity;
    VCS_NODE_COMMAND(SetPositionProfile, m_epos_handle, _position_mode_velocity, _position_mode_acceleration, _position_mode_deceleration);
    return true;
}