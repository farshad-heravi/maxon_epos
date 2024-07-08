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
#include <vector>
#include <thread>
#include "Definitions.h"
#include "EposMotor.hpp"
#include "Macros.hpp"
#include "EposException.hpp"
#include "EposProfilePositionMode.hpp"
#include "EposProfileVelocityMode.hpp"
#include "EposCurrentMode.hpp"

#include <unistd.h>

// #include "maxon_epos_msgs/MotorState.h"

/**
 * @brief Constructor
 */
EposMotor::EposMotor(std::string motor_name, std::string EposModel, std::string protocol_stack, std::string interface, std::string port,
                int baudrate, int timeout,
                int encoder_type, int encoder_resolution, int gear_ratio, int encoder_inverted_polarity, std::string control_mode) : _motor_name(motor_name),
        _EposModel(EposModel), _protocol_stack(protocol_stack), _interface(interface), _port(port),
        _encoder_type(encoder_type), _encoder_resolution(encoder_resolution), _gear_ratio(gear_ratio), _encoder_inverted_polarity(encoder_inverted_polarity),
        _baudrate(baudrate), _timeout (timeout),
        _control_mode (control_mode), _target_pos(0)
{
    // m_position = 0; //, m_velocity(0), m_effort(0), m_current(0)

    _position_mode_velocity = 6250000; // ,,,
    _position_mode_acceleration = 10000; // ,,,
    _position_mode_deceleration = 10000; // ,,,
    _update_interval = 30;
}

/**
 * @brief Destructor
 *        change the device state to "disable"
 */
EposMotor::~EposMotor()
{
    try {
        VCS_NODE_COMMAND_NO_ARGS(SetDisableState, m_epos_handle);
        _m_readLoop = false;
        if (m_pReadThread != nullptr)
        {
            m_pReadThread->join();
            delete m_pReadThread;
            m_pReadThread = nullptr;
        }
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

    _m_readLoop = true;
    m_pReadThread = new std::thread([this] { this->ReadThread(this); });    // start readloop
    sleep(1);
    _target_pos = m_position;
    enableMotor();      // enable the motor
    std::vector<int> readings = m_control_mode->read();     // read current position and set it to _home_qc
}

// //! \brief Entry point for the write thread.
// //! \param pModule A pointer to the Module instance associated with the write thread.
void EposMotor::ReadThread(EposMotor * pModule)
{
    pModule->ReadLoop();
}

void EposMotor::ReadLoop()
{
    std::cout << "Epos Read thread started." << std::endl;
    while (_m_readLoop)
    {
        std::chrono::high_resolution_clock::time_point last=std::chrono::high_resolution_clock::now();
        
        int positionls = 0, velocityls = 0;
        short currentls = 0;
        try{
            VCS_NODE_COMMAND(GetPositionIs, m_epos_handle, &positionls);
            VCS_NODE_COMMAND(GetVelocityIsAveraged, m_epos_handle, &velocityls);
            VCS_NODE_COMMAND(GetCurrentIsAveraged, m_epos_handle, &currentls);
        } catch (const EposException &e) {
            std::cout << "Epos GetPositionIs error!" << std::endl;
            std::cout << e.what() << std::endl;
        }
        m_position = positionls;
        m_velocity = velocityls;
        m_effort = currentls;

        std::chrono::high_resolution_clock::time_point current=std::chrono::high_resolution_clock::now();
        int64_t ms=(std::chrono::duration_cast<std::chrono::milliseconds>( current - last )).count();
        if(ms < _update_interval)
            std::this_thread::sleep_for(std::chrono::milliseconds( _update_interval - ms ));
    }
    std::cout << "EPOS Write thread terminating." << std::endl;
}

std::vector<int> EposMotor::read()
{
    std::vector<int> output = {m_position, m_velocity, m_current};
    return output;
    // previous implementation :
    // // try {
    // //     if (m_control_mode) {
    // //         m_control_mode->read();
    // //     }
    // //     m_position = ReadPosition();
    // //     // std::cout << m_position << std::endl;
    // //     m_velocity = ReadVelocity();
    // //     m_current = ReadCurrent();
    // // } catch (const EposException &e) {
    // //     std::cout << "ERROR: " << e.what() << std::endl;
    // // }
    // std::vector<double> output = {m_position, m_velocity, m_current};
    // return output;
}

/*
@param position: target position in ticks
*/
void EposMotor::write(const int position, const int velocity, const int current)
{
    _target_pos = position;

    // previous implementation (not thread)
    // try {
    //     if (m_control_mode) {
    //         m_control_mode->write(position, velocity, current);
    //     }
    // } catch (const EposException &e) {
    //     std::cout << e.what() << std::endl;
    // }
}

void EposMotor::WriteThread(EposMotor * pModule)
{
    pModule->WriteLoop();
}

//! \brief Starts the thread that is spinning in the write loop that sends periodic SetJoint commands to the module over the CAN bus.
void EposMotor::Start()
{
    m_bIsRunning = true;
    m_pReadThread = new std::thread([this] { this->WriteThread(this); });
}

//! \brief Terminates and joins the thread that is spinning in the write loop that sends periodic SetJoint commands to the module over the CAN bus.
void EposMotor::Stop()
{
    m_bIsRunning = false;
    if (m_pWriteThread != nullptr)
    {
        m_pWriteThread->join();
        delete m_pWriteThread;
        m_pWriteThread = nullptr;
    }
}

//! \brief The write loop that will asynchronuously send setposition messages to the module over the CAN bus.
void EposMotor::WriteLoop()
{
    std::cout << "Epos Write thread started." << std::endl;
    while (m_bIsRunning)
    {
        std::chrono::high_resolution_clock::time_point last=std::chrono::high_resolution_clock::now();
        if (_target_pos != m_position && std::abs( _target_pos - m_position) > 4)  // 4 is error tolerance in ticks
        {
            double second = 0.001 * _update_interval;
            int max_increment = (int)(second * _position_mode_velocity);
            int increment = max_increment;
            if (_target_pos < m_position)
                increment = -increment;
            if ( std::abs(_target_pos - m_position) < max_increment )
                increment = _target_pos - m_position;
            try {
                int new_pos = m_position + increment, temp=0.0;
                if (m_control_mode){
                    m_control_mode->write(new_pos, temp, temp); // TODO now, only for position control
                } else
                    std::cout << "EposMotor's control mode is not available." << std::endl;
            } catch (const EposException &e) {
                int error = _target_pos - m_position;
                std::cout << "Motor ticks: " << error << " [ticks]" << std::endl;
                std::cout << e.what() << std::endl;
            }
        }
        std::chrono::high_resolution_clock::time_point current=std::chrono::high_resolution_clock::now();
        int64_t ms=(std::chrono::duration_cast<std::chrono::milliseconds>( current - last )).count();
        if(ms < _update_interval)
            std::this_thread::sleep_for(std::chrono::milliseconds( _update_interval - ms ));
    }
    std::cout << "EPOS Write thread terminating." << std::endl;
}


/**
 * @brief Initialize Epos Node Handle
 */
void EposMotor::initEposDeviceHandle()
{
    const DeviceInfo device_info(_EposModel, _protocol_stack, _interface, _port);
    const unsigned short node_id(1);

    // // create epos handle
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
    if (control_mode == "profile_position"){
        m_control_mode.reset(new EposProfilePositionMode());
    } else if (control_mode == "profile_velocity") {
        m_control_mode.reset(new EposProfileVelocityMode());
    } else if (control_mode == "current") {
        m_control_mode.reset(new EposCurrentMode());
    } else {
            throw EposException("Unsupported control mode (" + control_mode + ")");
    }
    m_control_mode->init(m_epos_handle);
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
        VCS_NODE_COMMAND(SetVelocityUnits, m_epos_handle, 0xA4, -3); // 0 for STANDARD; -1 VN_DECI; -2 VN_CENTI; -3 VN_MILLI

        // m_max_qc = 4 * gear_ratio * resolution

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