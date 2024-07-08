/**
 * @file   EposManager
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:18:31
 * 
 * @author modified by Farshad Nozad Heravi (f.n.heravi@gmail.com)
 * @date    June 2024
 */

#include "EposManager.hpp"
#include <iostream>
#include <unistd.h>

/**
 * @brief Constructor
 */
EposManager::EposManager(std::string EposModel, std::string motor_name, std::string protocol_stack, std::string interface, std::string port, int baudrate, int timeout, int encoder_type, int encoder_resolution, int gear_ratio, bool encoder_inverted_polarity, std::string control_mode)
    : _EposModel(EposModel), _protocol_stack(protocol_stack), _interface(interface), _port(port), _baudrate(baudrate), _timeout(timeout), _encoder_type(encoder_type), _encoder_resolution(encoder_resolution),
    _gear_ratio(gear_ratio), _encoder_inverted_polarity(encoder_inverted_polarity), _control_mode(control_mode),
    _motor_name(motor_name)
{}

/**
 * @brief Destructor
 */
EposManager::~EposManager() = default;


bool EposManager::init()
{
    _motor = std::shared_ptr<EposMotor>(
        new EposMotor( _motor_name, _EposModel, _protocol_stack, _interface, _port, _baudrate, _timeout, _encoder_type, _encoder_resolution, _gear_ratio, _encoder_inverted_polarity, _control_mode)
    );
    _motor->init();
    return true;
}

void EposManager::start()
{
    _motor->Start();
}

void EposManager::stop()
{
    _motor->Stop();
}

std::vector<int> EposManager::read()
{
    std::vector<int> output =_motor->read();
    return output;
}

/*
* @param command get a three-element vector as [posistion, velocity, current]
* gets the value corresponds to the active controller and ignores the other ones
*/
void EposManager::write(const std::vector<int> command)
{
    _motor->write(command[0], command[1], command[2]);
}

bool EposManager::set_velocity(int velocity)
{
    _motor->set_velocity(velocity);
    return true;
}