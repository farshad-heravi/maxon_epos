/**
 * @file   EposManager
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:08:23
 */

#ifndef _EposManager_HPP
#define _EposManager_HPP

#include <string.h>
#include <vector>
#include "EposMotor.hpp"

class EposManager {
    public:
        EposManager() = default;
        EposManager(std::string EposModel, std::string motor_name, std::string protocol_stack, std::string interface, std::string port, int baudrate, int timeout, int encoder_type, int encoder_resolution, int gear_ratio, bool encoder_inverted_polarity, std::string control_mode);
        virtual ~EposManager();

        bool init();
        std::vector<double> read();
        void write(const std::vector<double> command);
        bool set_velocity(int velocity);                // set constant velocity for profile_position

    private:
        
        std::shared_ptr<EposMotor> _motor;      //   object of EposMotor for the controlled motor
        std::string _motor_name;        // a name for the controlled motor
        std::string _EposModel;         // EPOS4
        std::string _protocol_stack;    // MAXON SERIAL V2
        std::string _interface;         // USB
        std::string _port;              // USB0
        int _baudrate;          // 0
        int _timeout;           // 0
        std::string _control_mode;      // profile_position
        int _encoder_type;      // ,,,
        int _encoder_resolution; // ,,,
        int _gear_ratio; // ,,,
        bool _encoder_inverted_polarity;     // ,,,
};

#endif // _EposManager_HPP
