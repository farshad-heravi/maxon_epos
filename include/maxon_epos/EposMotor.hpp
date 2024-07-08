/**
 * @file   EposMotor
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-03 16:12:59
 */

#ifndef _EposMotor_HPP
#define _EposMotor_HPP

#include <vector>
#include <thread>
#include <string>
#include <atomic>
#include "Device.hpp"
#include "ControlModeBase.hpp"

class EposMotor {
    public:
        EposMotor();
        EposMotor(std::string motor_name, std::string EposModel, std::string protocol_stack, std::string interface, std::string port,
                int baudrate, int timeout,
                int encoder_type, int encoder_resolution, int gear_ratio, int encoder_inverted_polarity, std::string control_mode);
        virtual ~EposMotor();
        bool set_velocity(int velocity);

        void init();
        void Start();
        void Stop();

        std::vector<int> read();
        void write(const int position, const int velocity, const int current);


    private:
        void initEposDeviceHandle();
        void initDeviceError();
        void initProtocolStackChanges();
        void initControlMode(std::string control_mode);
        void initEncoderParams();
        void initProfilePosition();
        void enableMotor();
        void ReadThread(EposMotor * pModule);
        void ReadLoop();
        void WriteThread(EposMotor * pModule);
        void WriteLoop();


        double ReadPosition();
        double ReadVelocity();
        double ReadCurrent();

        typedef std::shared_ptr<ControlModeBase> ControlModePtr;
        typedef std::map<std::string, ControlModePtr> ControlModeMap;

        NodeHandle m_epos_handle;
        ControlModePtr m_control_mode;
        ControlModeMap m_control_mode_map;

        int m_position;
        int m_velocity;
        int m_effort;
        int m_current;

        int _target_pos;                // target position in ticks
        bool _m_readLoop;
        int _update_interval;  // in ms

        //! Flag indicating if the write loop should keep on spinning.
        std::atomic_bool m_bIsRunning;
        std::atomic_bool m_readLoop;

        //! Pointer to the write thread. Will be nullptr if the module is stopped.
        std::thread* m_pWriteThread;
        std::thread* m_pReadThread;


        std::string _motor_name;
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
        int _position_mode_velocity, _position_mode_acceleration, _position_mode_deceleration;  // ,,,
};

#endif // _EposMotor_HPP
