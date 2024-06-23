/**
 * @file   ModeBase
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 11:31:01
 */

#ifndef _ModeBase_HPP
#define _ModeBase_HPP

#include <string>
#include <vector>
#include "Device.hpp"

class ControlModeBase {
public:
    virtual ~ControlModeBase();

    virtual void init();

    // activate operation mode
    virtual void activate() = 0;

    // read something required for operation mode
    virtual void read() = 0;

    // write commands of operation mode
    virtual void write(const double position, const double velocity, const double current) = 0;

protected:
    NodeHandle m_epos_handle;
};

#endif // _ModeBase_HPP
