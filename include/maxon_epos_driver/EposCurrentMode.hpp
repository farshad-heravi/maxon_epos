/**
 * @file   EposCurrentMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 17:08:10
 */

#ifndef _EposCurrentMode_HPP
#define _EposCurrentMode_HPP

#include "ControlModeBase.hpp"
#include "Device.hpp"


class EposCurrentMode : public ControlModeBase {
public:
    virtual ~EposCurrentMode();

    virtual void init();
    virtual void activate();
    virtual void read();
    virtual void write(const double position, const double velocity, const double current);

private:

};

#endif // _EposCurrentMode_HPP
