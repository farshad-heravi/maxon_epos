/**
 * @file   EposProfileVelocityMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 17:07:38
 */

#ifndef _EposProfileVelocityMode_HPP
#define _EposProfileVelocityMode_HPP

#include "ControlModeBase.hpp"
#include "Device.hpp"


class EposProfileVelocityMode : public ControlModeBase {
public:
    virtual ~EposProfileVelocityMode();

    virtual void init();
    virtual void activate();
    virtual void read();
    virtual void write(const double position, const double velocity, const double current);

private:

};

#endif // _EposProfileVelocityMode_HPP
