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

    virtual void init(NodeHandle &node_handle);
    virtual void activate();
    virtual void read();
    virtual void write(const int position, const int velocity, const int current);

private:

};

#endif // _EposProfileVelocityMode_HPP
