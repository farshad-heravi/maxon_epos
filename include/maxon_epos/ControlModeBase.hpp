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

    virtual void init(NodeHandle &node_handle);

    // activate operation mode
    virtual void activate() = 0;

    // read something required for operation mode
    std::vector<int>  read();   // returns {pos, vel, current}

    // write commands of operation mode
    virtual void write(const int position, const int velocity, const int current) = 0;

protected:
    NodeHandle m_epos_handle;
};

#endif // _ModeBase_HPP
