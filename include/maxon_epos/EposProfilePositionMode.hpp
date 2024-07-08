/**
 * @file   EposProfilePositionMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 17:04:39
 */

#ifndef _EposProfilePositionMode_HPP
#define _EposProfilePositionMode_HPP

#include "ControlModeBase.hpp"
#include "Device.hpp"
#include <iostream>


class EposProfilePositionMode : public ControlModeBase {
public:
    virtual ~EposProfilePositionMode();

    virtual void init(NodeHandle &node_handle);
    virtual void activate();
    virtual std::vector<int> read();
    virtual void write(const int position, const int velocity, const int current);

private:
    int m_max_qc;
    double m_position_cmd;
};

#endif // _EposProfilePositionMode_HPP
