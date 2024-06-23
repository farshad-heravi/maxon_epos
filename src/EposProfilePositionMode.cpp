/**
 * @file   EposProfilePositionMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:40:57
 */

#include "EposProfilePositionMode.hpp"

EposProfilePositionMode::~EposProfilePositionMode()
{}

void EposProfilePositionMode::init()
{
    ControlModeBase::init();
}

void EposProfilePositionMode::activate()
{
    VCS_NODE_COMMAND_NO_ARGS(ActivateProfilePositionMode, m_epos_handle);
}

void EposProfilePositionMode::read()
{}

void EposProfilePositionMode::write(const double position, const double velocity, const double current)
{
    int quad_count = static_cast<int>(position);
    VCS_NODE_COMMAND(MoveToPosition, m_epos_handle, quad_count, /* absolute */true, /* overwrite */true);
}
