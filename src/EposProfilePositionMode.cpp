/**
 * @file   EposProfilePositionMode
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-04 22:40:57
 */

#include "EposProfilePositionMode.hpp"

EposProfilePositionMode::~EposProfilePositionMode()
{}

void EposProfilePositionMode::init(NodeHandle &node_handle)
{
    ControlModeBase::init(node_handle);
}

void EposProfilePositionMode::activate()
{
    VCS_NODE_COMMAND_NO_ARGS(ActivateProfilePositionMode, m_epos_handle);
}

void EposProfilePositionMode::write(const int position, const int velocity, const int current)
{
    int quad_count = static_cast<int>(position);
    VCS_NODE_COMMAND(MoveToPosition, m_epos_handle, quad_count, /* absolute */true, /* overwrite */true);
}

std::vector<int> EposProfilePositionMode::read()
{
    int positionls = 0, velocityls = 0;
    short currentls = 0;
    try{
        VCS_NODE_COMMAND(GetPositionIs, m_epos_handle, &positionls);
        VCS_NODE_COMMAND(GetVelocityIsAveraged, m_epos_handle, &velocityls);
        VCS_NODE_COMMAND(GetCurrentIsAveraged, m_epos_handle, &currentls);
    } catch (EposException &e) {
        std::cout << "Epos GetPositionIs error!" << std::endl;
        std::cout << e.what() << std::endl;
    }

    // if(m_use_ros_unit){
    //     // quad-counts of the encoder -> rad
    //     pos = (positionls / static_cast<double>(m_max_qc_)) * 2. * M_PI;
    //     // rpm -> rad/s
    //     vel = velocityls * M_PI / 30.;
    //     // mA -> A
    //     cur = currentls / 1000.;
    //     return;
    // }
    std::vector<int> output = {positionls, velocityls, currentls};
    return output;
}