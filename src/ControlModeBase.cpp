/**
 * @file   ControlModeBase
 * @brief  
 * @author arwtyxouymz
 * @date   2019-06-05 15:28:40
 */

#include "ControlModeBase.hpp"
#include <iostream>
/**
 * @brief Destructor
 */
ControlModeBase::~ControlModeBase()
{}

void ControlModeBase::init(NodeHandle &node_handle)
{
    m_epos_handle = node_handle;
}

std::vector<int> ControlModeBase::read()
{
    // int positionls = 0, velocityls = 0;
    // short currentls = 0;
    // try{
    //     VCS_NODE_COMMAND(GetPositionIs, m_epos_handle, &positionls);
    //     VCS_NODE_COMMAND(GetVelocityIsAveraged, m_epos_handle, &velocityls);
    //     VCS_NODE_COMMAND(GetCurrentIsAveraged, m_epos_handle, &currentls);
    // } catch (const EposException &e) {
    //     std::cout << "Epos GetPositionIs error!" << std::endl;
    //     std::cout << e.what() << std::endl;
    // }

    // // if(m_use_ros_unit){
    // //     // quad-counts of the encoder -> rad
    // //     pos = (positionls / static_cast<double>(m_max_qc_)) * 2. * M_PI;
    // //     // rpm -> rad/s
    // //     vel = velocityls * M_PI / 30.;
    // //     // mA -> A
    // //     cur = currentls / 1000.;
    // //     return;
    // // }
    // std::vector<int> output = {positionls, velocityls, currentls};
    // return output;
}