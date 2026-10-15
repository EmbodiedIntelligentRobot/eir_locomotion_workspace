/**
 * @file plane_closed_interface.h
 * @author JameScottX
 * @brief 
 * @version 0.1
 * @date 2024-11-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once
#include "plane_closed_kinematics.h"


class PlaneClosedInterface : public PlaneClosedKinematics{
    
public:
    PlaneClosedInterface(std::string name, 
                        float initial_theta,
                        float swing_rod,
                        float swing_link,
                        float swing2_link,
                        float fix_link);
    ~PlaneClosedInterface();

    // Respect to q 
    float actuator_dir = 1.0;
    // Respect to x
    float joint_dir = 1.0;

    void set_dir(float actuator_dir, float joint_dir);

    float plane_closed_DK(float q);
    float plane_closed_IK(float q);

    float plane_closed_DK_tau2tau(float q, float torque);
    float plane_closed_IK_tau2tau(float q, float torque);
    
    float plane_closed_DK_vel2vel(float q, float velocity);
    float plane_closed_IK_vel2vel(float q, float velocity);

};




