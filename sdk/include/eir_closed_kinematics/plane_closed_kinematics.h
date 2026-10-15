/**
 * @file plane_closed_kinematics.h
 * @author JameScottX
 * @brief 
 * @version 0.1
 * @date 2024-11-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once 

#include "Eigen/Core"
#include "Eigen/Dense"
#include "math.h"
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <vector>
#include <boost/algorithm/string.hpp>


class PlaneClosedKinematics{

public:
    PlaneClosedKinematics(std::string name_,
                        float initial_theta_,
                        float swing_rod_,
                        float swing_link_,
                        float swing2_link_,
                        float fix_link_
                        );
    ~PlaneClosedKinematics(){};

    std::string name="";
    float initial_theta = 0.0f;
    float swing_rod = 0.0f;
    float swing_link = 0.0f;
    float swing2_link = 0.0f;
    float fix_link = 0.0f;

    float initial_theta_ik = 0.0f;
    float swing_rod_ik = 0.0f;
    float swing_link_ik = 0.0f;
    float swing2_link_ik = 0.0f;
    
    float passive_joint_bias = 0.0f;
    float active_joint_bias = 0.0f;

    inline void reset_parameters(float initial_theta_,
                        float swing_rod_,
                        float swing_link_,
                        float swing2_link_,
                        float fix_link_);
    float DK(float q);
    float DK_Jocabian(float q);
    float DK_Jocabian_Inv_T(float q);

    float IK(float x);                      // Desired x position
    float IK_Jocabian(float q);             // Only q is well-known, which is the motor position
    float DK_Jocabian_T(float q);

};

