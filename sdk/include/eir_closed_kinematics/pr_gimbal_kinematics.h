/**
 * @file pr_gimbal_kinematics.h
 * @author JameScottX
 * @brief
 * @version 0.1
 * @date 2023-05-19
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


#define PITCH_BIAS -0.0


enum PR_GIMBAL_TYPE{
    PR_GIMBAL_TYPE_NECK,
    PR_GIMBAL_TYPE_WAIST,
};


class PrGimbalKinematics{

public:
    PrGimbalKinematics(PR_GIMBAL_TYPE pr_type, std::string pamfile, std::string pitch_fitfile, std::string roll_fitfile);
    ~PrGimbalKinematics(){};

    double angle_1_start = 0;
    double angle_1_slice = 0.002;
    double angle_2_start = 0;
    double angle_2_slice = 0.002;

    std::vector<std::vector<double>> map_pitch;
    std::vector<std::vector<double>> map_roll;

    double pitch_fit_ploy33_coef[10];
    double roll_fit_ploy33_coef[10];

    // pitch 存在0度偏差
    Eigen::Vector2d bias_q;
    double ang1_bias = 0.0;
    double ang2_bias = 0.0;

    double ploy33_fit(double *coef, double &x, double &y);
    double pitch_DK_fit(Eigen::Vector2d &q);
    double roll_DK_fit(Eigen::Vector2d &q);

    void load_map(std::string matfile, std::vector<std::vector<double>> &map_x);

    double pr_gimbal_IK_ang1(Eigen::Vector2d &q);
    double pr_gimbal_IK_ang2(Eigen::Vector2d &q);

    Eigen::Vector2d pr_gimbal_IK_ori(Eigen::Vector2d &q);
    Eigen::Vector2d pr_gimbal_DK_ori(Eigen::Vector2d &q);
    Eigen::MatrixXd pr_gimbal_IK_Jocabian(Eigen::Vector2d &q);
    Eigen::MatrixXd pr_gimbal_DK_Jocabian(Eigen::Vector2d &q);

    PR_GIMBAL_TYPE get_pr_gimbal_type() const { return pr_type_; }

protected:
    PR_GIMBAL_TYPE pr_type_;
    double s2d98(const std::string &s);
    int s2i98(const std::string &s);

private:
    // ===== 几何常量 (从 ang_search_param.txt 读取或硬编码默认值) =====
    Eigen::Vector3d LP0;
    Eigen::Vector3d RP0;
    Eigen::Vector3d SHORT_ROD_MOTOR_P;
    Eigen::Vector3d LONG_ROD_MOTOR_P;
    double SHORT_ROD_MOTOR_P_LENGTH = 0.0;
    double LONG_ROD_MOTOR_P_LENGTH = 0.0;
    double nominal_short_ = 1.706;
    double nominal_long_  = -1.706;
    double short_rod_ = 65.0;
    double long_rod_  = 65.0;
    double swing_rod_ = 19.0;
    bool use_extended_format_ = false;

    // ===== acos 分支追踪 =====
    int branch_signs_[4];
    bool branch_locked_ = false;
    double prev_raw_ang1_ = 0.0;
    double prev_raw_ang2_ = 0.0;
    static constexpr double BRANCH_JUMP_THRESH = 0.85;

    // ===== 几何法内部函数 =====
    Eigen::Matrix3d rotation_yx(double pitch, double roll);
    void compute_acos_magnitudes(double pitch, double roll,
                                 double &m10, double &m11,
                                 double &m20, double &m21);
    void compute_IK_raw(double pitch, double roll,
                        double &raw_ang1, double &raw_ang2);
    void lock_branch_at_zero();
};
