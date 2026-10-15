/**
 * @file ankle_kinematics.h
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


// NaN回退策略枚举
enum NAN_FALLBACK_STRATEGY {
    NAN_FALLBACK_ZERO,              // 返回零值
    NAN_FALLBACK_LAST_VALID,        // 方案一：使用上一次有效值，速度/力矩为0
    NAN_FALLBACK_NEAREST_VALID      // 方案二：在表中查找最近的有效值
};

class AnkleKinematics{

public:
    AnkleKinematics(std::string pamfile, std::string pitch_fitfile, std::string roll_fitfile);
    ~AnkleKinematics(){};

    double angle_1_start = 0;
    double angle_1_slice = 0.002;
    double angle_1_end = 0;
    double angle_2_start = 0;
    double angle_2_slice = 0.002;
    double angle_2_end = 0;

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

    double ankle_IK_ang1(Eigen::Vector2d &q);
    double ankle_IK_ang2(Eigen::Vector2d &q);

    Eigen::Vector2d ankle_IK(Eigen::Vector2d &q);
    // 固定生产分支的无状态位置 IK；不会读写分支追踪状态，适合实时安全检查。
    Eigen::Vector2d ankle_IK_fixed(const Eigen::Vector2d &q) const;
    Eigen::Vector2d ankle_DK(Eigen::Vector2d &q);
    bool ankle_DK_input_in_domain(const Eigen::Vector2d &q) const noexcept;
    Eigen::Matrix2d ankle_IK_Jacobian_fixed(const Eigen::Vector2d &q) const;
    Eigen::Matrix2d ankle_IK_Jocabian(const Eigen::Vector2d &q);
    Eigen::Matrix2d ankle_DK_Jocabian(const Eigen::Vector2d &q);

    // 在表中查找最近的有效值（方案二）
    double find_nearest_valid_value(const std::vector<std::vector<double>> &map,
                                    uint64_t target_row, uint64_t target_col,
                                    int max_search_radius = 10);

protected:
    double s2d98(const std::string &s);
    int s2i98(const std::string &s);

private:
    // 几何参数 (从 ang_search_param.txt 读取)
    Eigen::Vector3d LP0;
    Eigen::Vector3d RP0;
    Eigen::Vector3d SHORT_ROD_MOTOR_P;
    Eigen::Vector3d LONG_ROD_MOTOR_P;
    double SHORT_ROD_MOTOR_P_LENGTH = 0.0;
    double LONG_ROD_MOTOR_P_LENGTH = 0.0;
    double POINT_X_ROD = 0.0;
    double nominal_short_ = 0.426;   // motor_nominal_angle_zero_rad.short_rod
    double nominal_long_  = -0.405;  // motor_nominal_angle_zero_rad.long_rod
    double short_rod_ = 112.0;
    double long_rod_  = 173.0;
    double swing_rod_ = 28.0;
    bool use_extended_format_ = false;

    // ===== acos 分支追踪 =====
    int branch_signs_[4];   // ±1 for (ang_1_0, ang_1_1, ang_2_0, ang_2_1)
    bool branch_locked_;    // 分支是否已锁定
    double prev_raw_ang1_;  // 上一帧 raw 输出 (用于检测跳变)
    double prev_raw_ang2_;
    static constexpr double BRANCH_JUMP_THRESH = 0.85;  // rad, 跳变检测阈值
    static constexpr double PITCH_PROBE_RAD = 0.001;     // 锁分支时的小探针

    // ===== 几何法内部函数 =====
    Eigen::Matrix3d rotation_yx(double pitch, double roll) const;
    void compute_acos_magnitudes(double pitch, double roll,
                                 double &m10, double &m11,
                                 double &m20, double &m21) const;
    void compute_IK_raw(double pitch, double roll,
                        double &raw_ang1, double &raw_ang2);
    void lock_branch_at_zero();
};
