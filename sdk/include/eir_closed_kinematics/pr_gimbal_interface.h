/**
 * @file pr_gimbal_interface.h
 * @author JameScottX
 * @brief 
 * @version 0.1
 * @date 2023-05-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "pr_gimbal_kinematics.h"
#include <chrono>
#include <atomic>
#include <array>

enum PR_GIMBAL_MODE{
    PR_GIMBAL_MOTOR_POSITION_VELOCITY,                      // 使用电机的位置和速度
    PR_GIMBAL_ENCODER_POSITION_VELOCITY,                    // 使用编码器的位置和速度
};

struct PrGimbalDiagnosticSnapshot {
    uint64_t sequence{0};
    bool snapshot_valid{false};
    Eigen::Vector2d dk_input{Eigen::Vector2d::Zero()};
    Eigen::Vector2d dk_raw{Eigen::Vector2d::Zero()};
    Eigen::Vector2d dk_output{Eigen::Vector2d::Zero()};
    Eigen::Vector2d ik_input{Eigen::Vector2d::Zero()};
    Eigen::Vector2d ik_output{Eigen::Vector2d::Zero()};
    double jacobian_determinant{0.0};
    double jacobian_condition{0.0};
    uint32_t dk_flags{0};
    uint32_t ik_flags{0};
};

enum PR_GIMBAL_DIAGNOSTIC_BITS : uint32_t {
    PR_DIAG_AVAILABLE = 1U << 0,
    PR_DIAG_VALID = 1U << 1,
    PR_DIAG_FALLBACK = 1U << 2,
    PR_DIAG_RECOVERED = 1U << 3,
    PR_DIAG_OUT_OF_DOMAIN = 1U << 4,
    PR_DIAG_CLAMPED = 1U << 5,
    PR_DIAG_JACOBIAN_VALID = 1U << 6,
    PR_DIAG_NUMERIC_FAILURE = 1U << 7,
    PR_DIAG_UNAVAILABLE = 1U << 8,
};


class PrGimbalInterface : public PrGimbalKinematics
{
public:
    PrGimbalInterface(PR_GIMBAL_TYPE pr_type, 
                    PR_GIMBAL_MODE pr_mode,
                    std::string pamfile, 
                    std::string pitch_fitfile, 
                    std::string roll_fitfile,
                    std::string pitch_mapfile, 
                    std::string roll_mapfile,
                    std::string pitch_roll_rgefile);
    ~PrGimbalInterface();

    // 电机层面方向调整
    Eigen::Matrix2d actuator_dir = Eigen::Matrix2d::Zero();
    // 脚踝关节方向调整
    Eigen::Matrix2d joint_dir = Eigen::Matrix2d::Zero();

    Eigen::Vector2d pr_gimbal_IK(const Eigen::Vector2d &q);
    Eigen::Vector2d pr_gimbal_DK(const Eigen::Vector2d &q);

    Eigen::Vector2d pr_gimbal_IK_force2tau(const Eigen::Vector2d &q, const Eigen::Vector2d &force);
    Eigen::Vector2d pr_gimbal_IK_vel2vel(const Eigen::Vector2d &q, const Eigen::Vector2d &velocity);
    Eigen::Vector2d pr_gimbal_DK_vel2vel(const Eigen::Vector2d &q, const Eigen::Vector2d &velocity);
    
    Eigen::Vector2d order_change(const Eigen::Vector2d &q);

    PR_GIMBAL_MODE get_pr_gimbal_mode() const{return pr_mode_;}

    // 位置差分 DK 速度模式：用 DK(q_motor) 位置差分 + EMA 代替雅可比，避免 off-diagonal 噪声放大
    void set_use_pos_diff_dk_vel(bool enable) { use_pos_diff_dk_vel_ = enable; }
    void set_ema_alpha(double alpha) { ema_alpha_ = alpha; }

    Eigen::Vector2d pitch_rge;             // 先最小值，后最大值
    Eigen::Vector2d roll_rge;              // 先最小值，后最大值
    
    // 获取上一次有效值（用于外部检查）
    Eigen::Vector2d get_last_valid_pos() const { return last_valid_pos_; }
    
    // 重置缓存的有效值
    void reset_last_valid_values();
    PrGimbalDiagnosticSnapshot get_flight_recorder_diagnostics() const noexcept;

private:
    void publish_dk_diagnostic(const Eigen::Vector2d& input, const Eigen::Vector2d& raw,
                               const Eigen::Vector2d& output, uint32_t flags) noexcept;
    void publish_ik_diagnostic(const Eigen::Vector2d& input, const Eigen::Vector2d& output,
                               uint32_t flags) noexcept;
    void publish_jacobian_diagnostic(const Eigen::Matrix2d& jacobian) noexcept;
    void mark_dk_numeric_failure(uint32_t flags = 0) noexcept;
    void mark_ik_numeric_failure(uint32_t flags = 0) noexcept;
    PR_GIMBAL_MODE pr_mode_;
    
    // 缓存上一次的有效位置值
    Eigen::Vector2d last_valid_pos_ = Eigen::Vector2d::Zero();
    bool has_valid_pos_ = false;
    
    // 限频警告相关
    static constexpr double WARN_INTERVAL_SEC = 30.0;
    
    // 警告计数器
    std::atomic<uint64_t> dk_nan_count_{0};
    std::atomic<uint64_t> ik_nan_count_{0};
    std::atomic<uint64_t> vel_nan_count_{0};
    std::atomic<uint64_t> tau_nan_count_{0};
    std::atomic<uint64_t> range_warn_count_{0};
    std::atomic<uint64_t> diagnostic_sequence_{0};
    std::array<std::atomic<double>, 2> diagnostic_dk_input_{};
    std::array<std::atomic<double>, 2> diagnostic_dk_raw_{};
    std::array<std::atomic<double>, 2> diagnostic_dk_output_{};
    std::array<std::atomic<double>, 2> diagnostic_ik_input_{};
    std::array<std::atomic<double>, 2> diagnostic_ik_output_{};
    std::atomic<double> diagnostic_jacobian_determinant_{0.0};
    std::atomic<double> diagnostic_jacobian_condition_{0.0};
    std::atomic<uint32_t> diagnostic_dk_flags_{0};
    std::atomic<uint32_t> diagnostic_ik_flags_{0};
    
    // 上次警告时间
    std::chrono::steady_clock::time_point last_dk_warn_time_;
    std::chrono::steady_clock::time_point last_ik_warn_time_;
    std::chrono::steady_clock::time_point last_vel_warn_time_;
    std::chrono::steady_clock::time_point last_tau_warn_time_;
    std::chrono::steady_clock::time_point last_range_warn_time_;

    // 限频警告辅助函数
    bool should_warn(std::chrono::steady_clock::time_point &last_time);

    // 位置差分 DK 速度模式
    bool use_pos_diff_dk_vel_ = false;
    double ema_alpha_ = 0.15;
    bool pos_diff_init_ = false;
    Eigen::Vector2d prev_joint_q_ = Eigen::Vector2d::Zero();
    std::chrono::steady_clock::time_point prev_time_;
    Eigen::Vector2d ema_vel_ = Eigen::Vector2d::Zero();
};
