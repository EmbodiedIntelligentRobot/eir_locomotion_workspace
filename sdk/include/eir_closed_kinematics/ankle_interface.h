/**
 * @file ankle_interface.h
 * @author JameScottX
 * @brief 
 * @version 0.1
 * @date 2023-05-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include "ankle_kinematics.h"
#include <chrono>
#include <atomic>
#include <array>
#include <cstdint>

struct AnkleDiagnosticSnapshot {
    uint64_t sequence{0};
    bool snapshot_valid{false};
    bool dk_snapshot_valid{false};
    bool ik_snapshot_valid{false};
    Eigen::Vector2d dk_input{Eigen::Vector2d::Zero()};
    Eigen::Vector2d dk_raw{Eigen::Vector2d::Zero()};
    Eigen::Vector2d dk_output{Eigen::Vector2d::Zero()};
    Eigen::Vector2d ik_input{Eigen::Vector2d::Zero()};
    Eigen::Vector2d ik_output{Eigen::Vector2d::Zero()};
    uint32_t dk_flags{0};
    uint32_t ik_flags{0};
};

enum ANKLE_DIAGNOSTIC_BITS : uint32_t {
    ANKLE_DIAG_AVAILABLE = 1U << 0,
    ANKLE_DIAG_VALID = 1U << 1,
    ANKLE_DIAG_FALLBACK = 1U << 2,
    ANKLE_DIAG_RECOVERED = 1U << 3,
    ANKLE_DIAG_OUT_OF_DOMAIN = 1U << 4,
    ANKLE_DIAG_CLAMPED = 1U << 5,
    ANKLE_DIAG_NUMERIC_FAILURE = 1U << 6,
    ANKLE_DIAG_UNAVAILABLE = 1U << 7,
};

enum ANKLE_MODE{
    ANKLE_MOTOR_POSITION_VELOCITY,                      // 使用电机的位置和速度
    ANKLE_ENCODER_POSITION_VELOCITY,                    // 使用编码器的位置和速度
};

enum class AnkleMotorKdStrategy : uint8_t {
    ZERO_CROSS = 0,
    SOFT_QP = 1,
};

enum class AnkleMotorKdStatus : uint8_t {
    MAPPED = 0,
    HELD = 1,
    FALLBACK = 2,
};

struct AnkleMotorKdRequest {
    AnkleMotorKdStrategy strategy{AnkleMotorKdStrategy::ZERO_CROSS};
    double pitch_kd_target{0.0};
    double roll_kd_target{0.0};
    double pitch_weight{1.0};
    double roll_weight{1.0};
    double cross_weight{1.0};
    double fallback_motor_kd{3.0};
    double dt{0.002};
};

struct AnkleMotorKdResult {
    Eigen::Vector2d motor_kd{Eigen::Vector2d::Zero()};
    Eigen::Matrix2d ik_jacobian{Eigen::Matrix2d::Zero()};
    Eigen::Matrix2d equivalent_joint_kd{Eigen::Matrix2d::Zero()};
    double determinant{0.0};
    double sigma_min{0.0};
    double condition{0.0};
    double objective{0.0};
    AnkleMotorKdStatus status{AnkleMotorKdStatus::FALLBACK};
    bool valid{false};
    bool used_fallback{false};
    bool constraint_active{false};
};

enum class AnkleWorkspaceGuardState : uint8_t {
    SAFE = 0,
    DERATING = 1,
    BOUNDARY_HOLD = 2,
    RECOVERY_ONLY = 3,
    FAULT_LATCHED = 4,
};

struct AnkleWorkspaceGuardConfig {
    bool enabled{true};

    // 正常区、保护边界及恢复滞环。condition 越小越好，sigma_min 越大越好。
    double condition_soft{6.0};
    double condition_hard{8.0};
    double sigma_min_soft{0.35};
    double sigma_min_hard{0.25};
    double condition_recovery{5.5};
    double sigma_min_recovery{0.40};

    // 给机构误差和数值误差预留的保护裕量；实际命令边界更保守。
    double condition_model_margin{0.5};
    double sigma_min_model_margin{0.02};

    // 只限制位置命令每周期允许变化量，不使用任何速度反馈。
    double max_pitch_rate{2.0};
    double max_roll_rate{2.0};
    double max_dt{0.01};
    double max_motor_position_error{0.20};
    double min_derating_scale{0.05};
    double recovery_improvement_epsilon{1e-4};
    uint32_t recovery_clear_cycles{50};
    uint32_t path_check_subdivisions{16};
    uint32_t projection_iterations{24};
};

struct AnkleWorkspaceMetrics {
    Eigen::Matrix2d ik_jacobian{Eigen::Matrix2d::Zero()};
    double determinant{0.0};
    double sigma_min{0.0};
    double sigma_max{0.0};
    double condition{0.0};
    // 0 为带模型裕量的硬边界，1 为软边界；负值表示已越过硬边界。
    double safety_margin{0.0};
    bool finite{false};
    bool position_in_range{false};
    bool motor_in_dk_domain{false};
    bool hard_safe{false};
    bool soft_safe{false};
    bool recovery_safe{false};
};

struct AnkleWorkspaceGuardResult {
    Eigen::Vector2d requested_q{Eigen::Vector2d::Zero()};
    Eigen::Vector2d guarded_q{Eigen::Vector2d::Zero()};
    // 电机命令经控制器同一套 LUT DK 重构后的预测关节姿态。
    Eigen::Vector2d predicted_q{Eigen::Vector2d::Zero()};
    // 与 left/right_ankle_IK() 相同的实际驱动器顺序和方向。
    Eigen::Vector2d motor_position_command{Eigen::Vector2d::Zero()};
    AnkleWorkspaceMetrics actual_metrics{};
    AnkleWorkspaceMetrics guarded_metrics{};
    AnkleWorkspaceGuardState state{AnkleWorkspaceGuardState::FAULT_LATCHED};
    double projection_ratio{0.0};
    bool command_valid{false};
    bool projected{false};
    bool step_limited{false};
    bool motor_position_limited{false};
    bool motion_allowed{false};
    bool feedforward_allowed{false};
    bool fault_latched{false};
};


class AnkleInterface : public AnkleKinematics
{
public:
    AnkleInterface(ANKLE_MODE ankle_mode,
                    std::string pamfile, 
                    std::string pitch_fitfile, 
                    std::string roll_fitfile,
                    std::string pitch_mapfile, 
                    std::string roll_mapfile,
                    std::string pitch_roll_rgefile);
    ~AnkleInterface();

    // 电机层面方向调整
    Eigen::Matrix2d left_actuator_dir = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d right_actuator_dir = Eigen::Matrix2d::Zero();
    // 脚踝关节方向调整
    Eigen::Matrix2d left_joint_dir = Eigen::Matrix2d::Zero();
    Eigen::Matrix2d right_joint_dir = Eigen::Matrix2d::Zero();

    Eigen::Vector2d left_ankle_IK(const Eigen::Vector2d &q);
    Eigen::Vector2d right_ankle_IK(const Eigen::Vector2d &q);
    Eigen::Vector2d left_ankle_DK(const Eigen::Vector2d &q);
    Eigen::Vector2d right_ankle_DK(const Eigen::Vector2d &q);
    
    Eigen::Vector2d right_ankle_IK_force2tau(const Eigen::Vector2d &q, const Eigen::Vector2d &force);
    Eigen::Vector2d right_ankle_IK_vel2vel(const Eigen::Vector2d &q, const Eigen::Vector2d &velocity);
    Eigen::Vector2d right_ankle_DK_vel2vel(const Eigen::Vector2d &q, const Eigen::Vector2d &velocity);

    Eigen::Vector2d left_ankle_IK_force2tau(const Eigen::Vector2d &q, const Eigen::Vector2d &force);
    Eigen::Vector2d left_ankle_IK_vel2vel(const Eigen::Vector2d &q, const Eigen::Vector2d &velocity);
    Eigen::Vector2d left_ankle_DK_vel2vel(const Eigen::Vector2d &q, const Eigen::Vector2d &velocity);
    AnkleMotorKdResult left_ankle_joint_kd_to_motor_kd(
        const Eigen::Vector2d &q, const AnkleMotorKdRequest &request);
    AnkleMotorKdResult right_ankle_joint_kd_to_motor_kd(
        const Eigen::Vector2d &q, const AnkleMotorKdRequest &request);
    void reset_ankle_motor_kd_mapping() noexcept;

    bool configure_ankle_workspace_guard(
        const AnkleWorkspaceGuardConfig &config) noexcept;
    const AnkleWorkspaceGuardConfig &ankle_workspace_guard_config() const noexcept
    {
        return ankle_workspace_guard_config_;
    }
    AnkleWorkspaceMetrics evaluate_left_ankle_workspace(
        const Eigen::Vector2d &q) const noexcept;
    AnkleWorkspaceMetrics evaluate_right_ankle_workspace(
        const Eigen::Vector2d &q) const noexcept;
    AnkleWorkspaceGuardResult guard_left_ankle_joint_command(
        const Eigen::Vector2d &q_actual,
        const Eigen::Vector2d &motor_position_actual,
        const Eigen::Vector2d &q_desired,
        double dt) noexcept;
    AnkleWorkspaceGuardResult guard_right_ankle_joint_command(
        const Eigen::Vector2d &q_actual,
        const Eigen::Vector2d &motor_position_actual,
        const Eigen::Vector2d &q_desired,
        double dt) noexcept;
    void reset_ankle_workspace_guard() noexcept;

    static constexpr double MOTOR_KD_HOLD_TIMEOUT_SEC = 0.2;
    static constexpr double MOTOR_KD_SIGMA_MIN_LIMIT = 0.25;
    static constexpr double MOTOR_KD_CONDITION_LIMIT = 8.0;
    static constexpr double WORKSPACE_POSITION_SNAPSHOT_TOLERANCE = 1e-7;
    
    Eigen::Vector2d order_change(const Eigen::Vector2d &q);

    ANKLE_MODE get_ankle_mode() const{return ankle_mode_;}

    // 位置差分 DK 速度模式：用 DK(q_motor) 位置差分 + EMA 代替雅可比，避免 off-diagonal 噪声放大
    void set_use_pos_diff_dk_vel(bool enable) { use_pos_diff_dk_vel_ = enable; }
    void set_ema_alpha(double alpha) { ema_alpha_ = alpha; }

    // 获取上一次有效值（用于外部检查）
    Eigen::Vector2d get_last_valid_left_pos() const { return last_valid_left_pos_; }
    Eigen::Vector2d get_last_valid_right_pos() const { return last_valid_right_pos_; }
    
    // 重置缓存的有效值
    void reset_last_valid_values();
    AnkleDiagnosticSnapshot get_flight_recorder_diagnostics(bool left) const noexcept;

    Eigen::Vector2d pitch_rge;              // 先最小值，后最大值
    Eigen::Vector2d roll_rge;              // 先最小值，后最大值

private:
    void publish_dk_diagnostic(bool left, const Eigen::Vector2d& input,
                               const Eigen::Vector2d& raw, const Eigen::Vector2d& output,
                               uint32_t flags) noexcept;
    void publish_ik_diagnostic(bool left, const Eigen::Vector2d& input,
                               const Eigen::Vector2d& output, uint32_t flags) noexcept;
    void mark_dk_numeric_failure(bool left, uint32_t flags = 0) noexcept;
    void mark_ik_numeric_failure(bool left, uint32_t flags = 0) noexcept;
    Eigen::Vector2d clamp_joint_position(const Eigen::Vector2d &q) const;
    Eigen::Matrix2d command_jacobian(const Eigen::Vector2d &q, bool left);
    Eigen::Vector2d command_force(const Eigen::Vector2d &q, const Eigen::Vector2d &force, bool left);
    Eigen::Vector2d command_velocity(const Eigen::Vector2d &q, const Eigen::Vector2d &velocity, bool left);
    AnkleMotorKdResult joint_kd_to_motor_kd(
        const Eigen::Vector2d &q, bool left,
        const AnkleMotorKdRequest &request);
    AnkleMotorKdResult compute_joint_kd_to_motor_kd(
        const Eigen::Vector2d &q, bool left,
        const AnkleMotorKdRequest &request) const;
    AnkleWorkspaceMetrics evaluate_ankle_workspace(
        const Eigen::Vector2d &q, bool left) const noexcept;
    AnkleWorkspaceGuardResult guard_ankle_joint_command(
        const Eigen::Vector2d &q_actual,
        const Eigen::Vector2d &motor_position_actual,
        const Eigen::Vector2d &q_desired,
        bool left,
        double dt) noexcept;
    Eigen::Vector2d joint_to_motor_position_fixed(
        const Eigen::Vector2d &q, bool left) const noexcept;
    Eigen::Vector2d motor_to_joint_position_lut(
        const Eigen::Vector2d &motor_position, bool left) noexcept;
    Eigen::Vector2d clamp_physical_joint_position(
        const Eigen::Vector2d &q, bool left) const noexcept;

    struct AnkleMotorKdState {
        Eigen::Vector2d previous_motor_kd{Eigen::Vector2d::Zero()};
        double failure_duration{0.0};
        bool previous_valid{false};
    };
    std::array<AnkleMotorKdState, 2> ankle_motor_kd_state_{};

    struct AnkleWorkspaceGuardInternalState {
        Eigen::Vector2d previous_joint_command{Eigen::Vector2d::Zero()};
        Eigen::Vector2d previous_motor_command{Eigen::Vector2d::Zero()};
        AnkleWorkspaceGuardState state{AnkleWorkspaceGuardState::SAFE};
        uint32_t recovery_clear_count{0};
        bool initialized{false};
        bool fault_latched{false};
    };
    AnkleWorkspaceGuardConfig ankle_workspace_guard_config_{};
    std::array<AnkleWorkspaceGuardInternalState, 2> ankle_workspace_guard_state_{};

    ANKLE_MODE ankle_mode_;
    
    // 缓存上一次的有效位置值
    Eigen::Vector2d last_valid_left_pos_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d last_valid_right_pos_ = Eigen::Vector2d::Zero();
    
    // 标记是否有有效值
    bool has_valid_left_pos_ = false;
    bool has_valid_right_pos_ = false;
    
    // 实时诊断计数器（不在实时线程同步打印）
    std::atomic<uint64_t> left_dk_nan_count_{0};
    std::atomic<uint64_t> right_dk_nan_count_{0};
    std::atomic<uint64_t> left_ik_nan_count_{0};
    std::atomic<uint64_t> right_ik_nan_count_{0};
    std::atomic<uint64_t> left_vel_nan_count_{0};
    std::atomic<uint64_t> right_vel_nan_count_{0};
    std::atomic<uint64_t> left_tau_nan_count_{0};
    std::atomic<uint64_t> right_tau_nan_count_{0};
    std::atomic<uint64_t> left_range_warn_count_{0};
    std::atomic<uint64_t> right_range_warn_count_{0};
    // DK and IK have different real-time writers.  Separate sequences avoid
    // multi-writer seqlock corruption and let DK validity remain observable
    // even while the control thread is publishing IK diagnostics.
    std::array<std::atomic<uint64_t>, 2> diagnostic_dk_sequence_{};
    std::array<std::atomic<uint64_t>, 2> diagnostic_ik_sequence_{};
    std::array<std::array<std::atomic<double>, 2>, 2> diagnostic_dk_input_{};
    std::array<std::array<std::atomic<double>, 2>, 2> diagnostic_dk_raw_{};
    std::array<std::array<std::atomic<double>, 2>, 2> diagnostic_dk_output_{};
    std::array<std::array<std::atomic<double>, 2>, 2> diagnostic_ik_input_{};
    std::array<std::array<std::atomic<double>, 2>, 2> diagnostic_ik_output_{};
    std::array<std::atomic<uint32_t>, 2> diagnostic_dk_flags_{};
    std::array<std::atomic<uint32_t>, 2> diagnostic_ik_flags_{};
    
    // 位置差分 DK 速度模式
    bool use_pos_diff_dk_vel_ = false;
    double ema_alpha_ = 0.15;
    bool pos_diff_left_init_ = false;
    bool pos_diff_right_init_ = false;
    Eigen::Vector2d prev_left_joint_q_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d prev_right_joint_q_ = Eigen::Vector2d::Zero();
    std::chrono::steady_clock::time_point prev_left_time_;
    std::chrono::steady_clock::time_point prev_right_time_;
    Eigen::Vector2d left_ema_vel_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d right_ema_vel_ = Eigen::Vector2d::Zero();
};
