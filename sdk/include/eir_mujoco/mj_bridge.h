
#ifndef MJ_BRIDGE_H_
#define MJ_BRIDGE_H_

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mujoco/mujoco.h>

// 共享接口（从 eir_lowlevel_sdk 目录）
#include "eir_lowlevel_sdk/DataTypes.h"
#include "eir_lowlevel_sdk/EirLowLevelApi.h"
#include "finger_coupling.h"
#include "depth_camera.h"

// 传感器处理模块（定义 vector_t, StateSnapshot, SensorProcessor）
#include "sensor_processing.h"

#define FREE_JOINT_NUM 4 // xyz slide + ball

// 末端执行器 (灵巧手) 配置常量，与硬件保持一致
#define EE_COUNT 2           // 末端执行器数量 (左右手)
#define FINGER_COUNT 6       // 每个 EE 的自由度
#define TOTAL_EE_DOF (EE_COUNT * FINGER_COUNT)  // 总自由度 = 12


namespace MJ_BRIDGE {
class MJ_Bridge;
}

namespace MJ_SIM {

// 全局运行标志，控制仿真循环
extern std::atomic<bool> mujoco_run_flag;

/**
    * @brief 设置物理线程绑定的 CPU 核心
    * @param core CPU 核心编号 (0, 1, 2, ...)，-1 表示不绑定
    * @note 必须在 mj_loop() 之前调用才生效
    */
void setPhysicsThreadCpuCore(int core);

/**
    * @brief 获取当前设置的物理线程 CPU 核心
    * @return CPU 核心编号，-1 表示未绑定
    */
int getPhysicsThreadCpuCore();

/**
    * @brief 启动 MuJoCo 仿真循环（阻塞调用，包含渲染）
    * @param xml_name 模型文件路径
    * @param bridge 已创建的 MJ_Bridge 实例，用于状态读写。如果为 nullptr，则内部创建临时实例。
    * @note bridge 的生命周期必须覆盖整个 mj_loop 执行期间
    */
void mj_loop(const std::string& xml_name, MJ_BRIDGE::MJ_Bridge* bridge = nullptr);

// 请求 mj_loop 退出
void request_exit();

}  // namespace MJ_SIM


namespace MJ_BRIDGE {

using scalar_t = double;
using vector3_t = Eigen::Vector3d;
using vector4_t = Eigen::Vector4d;
using quaternion_t = Eigen::Quaterniond;
inline constexpr std::size_t kFinalTorqueSnapshotCapacity = 64;

struct FinalTorqueSnapshot {
    std::uint64_t sequence{0};
    std::uint32_t count{0};
    bool valid{false};
    std::array<double, kFinalTorqueSnapshotCapacity> torque{};
};

// Simulator-side stair context.  The seven values follow the training
// ``stair_gait_context`` contract:
// active, sequence_started, expected_left, expected_right,
// target_landing_x, target_landing_y, target_landing_z.
struct StairContext {
    static constexpr std::size_t kSize = 7;
    std::array<float, kSize> data{};
    std::uint64_t timestamp_ns{0};
    bool valid{false};
};
// vector_t, StateSnapshot, SensorProcessor 定义在 sensor_processing.h

/**
 * @brief MJ_Bridge - MuJoCo 仿真桥接类
 * 
 * 直接实现 EirLowLevelApi 接口，移除了中间层 EHR_Mujoco_Interface。
 * 
 * 功能包括:
 * - MuJoCo 模型数据管理
 * - 关节/电机映射
 * - 传感器数据读取
 * - 仿真线程管理
 * - 状态监控和回调
 * - PD 控制计算
 * - 悬吊功能
 */
class MJ_Bridge : public EIR_Robotics::EirLowLevelApi {
public:
    // ========== 构造/析构 ==========
    MJ_Bridge();
    explicit MJ_Bridge(const std::string& xml_path);
    MJ_Bridge(mjModel* model, mjData* data);  // 兼容原有构造方式
    ~MJ_Bridge() override;

    // ========== EirLowLevelApi 接口实现 ==========
    bool Init() override;
    bool Init(const std::string& config_name) override;
    
    void SubscribeRobotState(std::function<void(const RobotStatePtr)> callback) override;
    bool PublishRobotCmd(const RobotCmd& cmd) override;
    bool Estop() override;
    // Nonblocking inhibition, safe alongside command publication/physics.
    // Latched for this bridge's lifetime; create a new bridge to resume.
    void requestEstop() noexcept;

    void SubscribeBMSState(std::function<void(const BmsStatePtr)> callback) override;
    bool PublishBMSCmd(const BmsCmd& cmd) override;
    void SubscriberEndEffectorState(std::function<void(const EEStatePtr)> callback) override;
    bool PublishEndEffectorCmd(const EECmd& cmd) override;
    void SubscriberJoystickState(std::function<void(const JoystickStatePtr)> callback) override;
    void SubscriberDiagnosticStatus(std::function<void(const DiagnosticStatusPtr)> callback) override;

    // ========== 仿真控制 ==========
    void start_sim();
    void stop_sim();
    bool is_running() const;

    // ========== 配置接口 ==========
    bool setJointSequence(const std::vector<std::string>& joint_names);
    bool setMotorSequence(const std::vector<std::string>& motor_names);
    // Grippers use SDK q/dq/tau/Kp/Kd; legacy hands retain position-only behavior.
    bool setFingerImpedanceControl(bool enabled);
    bool setFingerTorqueLimits(const std::vector<double>& limits);
    bool setFingerJointSequence(const std::vector<std::string>& finger_joint_names);
    bool setFingerCouplingTable(
        const std::string& table_path,
        const std::vector<FingerCouplingJoint>& couplings);
    void setXmlPath(const std::string& xml_path);
    const std::string& xml_path() const;
    
    /**
     * @brief 设置物理线程绑定的 CPU 核心
     * @param core CPU 核心编号 (0, 1, 2, ...)，-1 表示不绑定（由系统调度）
     * @note 必须在 Init() 之前调用才生效
     */
    static void setPhysicsThreadCpuCore(int core);
    
    /**
     * @brief 获取当前设置的物理线程 CPU 核心
     * @return CPU 核心编号，-1 表示未绑定
     */
    static int getPhysicsThreadCpuCore();

    // ========== MuJoCo 数据设置 (由 mj_loop 调用) ==========
    void setup(mjModel* model, mjData* data);
    void Print_Model_Info();

    // ========== 状态查询接口 ==========
    size_t getUserJointCount() const;
    size_t getUserMotorCount() const;
    size_t getMJActuatorCount() const;
    
    // 验证传感器完整性
    bool validateSensorCompleteness() const;
    std::vector<std::string> getMissingPositionSensorJoints() const;
    std::vector<std::string> getMissingVelocitySensorJoints() const;
    std::vector<std::string> getMissingPositionSensorActuators() const;
    std::vector<std::string> getMissingVelocitySensorActuators() const;
    std::vector<std::string> getMissingForceSensorActuators() const;

    // ========== 底层数据访问 (内部使用或调试) ==========
    // 关节状态（从关节传感器读取，用于运动学计算）
    vector_t getJointPositions() const;
    vector_t getJointVelocities() const;
    
    // 执行器状态（从执行器传感器读取，用于电机控制反馈）
    vector_t getActuatorPositions() const;
    vector_t getActuatorVelocities() const;
    vector_t getActuatorForces() const;
    
    // 命令力矩
    vector_t getJointTorques() const;

    // Flight recorder fast path: fixed-size, lock-free and allocation-free.
    // Returns valid=false when no torque has been applied or a consistent
    // sequence-guard snapshot cannot be obtained without waiting.
    FinalTorqueSnapshot getFinalTorqueSnapshot() const noexcept;

    // ========== 感知深度（仿真端纯 C++，无 ROS） ==========
    void setDepthCameraConfig(const DepthCameraConfig& config);
    const DepthCameraConfig& getDepthCameraConfig() const noexcept;
    void setDepthPublishEnabled(bool enabled) noexcept;
    bool isDepthPublishEnabled() const noexcept;
    bool isDepthCameraReady() const noexcept;
    bool getDepthStack(DepthStack& output) const noexcept;
    // Returns the latest simulator-only stair context snapshot.  The
    // publisher thread never accesses mjModel/mjData directly.
    bool getStairContext(StairContext& output) const noexcept;

    // IMU 状态
    quaternion_t getIMUQuaternion() const;
    vector4_t getIMUQuaternionVec() const;
    vector3_t getIMUGyro() const;
    vector3_t getIMUAcc() const;

    // 设置命令力矩 (内部使用)
    void setJointTorques(const vector_t& joint_torques);

    // ========== 仿真步进 (由 mj_loop 调用) ==========
    void reset();
    void bridgeStep();

    // ========== 悬吊功能 ==========
    void setSuspendEnable(bool enable);
    bool isSuspendEnabled() const;
    void setSuspendParams(scalar_t linear_stiffness, scalar_t angular_stiffness,
                          scalar_t linear_damping, scalar_t angular_damping);

    // ========== IMU 传感器查找 ==========
    void findQuatSensor(const std::string& sensor_name);
    void findGyroSensor(const std::string& sensor_name);
    void findAccSensor(const std::string& sensor_name);
    void findVelSensor(const std::string& sensor_name);

    // ========== 通信延时模拟接口 ==========
    // 注意：传感器噪声通过 MuJoCo XML 配置，例如：
    //   <jointpos name="l_leg_1_p" joint="l_leg_1" noise="5e-3"/>
    
    /**
     * @brief 设置传感器通信延时（模拟真实机器人数据传输延迟）
     * @param delay_ms 延时时间（毫秒），0 表示禁用延时模拟
     */
    void setSensorDelay(double delay_ms);
    
    /**
     * @brief 获取当前设置的传感器通信延时
     * @return 延时时间（毫秒）
     */
    double getSensorDelay() const;
    
    /**
     * @brief 检查延时模拟是否启用
     * @return 如果延时 > 0 则返回 true
     */
    bool isDelaySimulationEnabled() const;
    
#ifdef DEBUG
    /**
     * @brief 获取实时关节位置（不经过延时缓冲区，用于调试对比）
     * @return 当前仿真的实时关节位置
     */
    vector_t getRealtimeJointPositions() const;
    
    /**
     * @brief 获取延时调试信息
     * @param realtime_pos 输出：实时位置
     * @param delayed_pos 输出：延时位置
     * @param actual_delay_ms 输出：实际延时（毫秒）
     * @return 是否获取成功
     */
    bool getDelayDebugInfo(vector_t& realtime_pos, vector_t& delayed_pos, double& actual_delay_ms) const;
    
    /**
     * @brief 获取综合噪声调试信息（位置+速度+力矩+IMU）
     * @param pos_raw 输出：原始关节位置
     * @param pos_noisy 输出：加噪后位置
     * @param vel_raw 输出：原始关节速度
     * @param vel_noisy 输出：加噪后速度
     * @param torque_raw 输出：原始力矩
     * @param torque_noisy 输出：加噪后力矩
     * @param imu_gyro_raw 输出：原始陀螺仪
     * @param imu_gyro_noisy 输出：加噪后陀螺仪
     * @param imu_acc_raw 输出：原始加速度
     * @param imu_acc_noisy 输出：加噪后加速度
     * @return 是否获取成功
     */
    bool getNoiseDebugInfo(
        vector_t& pos_raw, vector_t& pos_noisy,
        vector_t& vel_raw, vector_t& vel_noisy,
        vector_t& torque_raw, vector_t& torque_noisy,
        vector3_t& imu_gyro_raw, vector3_t& imu_gyro_noisy,
        vector3_t& imu_acc_raw, vector3_t& imu_acc_noisy) const;
#endif  // DEBUG
    
    // ========== 静态工具函数 ==========
    static bool file_exists(const std::string& path);

private:
    std::atomic<bool> estop_requested_{false};
    // ========== 内部初始化 ==========
    void setupJointIndexMaps();
    void setSimState(const vector_t& pos, const vector_t& vel);
    void updateState();
    void applyAction();
    void applySuspend();

    // ========== 状态发布 ==========
    void publishRobotState();
    void publishEEState();  ///< 发布末端执行器状态
    RobotStatePtr BuildRobotState() const;
    EEStatePtr BuildEEState() const;  ///< 构建末端执行器状态
    void ApplyPendingRobotCmd();
    void updateFinalTorqueSnapshotLocked();
    void updateDepthSnapshot(const DepthStack& stack) noexcept;
    void updateStairContextSnapshot(const StairContext& context) noexcept;
    void updateStairContext();
    double stairTerrainHeight(double x, double y) const noexcept;
    uint64_t NowNanoseconds() const;

    // ========== 配置 ==========
    std::string xml_path_;

    // ========== MuJoCo 模型和数据 ==========
    mjModel* model_{nullptr};
    mjData* data_{nullptr};

    // ========== 关节状态缓存 (用户顺序) ==========
    vector_t joint_pos_vec_;
    vector_t joint_vel_vec_;
    
    // ========== 执行器状态缓存 (用户顺序) ==========
    vector_t actuator_pos_vec_;
    vector_t actuator_vel_vec_;
    vector_t actuator_force_vec_;
    
    // ========== 命令力矩缓存 ==========
    vector_t joint_torque_vec_;
    vector_t robot_cmd_;
    
    // ========== IMU 状态 ==========
    quaternion_t imu_quat_;
    vector4_t imu_quat_vec_;
    vector3_t imu_gyro_vec_;
    vector3_t imu_acc_vec_;

    // ========== 索引映射 (MuJoCo 模型顺序) ==========
    std::unordered_map<std::string, size_t> MJ_Joint_Id_Map;
    std::unordered_map<std::string, size_t> MJ_Motor_Id_Map;

    // ========== 用户指定的名称序列 ==========
    std::vector<std::string> User_Joint_Names;
    std::vector<std::string> User_Motor_Names;
    std::vector<int> user_motor_ids_;  ///< User motor order to MuJoCo actuator ID

    // ========== 手指 PD 控制 ==========
    bool finger_control_enabled_{false};
    std::vector<std::string> configured_finger_joint_names_;  ///< Fixed SDK-slot order
    std::vector<int> finger_joint_ids_;       ///< MuJoCo joint IDs
    std::vector<int> finger_actuator_ids_;    ///< MuJoCo actuator IDs (for ctrl)
    std::vector<double> finger_open_positions_;
    std::vector<double> finger_closed_positions_;
    /// Model-travel targets (0..100 percent), not EIRGRIPPER motor radians.
    std::vector<double> finger_targets_;
    bool finger_impedance_control_{false};
    std::array<double, 2> finger_torque_limits_{3.0, 3.0};
    std::array<double, TOTAL_EE_DOF> finger_velocity_targets_{};
    std::array<double, TOTAL_EE_DOF> finger_feedforward_torque_{};
    std::array<double, TOTAL_EE_DOF> finger_kp_{};
    std::array<double, TOTAL_EE_DOF> finger_kd_{};
    struct FingerCouplingRuntime
    {
        std::string curve_name;
        int active_joint_id{-1};
        int passive_joint_id{-1};
    };
    FingerCouplingTable finger_coupling_table_;
    std::vector<FingerCouplingJoint> configured_finger_couplings_;
    std::vector<FingerCouplingRuntime> finger_couplings_;
    std::unordered_map<int, std::pair<double, double>> coupled_active_ranges_;
    mutable std::mutex finger_command_mutex_;
    static constexpr double kFingerKp = 20.0; ///< Position gain
    static constexpr double kFingerKd = 1.0;  ///< Damping gain
    static constexpr double kFingerMaxTarget = 100.0;
    bool validateAndBuildFingerCouplingMap(
        const FingerCouplingTable& table,
        const std::vector<FingerCouplingJoint>& configured_couplings,
        std::vector<FingerCouplingRuntime>& runtime_couplings,
        std::unordered_map<int, std::pair<double, double>>& active_ranges) const;
    bool buildFingerCouplingMap();
    bool buildFingerJointMap();
    void applyFingerPositions();
    void applyFingerCouplings();

    // ========== 传感器映射 ==========
    std::unordered_map<size_t, size_t> Jnt_to_P_Sensor_map;
    std::unordered_map<size_t, size_t> Jnt_to_V_Sensor_map;
    std::unordered_map<size_t, size_t> Jnt_to_F_Sensor_map;
    std::unordered_map<size_t, size_t> Act_to_P_Sensor_map;
    std::unordered_map<size_t, size_t> Act_to_V_Sensor_map;
    std::unordered_map<size_t, size_t> Act_to_F_Sensor_map;

    // ========== 计数器 ==========
    size_t nJoints_{0};
    size_t nActuators_{0};

    // ========== IMU 传感器 ID ==========
    int IMU_Quat_SensorID_{-1};
    int IMU_Gyro_SensorID_{-1};
    int IMU_Acc_SensorID_{-1};
    int IMU_Vel_SensorID_{-1};

    // ========== 数据保护互斥锁 ==========
    mutable std::mutex data_mutex_;
    mutable std::mutex command_mutex_;

    // Final actuator torque actually accepted by setJointTorques().  These
    // atomics are intentionally independent of command_mutex_ so a recorder
    // reader never waits for the MuJoCo physics/command writer.
    std::atomic<std::uint64_t> final_torque_sequence_{0};
    std::atomic<std::uint32_t> final_torque_count_{0};
    std::atomic<bool> final_torque_valid_{false};
    std::array<std::atomic<double>, kFinalTorqueSnapshotCapacity> final_torque_snapshot_{};

    // Processed perception depth snapshot.  The physics thread is the sole
    // writer; readers use the same seqlock pattern as the flight-recorder
    // torque snapshot and never touch mjModel/mjData from a ROS thread.
    std::unique_ptr<DepthCamera> depth_camera_;
    DepthCameraConfig depth_camera_config_{};
    std::atomic<bool> depth_publish_enabled_{false};
    std::atomic<bool> depth_camera_ready_{false};
    // model_ becomes visible to start_sim() early in setup(); this separate
    // flag prevents Init() from racing the subsequent site binding.
    std::atomic<bool> depth_camera_setup_done_{false};
    std::atomic<std::uint64_t> depth_snapshot_sequence_{0};
    std::atomic<std::uint32_t> depth_snapshot_source_sequence_{0};
    std::atomic<std::uint64_t> depth_snapshot_timestamp_ns_{0};
    std::atomic<std::uint64_t> depth_snapshot_monotonic_ns_{0};
    std::atomic<bool> depth_snapshot_valid_{false};
    std::array<std::atomic<float>, DepthStack::kDataSize> depth_snapshot_{};
    std::uint32_t depth_capture_sequence_{0};
    double next_depth_capture_time_{0.0};
    mutable std::mutex depth_config_mutex_;

    // Stair-context state is updated only on the MuJoCo physics thread and
    // exported through a seqlock snapshot, just like the depth stream.
    std::atomic<std::uint64_t> stair_context_sequence_{0};
    std::atomic<std::uint64_t> stair_context_timestamp_ns_{0};
    std::atomic<bool> stair_context_valid_{false};
    std::array<std::atomic<float>, StairContext::kSize> stair_context_snapshot_{};
    std::vector<int> stair_geom_ids_;
    int stair_left_foot_body_id_{-1};
    int stair_right_foot_body_id_{-1};
    int stair_base_body_id_{-1};
    bool stair_active_{false};
    double stair_enter_elapsed_{0.0};
    double stair_exit_elapsed_{0.0};
    int stair_direction_{0};
    int stair_first_expected_foot_{0};
    // The context estimator mirrors the training-side support/alternation
    // state.  These values are simulator-only bookkeeping; a hardware
    // estimator must derive the same signals from depth, contact and
    // kinematics rather than from stair geometry.
    int stair_support_foot_{-1};
    int stair_last_completed_foot_{-1};
    double stair_support_ground_height_{0.0};
    bool stair_edge_locked_{false};
    std::array<double, 2> stair_contact_ground_height_{};
    double stair_target_step_height_{0.0};
    double stair_edge_x_{0.0};

    // ========== 悬吊功能参数 ==========
    std::atomic<bool> suspend_enable_{true};
    scalar_t suspend_linear_k_{10000.0};
    scalar_t suspend_angular_k_{10000.0};
    scalar_t suspend_linear_d_{100.0};
    scalar_t suspend_angular_d_{100.0};
    bool suspend_last_state_{false};  // 悬挂状态变化跟踪（替换原来的 static 变量）

    // ========== 回调管理 (从 EHR_Mujoco_Interface 合并) ==========
    using RobotStateCallbacks = std::vector<std::function<void(const RobotStatePtr)>>;
    RobotStateCallbacks robot_state_callbacks_;
    mutable std::mutex robot_state_callbacks_mutex_;

    // ========== 末端执行器 (EE) 回调管理 ==========
    using EEStateCallbacks = std::vector<std::function<void(const EEStatePtr)>>;
    EEStateCallbacks ee_state_callbacks_;
    mutable std::mutex ee_state_callbacks_mutex_;

    // ========== 命令管理 (从 EHR_Mujoco_Interface 合并) ==========
    std::shared_ptr<const RobotCmd> last_robot_cmd_;
    mutable std::mutex robot_cmd_mutex_;

    // ========== 线程管理 ==========
    std::atomic<bool> sim_thread_running_{false};
    std::atomic<bool> sequences_configured_{false};  ///< 关节/电机序列配置完成标志，防止竞态
    std::atomic<bool> setup_complete_{false}; // Publish model/maps after setup finishes.
    std::thread sim_thread_;
    mutable std::mutex lifecycle_mutex_;

    // ========== 传感器处理（噪声 + 延时）==========
    std::unique_ptr<SensorProcessor> sensor_processor_;  ///< 传感器处理器
    
    // 内部辅助方法
    void initSensorProcessor();   ///< 初始化传感器处理器
    void pushStateToProcessor();  ///< 将当前状态推入处理器（噪声 + 缓冲）
    const StateSnapshot* getProcessedStatePtr() const;  ///< 获取处理后的状态指针
    uint64_t SimTimeNanoseconds() const;  ///< 获取 MuJoCo 仿真时间（纳秒）

};  // class MJ_Bridge

}  // namespace MJ_BRIDGE






#endif  // MJ_BRIDGE_H_
