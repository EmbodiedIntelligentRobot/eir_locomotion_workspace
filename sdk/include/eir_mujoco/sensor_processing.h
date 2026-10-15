/**
 * @file sensor_processing.h
 * @brief 传感器处理模块：噪声注入 + 通信延时模拟
 * 
 * 噪声参数从 MuJoCo XML 传感器定义中读取，例如：
 *   <jointpos name="l_leg_1_p" joint="l_leg_1" noise="5e-3"/>
 * 
 * 如果传感器定义了 noise 属性，则在读取时添加高斯噪声；
 * 如果没有定义，则不添加噪声。
 */

#ifndef SENSOR_PROCESSING_H
#define SENSOR_PROCESSING_H

#include <atomic>
#include <array>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include <Eigen/Dense>

namespace MJ_BRIDGE {

// ==================== 类型定义 ====================
using vector_t = Eigen::VectorXd;

// ==================== 高斯噪声发生器 ====================
class GaussianNoise {
public:
    explicit GaussianNoise(uint32_t seed = 42);
    
    /// @brief 采样单个高斯噪声值
    /// @param sigma 标准差（从 XML noise 属性读取）
    /// @return 噪声值
    double sample(double sigma);
    
    /// @brief 重新设置随机种子
    void reseed(uint32_t seed);
    
private:
    std::mt19937 rng_;
    std::normal_distribution<double> dist_;
};

// ==================== 状态快照（存储一帧传感器数据）====================
struct StateSnapshot {
    uint64_t timestamp_ns{0};        ///< 时间戳 (纳秒)
    
    // 关节状态（用户顺序）
    vector_t joint_pos;
    vector_t joint_vel;
    vector_t joint_torque;
    
    // 执行器状态（用户顺序）
    vector_t actuator_pos;
    vector_t actuator_vel;
    vector_t actuator_force;
    
    // IMU 状态
    Eigen::Quaterniond imu_quat{1.0, 0.0, 0.0, 0.0};  ///< 四元数 (w, x, y, z)
    Eigen::Vector3d imu_gyro{0.0, 0.0, 0.0};          ///< 角速度 (rad/s)
    Eigen::Vector3d imu_acc{0.0, 0.0, 0.0};           ///< 加速度 (m/s²)
    Eigen::Vector3d imu_vel{0.0, 0.0, 0.0};           ///< 线速度 (m/s)
    
    /// @brief 调整向量大小
    void resize(size_t joint_count, size_t motor_count);
};

// ==================== 状态环形缓冲区（SPSC，无锁）====================
template<typename T, size_t Capacity>
class StateRingBuffer {
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be power of 2");
    static constexpr size_t Mask = Capacity - 1;
    
public:
    StateRingBuffer() = default;
    
    /// @brief 预分配所有 slot 的内存
    void preallocate(size_t joint_count, size_t motor_count) {
        for (auto& slot : buffer_) {
            slot.resize(joint_count, motor_count);
        }
    }
    
    /// @brief 获取写入槽位引用（生产者调用）
    T& acquireWriteSlot() {
        uint64_t seq = seq_.load(std::memory_order_relaxed);
        return buffer_[seq & Mask];
    }
    
    /// @brief 提交写入（生产者调用）
    void commitWrite() {
        uint64_t seq = seq_.load(std::memory_order_relaxed);
        seq_.store(seq + 1, std::memory_order_release);
    }
    
    /// @brief 获取最新数据指针（消费者调用）
    const T* getLatestPtr() const {
        uint64_t seq = seq_.load(std::memory_order_acquire);
        if (seq == 0) return nullptr;
        return &buffer_[(seq - 1) & Mask];
    }
    
    /// @brief 获取指定延时的数据指针（最近匹配，确保延迟精度）
    /// 搜索时间戳最接近 target_time 的条目，避免 "至少 N ms" 的累积偏差
    const T* getDelayedPtr(uint64_t current_time_ns, uint64_t delay_ns) const {
        uint64_t seq = seq_.load(std::memory_order_acquire);
        if (seq == 0) return nullptr;

        uint64_t target_time = (delay_ns < current_time_ns) ? (current_time_ns - delay_ns) : 0;
        size_t available = std::min(seq, static_cast<uint64_t>(Capacity));

        // 搜索最接近 target_time 的条目
        size_t best_idx = (seq - 1) & Mask;
        uint64_t best_diff = UINT64_MAX;
        for (size_t i = 0; i < available; ++i) {
            size_t idx = (seq - 1 - i) & Mask;
            uint64_t ts = buffer_[idx].timestamp_ns;
            uint64_t diff = (ts > target_time) ? (ts - target_time) : (target_time - ts);
            if (diff < best_diff) {
                best_diff = diff;
                best_idx = idx;
            } else {
                // 时间戳单调递增（从最新往最旧搜），一旦 diff 开始变大就停止
                break;
            }
        }

        return &buffer_[best_idx];
    }
    
    size_t size() const {
        uint64_t seq = seq_.load(std::memory_order_acquire);
        return std::min(seq, static_cast<uint64_t>(Capacity));
    }
    
    uint64_t sequence() const {
        return seq_.load(std::memory_order_acquire);
    }
    
    void clear() {
        seq_.store(0, std::memory_order_release);
    }
    
private:
    std::array<T, Capacity> buffer_;
    std::atomic<uint64_t> seq_{0};
};

// ==================== 缓冲区配置 ====================
constexpr size_t SENSOR_BUFFER_CAPACITY = 128;  ///< 约 128ms @ 1kHz
using SensorBuffer = StateRingBuffer<StateSnapshot, SENSOR_BUFFER_CAPACITY>;

// ==================== 噪声配置（从 MuJoCo XML 读取）====================
struct SensorNoiseConfig {
    std::vector<double> joint_pos_noise;    ///< 关节位置噪声 sigma（从 jointpos 传感器）
    std::vector<double> joint_vel_noise;    ///< 关节速度噪声 sigma（从 jointvel 传感器）
    std::vector<double> actuator_pos_noise; ///< 执行器位置噪声 sigma
    std::vector<double> actuator_vel_noise; ///< 执行器速度噪声 sigma
    std::vector<double> actuator_force_noise; ///< 执行器力噪声 sigma
    double imu_gyro_noise{0.0};             ///< IMU 陀螺仪噪声 sigma
    double imu_acc_noise{0.0};              ///< IMU 加速度计噪声 sigma
    
    void resize(size_t joint_count, size_t motor_count) {
        joint_pos_noise.resize(joint_count, 0.0);
        joint_vel_noise.resize(joint_count, 0.0);
        actuator_pos_noise.resize(motor_count, 0.0);
        actuator_vel_noise.resize(motor_count, 0.0);
        actuator_force_noise.resize(motor_count, 0.0);
    }
};

// ==================== 传感器处理器 ====================
class SensorProcessor {
public:
    /// @brief 构造函数
    SensorProcessor(size_t joint_count, size_t motor_count);
    
    /// @brief 设置噪声配置（从 MuJoCo XML 读取后调用）
    void setNoiseConfig(const SensorNoiseConfig& config);
    
    /// @brief 设置延时
    void setDelayMs(double delay_ms);
    
    /// @brief 获取当前延时设置
    double getDelayMs() const;
    
    /// @brief 检查延时是否启用
    bool isDelayEnabled() const;

    /// @brief 检查是否需要噪声或延时处理
    bool isProcessingEnabled() const;
    
    /// @brief 推送一帧数据到缓冲区（自动应用噪声）
    void push(
        const vector_t& joint_pos,
        const vector_t& joint_vel,
        const vector_t& joint_torque,
        const vector_t& actuator_pos,
        const vector_t& actuator_vel,
        const vector_t& actuator_force,
        const Eigen::Vector4d& imu_quat,
        const Eigen::Vector3d& imu_gyro,
        const Eigen::Vector3d& imu_acc,
        const Eigen::Vector3d& imu_vel,
        uint64_t timestamp_ns
    );
    
    /// @brief 获取延时后的状态
    const StateSnapshot* getDelayedState(uint64_t current_time_ns) const;
    
    /// @brief 获取最新状态
    const StateSnapshot* getLatestState() const;
    
    /// @brief 重置处理器状态
    void reset();
    
private:
    std::unique_ptr<SensorBuffer> buffer_;
    std::atomic<uint64_t> delay_ns_{0};
    size_t joint_count_;
    size_t motor_count_;
    
    // 噪声相关
    SensorNoiseConfig noise_config_;
    GaussianNoise noise_gen_;
    bool has_noise_{false};
};

}  // namespace MJ_BRIDGE

#endif  // SENSOR_PROCESSING_H
