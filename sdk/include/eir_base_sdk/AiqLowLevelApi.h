/**
 * @file AiqLowLevelApi.h
 *
 * @brief EtherCAT 底层 API 实现类
 *
 * 继承 EirLowLevelApi 接口，实现实机 EtherCAT 通信。
 *
 * Copyright (c) 2025, EIR Co.Ltd. All rights reserved.
 */
#ifndef AIQ_LOW_LEVEL_API_H
#define AIQ_LOW_LEVEL_API_H

#include "eir_lowlevel_sdk/EirLowLevelApi.h"
#include <memory>
#include <cstdint>
#include <string>

namespace EIR_Robotics {

// 前向声明
class AiqLowLevelApiImpl;

//  模块定义 (ModuleCode) - 8 bits
typedef enum {
    MODULE_SALVE = 0x01,  // 01: 从站
    MODULE_MOTOR = 0x02,  // 02: 电机
    MODULE_IMU = 0x03, // 03: 陀螺仪
    MODULE_REMOTE = 0x04,  // 04: 遥控器
    MODULE_BATTERY = 0x05,  // 05: 电池
    MODULE_FORCESENSOR = 0x06, // 06: 六维力传感器
    MODULE_HAND = 0x07,  // 07: 灵巧手
    MODULE_ULTRASONIC = 0x08,  // 08: 超声波
    MODULE_ENCODER = 0x09, // 09: 编码器
} ModuleCode_t;

//  状态码定义 (Status Code) - 12 bits
typedef enum {
    STATUS_OVERHEAT = 0x001, // 001: 过热
    STATUS_OVERCURRENT = 0x002, // 002: 过流
    STATUS_OVERVOLTAGE = 0x003, // 001: 过压
    STATUS_UNDERVOLTAGE = 0x004, // 002: 欠压
    STATUS_ENCODERERROR = 0x005, // 001: 编码器错误
    STATUS_STOPOVERVOL = 0x006, // 002: 刹车过压
    STATUS_DRIVEERROR = 0x007, // 001: 驱动错误
    STATUS_OFFLINE = 0x00A, // 002: 离线
    STATUS_NORMAL   = 0x000, // 正常
    // ... 其他故障
} MotorStatusCode_t;

//  等级定义 (Level) - 4 bits
typedef enum {
    LEVEL_NORMAL = 0, // 0: Normal
    LEVEL_INFO   = 1, // 1: INFO
    LEVEL_WARN   = 2, // 2: WARN
    LEVEL_ERROR  = 3, // 3: ERROR
    LEVEL_FATAL  = 4, // 4: FATAL
} Level_t;

#define OTHER_DEVICE_OFFLINE 0X01


/**
 * @brief 实时性能统计
 */
struct RealtimeStats {
    uint64_t sample_count;          // 采样数量
    uint32_t period_min_ns;         // 最小周期 (ns)
    uint32_t period_max_ns;         // 最大周期 (ns)
    double period_avg_ns;           // 平均周期 (ns)
    double period_jitter_ns;        // 周期抖动 (ns, 标准差)
    uint32_t exec_min_ns;           // 最小执行时间 (ns)
    uint32_t exec_max_ns;           // 最大执行时间 (ns)
    double exec_avg_ns;             // 平均执行时间 (ns)
    uint32_t latency_min_ns;        // 最小唤醒延迟 (ns)
    uint32_t latency_max_ns;        // 最大唤醒延迟 (ns)
    double latency_avg_ns;          // 平均唤醒延迟 (ns)
    double latency_jitter_ns;       // 延迟抖动 (ns, 标准差)
    uint64_t overrun_count;         // 超时次数
};

/**
 * @brief Wi-Fi 接入点信息
 */
struct ApInfo {
    std::string ssid;
    int rssi_dbm = 0;
    std::string bssid;
};

/**
 * @brief 无线网卡信息
 */
struct NetInfo {
    std::string interface_name;
    std::string ip;
    std::string mac;
};

/**
 * @brief 网络信息后台连接状态
 */
enum class NetworkConnectionState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED
};

/**
 * @brief 网络信息缓存快照
 *
 * SDK 后台连接固定目标 192.168.2.254（root），每 2 秒查询 apclix0。
 * 查询失败时保留最后一次有效数据，并通过 stale 和 error_message 报告状态。
 */
struct NetworkInfoSnapshot {
    bool valid = false;
    bool stale = true;
    NetworkConnectionState connection_state = NetworkConnectionState::DISCONNECTED;
    int64_t last_success_timestamp_ms = 0;
    std::string error_message;
    ApInfo ap_info;
    NetInfo net_info;
};

/**
 * @brief EtherCAT 底层 API 实现类
 *
 * 继承 EirLowLevelApi 接口，负责实时 EtherCAT 通信、电机控制和状态管理。
 */
class PUBLIC_API AiqLowLevelApi : public EirLowLevelApi {
public:
    AiqLowLevelApi();
    ~AiqLowLevelApi() override;

    // 禁止拷贝
    AiqLowLevelApi(const AiqLowLevelApi&) = delete;
    AiqLowLevelApi& operator=(const AiqLowLevelApi&) = delete;

    // ========== EirLowLevelApi 接口实现 ==========
    bool Init(const std::string& config_name) override;
    bool Init() override;

    void SubscribeRobotState(std::function<void(const RobotStatePtr)> callback) override;
    bool PublishRobotCmd(const RobotCmd& cmd) override;
    bool Estop() override;

    void SubscribeBMSState(std::function<void(const BmsStatePtr)> callback) override;
    bool PublishBMSCmd(const BmsCmd& cmd) override;
    void SubscriberEndEffectorState(std::function<void(const EEStatePtr)> callback) override;
    bool PublishEndEffectorCmd(const EECmd& cmd) override;
    void SubscribeHeadState(std::function<void(const HeadStatePtr)> callback) override;
    bool PublishHeadCmd(const HeadCmd& cmd) override;

    // ========== 实机特有接口 ==========
    void Shutdown();                            // 安全关闭所有资源
    uint64_t GetAlStateChangeCount() const;     // 获取 AL 状态变化次数
    NetworkInfoSnapshot GetNetworkInfo() const; // 获取后台线程缓存的网络信息，不阻塞 SSH
    
    // ========== 实时性能监控接口 ==========
    void SubscribeRealtimeStats(std::function<void(const RealtimeStats&)> callback);  // 订阅实时性能统计
    void ResetRealtimeStats();                  // 重置性能统计

    // ========== 告警监控接口 ==========
    void SubscribeAlarmState(std::function<void(const AlarmStatePtr)> callback) override;  // 订阅告警状态

private:
    std::unique_ptr<AiqLowLevelApiImpl> impl_;  // PIMPL 实现指针
};

}  // namespace EIR_Robotics

#endif  // AIQ_LOW_LEVEL_API_H
