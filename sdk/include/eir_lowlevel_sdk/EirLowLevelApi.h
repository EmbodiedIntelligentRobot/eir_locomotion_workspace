/**
 * @file EirLowLevelApi.h
 *
 * @brief 机器人底层 API 接口基类
 *
 * 这是一个纯接口类，定义了机器人硬件和仿真通用的底层接口。
 *
 * Copyright (c) 2025, EIR Co.Ltd. All rights reserved.
 */
#ifndef EIR_LOW_LEVEL_API_H
#define EIR_LOW_LEVEL_API_H

#include <functional>

#include "DataTypes.h"
#include "Visibility.h"

namespace EIR_Robotics {

/**
 * @brief 机器人底层 API 接口基类
 *
 * 定义了与机器人硬件交互的标准接口，支持仿真和实机两种实现。
 */
class PUBLIC_API EirLowLevelApi {
public:
    EirLowLevelApi() = default;
    virtual ~EirLowLevelApi() = default;

    // 禁止拷贝
    EirLowLevelApi(const EirLowLevelApi&) = delete;
    EirLowLevelApi& operator=(const EirLowLevelApi&) = delete;

    // ========== 核心接口 (纯虚函数) ==========

    /**
     * @brief 初始化
     * @param config_name 配置文件名，如"eqr02.yaml"或"aiq.yaml"
     * @return 初始化成功返回true，失败返回false
     */
    virtual bool Init(const std::string& config_name) = 0;

    /**
     * @brief 默认初始化
     * @return 初始化成功返回true，失败返回false
     */
    virtual bool Init() = 0;

    /**
     * @brief 订阅机器人状态
     * @param callback 状态回调函数
     */
    virtual void SubscribeRobotState(std::function<void(const RobotStatePtr)> callback) = 0;

    /**
     * @brief 发布机器人控制命令
     * @param cmd 控制命令
     * @return 发布成功返回true
     */
    virtual bool PublishRobotCmd(const RobotCmd& cmd) = 0;

    /**
     * @brief 紧急停止
     * @return 执行成功返回true
     */
    virtual bool Estop() = 0;

    // ========== 可选接口 (默认空实现) ==========

    /**
     * @brief 安全关闭所有资源
     * 
     * 在对象销毁前调用，用于清理资源。默认空实现，子类可按需重写。
     */
    virtual void Shutdown() {}

    virtual void SubscribeBMSState(std::function<void(const BmsStatePtr)> callback) {
        (void)callback;
    }
    virtual bool PublishBMSCmd(const BmsCmd& cmd) {
        (void)cmd;
        return false;
    }
    virtual void SubscriberEndEffectorState(std::function<void(const EEStatePtr)> callback) {
        (void)callback;
    }
    virtual bool PublishEndEffectorCmd(const EECmd& cmd) {
        (void)cmd;
        return false;
    }
    virtual void SubscribeHeadState(std::function<void(const HeadStatePtr)> callback) {
        (void)callback;
    }
    virtual bool PublishHeadCmd(const HeadCmd& cmd) {
        (void)cmd;
        return false;
    }
    // ========== 告警监控接口 ==========
    virtual void SubscribeAlarmState(std::function<void(const AlarmStatePtr)> callback)  // 订阅告警状态
    {
        (void)callback;
    }

    virtual void SubscriberJoystickState(std::function<void(const JoystickStatePtr)> callback) {
        (void)callback;
    }
    virtual void SubscriberDiagnosticStatus(std::function<void(const DiagnosticStatusPtr)> callback) {
        (void)callback;
    }
};

}  // namespace EIR_Robotics

#endif  // EIR_LOW_LEVEL_API_H