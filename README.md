# eir_locomotion_workspace

AIQ11 SPORT / PRO 的纯 RL 运控工作空间。C++ runtime 提供统一的仿真与硬件接口，控制应用支持 AMP、MIMIC 与 PRO 感知行走，不依赖 ros2_control。

```text
src/        runtime 子模块与深度消息源码
apps/       AIQ 控制应用子模块、遥控 GUI 源码
sdk/        头文件、预编译库、相机处理包和机器人资源
doc/        AIQ 使用说明、公共构建与交付文档
```

- [获取与构建](doc/公共/构建与SDK.md)
- [启动仿真](doc/AIQ/仿真使用.md)
- [相机接入](doc/AIQ/实机相机接入.md)
- [验证范围](doc/公共/验证记录.md) · [发布前测试需求](doc/AIQ/发布前测试需求.md)
- [许可说明](doc/公共/许可说明.md)

底层实现与硬件及安全机制紧密相关，通过 SDK 提供接口和库。此公开版本不包含内部实现、内部部署管理工具或 EQR 应用。自有开放源码采用 [BSD-3-Clause](LICENSE)，第三方组件保留原许可证。
