# 相机 SDK 交付

保留完整 `sdk/ros2/eir_loco_depth_camera`，不能只复制其中的 `.so`。依赖、启动及 USB 参数见 [SDK README](../../sdk/ros2/eir_loco_depth_camera/README.md)；接口与策略边界见 [相机接入](实机相机接入.md)。

该包只提供接口、节点入口、配置、许可证和原生扩展；处理实现源码不在公开版本中。CPU 架构、Python ABI、ROS 和系统库必须与构建环境兼容。
