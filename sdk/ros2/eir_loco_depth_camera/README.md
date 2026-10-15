# 相机深度处理 SDK

平台：Linux x86_64、CPython 3.10、ROS 2 Humble；精确构建环境及 SHA256 见 `来源.json`。其他架构/Python ABI 需重新构建。

处理实现为 15 个原生 `.so` 模块。仅导入接口、日志桥接、节点入口、配置和 launch 以可读文件提供；不含开发机符号链接、`.egg-link`、生成的 C 或处理实现 `.py`。保留原 Apache-2.0，许可证位于 `share/eir_loco_depth_camera/LICENSE`。

先安装 ROS 2、numpy、OpenCV，并构建开放的 `src/depth_transport_msgs`，再加载环境：

```bash
source /opt/ros/humble/setup.bash
source /你的消息包安装目录/setup.bash
source /本SDK实际路径/setup.bash
ros2 pkg prefix eir_loco_depth_camera
ros2 launch eir_loco_depth_camera pro_perception.launch.py start_driver:=false
```

默认只订阅原图与 CameraInfo、处理和组帧，不打开相机、不启动机器人或 GUI。Ctrl+C 停止。

启用采集使用 `start_driver:=true`，默认绑定 USB UID `4-3`，不需要填写序列号；默认关闭网络设备枚举。通过 `usb_port:=实际USB_UID` 或本机环境变量 `EIR_DEPTH_CAMERA_USB_PORT` 覆盖默认值。用 `ros2 run orbbec_camera list_devices_node` 查看驱动报告的 USB UID，不使用 `/dev/videoN` 编号。

改用序列号时先将 `EIR_DEPTH_CAMERA_USB_PORT` 设为空，再传入 `serial_number:=实际序列号`，或设置 `EIR_DEPTH_CAMERA_SERIAL`。ROS 2 Humble 的 CLI 不接受空启动参数，用环境变量清空端口。同名启动参数覆盖环境变量；端口与序列号同时非空会报错。只有单相机自动选择场景才显式将两者都设为空；共享配置不写入单机序列号。

采集需另外提供官方 `orbbec_camera` 驱动及其配套 SDK。相机 SDK 不在本处理包内；本包不是 `ehr_depth_transport` 的同名替换。请使用上述包名与本项目的 `depth_transport_msgs/DepthStack` 接口，不复制参考工程的 `.egg-link`。

默认 `pro_perception.launch.py start_driver:=true` 和 `gemini336_raw_test.launch.py` 通过 `camera_supervisor` 启动官方采集进程。连续 5 秒没有时间戳递增、采集年龄不超过 1 秒的原始帧（包括启动后一直无帧），只停止并重新启动采集进程；宽限 5 秒，仍不退出才强制结束其进程组，组退出后等 1 秒重建。主控、处理和组帧节点不随之重启。监督使用单调时钟；不改写或重发原帧时间戳。主控原有的 100 ms 深度超时与故障锁定独立生效，相机恢复不会自动恢复运动。

停止时监督进程只向采集入口发送一次 SIGINT，ROS launch 负责转发；直接 SDK 采集节点在清理期间忽略重复 SIGINT/SIGTERM，保证线程和 Pipeline 清理不被再次打断。服务应由主进程逐级停止，使用 `KillMode=mixed` 并保留整组强制退出超时。外部驱动仍需在目标机验收；本包不能保证被外部直接反复发信号或被 SIGKILL 的驱动完成清理。

只用 `depth_transport.launch.py start_publisher:=true` 或 `ros2 run ... depth_camera_publisher` 的可选直接 SDK 路径有停止保护，但没有默认的 5 秒进程监督；标准实机入口使用上述官方驱动路径。已有服务不会因更新 SDK 自动改写 systemd 单元。

细节见工作空间 `doc/AIQ/相机SDK交付.md` 与 `doc/AIQ/实机相机接入.md`。
