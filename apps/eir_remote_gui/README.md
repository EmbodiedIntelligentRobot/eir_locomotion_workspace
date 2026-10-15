# eir_remote_gui

随公开 workspace 提供的轻量 ROS 2 遥控 GUI，保留原 BSD-3-Clause 许可证。模式、速度范围和话题从 AIQ 控制配置读取；不包含控制器或部署管理工具。

依赖 ROS 2 Humble、随控制器提供的 ehr_message、PyQt5 和 PyYAML。深度显示另需 rqt_image_view。

```bash
source /opt/ros/humble/setup.bash
source build/interfaces/setup.bash
/usr/bin/python3 apps/eir_remote_gui/remote_gui.py --config apps/eir_ctrl_demo_aiq11/config/pro.yaml
```

可视化仿真默认自动打开 GUI。摇杆横轴控制旋转，下方滑块控制侧移；自动回零可选，窗口失焦不主动停止发布。输入接管与反馈异常处理沿用控制配置和现有保护。进入感知模式后打开深度显示窗口，退出模式或 GUI 时清理所属窗口。

可选本地 CSV 回放需另行安装 motion_stream_tools 及所需消息；缺少该依赖不影响基础模式和速度控制。

来源与验证范围见 workspace 的 doc/公共/公开发布维护.md 和 doc/公共/验证记录.md。
