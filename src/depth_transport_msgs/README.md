# depth_transport_msgs

保留原有 `DepthStack` 接口（来源提交 `b4b34c5`，Apache-2.0）。

数据按 `[frame,y,x]` 排列，帧由旧到新，已归一化到 [0,1]；`header.stamp` 是最新帧的原始采集时间。尺寸、处理版本与历史间隔必须由模型配置共同确定，不能仅根据消息名称判断兼容性。

当前接入见 [相机接入文档](../../doc/AIQ/实机相机接入.md)。
