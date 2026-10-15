# 获取与构建

以下命令在 Linux x86_64、Ubuntu 22.04 / ROS 2 Humble 环境执行。SDK 为现有 Linux x86_64 ABI 构建；其他平台需要匹配的 SDK。

## 获取源码

```bash
git lfs install
git clone --branch publish --recurse-submodules https://github.com/EmbodiedIntelligentRobot/eir_locomotion_workspace.git
cd eir_locomotion_workspace
git lfs pull
git submodule foreach 'git lfs pull'
```

子模块以顶层提交锁定的版本为准。不要用 `git submodule update --remote` 代替版本锁定。SDK、网格、PT 与 ONNX 模型使用 Git LFS，只有指针文件的目录不能运行。

## 只构建 runtime

需要 CMake ≥ 3.18、支持 C++20 的编译器、Eigen3、GLFW 和 GTest。公共 API 使用 C++17。MuJoCo 3.14.0 随 SDK 提供。

```bash
cmake -S . -B build/runtime -DCMAKE_BUILD_TYPE=Release \
  -DEIR_WITH_HARDWARE=OFF -DEIR_BUILD_CTRL=OFF
cmake --build build/runtime --parallel 3
ctest --test-dir build/runtime --output-on-failure
```

不构建测试时设置 `-DBUILD_TESTING=OFF`，不再需要 GTest。runtime 本身不依赖 ROS。

## 构建 AIQ 控制应用

另需 ROS 2 Humble、colcon、yaml-cpp、OpenSSL、PyYAML。GUI 需要 PyQt5，深度显示需要 rqt_image_view。双后端另需与系统 C++ ABI 匹配的 LibTorch；本项目验证版本为 2.7.0 CPU。

先从随包源码构建消息接口，不要同时加载另一套同名消息包：

```bash
source /opt/ros/humble/setup.bash
colcon --log-base build/interfaces-log build \
  --base-paths apps/eir_ctrl_demo_aiq11/interfaces src/depth_transport_msgs \
  --build-base build/interfaces-build --install-base build/interfaces \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source build/interfaces/setup.bash
```

ONNX 单后端不需要另行安装 LibTorch：

```bash
cmake -S . -B build/aiq -DCMAKE_BUILD_TYPE=Release \
  -DEIR_BUILD_CTRL=ON -DEIR_WITH_HARDWARE=OFF \
  -DEIR_CTRL_WITH_TORCH=OFF -DEIR_CTRL_WITH_DEPTH_ROS=ON \
  -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build build/aiq --parallel 3
```

双后端将 `EIR_CTRL_WITH_TORCH` 设为 `ON`，并传入 `-DTorch_DIR=/实际LibTorch路径/share/cmake/Torch`。SDK 已包含 ONNX Runtime 1.23.2；运行后端由 YAML 指定，缺少对应后端时拒绝启动，不自动回退。

本项目不自动安装系统依赖。模型、配置和控制参数保持随包版本。首次运行见 [仿真使用](../AIQ/仿真使用.md)。

## SDK 边界

`sdk/include` 是公共接口，`sdk/lib` 是预编译库，`sdk/share` 是模型和标定资源，`sdk/ros2` 是相机处理交付包。`eir_lowlevel_sdk` 为头文件接口，没有单独的 `.so`。

公开构建和运行无需内部源码或内网 Git 仓库。SDK 的安装不随 runtime 自动完成；搬迁交付时须同时保留 SDK、策略、模型和配置的相对目录。头文件、库及下游应用必须成套更新，不能混用新头文件与旧动态库。

硬件编译可设置 `-DEIR_WITH_HARDWARE=ON`；实际部署仍需目标机的已校准配置及独立验收，不直接沿用仿真参数上机。
