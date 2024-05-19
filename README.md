

借助ros brdige websocket实现的跨平台flutter gui 人机交互软件

目录 
<!-- TOC -->

- [一,ROS Bridge WebSocket 安装与运行指南](#%E4%B8%80ros-bridge-websocket-%E5%AE%89%E8%A3%85%E4%B8%8E%E8%BF%90%E8%A1%8C%E6%8C%87%E5%8D%97)
    - [ROS 1](#ros-1)
        - [安装 rosbridge_suite](#%E5%AE%89%E8%A3%85-rosbridge_suite)
        - [运行 rosbridge_websocket](#%E8%BF%90%E8%A1%8C-rosbridge_websocket)
    - [ROS 2](#ros-2)
        - [安装 rosbridge_suite](#%E5%AE%89%E8%A3%85-rosbridge_suite)
        - [运行 rosbridge_websocket](#%E8%BF%90%E8%A1%8C-rosbridge_websocket)

<!-- /TOC -->


# 一,ROS Bridge WebSocket 安装与运行指南

## ROS 1

### 安装 rosbridge_suite

1. **确保已安装 ROS 1**（例如：ROS Melodic 或 ROS Noetic）。如果没有，请参考 [ROS 安装指南](http://wiki.ros.org/ROS/Installation) 进行安装。

2. **安装 `rosbridge_suite` 包**：

   ```bash
   sudo apt-get install ros-<your-ros-distro>-rosbridge-suite
   ```

   将 `<your-ros-distro>` 替换为你的 ROS 版本，例如 `melodic` 或 `noetic`。

### 运行 rosbridge_websocket

1. **启动 ROS 核心**：

   ```bash
   roscore
   ```

2. **在新的终端中，启动 rosbridge_websocket 节点**：

   ```bash
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```

3. **验证 rosbridge_websocket 是否正在运行**：

   打开浏览器，导航到 `http://localhost:9090`，如果连接成功，说明 WebSocket 服务器已启动并运行。

## ROS 2

### 安装 rosbridge_suite

1. **确保已安装 ROS 2**（例如：ROS Foxy、Galactic 或 Humble）。如果没有，请参考 [ROS 2 安装指南](https://docs.ros.org/en/foxy/Installation.html) 进行安装。

2. **安装 `rosbridge_suite` 包**：

   ```bash
   sudo apt-get install ros-<your-ros2-distro>-rosbridge-suite
   ```

   将 `<your-ros2-distro>` 替换为你的 ROS 2 版本，例如 `foxy`、`galactic` 或 `humble`。

3. **在每个新的终端会话中，source 你的 ROS 2 环境**：

   ```bash
   source /opt/ros/<your-ros2-distro>/setup.bash
   ```

### 运行 rosbridge_websocket

1. **在新的终端中，启动 rosbridge_websocket 节点**：

   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

2. **验证 rosbridge_websocket 是否正在运行**：

   打开浏览器，导航到 `http://localhost:9090`，如果连接成功，说明 WebSocket 服务器已启动并运行。

---

通过上述步骤，你可以在 ROS 1 和 ROS 2 环境中安装并运行 rosbridge WebSocket 服务器。如果你有任何问题或需要进一步的帮助，请参考 [ROS 官方文档](http://wiki.ros.org/) 或 [ROS 2 官方文档](https://docs.ros.org/en/foxy/index.html)。

---
