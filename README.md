
![web](https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/web_build.yaml/badge.svg)
![android](https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/android_build.yaml/badge.svg)
![linux](https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/linux_build.yaml/badge.svg)
![windows](https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/windows_build.yaml/badge.svg)

借助ros brdige websocket实现的跨平台flutter gui 人机交互软件
本项目已接入CI,保证多环境可用,并自动打包多平台二进制Release版本

目录 
<!-- TOC -->

- [一,ROS Bridge WebSocket 安装与运行指南](#一ros-bridge-websocket-安装与运行指南)
    - [ROS 1](#ros-1)
        - [安装 rosbridgesuite](#安装-rosbridgesuite)
        - [运行 rosbridgewebsocket](#运行-rosbridgewebsocket)
    - [ROS 2](#ros-2)
        - [安装 rosbridgesuite](#安装-rosbridgesuite)
        - [运行 rosbridgewebsocket](#运行-rosbridgewebsocket)
- [web端运行指南](#web端运行指南)

<!-- /TOC -->


# 一,ROS Bridge WebSocket 安装与运行指南

软件借助ros bridge websoct实现与ros之间的通信，因此需要先在自己的机器人系统上安装ros bridget websocket并运行，这里区分ros1 与 ros2分别介绍安装教程

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

# web端运行指南
从[Release界面](https://github.com/chengyangkj/ROS_Flutter_Gui_App/releases)下载最新的web端版本(ros_flutter_gui_app_web.tar.gz)  
解压到本地，借助Apache等网站服务器部署即可  

进入压缩包目录：
```shell
cd ros_flutter_gui_app_web
```
我这里采用python的来搭建一个简单的网站服务器：

```shell
python -m http.server 8000
```
由于这里指定的端口为8000，在浏览器输入`本机ip:8000`即可访问站点