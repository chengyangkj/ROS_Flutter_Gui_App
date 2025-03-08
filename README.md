<div align="center">

# ROS Flutter GUI App

[中文](#中文) | [English](README_EN.md)

<p align="center">
<img src="https://img.shields.io/github/last-commit/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub last commit"/>
<img src="https://img.shields.io/github/stars/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub stars"/>
<img src="https://img.shields.io/github/forks/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub forks"/>
<img src="https://img.shields.io/github/issues/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub issues"/>
<a href="http://qm.qq.com/cgi-bin/qm/qr?_wv=1027&k=mvzoO6tJQtu0ZQYa_itHW7JrT0i4OCdK&authKey=exOT53pUpRG85mwuSMstWKbLlnrme%2FEuJE0Rt%2Fw6ONNvfHqftoWMay03mk1Qi7yv&noverify=0&group_code=797497206"><img alt="QQ Group" src="https://img.shields.io/badge/QQ%e7%be%a4-797497206-purple"/></a>
</p>

<p align="center">
<img src="https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/web_build.yaml/badge.svg" alt="web"/>
<img src="https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/android_build.yaml/badge.svg" alt="android"/>
<img src="https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/linux_build.yaml/badge.svg" alt="linux"/>
<img src="https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/windows_build.yaml/badge.svg" alt="windows"/>
</p>

</div>

## 简介

ROS Flutter GUI App 是一个基于 Flutter 开发的跨平台 ROS 机器人人机交互界面，支持 ROS1/ROS2，可运行于 Android、iOS、Web、Linux、Windows 等多个平台。通过 rosbridge websocket 实现与 ROS 系统的通信。

### 主要特性

- 🌟 跨平台支持 - Android、iOS、Web、Linux、Windows
- 🤖 支持 ROS1/ROS2 
- 🗺️ 地图显示与导航功能
- 📹 相机图像显示
- 🎮 机器人遥控功能
- 🔋 电池状态监控
- 📍 多点导航任务
- 🛠️ 高度可配置

### 演示

![主界面](./doc/image/main.gif)
![建图](./doc/image/mapping.gif)

## 功能列表

| 功能           | 状态 | 备注                  |
| -------------- | ---- | --------------------- |
| ROS1/ROS2通信  | ✅    |                       |
| 地图显示       | ✅    |                       |
| 机器人位置显示 | ✅    |                       |
| 速度控制       | ✅    |                       |
| 重定位         | ✅    |                       |
| 单点/多点导航  | ✅    |                       |
| 规划轨迹显示   | ✅    |                       |
| 电池监控       | ✅    |                       |
| 相机显示       | ✅    | 需要 web_video_server |
| 地图编辑       | ❌    | 开发中                |
| 拓扑地图       | ❌    | 计划中                |

## 快速开始

### 安装

1. 从 [Release](https://github.com/chengyangkj/ROS_Flutter_Gui_App/releases) 下载对应平台的安装包

2. 安装 ROS 依赖:

```bash
# ROS1
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite

# ROS2
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
```

### 配置

1. 启动 rosbridge:

```bash
# ROS1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

2. 运行应用并配置连接参数

## 详细文档

- [安装指南](docs/installation.md) - 包含各平台的安装步骤和环境配置说明
- [配置说明](docs/configuration.md) - 详细的参数配置说明和默认值
- [使用教程](docs/usage.md) - 软件功能使用说明和最佳实践

## Star History

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=chengyangkj/Ros_Flutter_Gui_App&type=Timeline&theme=dark" />
  <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=chengyangkj/Ros_Flutter_Gui_App&type=Timeline" />
  <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=chengyangkj/Ros_Flutter_Gui_App&type=Timeline" width="75%" />
</picture>

## 贡献指南

欢迎提交 Issue 和 Pull Request。详见 [贡献指南](CONTRIBUTING.md)。

## 致谢

- [ros_navigation_command_app](https://github.com/Rongix/ros_navigation_command_app)
- [roslibdart](https://pub.dev/packages/roslibdart)
- [matrix_gesture_detector](https://pub.dev/packages/matrix_gesture_detector)

## 许可证

本项目采用 [CC BY-NC-SA 4.0](LICENSE) 许可证。
