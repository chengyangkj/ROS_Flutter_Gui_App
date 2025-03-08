# 安装指南

## 下载安装包

从 [Release页面](https://github.com/chengyangkj/ROS_Flutter_Gui_App/releases) 下载对应平台的最新版本:

- Windows: `ros_flutter_gui_app_windows.zip`
- Linux: `ros_flutter_gui_app_linux.tar.gz`
- Android: `ros_flutter_gui_app_android.apk`
- Web: `ros_flutter_gui_app_web.tar.gz`

## 环境要求

### ROS环境

- ROS1 (Melodic/Noetic) 或 ROS2 (Foxy/Galactic/Humble)
- rosbridge_suite
- web_video_server (可选,用于相机显示功能)

### 安装依赖

#### ROS1

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
sudo apt install ros-${ROS_DISTRO}-web-video-server  # 可选
```

#### ROS2

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
# web_video_server需要从源码编译
cd ~/ros2_ws/src
git clone https://github.com/RobotWebTools/web_video_server.git
cd ~/ros2_ws
colcon build
```

## 平台特定说明

### Web版本部署

1. 解压web版本压缩包
2. 使用Python快速启动HTTP服务:
```bash
cd ros_flutter_gui_app_web
python -m http.server 8000
```
3. 访问 `http://localhost:8000`

### Android版本

直接安装APK文件即可。

### Linux/Windows版本

解压后运行可执行文件即可。 