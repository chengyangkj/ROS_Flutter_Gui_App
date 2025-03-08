# Installation Guide

## Download Package

Download the latest version for your platform from [Release Page](https://github.com/chengyangkj/ROS_Flutter_Gui_App/releases):

- Windows: `ros_flutter_gui_app_windows.zip`
- Linux: `ros_flutter_gui_app_linux.tar.gz`
- Android: `ros_flutter_gui_app_android.apk`
- Web: `ros_flutter_gui_app_web.tar.gz`

## Requirements

### ROS Environment

- ROS1 (Melodic/Noetic) or ROS2 (Foxy/Galactic/Humble)
- rosbridge_suite
- web_video_server (optional, for camera display)

### Install Dependencies

#### ROS1

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
sudo apt install ros-${ROS_DISTRO}-web-video-server  # optional
```

#### ROS2

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
# web_video_server needs to be built from source
cd ~/ros2_ws/src
git clone https://github.com/RobotWebTools/web_video_server.git
cd ~/ros2_ws
colcon build
```

## Platform-Specific Instructions

### Web Version Deployment

1. Extract the web version package
2. Start a quick HTTP server using Python:
```bash
cd ros_flutter_gui_app_web
python -m http.server 8000
```
3. Visit `http://localhost:8000`

### Android Version

Simply install the APK file.

### Linux/Windows Version

Extract and run the executable file. 