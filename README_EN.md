# ROS Flutter GUI App

[中文](README.md) | [English](README_EN.md)

![GitHub last commit](https://img.shields.io/github/last-commit/chengyangkj/ROS_Flutter_Gui_App?style=flat-square)
![GitHub stars](https://img.shields.io/github/stars/chengyangkj/ROS_Flutter_Gui_App?style=flat-square)
![GitHub forks](https://img.shields.io/github/forks/chengyangkj/ROS_Flutter_Gui_App?style=flat-square)
![GitHub issues](https://img.shields.io/github/issues/chengyangkj/ROS_Flutter_Gui_App?style=flat-square)
<a href="http://qm.qq.com/cgi-bin/qm/qr?_wv=1027&k=mvzoO6tJQtu0ZQYa_itHW7JrT0i4OCdK&authKey=exOT53pUpRG85mwuSMstWKbLlnrme%2FEuJE0Rt%2Fw6ONNvfHqftoWMay03mk1Qi7yv&noverify=0&group_code=797497206">
<img alt="Static Badge" src="https://img.shields.io/badge/QQ%e7%be%a4-797497206-purple">
</a>

![web](https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/web_build.yaml/badge.svg)
![android](https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/android_build.yaml/badge.svg)
![linux](https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/linux_build.yaml/badge.svg)
![windows](https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/windows_build.yaml/badge.svg)

Cross-platform Flutter GUI human-machine interaction software implemented with ros bridge websocket. This project is integrated with CI to ensure multi-environment availability and automatically package multi-platform binary Release versions.

*Table of Contents* 
<!-- TOC -->

- [ROS Flutter GUI App](#ros-flutter-gui-app)
- [1. User Guide](#1-user-guide)
  - [1.1 Download Project Release Package](#11-download-project-release-package)
  - [1.2 Run Project](#12-run-project)
    - [1.2.1 Web End Operation Guide](#121-web-end-operation-guide)
  - [1.3 Robot End Environment Configuration](#13-robot-end-environment-configuration)
    - [ROS 1](#ros-1)
      - [Install rosbridge\_suite](#install-rosbridge_suite)
      - [Run rosbridge\_websocket](#run-rosbridge_websocket)
    - [ROS 2](#ros-2)
      - [Install rosbridge\_suite](#install-rosbridge_suite-1)
      - [Run rosbridge\_websocket](#run-rosbridge_websocket-1)
  - [1.4 Software Operation](#14-software-operation)
  - [1.5 Configuration Instructions](#15-configuration-instructions)
  - [1.6 Function Description](#16-function-description)
    - [1.6.1 Map Display](#161-map-display)
    - [1.6.2 Robot Position Display](#162-robot-position-display)
    - [1.6.3 Robot Speed Control](#163-robot-speed-control)
    - [1.6.4 Camera Image Display](#164-camera-image-display)
      - [ROS 1 Install web\_video\_server](#ros-1-install-web_video_server)
      - [ROS 2 Install web\_video\_server Guide](#ros-2-install-web_video_server-guide)
        - [1.5.3 Multi-point Navigation](#153-multi-point-navigation)
      - [Software Configuration](#software-configuration)
- [References](#references)
  - [License](#license)

*Project Screenshots*

![light](./doc/image/camera.png)

![main.gif](./doc/image/main.gif)
![mapping.gif](./doc/image/mapping.gif)

Features/TODO:
  
| Feature                      | Status | Remarks                                                                                                      |
| ---------------------------- | ------ | ------------------------------------------------------------------------------------------------------------ |
| ROS1/ROS2 Communication      | ✅      |                                                                                                              |
| Map Display                  | ✅      |                                                                                                              |
| Real-time Robot Position     | ✅      |                                                                                                              |
| Robot Speed Dashboard        | ✅      |                                                                                                              |
| Manual Robot Control         | ✅      |                                                                                                              |
| Robot Relocation             | ✅      |                                                                                                              |
| Single/Multi-point Navigation| ✅      |                                                                                                              |
| Global/Local Path Planning   | ✅      |                                                                                                              |
| Topological Map Function     | ❌      |                                                                                                              |
| Battery Level Display        | ✅      |                                                                                                              |
| Map Editing Function         | ❌      |                                                                                                              |
| Robot Navigation Task Chain  | ✅      |                                                                                                              |
| Map Loading                  | ❌      |                                                                                                              |
| Map Saving                   | ❌      |                                                                                                              |
| Camera Image Display         | ✅      | Depends on [web_video_server](https://github.com/RobotWebTools/web_video_server.git), not supported on web, others supported |
| Camera Image Display         | ✅      | Depends on [web_video_server](https://github.com/RobotWebTools/web_video_server.git), not supported on web, others supported |
| Robot Body Contour Display   | ❌      | Supports configuration of non-standard body shapes                                                            |
| Layer Camera View Adjustment | ✅      |                                                                                                              |

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=chengyangkj/Ros_Flutter_Gui_App&type=Timeline&theme=dark" />
  <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=chengyangkj/Ros_Flutter_Gui_App&type=Timeline" />
  <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=chengyangkj/Ros_Flutter_Gui_App&type=Timeline" width="75%" />
</picture>

# 1. User Guide
## 1.1 Download Project Release Package

Download the required version for your environment (windows, linux, web, android) from the [Release page](https://github.com/chengyangkj/ROS_Flutter_Gui_App/releases).

## 1.2 Run Project

Unzip the downloaded package. The app can be run directly after downloading. For the web version, a web server is needed. Here's a guide for using the web version:

### 1.2.1 Web End Operation Guide

Download the latest web version (ros_flutter_gui_app_web.tar.gz) from the [Release page](https://github.com/chengyangkj/ROS_Flutter_Gui_App/releases).  
Unzip it locally and deploy it using a web server like Apache.  

Navigate to the package directory:
```shell
cd ros_flutter_gui_app_web
```
Here, I use Python to set up a simple web server:

```shell
python -m http.server 8000
```
Since the port specified here is 8000, enter `localhost:8000` in Google Chrome (other browsers may not work and could show a blank page) to access the site.

## 1.3 Robot End Environment Configuration

The software uses ros bridge websocket to communicate with ROS, so you need to install and run ros bridge websocket on your robot system. Since ROS Bridge websocket is compatible with both ROS1 and ROS2, the installation guide is provided separately for ROS1 and ROS2.

### ROS 1

#### Install rosbridge_suite

1. **Ensure ROS 1 is installed** (e.g., ROS Melodic or ROS Noetic). If not, refer to the [ROS Installation Guide](http://wiki.ros.org/ROS/Installation).

2. **Install the `rosbridge_suite` package**:

   ```bash
   sudo apt-get install ros-<your-ros-distro>-rosbridge-suite
   ```

   Replace `<your-ros-distro>` with your ROS version, such as `melodic` or `noetic`.

#### Run rosbridge_websocket

1. **Start the ROS core**:

   ```bash
   roscore
   ```

2. **In a new terminal, start the rosbridge_websocket node**:

   ```bash
   roslaunch rosbridge_server rosbridge_websocket.launch
   ```

3. **Verify that rosbridge_websocket is running**:

   Open a browser and navigate to `http://localhost:9090`. If the connection is successful, the WebSocket server is up and running.

### ROS 2

#### Install rosbridge_suite

1. **Ensure ROS 2 is installed** (e.g., ROS Foxy, Galactic, or Humble). If not, refer to the [ROS 2 Installation Guide](https://docs.ros.org/en/foxy/Installation.html).

2. **Install the `rosbridge_suite` package**:

   ```bash
   sudo apt-get install ros-<your-ros2-distro>-rosbridge-suite
   ```

   Replace `<your-ros2-distro>` with your ROS 2 version, such as `foxy`, `galactic`, or `humble`.

3. **Source your ROS 2 environment in each new terminal session**:

   ```bash
   source /opt/ros/<your-ros2-distro>/setup.bash
   ```

#### Run rosbridge_websocket

1. **In a new terminal, start the rosbridge_websocket node**:

   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

2. **Verify that rosbridge_websocket is running**:

   Open a browser and navigate to `http://localhost:9090`. If the connection is successful, the WebSocket server is up and running.

## 1.4 Software Operation

Open the software and set the topics:

![setting_button](./doc/image/setting_button.png)

Settings interface:
![setting_button](./doc/image/setting_list.png)

## 1.5 Configuration Instructions
|Configuration Name|Message Type|Description|
|---|---|---|
|battery_topic|sensor_msgs/BatteryState|Topic for robot battery level, subscribed by the software |
|mapTopic|nav_msgs/OccupancyGrid|Topic name for robot map, subscribed by the software |
|laserTopic|sensor_msgs/LaserScan| Topic name for laser, subscribed by the software|
|localPathTopic|nav_msgs/Path|Topic name for robot local path, subscribed by the software |
|globalPathTopic|nav_msgs/Path|Topic name for robot global path, subscribed by the software |
|odomTopic|nav_msgs/Odometry|Topic name for robot odometry, subscribed by the software |
|relocTopic|geometry_msgs/PoseWithCovarianceStamped|Topic name for robot relocation, published by the software |
|navGoalTopic|geometry_msgs/PoseStamped|Topic name for robot navigation goal, published by the software |
|SpeedCtrlTopic|geometry_msgs/Twist|Topic name for robot speed control, published by the software|
|maxVx|double|Maximum vx speed for manual control in the software |
|maxVy|double|Maximum vy speed for manual control in the software |
|maxVw|double|Maximum vw speed for manual control in the software |
|mapFrameName|string|TF frame name for map coordinate system|
|baseLinkFrameName|string|TF frame name for robot base coordinate system|
|imagePort|string|Port for camera image web video server|
|imageTopic|string|Topic for the camera image to be displayed|
|imageWidth|int|Width of the camera image to be displayed, default is 640|
|imageHeight|int|Height of the camera image to be displayed, default is 480|
|ConfigTemplate|string|Template configuration, used for initialization during software upgrade and initialization, supports types ("turtlebot3:ros2","turtlebot3:ros1","jackal","turtlebot4","walking")|

After setting, click the connect button to connect to rosbridge_websocket. Once connected, the software will automatically subscribe to the set topics and display the topic data:
![connect](./images/connect.png)

## 1.6 Function Description

### 1.6.1 Map Display

The software will automatically subscribe to the set map topic, configuration item [mapTopic], and display the map data. The map data will be displayed in a 2D grid format on the interface. Clicking on a grid on the map will show the coordinates and value of the grid.

### 1.6.2 Robot Position Display

The software subscribes to ROS's tf, manually constructs the tf tree, and implements the tf2_dart class. Through the tf2_dart class, the robot's position on the map can be obtained and displayed on the interface.

### 1.6.3 Robot Speed Control

The software will automatically publish the set manual control speed, configuration item [SpeedCtrlTopic], and display the robot speed control data. Clicking the speed control button on the interface can control the robot's speed.
The left joystick can control the robot's speed, with the top left being the positive direction, the bottom right being the negative direction, and the middle being stop.
The right joystick can control both the robot's speed and rotation, with the top left being the positive direction, the bottom right being the negative direction, the left being left rotation, the right being right rotation, and the middle being stop.

### 1.6.4 Camera Image Display

Camera image display depends on the `web_video_server` package, which automatically converts all image topics in the system to mjpeg format HTTP video streams.

The following guide provides a reference method for installing and verifying this package in **ROS 1** and **ROS 2**.

#### ROS 1 Install web_video_server

1. **Install the `web_video_server` package**

   In **ROS 1**, run the following command to install the `web_video_server` package:

   ```bash
   sudo apt install ros-noetic-web-video-server
   ```

2. Start the camera node

Start your camera node to ensure there is an image topic.

3. Start the web_video_server

Start the web_video_server node, which will publish the image stream for web clients to access:

```bash
rosrun web_video_server web_video_server
```

4. Verify the video stream

Open the following link in a web browser to view the video stream (assuming the camera topic is /usb_cam/image_raw):

```bash
http://localhost:8080/stream?topic=/usb_cam/image_raw
```
If the image displays correctly, the web_video_server is successfully configured.

#### ROS 2 Install web_video_server Guide
Install the web_video_server package

In ROS 2, you need to compile web_video_server from source. First, ensure your workspace is initialized and set up:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/RobotWebTools/web_video_server.git
```
1. Install dependencies

Install the required dependencies:

```bash
cd ~/ros2_ws/
rosdep install --from-paths src --ignore-src -r -y
```

2. Compile the workspace

Use the colcon tool to compile the workspace:

```bash
colcon build
```

3. Start the camera node

Start your camera node to ensure there is an image topic.

4. Start the web_video_server node:

```bash
ros2 run web_video_server web_video_server
```
4. Verify the video stream

Open the following link in a web browser to view the video stream (assuming the camera topic is /usb_cam/image_raw):

```bash
http://localhost:8080/stream?topic=/usb_cam/image_raw
```
If the image displays correctly, the web_video_server is successfully configured.

##### 1.5.3 Multi-point Navigation

Long press the left navigation point button to manage waypoints and perform multi-point navigation.

#### Software Configuration

In the software, you need to configure the topic address to be displayed and the port of the web video server (if changed, it needs to be modified, default is 8080):

The following items need to be configured:
- imagePort: Port for the camera image web video server
- imageTopic: Topic for the camera image to be displayed
- imageWidth: Width of the camera image to be displayed, default is 640
- imageHeight: Height of the camera image to be displayed, default is 480

# References

- Some UI effects are referenced from [ros_navigation_command_app](https://github.com/Rongix/ros_navigation_command_app). Only the UI display effects are referenced; the code implementation in this repository is original.
- [roslibdart](https://pub.dev/packages/roslibdart), implements ROS bridge websocket communication in Flutter, allowing direct end-to-end communication with ROS.
- [matrix_gesture_detector](https://pub.dev/packages/matrix_gesture_detector), the software's gesture recognition is based on this pub package.

## License

This project is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. You may:

- **Share** — copy and redistribute the material in any medium or format
- **Adapt** — remix, transform, and build upon the material

Under the following terms:

- **Attribution** — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
- **NonCommercial** — You may not use the material for commercial purposes. Permission from the author is required.
- **ShareAlike** — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.

For commercial use, please contact the author for authorization.

For more details, please refer to the [LICENSE](./LICENSE) file.
