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

借助ros bridge websocket实现的跨平台flutter gui 人机交互软件
本项目已接入CI,保证多环境可用,并自动打包多平台二进制Release版本

*目录* 
<!-- TOC -->

- [一，使用教程](#一使用教程)
    - [1.1 下载项目 Release 包](#11-下载项目-release-包)
    - [1.2 运行项目](#12-运行项目)
        - [1.2.1 web端运行指南](#121-web端运行指南)
    - [1.3 机器人端环境配置](#13-机器人端环境配置)
        - [ROS 1](#ros-1)
            - [安装 rosbridgesuite](#安装-rosbridgesuite)
            - [运行 rosbridgewebsocket](#运行-rosbridgewebsocket)
        - [ROS 2](#ros-2)
            - [安装 rosbridgesuite](#安装-rosbridgesuite)
            - [运行 rosbridgewebsocket](#运行-rosbridgewebsocket)
    - [1.4 软件运行](#14-软件运行)
    - [1.5 功能说明](#15-功能说明)
        - [1.5.1 地图显示](#151-地图显示)
        - [1.5.2 机器人位置显示](#152-机器人位置显示)
        - [1.5.3 机器人速度控制](#153-机器人速度控制)
        - [1.5.2 相机图像显示](#152-相机图像显示)
            - [ROS 1 安装webvideoserver教程](#ros-1-安装webvideoserver教程)
            - [ROS 2 安装webvideoserver教程](#ros-2-安装webvideoserver教程)
            - [软件配置](#软件配置)
- [引用](#引用)

<!-- /TOC -->


*项目截图*

![light](./doc/image/camera.png)

![main.gif](./doc/image/main.gif)
![mapping.gif](./doc/image/mapping.gif)

功能/TODO:
  
| 功能                        | 状态 | 备注                 |
| --------------------------- | ---- | -------------------- |
| ROS1/ROS2通信               | ✅    |                      |
| 地图显示           | ✅    |                      |
| 机器人实时位置显示          | ✅    |                      |
| 机器人速度仪表盘            | ✅     |                      |
| 机器人手动控制              | ✅    |                      |
| 机器人重定位                | ✅    |                      |
| 机器人单点/多点导航         |  ✅    |                      |
| 机器人全局/局部规划轨迹显示 | ✅    |                      |
| 机器人拓扑地图功能          | ❌    |                      |
| 电池电量显示                | ✅     |                      |
| 地图编辑功能                | ❌    |                      |
| 机器人导航任务链            |❌    |               |
| 地图加载                    | ❌    |                      |
| 地图保存                    | ❌    |                      |
| 相机图像显示                | ✅    |  依赖[web_video_server](https://github.com/RobotWebTools/web_video_server.git) 暂不支持web端使用，其余均支持|
| 机器人车身轮廓显示          | ❌    | 支持配置异形车身     |
| 图层相机视角调整          | ✅     |      |

  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=chengyangkj/Ros_Flutter_Gui_App&type=Timeline&theme=dark" />
    <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=chengyangkj/Ros_Flutter_Gui_App&type=Timeline" />
    <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=chengyangkj/Ros_Flutter_Gui_App&type=Timeline" width="75%" />
  </picture>

# 一，使用教程
## 1.1 下载项目 Release 包

从[Release界面](https://github.com/chengyangkj/ROS_Flutter_Gui_App/releases) 下载所需要的对应环境的版本（windows，linux，web，android）

## 1.2 运行项目

解压下载的压缩包，APP端下载后即可运行，web端运行时需要借助网站服务器这里介绍下web端使用：

### 1.2.1 web端运行指南

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
由于这里指定的端口为8000，在google浏览器(其他浏览器未测试，可能出现空白界面问题)输入`本机ip:8000`即可访问站点

## 1.3 机器人端环境配置

软件借助ros bridge websocket实现与ros之间的通信，因此需要先在自己的机器人系统上安装ros bridget websocket并运行，由于ROS Bridge websocket的实现兼容ros1与ros2，因此这里区分ros1 与 ros2分别介绍安装教程
 
### ROS 1

#### 安装 rosbridge_suite

1. **确保已安装 ROS 1**（例如：ROS Melodic 或 ROS Noetic）。如果没有，请参考 [ROS 安装指南](http://wiki.ros.org/ROS/Installation) 进行安装。

2. **安装 `rosbridge_suite` 包**：

   ```bash
   sudo apt-get install ros-<your-ros-distro>-rosbridge-suite
   ```

   将 `<your-ros-distro>` 替换为你的 ROS 版本，例如 `melodic` 或 `noetic`。

#### 运行 rosbridge_websocket

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

### ROS 2

#### 安装 rosbridge_suite

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

#### 运行 rosbridge_websocket

1. **在新的终端中，启动 rosbridge_websocket 节点**：

   ```bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

2. **验证 rosbridge_websocket 是否正在运行**：

   打开浏览器，导航到 `http://localhost:9090`，如果连接成功，说明 WebSocket 服务器已启动并运行。

## 1.4 软件运行

打开软件，进行topic设置：

![setting_button](./doc/image/setting_button.png)

设置界面：
![setting_button](./doc/image/setting_list.png)

配置说明
|配置名|消息类型|说明|
|---|---|---|
|battery_topic|sensor_msgs/BatteryState|机器人电池电量的topic，软件订阅 |
|mapTopic|nav_msgs/OccupancyGrid|机器人地图话题名，软件订阅 |
|laserTopic|sensor_msgs/LaserScan| 激光话题名，软件订阅|
|localPathTopic|nav_msgs/Path|机器人局部路径话题名，软件订阅 |
|globalPathTopic|nav_msgs/Path|机器人全局路径话题名，软件订阅 |
|odomTopic|nav_msgs/Odometry|机器人里程计话题名，软件订阅 |
|relocTopic|geometry_msgs/PoseWithCovarianceStamped|机器人重定位topic名，软件发布 |
|navGoalTopic|geometry_msgs/PoseStamped|机器人导航目标点话题名，软件发布 |
|SpeedCtrlTopic|geometry_msgs/Twist|机器人速度控制话题名，软件发布|
|maxVx|double|软件手动控制时最大vx速度 |
|maxVydouble|软件手动控制时最大vy速度 |
|maxVw|double|软件手动控制时最大vw速度 |
|mapFrameName|string|地图坐标系tf fram名|
|baseLinkFrameName|string|机器人底盘坐标系tf fram名|
|imagePort|string|相机图像web video server 服务器短裤|
|imageTopic|string|要展示的相机图像的topic|
|imageWidth|int|要展示的相机图像的宽默认640|
|imageHeight|int|要展示的相机图像的高默认480|

设置完成后，点击connect按钮，连接到rosbridge_websocket，连接成功后，软件会自动订阅设置的topic，并显示topic的数据：
![connect](./images/connect.png)

## 1.5 功能说明

### 1.5.1 地图显示

软件会自动订阅设置的地图topic，配置项[mapTopic]，并显示地图数据，地图数据会以2D栅格的形式显示在界面上，点击地图上的栅格，会显示栅格的坐标和栅格的值。

### 1.5.2 机器人位置显示

软件订阅ros的tf，手动构建tf树，实现tf2_dart类，通过tf2_dart类，可以获取机器人在地图上的位置，并显示在界面上。

### 1.5.3 机器人速度控制

软件会自动发布设置的手动控制速度，配置项[SpeedCtrlTopic]，并显示机器人速度控制数据，点击界面上的速度控制按钮，可以控制机器人的速度。
左侧遥感可以控制机器人的速度，遥感左上角为正方向，遥感右下角为负方向，遥感中间为停止。
右侧遥感既可控制机器人速度，又可控制机器人旋转，遥感左上角为正方向，遥感右下角为负方向，左侧为向左旋转，右侧向右旋转，遥感中间为停止。


### 1.5.2 相机图像显示

相机图像显示依赖 `web_video_server` 包，这个包会自动将系统中所有的图像topic转换为mjpeg格式的http视频流

以下教程是在 **ROS 1** 和 **ROS 2** 中安装和验证该包的参考方法。

#### ROS 1 安装web_video_server教程

1. **安装 `web_video_server` 包**

   在 **ROS 1** 中，运行以下命令安装 `web_video_server` 包：

   ```bash
   sudo apt install ros-noetic-web-video-server
   ```

2. 启动相机节点

启动自己的相机节点，确保有图片topic

3. 启动 web_video_server

启动 web_video_server 节点，它将发布图像流供 Web 客户端访问：

```bash
rosrun web_video_server web_video_server
```

4. 验证视频流

在 Web 浏览器中打开以下链接，查看视频流（假设相机话题为 /usb_cam/image_raw）：

```bash
http://localhost:8080/stream?topic=/usb_cam/image_raw
```
如果图像正常显示，则 web_video_server 已成功配置。

#### ROS 2 安装web_video_server教程
安装 web_video_server 包

在 ROS 2 中，您需要从源代码编译 web_video_server。首先，确保您的工作空间已经初始化并设置：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/RobotWebTools/web_video_server.git
```
1. 安装依赖

安装所需的依赖：

```bash
cd ~/ros2_ws/
rosdep install --from-paths src --ignore-src -r -y
```

2. 编译工作空间

使用 colcon 工具编译工作空间：

```bash
colcon build
```

3. 启动相机节点

启动自己的相机节点，确保有图片topic

4. 启动 web_video_server 节点：

```bash
ros2 run web_video_server web_video_server
```
4. 验证视频流

在 Web 浏览器中打开以下链接，查看视频流（假设相机话题为 /usb_cam/image_raw）：

```bash
http://localhost:8080/stream?topic=/usb_cam/image_raw
```
如果图像正常显示，则 web_video_server 已成功配置。

#### 软件配置

在软件中需要配置要显示的话题topic地址，以及web video server的端口（如果更改的话需要修改，默认是8080:

需要配置如下两项：
- imagePort 相机图像web video server 服务器端口
- imageTopic 要展示的相机图像的topic
- imageWidth 要展示的相机图像的宽默认640
- imageHeight 要展示的相机图像的高默认480

# 引用

- 部分UI界面效果参考自[ros_navigation_command_app](https://github.com/Rongix/ros_navigation_command_app)，仅参考UI显示效果，本仓库的代码的实现均为原创
- [roslibdart](https://pub.dev/packages/roslibdart)，实现flutter 中的ros bridge websocket的通信，借助此库可以直接与ros进行端对端通信
- [matrix_gesture_detector](https://pub.dev/packages/matrix_gesture_detector) 软件的手势识别在此pub包的基础上做更改
