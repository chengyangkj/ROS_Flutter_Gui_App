<div align="center">

# ROS Flutter GUI App

[中文](#中文) | [English](README_EN.md)

<p align="center">
<img src="https://img.shields.io/github/last-commit/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub last commit"/>
<img src="https://img.shields.io/github/stars/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub stars"/>
<img src="https://img.shields.io/github/forks/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub forks"/>
<img src="https://img.shields.io/github/issues/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub issues"/>
<a href="http://qm.qq.com/cgi-bin/qm/qr?_wv=1027&k=mvzoO6tJQtu0ZQYa_itHW7JrT0i4OCdK&authKey=exOT53pUpRG85mwuSMstWKbLlnrme%2FEuJE0Rt%2Fw6ONNvfHqftoWMay03mk1Qi7yv&noverify=0&group_code=797497206"><img alt="QQ Group" src="https://img.shields.io/badge/QQ%e7%be%a4-797497206-purple"/></a>
<img src="https://img.shields.io/badge/Flutter-3.27.4-blue?style=flat-square" alt="Flutter Version"/>
</p>

<p align="center">
<img src="https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/web_build.yaml/badge.svg" alt="web"/>
<img src="https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/android_build.yaml/badge.svg" alt="android"/>
<img src="https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/linux_build.yaml/badge.svg" alt="linux"/>
<img src="https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/windows_build.yaml/badge.svg" alt="windows"/>
</p>


</div>

## 简介

ROS Flutter GUI App 是一个基于 Flutter Flame 游戏框架开发的跨平台 ROS 机器人人机交互界面，支持 ROS1/ROS2，可运行于 Android、iOS、Web、Linux、Windows 等多个平台。通过 rosbridge websocket 实现与 ROS 系统的通信。

### 主要特性

- 🌟 跨平台支持 - Android、iOS、Web、Linux、Windows
- 🤖 支持 ROS1/ROS2 
- 🗺️ 地图显示与导航功能
- 📹 相机图像显示
- 🎮 机器人遥控功能
- 🔋 电池状态监控
- 📍 多点导航任务
- 🛠️ 高度可配置

### TODO:

- 大地图无法显示问题: 需订阅解析rosbridge cbor-raw压缩数据,解析原始cdr数据
- 大地图显示卡顿优化

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
| 相机显示       | ✅    | 需要 web_video_server，不支持web |
| 地图编辑       | ✅    |                 |
| 拓扑地图       | ✅    |                 |
| 诊断信息显示    | ✅    | /diagnostics topic制表显示及错误Toast弹窗 |

## 快速开始

### 下载安装包

从 [Release页面](https://github.com/chengyangkj/ROS_Flutter_Gui_App/releases) 下载对应平台的最新版本:

- Windows: `ros_flutter_gui_app_windows.zip`
- Linux: `ros_flutter_gui_app_linux.tar.gz`
- Android: `ros_flutter_gui_app_android.apk`
- Web: `ros_flutter_gui_app_web.tar.gz`

### 环境要求

#### ROS环境

- ROS1 (Melodic/Noetic) 或 ROS2 (Foxy/Galactic/Humble)
- rosbridge_suite
- web_video_server (可选,用于相机显示功能)

#### 安装依赖

**ROS1**

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
sudo apt install ros-${ROS_DISTRO}-web-video-server  # 可选
```

**ROS2**

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-suite
# web_video_server需要从源码编译
cd ~/ros2_ws/src
git clone https://github.com/RobotWebTools/web_video_server.git
cd ~/ros2_ws
colcon build
```

### 平台特定说明

#### Web版本部署

1. 解压web版本压缩包
2. 使用Python快速启动HTTP服务:
```bash
cd ros_flutter_gui_app_web
python -m http.server 8000
```
3. 访问 `http://localhost:8000`

#### Android版本

直接安装APK文件即可。

#### Linux/Windows版本

解压后运行可执行文件即可。

### 配置

1. 启动 **rosbridge** (机器人侧):

```bash
# ROS1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

2. 启动 **ros_gui_backend**（机器人侧）

自 v2.0 起，大地图通过 HTTP 瓦片加载，需与 [ros_gui_backend](https://github.com/chengyangkj/ros_gui_backend) 配套使用（仅需 rosbridge 的旧体验请使用 v1.x）。


在机器人（或同一局域网能订阅到 `/map` 的机器）上安装运行，以后端仓库 [README](https://github.com/chengyangkj/ros_gui_backend/blob/main/README.md) 为准；典型流程如下。

**编译安装**（仓库根目录，需已安装对应 ROS 并 `source` 其 `setup.bash`）：

```bash
sudo apt-get install libsdl2-dev libsdl2-image-dev -y
./build.sh
```

产物在 `build/install/`，可执行文件一般在 `build/install/bin/` 下

**启动** ：

```bash
cd build/install/bin
export LD_LIBRARY_PATH=../lib:$LD_LIBRARY_PATH
./ros_gui_backend --config ./cfg/config.yaml
```

若安装包内提供 `start.sh`，也可在同一目录执行 `./start.sh`（内部一般为设置 `LD_LIBRARY_PATH` 后带默认配置启动）。不传 `--config` 时程序使用内置默认：HTTP 瓦片端口 **7684**，订阅 **`/map`**，发布 **`/map_manager/map`** 等；可通过 YAML 的 `web_server.port`、`ros_gui_node.sub_map_topic` 等覆盖（字段含义见后端仓库说明）。

**使用要点**：进程需与导航栈同机或能收到地图 topic；默认在 `0.0.0.0:7684` 提供瓦片与 REST（地图列表、切换、拓扑等，详见后端 README）。确保防火墙放行该端口（及 rosbridge 端口）。

**与 App 对接**：瓦片与地图管理 HTTP 基址恒为 `http://<连接页机器人 IP>:<瓦片服务端口>`，默认端口 **7684**。若后端 HTTP 端口不同，在 **设置 → 基础设置 → 瓦片服务端口** 中修改；留空或填 **7684** 表示使用默认。请保证 `ros_gui_backend` 在该 IP 与端口上可达（与 rosbridge 不同机时需自行保证网络可达，应用侧主机名始终与连接页 IP 一致）。

3. 在连接页填写 rosbridge 的 IP 与端口（一般为 **9090**），连接成功后进入 **设置** 页完成适配。

### 适配说明（设置页）

先在 **机器人类型** 中选择 **ROS1** 或 **ROS2** 默认模版，再按实际话题名修改下列项。界面为中文时，名称与下表 **设置项** 一致。下表 **ROS2 默认模版** 列为选用 **ROS2** 模版后的写入值；未写入 prefs 的项取应用内代码回退默认值。

**基础设置**

| 设置项 | 对应功能 | ROS2 默认模版 |
| ------ | -------- | ------------- |
| 瓦片服务端口 | 与连接页 IP 组成 `http://<IP>:<端口>`，用于瓦片与 REST | `7684` |
| 地图坐标系 | 与 OccupancyGrid 的 `frame_id` 一致，用于地图与 TF 对齐 | `map` |
| 机器人基座坐标系 | 与里程计、机器人模型所用 base 系一致（常见为 `base_link`） | `base_link` |
| 图像端口 | `web_video_server` 的 HTTP 端口，与 **图像话题** 一起用于相机画面 | `8080` |

**话题设置**

| 设置项 | 对应功能 | ROS2 默认模版 |
| ------ | -------- | ------------- |
| 地图话题 | 订阅栅格地图（`nav_msgs/OccupancyGrid`） | `map` |
| 激光雷达话题 | 主界面激光扫描显示（`sensor_msgs/LaserScan`） | `scan` |
| 点云话题 | 点云图层（`sensor_msgs/PointCloud2`） | `points` |
| 全局路径话题 | 全局规划路径线（`nav_msgs/Path`） | `/plan` |
| 局部路径话题 | 局部规划路径线（`nav_msgs/Path`） | `/local_plan` |
| 轨迹路径话题 | 已执行/变换后的全局轨迹等路径显示（`nav_msgs/Path`） | `/transformed_global_plan` |
| 重定位话题 | 发布初始位姿（ROS2 多为 `/initialpose`，ROS1 同名或栈定义话题） | `/initialpose` |
| 导航目标话题 | 单点导航目标（ROS2 多为 `/goal_pose`，ROS1 常见 `move_base_simple/goal`） | `/goal_pose` |
| 里程计话题 | 机器人位姿与速度（`nav_msgs/Odometry`） | `/wheel/odometry` |
| 速度控制话题 | 摇杆/遥控发布的 `geometry_msgs/Twist`（常见 `/cmd_vel`） | `/cmd_vel` |
| 电池状态话题 | 电池电量显示（`sensor_msgs/BatteryState`，使用字段 `percentage`，代码按 0–1 比例换算为百分比） | `/battery_status` |
| 图像话题 | 供 `web_video_server` 拉流的图像 topic | `/camera/image_raw` |
| 机器人尺寸话题 | 代价地图上机器人 footprint 多边形（如 `PolygonStamped`） | `/local_costmap/published_footprint` |
| 局部代价地图话题 | 局部 costmap 叠加显示 | `/local_costmap/costmap` |
| 全局代价地图话题 | 全局 costmap 叠加显示 | `/global_costmap/costmap` |

选用 **ROS1** 模版时，与上表不同的默认话题为：全局路径 `/move_base/DWAPlannerROS/global_plan`，局部路径 `/move_base/DWAPlannerROS/local_plan`，导航目标 `move_base_simple/goal`，里程计 `/odom`，图像 `/camera/rgb/image_raw`；其余与 ROS2 默认模版相同（含轨迹路径回退 `/transformed_global_plan`）。

若某功能不用（例如未接电池话题），可保留默认或改为机器人侧实际发布的名称，避免订阅无效 topic。

## Star History

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=chengyangkj/Ros_Flutter_Gui_App&type=Timeline&theme=dark" />
  <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=chengyangkj/Ros_Flutter_Gui_App&type=Timeline" />
  <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=chengyangkj/Ros_Flutter_Gui_App&type=Timeline" width="75%" />
</picture>

## 贡献指南

欢迎提交 Issue 和 Pull Request。详见 [贡献指南](CONTRIBUTING.md)。

## 致谢

- [ros_navigation_command_app 参考界面显示效果](https://github.com/Rongix/ros_navigation_command_app)
- [roslibdart ros通信库](https://pub.dev/packages/roslibdart)

## 许可证

本项目采用 [CC BY-NC-SA 4.0](LICENSE) 许可证。
