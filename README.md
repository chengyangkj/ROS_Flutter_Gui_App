# ROS Flutter GUI App

[中文说明](#1-项目简介) | [English](README_EN.md)

<p align="center">
  <img src="https://img.shields.io/github/last-commit/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub last commit"/>
  <img src="https://img.shields.io/github/stars/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub stars"/>
  <img src="https://img.shields.io/github/forks/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub forks"/>
  <img src="https://img.shields.io/github/issues/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub issues"/>
  <a href="http://qm.qq.com/cgi-bin/qm/qr?_wv=1027&amp;k=mvzoO6tJQtu0ZQYa_itHW7JrT0i4OCdK&amp;authKey=exOT53pUpRG85mwuSMstWKbLlnrme%2FEuJE0Rt%2Fw6ONNvfHqftoWMay03mk1Qi7yv&amp;noverify=0&amp;group_code=797497206"><img alt="QQ 群" src="https://img.shields.io/badge/QQ%e7%be%a4-797497206-purple"/></a>
  <img src="https://img.shields.io/badge/Flutter-3.29.3-blue?style=flat-square" alt="Flutter version"/>
</p>

<p align="center">
  <img src="https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/app_build.yaml/badge.svg" alt="app build"/>
  <img src="https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/ros_humble_build.yaml/badge.svg" alt="ROS 2 Humble backend"/>
</p>

## 1. 项目简介

基于 C/S 架构的 ROS 人机交互软件。借助 Flutter 的跨端能力，可在 Web、Android、iOS、macOS 等多端运行（界面布局侧重移动端）。

自 v2.0 起，为提高渲染与通信效率，采用**自定义 C++ 后端**与前端直连，**不再依赖 rosbridge**。仍使用 rosbridge 的版本见 [v1.2.5](https://github.com/chengyangkj/ROS_Flutter_Gui_App/tree/v1.2.5)。

| 模块 | 说明 |
| --- | --- |
| **backend** | 运行于机器人侧，与 ROS 2 对接，提供 HTTP、WebSocket 等接口；当前主要兼容 ROS 2（ROS 1 接口预留，欢迎 PR） |
| **app** | Flutter 客户端 / Web 前端，与后端通信并完成展示 |

更细的接口、目录与配置见：

| 文档 | 内容 |
| --- | --- |
| [backend/README.md](backend/README.md) · [backend/README_EN.md](backend/README_EN.md) | 后端编译与部署、进程结构、数据目录、`config.yaml`、HTTP / ROS 摘要 |
| [app/README.md](app/README.md) · [app/README_EN.md](app/README_EN.md) | 前端依赖与构建、连接方式、功能与设置、使用流程 |

---

## 2. 项目预览

界面以**瓦片地图**为画布，叠加机器人位姿、激光/路径/代价等图层；顶部为速度/电池/导航与诊断状态，左右为图层与工具，可拖动的**相机浮窗**便于边看图边操作。

<p align="center">
  <img src="doc/image/main_page.jpg" alt="主界面：地图与图层" width="78%" />
</p>
<p align="center"><sub>主界面 · 瓦片地图 · 图层开关 · 遥控 / 导航入口</sub></p>

<p align="center">
  <img src="doc/image/map_edit_page.jpg" alt="地图编辑" width="78%" />
</p>
<p align="center">
  <img src="doc/image/map_manager_page.jpg" alt="地图管理" width="78%" />
</p>
<p align="center"><sub>地图编辑管理· 障碍与拓扑 </sub></p>

<p align="center">
  <img src="doc/image/ssh_page.jpg" alt="ssh" width="78%" />
</p>
<p align="center">
  <img src="doc/image/ssh_quick_cmd_page.jpg" alt="ssh qucik cmd" width="78%" />
</p>
<p align="center"><sub>SSH 功能</sub></p>



---

## 3. 功能一览

| 类别 | 内容 |
| --- | --- |
| **地图** | 高性能瓦片栅格地图；全局/局部路径、轨迹、代价地图、footprint、拓扑等图层（数据经 WebSocket 推送） |
| **感知** | 激光扫描、点云叠加显示；可选 **相机 MJPEG** 浮窗（话题由后端配置） |
| **导航** | 导航状态与目标点；重定位；停止导航；与 Nav2 / 后端接口协同 |
| **编辑与管理** | 地图与拓扑编辑、列表与切换（HTTP REST） |
| **诊断** | 订阅诊断消息，主界面提示 ERROR / WARN 等 |
| **运维** | **SSH 终端**与**快捷指令**（经后端 `/ws/ssh` 隧道，详见 [app/README.md](app/README.md) §4.1） |

---

## 4. 编译与部署

### 4.1 从源码构建（推荐：仓库根目录一键）

```bash
git clone https://github.com/chengyangkj/ROS_Flutter_Gui_App.git
cd ROS_Flutter_Gui_App
./build.sh
```

根目录 `build.sh` 会依次：生成 Dart `protobuf` → 构建 backend → `flutter build web`，并将 Web 产物同步到 `backend/build/install/bin/dist`（供后端静态托管）。仅构建后端时可进入 `backend/` 按该目录 README 使用 CMake；仅构建前端时需先完成协议生成，见 [app/README.md](app/README.md)。

### 4.2 仅构建前端（示例）

```bash
cd app
flutter pub get
flutter build web --release
```

其他目标（APK、Linux、Windows 等）参见 [app/README.md](app/README.md)。建议 Flutter / Dart 版本与工程 `pubspec.yaml` 中 `environment.sdk` 一致。

---

## 5. 预构建产物（GitHub Releases）

如果不想手动配置环境编译运行，可在 **Releases** 页面可下载与当前 tag 对应的后端压缩包与多平台客户端，例如（名称随版本变化，以 Release 列表为准）：

| 类型 | 典型文件名 |
| --- | --- |
| 后端（含 Web `dist`，x86_64） | `backend-<tag>-x86_64.zip` |
| 后端（arm64） | `backend-<tag>-arm64.zip` |
| Web | `app-<tag>-web.tar.gz` |
| Linux | `app-<tag>-linux-x64.tar.gz` |
| Android | `app-<tag>-android.apk` |
| Windows | `app-<tag>-windows-x64.zip` |

---

## 6. 使用说明

### 6.1 启动后端

```bash
cd backend/build/install/bin
./start.sh
```

（若从 Release 解压部署，请将路径换为解压后的 `bin` 目录。）

### 6.2 打开前端

先启动 6.1 中的后端。默认 HTTP 端口为 **8080**，浏览器访问例如：

`http://127.0.0.1:8080`

亦可从 Release 下载对应平台的客户端，配置同一后端地址进行连接。

---

## 7. 仓库结构

| 路径 | 说明 |
| --- | --- |
| `protocol/` | `.proto` 定义；根目录 `build.sh` 生成 Dart 与后端共用的协议代码 |
| `backend/` | C++ 后端，CMake 工程 |
| `app/` | Flutter 应用 |
| `build.sh` | 根目录一键构建（protobuf → backend → `flutter build web` + 拷贝 `dist`） |


## 8. 📊 Star 历史

<div align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=chengyangkj/ROS_Flutter_Gui_App&type=Timeline&theme=dark" />
    <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=chengyangkj/ROS_Flutter_Gui_App&type=Timeline" />
    <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=chengyangkj/ROS_Flutter_Gui_App&type=Timeline" width="75%" />
  </picture>
</div>
