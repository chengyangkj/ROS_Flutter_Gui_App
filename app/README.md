# 前端（Flutter GUI）

[English](README_EN.md)

本目录为 **Flutter** 跨平台客户端，与仓库根目录 **`backend/`** 配套：大地图走 **HTTP 栅格瓦片** 与地图/拓扑 **REST**；机器人状态与控制通过 **WebSocket（protobuf）** 及同一后端的 **HTTP** 接口。支持 Web、Android、iOS、Linux、Windows 等（以当前工程启用的平台为准）。

更完整的仓库说明见根目录 [README.md](../README.md)。

---

## 1. 依赖与环境

- **Flutter** SDK（`flutter doctor` 无阻塞项）
- **Protocol Buffers**：`protoc`、[protoc_plugin](https://pub.dev/packages/protoc_plugin)（`dart pub global activate protoc_plugin`，并保证 `~/.pub-cache/bin` 在 `PATH`）
- 机器人侧需运行本仓库 **后端**，并保证导航等 ROS 栈在线；详见 [backend/README.md](../backend/README.md)

可选：若使用 **web_video_server** 等拉相机流，需与设置中的图像端口、图像话题一致（详见下文「基础设置」）。

---

## 2. 构建

### 2.1 与仓库一体构建（推荐）

在仓库根目录执行 **`./build.sh`**：会生成 `app/lib/protobuf/*`、构建 `backend`，并执行 **`flutter pub get`** 与 **`flutter build web`**。Web 产物目录：**`app/build/web/`**。

### 2.2 仅开发前端

```bash
cd app
flutter pub get
flutter run -d chrome    # 或其它设备
```

若 `.proto` 有变更且未跑根目录 `build.sh`，需在 `protocol/` 下自行用 `protoc` 生成 Dart 输出到 `app/lib/protobuf/`（参数与根 `build.sh` 一致）。

发布 Web：

```bash
cd app
flutter build web
```

将 **`build/web/`** 部署到任意静态文件服务器；本地可 **`python -m http.server`** 在产物目录下试跑。

---

## 3. 与后端的连接关系

| 项 | 说明 |
| --- | --- |
| 连接页 | 填写机器人（或后端所在主机）**IP**，以及后端 **HTTP 监听端口**（与 `config.yaml` 里 `web_server.port` 一致，默认 **8080**） |
| HTTP | 瓦片 URL、`/api/settings`、地图列表与编辑等 REST，基址为 `http://<IP>:<端口>` |
| WebSocket | 浏览器使用 `ws://` 或 `wss://`（页面为 HTTPS 时）连接 **`/ws/robot`**，推送占用格、位姿、路径、诊断等 protobuf |
| 话题与坐标系 | 与 ROS 话题相关的字符串由后端 **`gui_app_settings.json`** 与 **`/api/settings`** 维护；前端设置页修改后会 **POST** 到后端，不再全部落本地 SharedPreferences |

无需在客户端单独配置 **rosbridge** 即可使用上述主流程；旧版纯 rosbridge 地图模式与本 monorepo 当前默认路径不同。

---

## 4. 功能概览

| 功能 | 说明 |
| --- | --- |
| 连接与配置 | 连接页配置 IP 与后端端口；设置页：语言、朝向、手柄映射、图像相关、速度上限等仍存本地；ROS 话题/帧名走后端 |
| 地图显示 | 瓦片底图；叠加激光、点云、全局/局部路径、轨迹、代价地图、footprint、拓扑等（数据来自 WS） |
| 位姿 | 后端在地图坐标系下封装位姿并推送 |
| 重定位与导航 | 通过后端 HTTP 发布初始位姿与导航目标；拓扑与地图编辑走 HTTP |
| 遥控 | 屏幕摇杆与手柄映射；速度经后端 **`/robot/cmd_vel`** |
| 相机 | 图像话题订阅由后端转发到 WS；失败时见占位或无画面 |
| 地图编辑 | 障碍与拓扑编辑通过 REST 与后端交互 |
| 诊断 | 后端推送 `DiagnosticArray`；主界面可对 ERROR/WARN Toast |
| 电池 | 后端推送电池状态 |
| 国际化 | 中/英；横竖屏等应用侧设置 |
| SSH | 见下文 **§4.1** |

### 4.1 SSH 远程（快捷指令 / 终端）

与地图、遥控使用**同一后端 HTTP 地址**。SSH 不是浏览器直连 TCP，而是：

1. **连接机器人**：在连接页填写后端 IP 与端口并连上后，**SSH 隧道目标主机与当前机器人 IP 一致**（保存到后台的 `sshHost` 与 `robotIp` 同步）。
2. **隧道**：客户端通过 **`ws://` / `wss://`**（页面为 HTTPS 时用 `wss`）访问后端的 **`/ws/ssh`**，由后端再 **TCP 连接到** `gui_app_settings.json` 中的 `sshHost:sshPort`（一般为远端 `sshd`）。因此需在 **设置 → SSH** 中配置 **端口、SSH 用户名与密码** 并保存到后台。
3. **快捷指令**：主界面可打开「SSH 快捷指令」列表。每条指令可单独打开 **「sudo 执行」**：开启后，实际远端执行形式为  
   `echo '<SSH密码>' | sudo -S sh -c '<命令>'`  
   以便用 **当前填写的 SSH 登录密码** 通过 `sudo -S` 提权（请确保该用户具备 sudo 且密码与 SSH 密码一致，否则应关闭 sudo 选项或改用无 sudo 的命令）。**命令内容可写实际 shell 行**（如 `shutdown -h now`）；若仍带前缀 `sudo `，发送前会自动去掉一层，避免重复。
4. **SSH 终端**：交互式 shell；Web 与移动端均可使用隧道，无需浏览器原生 TCP。
5. **安全提示**：密码经 WebSocket 传输时，若页面为 HTTP 则链路未加密；公网请使用 **HTTPS + WSS**。sudo 通过命令行传密码在部分系统上可能被 `ps` 看到，高安全场景请使用密钥或专用运维通道。

配置与指令列表持久化在机器人侧 **`gui_app_settings.json`**（经 **`/api/settings`**），与仓库内后端 **`GuiAppSettings`** 字段一致。

---

## 5. 推荐使用流程

1. 在机器人上 **`source`** ROS 后启动 **backend**（见 [backend/README.md](../backend/README.md)），并启动导航等必要节点。
2. 打开应用（浏览器或桌面），在连接页输入 **IP** 与 **后端 HTTP 端口**（默认 **8080**），连接成功后会拉取 **`/api/settings`** 应用 GUI 配置。
3. 在 **设置** 中按需调整应用侧选项（图像、速度上限、语言等）；若需改 ROS 话题名，保存时会同步到后端（需后端可达）。
4. 在主界面使用地图图层、遥控、导航、诊断、地图编辑等。

---

## 6. 设置项说明

### 6.1 基础设置（多为本地持久化）

| 设置项 | 作用 | 典型值 |
| --- | --- | --- |
| IP | 后端所在主机 | 连接页同步 |
| 端口 | 历史字段名，现用作 **后端 HTTP 端口** | **8080** |
| HTTP 服务端口（若单独一项） | 与瓦片/base URL 一致 | 默认或自定义 |
| 最大线/角速度 | 遥控上限 | 模版如 `0.9` |
| 机器人图标尺寸 | 地图上显示大小 | 模版写入 |
| 图像端口 / 宽 / 高 | 相机 MJPEG 等 | `8080`、`640×480` |

与 **`map_frame` / `base_link`、各类 ROS topic** 等对应项在设置页仍可编辑，但持久化在后端，见 [backend/README.md](../backend/README.md) 中 **`/api/settings`** 与 **`NodeConfig` / `GuiAppSettings`**。

### 6.2 话题与模版

**ROS1 / ROS2** 两套模版用于快速填充后端侧默认话题；切换模版会重置本地/App 侧部分默认值并尝试同步到后端。具体键名与消息类型与上游 ROS 栈一致，可对照后端 `cfg/config.yaml` 中 **`ros_gui_node`** 段。

---

## 7. 已知限制

- 大地图以 **瓦片 + 后端** 为主路径；极端弱网下仍需关注加载与缓存。
- **Web** 上相机与跨域、HTTPS/WSS 策略相关，失败时可能无画面。
- 超大点云/高频话题仍依赖后端与网络压缩策略。

---

## 8. 许可与致谢

**[LICENSE](LICENSE)** 
