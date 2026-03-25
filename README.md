# ROS GUI Backend（ros_gui_backend）

部署在机器人侧的 **ROS1 / ROS2** 配套后端：单进程内同时提供 **HTTP** 与 **ROS** 节点能力，面向跨平台 GUI 等上位机做能力扩展。**地图与栅格瓦片、拓扑与相关 REST** 是目前已实现的一块业务，后续会在同一架构上继续增加其他功能模块。默认数据目录为 **`~/.maps`**（无 `HOME` 时为当前目录下 **`.maps`**），现阶段主要存放地图相关数据。

---

## 1. 编译

### 1.1 依赖

- Ubuntu 示例：

```bash
sudo apt-get install libsdl2-dev libsdl2-image-dev -y
```

- 已安装 **ROS1** 或 **ROS2**，编译与运行前需 `source /opt/ros/<distro>/setup.bash`。
- 工程通过 CMake 拉取 **yaml-cpp**、**fmt**、**Drogon** 等（见 `cmake/` 下脚本）。

### 1.2 编译命令

```bash
# 标准流程：安装上述系统依赖 + 配置 + 构建 + 安装到 build/install
./build.sh

# 国内镜像：为 Drogon 等依赖指定 Gitee 源（参数透传给 CMake）
./build_cn.sh
```

安装前缀默认为 **`build/install`**（见 `build.sh` 中 `CMAKE_INSTALL_PREFIX`）。

### 1.3 切换 ROS 版本

```bash
# ROS2（默认）
./build.sh

# ROS1
./build.sh -DROS_VERSION=1
```

### 1.4 产物路径

- 可执行文件：**`build/install/bin/ros_gui_backend`**
- 随安装打包的脚本与示例配置：**`build/install/bin/start.sh`**、**`build/install/bin/cfg/config.yaml`**

---

## 2. 运行

运行前 **`source`** 对应 ROS 环境。

### 2.1 推荐方式

```bash
cd build/install/bin
./start.sh
```

`start.sh` 内容为设置 **`LD_LIBRARY_PATH=../lib:...`** 后执行：

`./ros_gui_backend --config ./cfg/config.yaml`

### 2.2 直接启动

```bash
cd build/install/bin
export LD_LIBRARY_PATH=../lib:$LD_LIBRARY_PATH
./ros_gui_backend --config /path/to/config.yaml
```

### 2.3 命令行参数

当前入口仅解析配置文件路径：

| 序号 | 参数 | 说明 |
| --- | --- | --- |
| 1 | `--config <path>` | YAML 配置文件路径 |
| 2 | `--config=<path>` | 同上 |

未指定 `--config` 时，不读取文件，**`MapManager`**、**`GuiBackendConfig`**、**`WebServerConfig`** 使用代码内建默认值（例如 HTTP 端口 **7684**，发布 **`/map_manager/map`**，订阅 **`/map`** 等，与示例 `cfg/config.yaml` 一致）。

---

## 3. 进程架构

```
main.cpp
└── Application
    ├── MapManager（单例）
    │   ├── 地图根目录 ~/.maps、当前图、YAML/PGM、拓扑 JSON、瓦片目录
    │   └── 与 map_io、tiles 生成逻辑协作
    ├── WebServer（Drogon）
    │   └── /tiles/*、地图列表、切换、删除、拓扑查询、编辑提交等
    └── RosGuiNode
        ├── ros2/ 或 ros1/ 实现
        └── 发布栅格地图、ROS2 下订阅 /map 更新内存图并提供 Nav2 相关服务
```

### 3.1 源码目录职责

| 序号 | 路径 | 职责 |
| --- | --- | --- |
| 1 | `src/app/` | `main`、`Application`、YAML 设置加载 |
| 2 | `src/node/` | ROS1/ROS2 节点适配 |
| 3 | `src/core/` | 地图核心、`web_server`、瓦片生成等 |
| 4 | `src/common/` | 日志等公共模块 |

### 3.2 启动时地图加载

**`MapManager::Initialize()`**：若 `~/.maps/current_map` 指向的地图目录下存在 **`<当前地图名>.yaml`**，则尝试加载；否则可在运行后通过 ROS2 订阅 **`/map`** 等方式更新内存地图（见下文 ROS 接口）。

---

## 4. 数据目录约定

```
~/.maps/
├── current_map              # 文本：当前地图目录名
├── <地图名>/
│   ├── <地图名>.yaml        # 栅格地图元数据（列表与切换逻辑依赖此路径）
│   ├── <地图名>.pgm         # 栅格图像（或 yaml 内 image 所指文件）
│   ├── <地图名>.topology    # `/updateMapEdit` 等流程写入的拓扑 JSON
│   ├── map.topology         # `GET /getTopologyMap?map_name=` 当前实现从此文件名读取
│   └── tiles/               # 瓦片缓存 {z}/{x}/{y}.png
└── ...
```

识别为「已安装地图」的目录需满足：存在 **`<地图名>/<地图名>.yaml`**。

---

## 5. HTTP 接口（Drogon）

默认监听 **`0.0.0.0:<端口>`**（配置项 `web_server.port`，默认 **7684**）。已注册 **CORS**（含 `OPTIONS` 预检）。

### 5.1 瓦片与元数据

| 序号 | 方法 | 路径 | 说明 |
| --- | --- | --- | --- |
| 1 | GET | `/tiles/{z}/{x}/{y}` | 当前地图瓦片 PNG |
| 2 | GET | `/tiles/{map_name}/{z}/{x}/{y}` | 指定地图瓦片 |
| 3 | GET | `/tiles/meta` | 当前地图瓦片元信息 JSON |
| 4 | GET | `/tiles/{map_name}/meta` | 指定地图元信息（读取该目录下 `map.yaml`） |
| 5 | POST | `/tiles/config` | 请求体 JSON：调整当前图 `extra_zoom_levels` |
| 6 | POST | `/tiles/{map_name}/config` | 同上，指定地图 |
| 7 | GET | `/tiles/` | 纯文本接口说明 |

客户端常在与路径末尾加 **`.png`**，需与 Drogon 路由或反向代理配置一致。

### 5.2 地图管理

| 序号 | 方法 | 路径 | 说明 |
| --- | --- | --- | --- |
| 1 | GET | `/getAllMapList` | 地图名 JSON 数组 |
| 2 | GET | `/currentMap` | `{"current_map":"..."}` |
| 3 | GET | `/setCurrentMap?name=<地图名>` | 加载 `<名>/<名>.yaml` 并更新 current_map |
| 4 | GET | `/deleteMap?map_name=<地图名>` | 删除地图目录 |
| 5 | GET | `/getTopologyMap` | 当前内存中的拓扑 JSON |
| 6 | GET | `/getTopologyMap?map_name=<地图名>` | 从 `<地图目录>/map.topology` 读取 |

### 5.3 编辑

| 序号 | 方法 | 路径 | 说明 |
| --- | --- | --- | --- |
| 1 | GET | `/updateMapEdit` | Query：`session_id`、`map_name`、`topology_json`、`obstacle_edits_json`（均为 URL 编码字符串） |

---

## 6. ROS 接口

节点名：**`ros_gui_app_backend`**（ROS1 `ros::init` / ROS2 `rclcpp::Node` 名称）。

### 6.1 ROS2

| 序号 | 类型 | 名称 | 说明 |
| --- | --- | --- | --- |
| 1 | 发布 | 配置项 `ros_gui_node.pub_map_topic`（默认 **`/map_manager/map`**） | `nav_msgs/OccupancyGrid`，transient local |
| 2 | 订阅 | 配置项 `ros_gui_node.sub_map_topic`（默认 **`/map`**） | 栅格地图，更新内存与瓦片相关数据 |
| 3 | 服务 | `ros_gui_app_backend/map` | `nav_msgs/srv/GetMap` |
| 4 | 服务 | `ros_gui_app_backend/load_map` | `nav2_msgs/srv/LoadMap` |
| 5 | 服务 | `ros_gui_app_backend/save_map` | `nav2_msgs/srv/SaveMap` |

QoS、保存超时、`free_thresh` / `occupied_thresh` 默认值等见 **`cfg/config.yaml`** 中 `ros_gui_node` 段。

### 6.2 ROS1

| 序号 | 类型 | 名称 | 说明 |
| --- | --- | --- | --- |
| 1 | 发布 | 配置项 `pub_map_topic` | `nav_msgs/OccupancyGrid` |
| 2 | 服务 | **`/static_map`** | `nav_msgs/GetMap` |

当前 ROS1 实现**未**订阅 `/map` 回写内存地图；与 ROS2 行为差异以源码 `src/node/ros1/` 为准。

---

## 7. 配置文件示例

安装目录内 **`bin/cfg/config.yaml`** 结构如下（字段由 `Application` 解析）：

- **`map_manager`**：`frame_id` 等  
- **`ros_gui_node`**：`pub_map_topic`、`sub_map_topic`、`save_map_timeout`、`free_thresh_default`、`occupied_thresh_default`、`map_subscribe_transient_local`  
- **`web_server`**：`port`

---

## 8. 第三方与构建说明

- **HTTP**：**Drogon**
- **序列化 / 配置**：**nlohmann/json**、**yaml-cpp**、**fmt**
- **图像与瓦片**：**OpenCV**、**SDL2** / **SDL2_image**、**Boost.Filesystem**

本仓库以 **CMake** 独立构建，**不**依赖 `topology_msgs` 等额外 ROS 消息包作为编译条件；拓扑数据主要通过磁盘 JSON 与 HTTP 与上位机交互。
