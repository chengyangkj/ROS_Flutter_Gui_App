# ROS1/ROS2 Map Manager

基于 Nav2 的地图服务，支持 ROS1/ROS2，集成 HTTP 瓦片服务、栅格地图与拓扑地图管理。地图数据存储在 `~/.maps` 目录。

## 编译

### 依赖

```bash
# Ubuntu
sudo apt-get install libsdl2-dev libsdl2-image-dev -y
```

需已安装 ROS1 或 ROS2，并 `source /opt/ros/<distro>/setup.bash`。

### 编译命令

```bash
# 标准编译（build.sh 会执行 apt 安装依赖 + cmake + make + install）
./build.sh

# 国内加速（使用 Gitee 镜像拉取 topology_msgs 等）
./build_cn.sh
```

输出目录：`build/install/`，可执行文件在 `build/install/bin/map_manager`。

### 切换 ROS 版本

```bash
# ROS2（默认）
./build.sh

# ROS1
./build.sh -DROS_VERSION=1
```

## 运行

```bash
# 进入安装目录并执行（需先 source ROS）
cd build/install/bin
./map_manager.sh
```

或直接运行可执行文件：

```bash
./build/install/bin/map_manager [选项]
```

运行前需 `source` 对应 ROS 环境。

## 架构

```
main.cpp                    # 入口：解析参数，初始化 MapManager 与 ROS 节点
├── MapManager              # 核心：地图加载、瓦片生成、HTTP 接口
│   ├── map_io              # 栅格地图与拓扑地图读写
│   ├── tiles_map_generator # 瓦片生成
│   └── HTTP Server         # 瓦片与 REST 接口
└── MapServerNode           # ROS 接口：topic/service，桥接 MapManager
    ├── ros1/map_server_node
    └── ros2/map_server_node
```

### 模块说明

| 模块 | 路径 | 职责 |
|------|------|------|
| app | src/app/ | main、MapManager（地图逻辑、HTTP） |
| node | src/node/ | ROS 节点，提供 topic/service |
| core | src/core/ | map_io、tiles_map_generator、topology_map |
| common | src/common/ | 日志等公共组件 |

### 数据目录

```
~/.maps/
├── current_map          # 当前地图名
├── map_name_1/          # 地图目录
│   ├── map_name_1.yaml  # 或 map.yaml（栅格元数据）
│   ├── map_name_1.pgm   # 栅格图像
│   ├── map_name_1.topology  # 拓扑地图 JSON
│   └── tiles/           # 瓦片缓存
│       └── {z}/{x}/{y}.png
└── map_name_2/
    └── ...
```

## 启动方式

```bash
# ROS2
./map_manager [选项]

# 常用选项
--yaml_filename, -y    初始地图 yaml（不存在则不加载，从 ROS topic 获取）
--pub_map_topic, -t    发布地图 topic，默认 /map_manager/map
--sub_map_topic        订阅地图 topic，默认 /map
--frame_id, -f         frame_id，默认 map
--tiles_http_port      HTTP 瓦片服务端口，默认 7684
--config, -c           配置文件路径
```

地图来源优先顺序：若 `yaml_filename` 存在则加载；否则若有 current_map 且对应 yaml 存在则加载；否则不加载，等待 ROS topic 发布地图。

## HTTP 接口

默认 `http://0.0.0.0:7684`，支持 CORS。

### 瓦片

| 接口 | 说明 |
|------|------|
| GET /tiles/{z}/{x}/{y}.png | 当前地图瓦片 |
| GET /tiles/{map_name}/{z}/{x}/{y}.png | 指定地图瓦片 |
| GET /tiles/meta | 当前地图元信息 |
| GET /tiles/{map_name}/meta | 指定地图元信息 |
| POST /tiles/config | 设置当前地图 extra_zoom_levels |
| POST /tiles/{map_name}/config | 设置指定地图 extra_zoom_levels |
| GET /tiles/ | 接口说明页 |

### 地图管理

| 接口 | 说明 |
|------|------|
| GET /getAllMapList | 地图名列表 |
| GET /currentMap | 当前地图名 |
| GET /setCurrentMap?name=xxx | 切换当前地图 |
| GET /deleteMap?map_name=xxx | 删除地图目录 |
| GET /getTopologyMap | 当前拓扑 JSON |
| GET /getTopologyMap?map_name=xxx | 指定地图拓扑 JSON |

### 编辑

| 接口 | 说明 |
|------|------|
| GET /updateMapEdit | 提交编辑，query: session_id, map_name, topology_json, obstacle_edits_json |

## ROS 接口

- **服务**（节点名 map_manager）：`map_manager/map`(GetMap)、`map_manager/load_map`、`map_manager/save_map`
- **发布**：`/map_manager/map`（occupancy grid）、`/map/topology`（拓扑）
- **订阅**：`/map`（occupancy grid，收到后更新瓦片并保存）
- 收到地图更新且 current_map 为空时，自动设为 `map` 并保存到 `~/.maps/map/`

话题名可通过 `--pub_map_topic`、`--sub_map_topic` 修改。

## 依赖

- ROS1 或 ROS2
- Boost、OpenCV、yaml-cpp、cpp-httplib
- topology_msgs（ROS2）
