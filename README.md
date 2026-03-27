# 地图服务与 GUI（nav2_map_server）

单仓库集成机器人侧 **后端**（ROS + HTTP + WebSocket）与 **Flutter** 上位机：后端负责栅格瓦片、地图/拓扑 REST、GUI 话题配置持久化及向客户端推送融合后的机器人数据；前端通过 **HTTP** 与 **`/ws/robot`** 与后端通信。

**详细设计、接口与目录说明请阅：**

| 文档 | 内容 |
| --- | --- |
| [backend/README.md](backend/README.md) | 后端编译与部署、进程结构、数据目录、`config.yaml`、HTTP/ROS 接口摘要 |
| [app/README.md](app/README.md) | 前端依赖与构建、与后端连接方式、功能与设置、使用流程 |

---

## 1. 部署

### 1.1 后端部署

1. **系统依赖（示例）**：Ubuntu 上安装 **SDL2 / SDL2_image**、**protobuf-compiler**，并已安装 **ROS1 或 ROS2**，每次构建/运行前 **`source /opt/ros/<distro>/setup.bash`**。  
2. **一键构建**（推荐，在仓库根目录）：依赖 **`flutter`**、**`protoc`**、**`protoc-gen-dart`** 均在 `PATH` 中，执行：
   ```bash
   ./build.sh
   ```
   将生成 Dart protobuf、编译 **backend**（安装前缀默认 **`backend/build/install`**），并构建 **Flutter Web**（`app/build/web`）。  
3. **仅部署后端**：也可进入 **`backend/`** 按 [backend/README.md](backend/README.md) 单独 `cmake` 安装；运行 **`backend/build/install/bin/start.sh`**（或带 **`--config`** 的可执行文件）。  
4. 默认 HTTP 端口 **7684**，地图数据目录 **`~/.maps`**；详见后端文档。

### 1.2 前端部署

1. **依赖**：**Flutter**；与根目录 **`build.sh`** 一致时还需 **`protoc`** 与 **`protoc_gen_dart`**。  
2. **构建**：根目录 **`./build.sh`** 会在末尾执行 **`flutter pub get`** 与 **`flutter build web`**；仅前端时可进入 **`app/`** 自行 **`flutter build web`**（需已生成 **`app/lib/protobuf/`**，见 [app/README.md](app/README.md)）。  
3. **发布**：将 **`app/build/web/`** 置于 **Nginx**、对象存储或 **`python -m http.server`** 等静态资源服务；**Web** 访问 `https` 时，客户端会以 **`wss://`** 连后端 **`/ws/robot`**，需网关与证书配置一致。

---

## 2. 使用

### 2.1 后端使用

1. 在机器人（或网关机）上 **`source`** ROS，启动 **`ros_gui_backend`**（推荐 **`cd backend/build/install/bin && ./start.sh`**）。  
2. 保证 **`config.yaml`** 中 **`web_server.port`** 与防火墙策略对前端可达；ROS 侧话题与 **`ros_gui_node`** 段与现场 Nav2/导航栈一致。  
3. 地图切换、拓扑、**`/api/settings`** 等行为见 [backend/README.md](backend/README.md)。

### 2.2 前端使用

1. 浏览器或桌面客户端打开已部署的 Flutter 应用（**Web** 需通过上述静态服务访问）。  
2. **连接页**填写运行后端的 **IP** 与 **HTTP 端口**（默认 **7684**）；成功后将拉取 **`/api/settings`** 并建立 **WebSocket**。  
3. **设置** 中区分：应用本地项（语言、图像尺寸、摇杆上限等）与由后端持久化的话题/坐标系等；地图、遥控、导航、编辑、诊断等见 [app/README.md](app/README.md) 中的流程与表格。

---

## 3. 仓库结构（索引）

| 路径 | 说明 |
| --- | --- |
| `protocol/` | `.proto`，根 `build.sh` 生成 **Dart** 与后端共用约定 |
| `backend/` | C++ 后端，CMake 子工程 |
| `app/` | Flutter 工程 |
| `build.sh` | 根目录一键：protobuf → backend → `flutter build web` |
