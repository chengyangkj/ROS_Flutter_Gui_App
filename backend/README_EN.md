# Backend (`ros_gui_backend`)

[中文](README.md)

Single-process service on the robot: **ROS 1 / ROS 2** node logic plus **HTTP (Drogon)** and **WebSocket robot stream** in one process, serving Flutter / Web with tiled maps, map-management REST, persisted GUI topic config, and more. Default map data root is **`~/.maps`** (or **`./.maps`** when `HOME` is unset).

---

## 1. Build & dependencies

### 1.1 System packages (example: Ubuntu)

```bash
sudo apt-get install libsdl2-dev libsdl2-image-dev protobuf-compiler libc-ares-dev -y
```

- Install **ROS 1** or **ROS 2**; `source /opt/ros/<distro>/setup.bash` before build/run.
- CMake pulls **yaml-cpp**, **fmt**, **Drogon**, etc. via scripts under `cmake/`.

### 1.2 One-shot build from repository root (recommended)

At **`nav2_map_server` repo root**:

```bash
./build.sh
```

Runs: **protocol → Dart protobuf** → **this backend (CMake)** → **Flutter Web**. For backend alone you can CMake only in `backend/` (next section).

### 1.3 Build this directory only

```bash
cd backend
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=build/install
cmake --build build --parallel
```

Switch ROS version:

```bash
cmake -S . -B build -DROS_VERSION=1   # ROS 1; default is ROS 2
```

You can pass mirror CDN flags into CMake when needed; use `backend/build_cn.sh` if present.

### 1.4 Install layout

- Binary: **`backend/build/install/bin/ros_gui_backend`**
- Launcher + sample config: **`backend/build/install/bin/start.sh`**, **`backend/build/install/bin/cfg/config.yaml`**

---

## 2. Run

**`source`** the right ROS workspace first.

### 2.1 Recommended

```bash
cd backend/build/install/bin
./start.sh
```

`start.sh` sets **`LD_LIBRARY_PATH=../lib:...`** then runs:

`./ros_gui_backend --config ./cfg/config.yaml`

### 2.2 Manual

```bash
cd backend/build/install/bin
export LD_LIBRARY_PATH=../lib:$LD_LIBRARY_PATH
./ros_gui_backend --config /path/to/config.yaml
```

### 2.3 CLI flags

| Flag | Meaning |
| --- | --- |
| `--config <path>` | YAML config path |
| `--config=<path>` | Same |

If **`--config`** is omitted, no file is read and **`MapManager`**, **`NodeConfig`**, **`WebServerConfig`** use built-in defaults (e.g. HTTP **8080**, publish **`/map_manager/map`**, subscribe **`/map`**, aligned with sample `cfg/config.yaml`).

---

## 3. Process & source layout

```
main.cpp
└── Application
    ├── MapManager (singleton)
    │   ├── Map root ~/.maps, active map, YAML/PGM, topology JSON, tile dirs
    │   └── Works with map_io & tile generation
    ├── WebServer (Drogon)
    │   ├── /tiles/*, map list/switch/delete, topology, edit, /api/settings, /robot/*, …
    │   └── WebSocket /ws/robot (protobuf RobotMessage stream)
    └── RosGuiNode
        ├── ros2/ or ros1/ implementation
        └── Subscribes to laser/path/odom, pushes to WS; ROS 2 subscribes /map, updates in-memory map and Nav2-related services
```

| Path | Role |
| --- | --- |
| `src/app/` | `main`, `Application`, YAML load |
| `src/node/` | ROS node adapters, `node_config.hpp` |
| `src/core/` | Map core, `web_server`, robot_stream, tiles |
| `src/common/` | Logging, `config/` (GuiAppSettings persistence, etc.) |

**`MapManager::Initialize()`**: if **`~/.maps/current_map`** points at a folder containing **`<map_name>.yaml`**, it loads; otherwise the map can be filled later via ROS 2 **`/map`**, etc.

---

## 4. Data directories

```
~/.maps/
├── current_map              # text: active map folder name
├── <map_name>/
│   ├── <map_name>.yaml
│   ├── <map_name>.pgm
│   ├── <map_name>.topology
│   ├── map.topology         # GET /getTopologyMap?map_name= …
│   └── tiles/               # tiles {z}/{x}/{y}.png
└── ...
```

Installed maps need **`<map_name>/<map_name>.yaml`**.

GUI topic preferences are stored in **`gui_app_settings.json`** (path resolved by `Application` / `Config`, often cwd).

---

## 5. HTTP API summary

Listens on **`0.0.0.0:<port>`** (`web_server.port`, default **8080**). **CORS** registered (including `OPTIONS`).

### 5.1 Tiles & metadata

| Method | Path | Notes |
| --- | --- | --- |
| GET | `/tiles/{z}/{x}/{y}` | Current map PNG tile |
| GET | `/tiles/{map_name}/{z}/{x}/{y}` | Named map tile |
| GET | `/tiles/meta` | Current map tile meta JSON |
| GET | `/tiles/{map_name}/meta` | Named map meta |
| POST | `/tiles/config` | JSON: `extra_zoom_levels` |
| POST | `/tiles/{map_name}/config` | Same |
| GET | `/tiles/` | Human-readable API blurb |

Clients often append **`.png`**—keep routing / reverse proxy consistent.

### 5.2 Map management

| Method | Path | Notes |
| --- | --- | --- |
| GET | `/getAllMapList` | JSON array of map names |
| GET | `/currentMap` | `{"current_map":"..."}` |
| GET | `/setCurrentMap?name=` | Switch active map |
| GET | `/deleteMap?map_name=` | Delete map folder |
| GET | `/getTopologyMap` | In-memory topology |
| GET | `/getTopologyMap?map_name=` | Read topology from disk |

### 5.3 Editing & misc.

| Method | Path | Notes |
| --- | --- | --- |
| GET | `/updateMapEdit` | Query: `session_id`, `map_name`, `topology_json`, `obstacle_edits_json` (URL-encoded) |
| GET | `/api/settings` | GUI topics / frames JSON |
| POST | `/api/settings` | Merge write + rebind ROS stream |
| POST | `/robot/cmd_vel`, `/robot/nav_goal`, `/robot/initial_pose`, … | Control endpoints used by the app |

Authoritative route list: **`web_server.cpp`**.

---

## 6. ROS interfaces

Node name: **`ros_gui_app_backend`**.

### 6.1 ROS 2

| Kind | Name | Notes |
| --- | --- | --- |
| Pub | **`/map_manager/map`** (constant in code) | `nav_msgs/OccupancyGrid`, transient local |
| Sub | **`/map`** (constant) | Refresh in-memory map + tile pipeline |
| Srv | `ros_gui_app_backend/map` | `nav_msgs/srv/GetMap` |
| Srv | `ros_gui_app_backend/load_map` | `nav2_msgs/srv/LoadMap` |
| Srv | `ros_gui_app_backend/save_map` | `nav2_msgs/srv/SaveMap` |

**SaveMap** QoS for temporary subs, timeouts, default `free_thresh` / `occupied_thresh` live as constants in **`node/ros2/node.cpp`** (same as prior YAML defaults)—not overridden from config.

### 6.2 ROS 1

| Kind | Name | Notes |
| --- | --- | --- |
| Pub | **`/map_manager/map`** (constant) | `nav_msgs/OccupancyGrid` |
| Srv | **`/static_map`** | `nav_msgs/GetMap` |

ROS 1 **does not** subscribe `/map` to rewrite the in-memory map; see **`src/node/ros1/`** for details.

---

## 7. Configuration file

Installed **`bin/cfg/config.yaml`** is parsed by **`Application`**:

- **`map_manager`**: `frame_id`, etc.
- **`ros_gui_node`**: `map_frame`, `base_link_frame`, GUI/ROS topic fields (**`NodeConfig`**, merged with **`gui_app_settings.json`** / **`GuiAppSettings`**); map pub/sub names and SaveMap defaults are fixed in source
- **`web_server`**: `port`

---

## 8. Third-party & notes

- **HTTP**: Drogon  
- **JSON / config**: nlohmann/json, yaml-cpp, fmt  
- **Images / tiles**: OpenCV, SDL2 / SDL2_image, Boost.Filesystem  

Build does **not** hard-depend on `topology_msgs`; topology is mostly disk JSON + HTTP to the GUI.
