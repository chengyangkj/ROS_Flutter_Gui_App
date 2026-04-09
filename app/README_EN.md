# Frontend (Flutter GUI)

[中文](README.md)

Cross-platform **Flutter** client paired with **`backend/`** in this repo: large maps use **HTTP raster tiles** and map/topology **REST**; robot state and control use **WebSocket (protobuf)** plus the same backend’s **HTTP** APIs. Web, Android, iOS, Linux, Windows, etc. are supported per enabled platforms.

Full repo overview: root [README.md](../README.md) · [README_EN.md](../README_EN.md).

---

## 1. Dependencies & environment

- **Flutter** SDK (`flutter doctor` clean)
- **Protocol Buffers**: `protoc`, [protoc_plugin](https://pub.dev/packages/protoc_plugin) (`dart pub global activate protoc_plugin`, put `~/.pub-cache/bin` on `PATH`)
- Run this repo’s **backend** on the robot with the navigation stack up; see [backend/README.md](../backend/README.md) · [backend/README_EN.md](../backend/README_EN.md)

Optional: for **web_video_server** or similar camera streams, match image port and topic in settings (see **Basic settings** below).

---

## 2. Build

### 2.1 Monorepo build (recommended)

From repo root run **`./build.sh`**: generates `app/lib/protobuf/*`, builds **backend**, runs **`flutter pub get`** and **`flutter build web`**. Web output: **`app/build/web/`**.

### 2.2 Frontend-only development

```bash
cd app
flutter pub get
flutter run -d chrome    # or another device
```

If `.proto` changes and you did not run root `build.sh`, regenerate Dart under `app/lib/protobuf/` from `protocol/` with `protoc` (same args as root `build.sh`).

Production Web:

```bash
cd app
flutter build web
```

Serve **`build/web/`** from any static host; locally you can use **`python -m http.server`** inside that folder.

---

## 3. How the client connects to the backend

| Item | Description |
| --- | --- |
| Connect screen | Robot (or backend host) **IP** and backend **HTTP port** (matches `web_server.port` in `config.yaml`, default **8080**) |
| HTTP | Tile URLs, `/api/settings`, map list & edit REST; base `http://<IP>:<port>` |
| WebSocket | Browser uses `ws://` or `wss://` (when page is HTTPS) for **`/ws/robot`**—occupancy updates, pose, path, diagnostics, etc. as protobuf |
| Topics & frames | ROS/RW strings come from backend **`gui_app_settings.json`** and **`/api/settings`**; changes from the Settings screen are **POST**ed to the backend, not only local SharedPreferences |

You **do not** need a separate **rosbridge** setup for this flow; older rosbridge-only map modes differ from this monorepo’s default.

---

## 4. Feature overview

| Feature | Description |
| --- | --- |
| Connect & settings | Connect screen: IP + backend port; settings: language, orientation, gamepad mapping, camera-related, speed limits stay local; ROS topics / frame names live on the backend |
| Map | Tile basemap; overlays: laser, point cloud, global/local path, trajectory, costmaps, footprint, topology (WS data) |
| Pose | Pose in map frame, packaged by backend |
| Reloc & nav | Initial pose and goals via backend HTTP; topology & map edit via HTTP |
| Teleop | On-screen joystick & gamepad mapping; velocities via **`/robot/cmd_vel`** |
| Camera | Image topic bridged through backend to WS; failures show placeholder or blank |
| Map edit | Obstacles & topology via REST |
| Diagnostics | `DiagnosticArray` from backend; toasts for ERROR/WARN on main UI |
| Battery | From backend stream |
| i18n | Chinese / English; portrait/landscape app settings |
| SSH | See **§4.1** |

### 4.1 SSH remote (quick commands / terminal)

Same backend **HTTP host/port** as the map and teleop stack. SSH is **not** raw browser TCP:

1. **Connect**: After filling IP/port on the connect screen, **SSH tunnel target matches the current robot IP** (`sshHost` in sync with `robotIp` on the server).
2. **Tunnel**: Client opens **`/ws/ssh`** over `ws` / `wss` (use `wss` when the page is HTTPS); backend opens **TCP** to `sshHost:sshPort` from `gui_app_settings.json` (usually remote `sshd`). Configure **port, SSH user, password** under **Settings → SSH** and save to the backend.
3. **Quick commands**: List on main UI. Each entry can enable **sudo**: remote command runs as  
   `echo '<SSH password>' | sudo -S sh -c '<command>'`  
   so the **current SSH login password** feeds `sudo -S` (user must have sudo and password must match; otherwise turn off sudo or avoid privileged commands). You can enter normal shell lines (e.g. `shutdown -h now`); a leading `sudo ` is stripped once if present.
4. **SSH terminal**: Interactive shell; Web and mobile use the tunnel—no native browser TCP.
5. **Security**: Passwords over WS are cleartext if the page is HTTP; use **HTTPS + WSS** on public networks. Passing sudo passwords on the command line may be visible to `ps` on some systems—use keys or a hardened ops path for high-security deployments.

Settings and command lists persist on the robot in **`gui_app_settings.json`** (via **`/api/settings`**), matching backend **`GuiAppSettings`**.

---

## 5. Recommended workflow

1. On the robot **`source`** ROS, start **backend** (see backend README), plus navigation and required nodes.
2. Open the app (browser or desktop); on the connect screen enter **IP** and **backend HTTP port** (default **8080**); after connect, **`/api/settings`** loads GUI config.
3. Adjust app-only options (camera, speed caps, language, …) in **Settings**; changing ROS topic names saves to the backend (backend must be reachable).
4. Use layers, teleop, nav, diagnostics, map edit on the main screen.

---

## 6. Settings reference

### 6.1 Basic settings (mostly local)

| Setting | Role | Typical value |
| --- | --- | --- |
| IP | Backend host | Same as connect screen |
| Port | Legacy name; used as **backend HTTP port** | **8080** |
| HTTP service port (if separate) | Must match tile base URL | default or custom |
| Max linear / angular speed | Teleop caps | e.g. `0.9` |
| Robot icon size | On-map marker | template |
| Image port / width / height | MJPEG camera | `8080`, `640×480` |

**`map_frame` / `base_link` / ROS topics** remain editable in Settings but persist on the backend—see backend README **`/api/settings`** and **`NodeConfig` / `GuiAppSettings`**.

### 6.2 Topic templates

**ROS 1 / ROS 2** templates pre-fill backend-side defaults; switching template resets some local/app defaults and tries to sync upstream. Keys and message types follow your ROS stack—compare backend `cfg/config.yaml` **`ros_gui_node`** section.

---

## 7. Known limitations

- Large maps are **tiles + backend**; very poor networks still affect load and cache.
- **Web** camera depends on CORS and HTTPS/WSS policy; failures may show no image.
- Huge point clouds / high-rate topics still depend on backend and network efficiency.

---

## 8. License & credits

**[LICENSE](../LICENSE)**
