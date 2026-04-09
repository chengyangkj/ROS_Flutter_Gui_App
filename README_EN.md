# ROS Flutter GUI App

[中文说明](README.md) | [English](#introduction)

<p align="center">
  <img src="https://img.shields.io/github/last-commit/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub last commit"/>
  <img src="https://img.shields.io/github/stars/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub stars"/>
  <img src="https://img.shields.io/github/forks/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub forks"/>
  <img src="https://img.shields.io/github/issues/chengyangkj/ROS_Flutter_Gui_App?style=flat-square" alt="GitHub issues"/>
  <a href="http://qm.qq.com/cgi-bin/qm/qr?_wv=1027&amp;k=mvzoO6tJQtu0ZQYa_itHW7JrT0i4OCdK&amp;authKey=exOT53pUpRG85mwuSMstWKbLlnrme%2FEuJE0Rt%2Fw6ONNvfHqftoWMay03mk1Qi7yv&amp;noverify=0&amp;group_code=797497206"><img alt="QQ Group" src="https://img.shields.io/badge/QQ%e7%be%a4-797497206-purple"/></a>
  <img src="https://img.shields.io/badge/Flutter-3.29.3-blue?style=flat-square" alt="Flutter version"/>
</p>

<p align="center">
  <img src="https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/app_build.yaml/badge.svg" alt="app build"/>
  <img src="https://github.com/chengyangkj/ROS_Flutter_Gui_App/actions/workflows/ros_humble_build.yaml/badge.svg" alt="ROS 2 Humble backend"/>
</p>

## Introduction

ROS human–machine interface software in a **client–server** layout. Flutter runs the UI on Web, Android, iOS, macOS, and more (layout is mobile-oriented).

From **v2.0** onward, a **custom C++ backend** talks directly to the client for better rendering and throughput—**rosbridge is no longer required**. The last rosbridge-based release is [v1.2.5](https://github.com/chengyangkj/ROS_Flutter_Gui_App/tree/v1.2.5).

| Module | Description |
| --- | --- |
| **backend** | Runs on the robot, interfaces with ROS 2, exposes HTTP, WebSocket, etc.; primarily ROS 2 today (ROS 1 hooks reserved—PRs welcome) |
| **app** | Flutter / Web client talking to that backend |

More detail on APIs, directories, and configuration:

| Doc | Contents |
| --- | --- |
| [backend/README.md](backend/README.md) · [backend/README_EN.md](backend/README_EN.md) | Build & deploy backend, process layout, data dirs, `config.yaml`, HTTP / ROS summary |
| [app/README.md](app/README.md) · [app/README_EN.md](app/README_EN.md) | Frontend deps & build, connectivity, features & settings, workflow |

---

## 1. Build & deploy

### 1.1 From source (recommended: one command at repo root)

```bash
git clone https://github.com/chengyangkj/ROS_Flutter_Gui_App.git
cd ROS_Flutter_Gui_App
./build.sh
```

`build.sh` at the repo root: generates Dart `protobuf` → builds **backend** → `flutter build web`, then copies the Web artifact into `backend/build/install/bin/dist` (for static hosting by the backend). For backend-only, use CMake under `backend/` per that README. For frontend-only, run protocol generation first—see [app/README_EN.md](app/README_EN.md).

### 1.2 Frontend-only (example)

```bash
cd app
flutter pub get
flutter build web --release
```

Other targets (APK, Linux, Windows, etc.) are described in [app/README_EN.md](app/README_EN.md). Match Flutter / Dart to `environment.sdk` in `pubspec.yaml`.

---

## 2. Prebuilt binaries (GitHub Releases)

**Releases** ship backend archives and multi-platform clients for each tag (exact names vary—check the release list):

| Type | Typical filename |
| --- | --- |
| Backend (includes Web `dist`, x86_64) | `backend-<tag>-x86_64.zip` |
| Backend (arm64) | `backend-<tag>-arm64.zip` |
| Web | `app-<tag>-web.tar.gz` |
| Linux | `app-<tag>-linux-x64.tar.gz` |
| Android | `app-<tag>-android.apk` |
| Windows | `app-<tag>-windows-x64.zip` |

---

## 3. Usage

### 3.1 Start the backend

```bash
cd backend/build/install/bin
./start.sh
```

(If you deployed from a Release tarball, point to that package’s `bin` directory.)

### 3.2 Open the frontend

Start the backend from §3.1 first. Default HTTP port is **8080**—for example:

`http://127.0.0.1:8080`

You can also install a client from Releases and point it at the same backend address.

---

## 4. Repository layout

| Path | Description |
| --- | --- |
| `protocol/` | `.proto` definitions; root `build.sh` generates shared Dart + backend stubs |
| `backend/` | C++ backend, CMake project |
| `app/` | Flutter application |
| `build.sh` | One-shot build: protobuf → backend → `flutter build web` + copy `dist` |
